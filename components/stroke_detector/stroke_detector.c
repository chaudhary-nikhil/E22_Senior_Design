/**
 * @file stroke_detector.c
 * @brief On-device stroke detection and ideal stroke comparison
 *
 * C port of the Python StrokeProcessor's detection logic, optimized for
 * real-time execution on ESP32-S3. Uses world-frame vertical acceleration
 * derived from quaternion rotation to detect water-entry impacts.
 *
 * Research basis:
 *   - SwimBIT (PMC6915422): water entry creates distinct deceleration spike
 *   - Frontiers IMU swimming papers: jerk-based sharp change detection
 *   - Typical stroke rate: 40-80 strokes/min = 0.75-1.5s per stroke
 */

#include "stroke_detector.h"
#include "dtw_classifier.h"
#include "esp_log.h"
#include "haptic.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "STROKE_DET";

// ============================================================================
// Configuration (mirrors Python StrokeProcessor thresholds)
// ============================================================================
#define WATER_ENTRY_ACCEL_THRESHOLD 6.0f // m/s² total accel at impact
#define DOWNWARD_ACCEL_THRESHOLD -0.8f   // m/s² world_az for "moving down"
#define DOWNWARD_SAMPLE_WINDOW 8         // samples to check for downward motion
#define DOWNWARD_REQUIRED_COUNT 4        // min samples showing downward
#define IMPACT_REVERSAL_THRESHOLD 2.0f   // m/s² world_az must rise above
#define IMPACT_JERK_THRESHOLD 250.0f     // m/s³ sharp change detection
#define IMPACT_DELTA_A_THRESHOLD 4.0f    // m/s² fallback delta
#define ENTRY_GYRO_THRESHOLD 0.8f        // rad/s hand rotation
#define MIN_STROKE_INTERVAL_MS 1100      // 1.1s between strokes
/* Wall push-off is far stronger than stroke impacts; strokes often peak ~20–28 m/s² LIA. */
#define WALL_IMPACT_THRESHOLD 34.0f      // m/s² — must exceed typical stroke spikes
#define WALL_SUSTAINED_COUNT 4           // consecutive high-accel samples (stricter)
#define STROKE_TURN_COOLDOWN_MS 2000     // ignore wall/turn right after a stroke impact
#define TURN_LOCKOUT_MS 2500             // 2.5s lockout after turn
#define STROKE_INTEGRATION_TIMEOUT_MS 600 // 0.6s integration window

// Ideal stroke comparison
#define MAX_IDEAL_SAMPLES 200   // max samples for ideal stroke (~2s at 100Hz)
#define MAX_CURRENT_SAMPLES 200 // max samples to accumulate per stroke
#define DEFAULT_HAPTIC_THRESHOLD 1.08f /* unused legacy; presets set real value (~1.0 DTW scale) */
#define HAPTIC_WARMUP_STROKES 2       // skip haptic on first N strokes of a session

// History buffer
#define MAX_RECENT_SAMPLES 12

// ============================================================================
// Internal state
// ============================================================================
typedef struct {
  // Stroke detection state
  uint32_t stroke_count;
  uint32_t turn_count;
  uint32_t last_stroke_time_ms;
  uint32_t last_wall_impact_time_ms;
  uint32_t last_timestamp_ms;
  bool has_previous_sample;

  // History buffers
  float recent_world_az[MAX_RECENT_SAMPLES];
  int recent_world_az_count;
  int recent_world_az_idx; // circular index
  float prev_accel_mag;
  bool has_prev_accel_mag;
  int wall_high_count;

  // Stroke integration window
  bool stroke_integrating;
  uint32_t stroke_integration_start_ms;

  // Current stroke accumulation (for ideal comparison)
  float current_stroke_lia[MAX_CURRENT_SAMPLES * 3]; // lia_x, lia_y, lia_z
  int current_stroke_count;

  // Ideal stroke reference
  float ideal_lia[MAX_IDEAL_SAMPLES * 3]; // lia_x, lia_y, lia_z
  int ideal_sample_count;
  float ideal_entry_angle;
  bool ideal_loaded;

  // Latched entry angle from stroke detection moment (carried to integration completion)
  float latched_entry_angle;

  // Comparison results
  float last_deviation;

  // Haptic config
  float haptic_threshold;
  bool haptic_enabled;

  // User parameters
  float wingspan_cm;
  haptic_skill_level_t skill_level;
  /** Extra deviation beyond haptic_threshold to reach double / triple pulse tiers */
  float haptic_tier_strong_delta;
  float haptic_tier_moderate_delta;
  /** Degrees from ideal entry before ENTRY_BAD (beginner: looser, advanced: tighter) */
  float entry_angle_tol_deg;
} detector_state_t;

static detector_state_t s_state;

// ============================================================================
// Helper: Quaternion rotation (sensor frame → world frame)
// ============================================================================
static void rotate_to_world(float qw, float qx, float qy, float qz, float ax,
                            float ay, float az, float *wx, float *wy,
                            float *wz) {
  float ww = qw * qw, xx = qx * qx, yy = qy * qy, zz = qz * qz;
  float _wx = qw * qx, _wy = qw * qy, _wz = qw * qz;
  float xy = qx * qy, xz = qx * qz, yz = qy * qz;

  *wx = (ww + xx - yy - zz) * ax + 2 * (xy - _wz) * ay + 2 * (xz + _wy) * az;
  *wy = 2 * (xy + _wz) * ax + (ww - xx + yy - zz) * ay + 2 * (yz - _wx) * az;
  *wz = 2 * (xz - _wy) * ax + 2 * (yz + _wx) * ay + (ww - xx - yy + zz) * az;
}

// ============================================================================
// Helper: Compare current stroke against ideal using DTW (Dynamic Time Warping)
// ============================================================================
static float compare_strokes(const float *current, int cur_count,
                             const float *ideal, int ideal_count) {
  if (cur_count == 0 || ideal_count == 0)
    return 0.0f;

  // Utilize the machine learning Dynamic Time Warping (DTW) algorithm
  // optimized for the ESP32 to accurately classify stroke similarity
  // independent of exact pacing.
  return process_dtw_buffer(current, cur_count, ideal, ideal_count);
}

// ============================================================================
// Public API
// ============================================================================

/**
 * Skill presets — keep ordering: beginner (highest threshold = fewest cues) … competitive (lowest).
 *
 * DTW deviation from process_dtw_buffer() clusters near ~1.0 for typical strokes vs ideal; older
 * cuts (0.35–0.65) sat below that cluster and caused intermediate to cue almost every stroke.
 * Tuned to logs where DTW deviation sits ~0.98–1.06 on “fine” strokes, ~1.10+ when shape is
 * off, and ~1.15+ on clear misses. Entry-angle cues use entry_angle_tol_deg separately.
 */
static void stroke_detector_apply_skill_level(haptic_skill_level_t skill) {
  s_state.skill_level = skill;
  switch (skill) {
  case HAPTIC_SKILL_BEGINNER:
    s_state.haptic_threshold = 1.14f;
    s_state.haptic_tier_strong_delta = 0.24f;
    s_state.haptic_tier_moderate_delta = 0.13f;
    s_state.entry_angle_tol_deg = 26.0f;
    break;
  case HAPTIC_SKILL_INTERMEDIATE:
    /* Slightly above the 1.01–1.06 “meh” band so DTW alone does not dominate; entry tol relaxed */
    s_state.haptic_threshold = 1.11f;
    s_state.haptic_tier_strong_delta = 0.19f;
    s_state.haptic_tier_moderate_delta = 0.095f;
    s_state.entry_angle_tol_deg = 21.0f;
    break;
  case HAPTIC_SKILL_ADVANCED:
    s_state.haptic_threshold = 1.05f;
    s_state.haptic_tier_strong_delta = 0.15f;
    s_state.haptic_tier_moderate_delta = 0.075f;
    s_state.entry_angle_tol_deg = 13.0f;
    break;
  case HAPTIC_SKILL_COMPETITIVE:
    s_state.haptic_threshold = 1.00f;
    s_state.haptic_tier_strong_delta = 0.10f;
    s_state.haptic_tier_moderate_delta = 0.05f;
    s_state.entry_angle_tol_deg = 8.0f;
    break;
  default:
    s_state.haptic_threshold = 1.11f;
    s_state.haptic_tier_strong_delta = 0.19f;
    s_state.haptic_tier_moderate_delta = 0.095f;
    s_state.entry_angle_tol_deg = 21.0f;
    s_state.skill_level = HAPTIC_SKILL_INTERMEDIATE;
    break;
  }
}

void stroke_detector_init(void) {
  memset(&s_state, 0, sizeof(s_state));
  s_state.haptic_enabled = true;
  s_state.wingspan_cm = 180.0f;
  stroke_detector_apply_skill_level(HAPTIC_SKILL_INTERMEDIATE);
  ESP_LOGI(TAG, "Stroke detector init: skill=%d thresh=%.2f entry±%.0f°",
           (int)s_state.skill_level, s_state.haptic_threshold,
           s_state.entry_angle_tol_deg);
}

void stroke_detector_get_haptic_profile(stroke_detector_haptic_profile_t *out) {
  if (!out) {
    return;
  }
  out->haptic_threshold = s_state.haptic_threshold;
  out->tier_strong_delta = s_state.haptic_tier_strong_delta;
  out->tier_moderate_delta = s_state.haptic_tier_moderate_delta;
  out->entry_tol_deg = s_state.entry_angle_tol_deg;
  out->skill_level = s_state.skill_level;
}

void stroke_detector_reset_session(void) {
  // Reset only session-specific fields
  s_state.stroke_count = 0;
  s_state.turn_count = 0;
  s_state.last_stroke_time_ms = 0;
  s_state.last_wall_impact_time_ms = 0;
  s_state.has_previous_sample = false;
  
  memset(s_state.recent_world_az, 0, sizeof(s_state.recent_world_az));
  s_state.recent_world_az_count = 0;
  s_state.recent_world_az_idx = 0;
  s_state.prev_accel_mag = 0.0f;
  s_state.has_prev_accel_mag = false;
  s_state.wall_high_count = 0;
  
  s_state.stroke_integrating = false;
  s_state.stroke_integration_start_ms = 0;
  
  memset(s_state.current_stroke_lia, 0, sizeof(s_state.current_stroke_lia));
  s_state.current_stroke_count = 0;
  s_state.last_deviation = 0.0f;

  ESP_LOGI(TAG, "Stroke detector session reset (ideal persists: %s)", 
           s_state.ideal_loaded ? "YES" : "NO");
}

stroke_event_t stroke_detector_feed(const bno055_sample_t *sample) {
  stroke_event_t event = {0};
  event.haptic_reason = HAPTIC_REASON_NONE;

  uint32_t t_ms = sample->t_ms;

  // Skip first sample (need dt)
  if (!s_state.has_previous_sample) {
    s_state.has_previous_sample = true;
    s_state.last_timestamp_ms = t_ms;
    return event;
  }

  float dt = (float)(t_ms - s_state.last_timestamp_ms) / 1000.0f;
  s_state.last_timestamp_ms = t_ms;

  if (dt <= 0.0f || dt > 0.5f)
    return event;

  // Compute accel magnitude from LIA
  float lia_x = sample->lia_x, lia_y = sample->lia_y, lia_z = sample->lia_z;
  float accel_mag = sqrtf(lia_x * lia_x + lia_y * lia_y + lia_z * lia_z);

  // Compute world-frame vertical acceleration
  float world_ax, world_ay, world_az;
  rotate_to_world(sample->qw, sample->qx, sample->qy, sample->qz, lia_x, lia_y,
                  lia_z, &world_ax, &world_ay, &world_az);

  // Gyroscope magnitude
  float gyro_mag = sqrtf(sample->gx * sample->gx + sample->gy * sample->gy +
                         sample->gz * sample->gz);

  bool in_turn_lockout =
      ((t_ms - s_state.last_wall_impact_time_ms) < TURN_LOCKOUT_MS) &&
      (s_state.last_wall_impact_time_ms > 0);

  // --- UPDATE HISTORY BUFFER (circular) ---
  int az_idx = s_state.recent_world_az_idx % MAX_RECENT_SAMPLES;
  s_state.recent_world_az[az_idx] = world_az;
  s_state.recent_world_az_idx++;
  if (s_state.recent_world_az_count < MAX_RECENT_SAMPLES) {
    s_state.recent_world_az_count++;
  }

  // --- CHECK DOWNWARD MOTION HISTORY ---
  bool was_moving_downward = false;
  if (s_state.recent_world_az_count > DOWNWARD_SAMPLE_WINDOW) {
    int down_count = 0;
    for (int i = 1; i <= DOWNWARD_SAMPLE_WINDOW; i++) {
      int idx =
          (s_state.recent_world_az_idx - 1 - i + MAX_RECENT_SAMPLES * 100) %
          MAX_RECENT_SAMPLES;
      if (s_state.recent_world_az[idx] < DOWNWARD_ACCEL_THRESHOLD) {
        down_count++;
      }
    }
    was_moving_downward = (down_count >= DOWNWARD_REQUIRED_COUNT);
  }

  // --- IMPACT DETECTION ---
  bool is_impact_spike = (accel_mag > WATER_ENTRY_ACCEL_THRESHOLD) &&
                         (world_az > IMPACT_REVERSAL_THRESHOLD);

  // Jerk-based fallback
  float accel_delta = 0.0f, jerk = 0.0f;
  if (s_state.has_prev_accel_mag) {
    accel_delta = accel_mag - s_state.prev_accel_mag;
    jerk = (dt > 0.0f) ? (accel_delta / dt) : 0.0f;
  }
  s_state.prev_accel_mag = accel_mag;
  s_state.has_prev_accel_mag = true;

  bool is_impact_by_jerk =
      (accel_mag > WATER_ENTRY_ACCEL_THRESHOLD) &&
      (jerk > IMPACT_JERK_THRESHOLD || accel_delta > IMPACT_DELTA_A_THRESHOLD);

  bool impact_detected = is_impact_spike || is_impact_by_jerk;

  // --- STROKE INTERVAL CHECK ---
  bool interval_ok =
      ((t_ms - s_state.last_stroke_time_ms) >= MIN_STROKE_INTERVAL_MS) ||
      (s_state.last_stroke_time_ms == 0);

  // --- ENTRY ROTATION CHECK ---
  bool has_entry_rotation = (gyro_mag > ENTRY_GYRO_THRESHOLD);

  // --- FINAL STROKE DECISION ---
  bool stroke_detected = impact_detected && interval_ok && has_entry_rotation &&
                         !in_turn_lockout && !s_state.stroke_integrating &&
                         (was_moving_downward || is_impact_by_jerk);

  if (stroke_detected) {
    s_state.stroke_count++;
    s_state.last_stroke_time_ms = t_ms;
    s_state.recent_world_az_count = 0;
    s_state.recent_world_az_idx = 0;
    s_state.wall_high_count = 0; // stroke spikes must not accumulate toward "wall"

    // Start integration window for ideal comparison
    s_state.stroke_integrating = true;
    s_state.stroke_integration_start_ms = t_ms;
    s_state.current_stroke_count = 0;

    // Compute entry angle from quaternion pitch (water entry angle)
    // pitch = asin(2*(qw*qy - qz*qx)) per BNO055 quaternion convention
    float pitch_sin =
        2.0f * (sample->qw * sample->qy - sample->qz * sample->qx);
    if (pitch_sin > 1.0f)
      pitch_sin = 1.0f;
    if (pitch_sin < -1.0f)
      pitch_sin = -1.0f;
    float entry_angle_deg = asinf(pitch_sin) * (180.0f / (float)M_PI);
    event.entry_angle = fabsf(entry_angle_deg);
    s_state.latched_entry_angle = event.entry_angle;

    event.stroke_detected = true;
    ESP_LOGI(TAG,
             "Stroke #%u detected (accel=%.1f, world_az=%.1f, gyro=%.1f, "
             "entry=%.1f°)",
             (unsigned)s_state.stroke_count, accel_mag, world_az, gyro_mag,
             event.entry_angle);
  }

  // --- WALL / TURN (after stroke): hard strokes must not register as turns ---
  {
    uint32_t since_stroke = (s_state.last_stroke_time_ms > 0)
                                ? (t_ms - s_state.last_stroke_time_ms)
                                : STROKE_TURN_COOLDOWN_MS + 1;
    if (since_stroke >= STROKE_TURN_COOLDOWN_MS) {
      if (accel_mag > WALL_IMPACT_THRESHOLD) {
        s_state.wall_high_count++;
        if (s_state.wall_high_count >= WALL_SUSTAINED_COUNT) {
          uint32_t time_since_wall = t_ms - s_state.last_wall_impact_time_ms;
          if (time_since_wall > TURN_LOCKOUT_MS) {
            s_state.turn_count++;
            event.turn_detected = true;
          }
          s_state.last_wall_impact_time_ms = t_ms;
          s_state.recent_world_az_count = 0;
          s_state.wall_high_count = 0;
        }
      } else {
        s_state.wall_high_count = 0;
      }
    } else {
      s_state.wall_high_count = 0;
    }
  }

  // --- INTEGRATION WINDOW (accumulate LIA for comparison) ---
  if (s_state.stroke_integrating) {
    // Accumulate LIA during integration window for DTW comparison
    if (s_state.current_stroke_count < MAX_CURRENT_SAMPLES) {
      float scale = 1.0f;
      if (s_state.wingspan_cm > 50.0f && s_state.wingspan_cm < 250.0f) {
        scale = 180.0f / s_state.wingspan_cm;
      }

      int i = s_state.current_stroke_count * 3;
      s_state.current_stroke_lia[i + 0] = lia_x * scale;
      s_state.current_stroke_lia[i + 1] = lia_y * scale;
      s_state.current_stroke_lia[i + 2] = lia_z * scale;
      s_state.current_stroke_count++;
    }

    uint32_t time_in_stroke = t_ms - s_state.stroke_integration_start_ms;
    if (time_in_stroke > STROKE_INTEGRATION_TIMEOUT_MS) {
      s_state.stroke_integrating = false;

      // Record pull duration for coaching insights
      event.pull_duration_ms = (float)time_in_stroke;

      if (s_state.ideal_loaded) {
        if (s_state.current_stroke_count > 5) {
          int num_ideal = s_state.ideal_sample_count;
          s_state.last_deviation = compare_strokes(
              s_state.current_stroke_lia, s_state.current_stroke_count,
              s_state.ideal_lia, num_ideal);
          event.deviation_score = s_state.last_deviation;
          event.entry_angle = s_state.latched_entry_angle;

          bool technique_bad = (event.deviation_score > s_state.haptic_threshold);
          float entry_diff = fabsf(s_state.latched_entry_angle - s_state.ideal_entry_angle);
          bool entry_bad = (s_state.ideal_entry_angle > 5.0f) &&
                           (entry_diff > s_state.entry_angle_tol_deg);
          /* Pull duration flag: integration ends at ~600ms window end; threshold is legacy hook. */
          bool pull_short = (time_in_stroke < 250);

          // Set reason codes so app can explain WHY haptic fired
          event.haptic_reason = HAPTIC_REASON_NONE;
          if (technique_bad) event.haptic_reason |= HAPTIC_REASON_DTW_HIGH;
          if (entry_bad) event.haptic_reason |= HAPTIC_REASON_ENTRY_BAD;
          if (pull_short) event.haptic_reason |= HAPTIC_REASON_PULL_SHORT;

          ESP_LOGI(TAG, "Stroke deviation: %.3f (entry: %.1f, target: %.1f, thresh: %.3f, pull: %ums, reason: 0x%02x)",
                   event.deviation_score, s_state.latched_entry_angle, s_state.ideal_entry_angle, 
                   s_state.haptic_threshold, (unsigned)time_in_stroke, event.haptic_reason);

          bool past_warmup = (s_state.stroke_count > HAPTIC_WARMUP_STROKES);
          if (s_state.haptic_enabled && haptic_is_available() && past_warmup &&
              (technique_bad || entry_bad)) {
            if (event.deviation_score >
                    (s_state.haptic_threshold + s_state.haptic_tier_strong_delta) ||
                entry_bad) {
              haptic_play_pattern(HAPTIC_PATTERN_TRIPLE_PULSE);
              event.haptic_fired = true;
            } else if (event.deviation_score >
                       (s_state.haptic_threshold + s_state.haptic_tier_moderate_delta)) {
              haptic_play_pattern(HAPTIC_PATTERN_DOUBLE_PULSE);
              event.haptic_fired = true;
            } else if (technique_bad) {
              if (s_state.skill_level == HAPTIC_SKILL_BEGINNER) {
                haptic_play_pattern(HAPTIC_PATTERN_DEVIATION_MILD);
              } else {
                haptic_play_pattern(HAPTIC_PATTERN_SINGLE_LONG);
              }
              event.haptic_fired = true;
            }
          }

          if (event.haptic_fired) {
            ESP_LOGI(TAG, "Haptic fired! (dev=%.3f, entry_diff=%.1f, reason=0x%02x)", 
                     event.deviation_score, entry_diff, event.haptic_reason);
          } else {
            ESP_LOGI(TAG, "Stroke #%u: Score=%.3f (No haptic)",
                     (unsigned)s_state.stroke_count, event.deviation_score);
          }
        } else {
          ESP_LOGW(TAG, "Stroke #%u too short for DTW scoring (%d samples)",
                   (unsigned)s_state.stroke_count, s_state.current_stroke_count);
        }
      } else {
        ESP_LOGW(TAG, "Stroke #%u detected but no ideal stroke loaded",
                 (unsigned)s_state.stroke_count);
      }
    }
  }

  // Populate event (note: deviation_score already set if window closed this frame)
  event.stroke_count = s_state.stroke_count;
  event.turn_count = s_state.turn_count;

  return event;
}

esp_err_t stroke_detector_load_ideal(const float *lia_data,
                                     size_t num_samples, float ideal_entry_angle) {
  s_state.ideal_loaded = false; // Disable during modification
  
  if (num_samples > MAX_IDEAL_SAMPLES) {
    ESP_LOGW(TAG, "Ideal stroke too large (%zu samples, max %d) — truncating",
             num_samples, MAX_IDEAL_SAMPLES);
    num_samples = MAX_IDEAL_SAMPLES;
  }
  
  if (num_samples == 0 || lia_data == NULL) {
    s_state.ideal_sample_count = 0;
    ESP_LOGI(TAG, "Ideal stroke cleared");
    return ESP_OK; // Cleared
  }

  // Copy into static array
  memcpy(s_state.ideal_lia, lia_data, num_samples * 3 * sizeof(float));
  s_state.ideal_sample_count = (int)num_samples;
  s_state.ideal_entry_angle = ideal_entry_angle;
  s_state.ideal_loaded = true;

  ESP_LOGI(TAG, "Loaded ideal stroke: %zu samples, entry_angle: %.1f°", 
           num_samples, ideal_entry_angle);
  return ESP_OK;
}

bool stroke_detector_has_ideal(void) { return s_state.ideal_loaded; }

uint32_t stroke_detector_get_count(void) { return s_state.stroke_count; }

uint32_t stroke_detector_get_turn_count(void) { return s_state.turn_count; }

float stroke_detector_get_deviation(void) { return s_state.last_deviation; }

void stroke_detector_set_haptic_threshold(float threshold) {
  s_state.haptic_threshold = threshold;
  ESP_LOGI(TAG, "Haptic threshold set to %.2f", threshold);
}

void stroke_detector_enable_haptic(bool enable) {
  s_state.haptic_enabled = enable;
  ESP_LOGI(TAG, "Haptic triggering %s", enable ? "enabled" : "disabled");
}

void stroke_detector_set_user_params(float wingspan_cm, haptic_skill_level_t skill_level) {
  if (wingspan_cm > 50.0f && wingspan_cm < 250.0f) {
    s_state.wingspan_cm = wingspan_cm;
  }
  stroke_detector_apply_skill_level(skill_level);

  ESP_LOGI(TAG,
           "User params: wingspan=%.1fcm skill=%d thresh=%.2f dStrong=+%.2f dMod=+%.2f "
           "entry±%.0f°",
           s_state.wingspan_cm, (int)s_state.skill_level, s_state.haptic_threshold,
           s_state.haptic_tier_strong_delta, s_state.haptic_tier_moderate_delta,
           s_state.entry_angle_tol_deg);
}

float stroke_detector_get_ideal_entry_angle(void) { return s_state.ideal_entry_angle; }
float stroke_detector_get_last_deviation(void) { return s_state.last_deviation; }
