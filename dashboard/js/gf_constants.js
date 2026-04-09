/**
 * GoldenForm — browser localStorage keys (single place to avoid drift).
 * Loaded after gf_roles.js (which holds LS_EXPECTED_WEARABLES).
 */
const LS_IDEAL_KEY = 'goldenform_ideal_cache';
const LS_WIFI_SYNC_OK = 'goldenform_wifi_sync_ok';
const LS_KEY = 'goldenform_sessions';
const LS_CAL_KEY = 'goldenform_cal_snapshot_v1';
// Per-device calibration snapshots (keyed by device_id) for multi-wearable setups.
const LS_CAL_MAP_KEY = 'goldenform_cal_snapshots_by_device_v1';
// One-time UX: after all planned devices are calibrated, show a single "ready to log" prompt.
const LS_READY_TO_LOG_SHOWN = 'goldenform_ready_to_log_shown_v1';
/** Survives gf_bootstrap instance clears (not prefixed with goldenform_). */
const LS_USER_PROFILE_CACHE = 'gf_user_profile_cache_v1';

/** Above this vs-ideal LIA deviation, Analysis “Haptic” column shows ⚡ (cue vs your saved baseline). */
const GF_VS_IDEAL_HAPTIC_THRESH = 0.45;
