/**
 * @file app_main.c
 * @brief GoldenForm Firmware - ESP32-S3-WROOM-1
 */

#include "bno055.h"
#include "bus_i2c.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "haptic.h"
#include "led_strip.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protobuf_utils.h"
#include "sdkconfig.h"
#include "storage.h"
#include "stroke_detector.h"
#include "wifi_server.h"
#include <inttypes.h>
#include <stdio.h>

#define DEBUG_SD_CARD 0

static const char *TAG = "GOLDENFORM";

// ============================================================================
// Pin Definitions (from Kconfig, matching schematic)
// ============================================================================

// BNO055 I2C
#define I2C_SDA_GPIO CONFIG_GOLDENFORM_I2C_SDA_GPIO         // GPIO 4
#define I2C_SCL_GPIO CONFIG_GOLDENFORM_I2C_SCL_GPIO         // GPIO 5
#define BNO055_ADDR_GPIO CONFIG_GOLDENFORM_BNO055_ADDR_GPIO // GPIO 6

/* User button (menuconfig: GOLDENFORM_BUTTON_GPIO). Default 0 = DevKit BOOT.
 * Short press = record/stop → SD; hold ~BUTTON_HOLD_MS = Wi‑Fi sync.
 *
 * GPIO0 strapping: if IO0 is LOW when the chip resets, ESP32-S3 enters UART
 * download mode (ROM), not this app — release the button before power-on / EN reset.
 *
 * Input is polled in the main loop (same timing with or without USB UART); GPIO
 * ISRs are not used for the button.
 *
 * Wi‑Fi / sync current spikes + weak supplies can brown out the chip (looks like
 * a reset). Use adequate decoupling and a supply that can handle peak Wi‑Fi load. */
#define BUTTON_GPIO CONFIG_GOLDENFORM_BUTTON_GPIO

// LED Configuration
// Set LED_ENABLED to 0 to disable LED (required when no LED is wired or GPIO
// conflicts) GPIO48 is used for SD card CS, so LED is disabled to avoid
// conflict
#define LED_ENABLED 0
#define LED_GPIO 48 // Not used when LED_ENABLED=0

// Status LEDs on GPIO 40, 41, 42
#define POWER_LED_GPIO 40  // Always on when powered
#define STATUS_LED_GPIO 9 // ON during logging, BLINKS during syncing
#define ERROR_LED_GPIO 1  // ON when any error occurs (IMU, SD card, WiFi)

// ============== Application State Machine ==============
typedef enum {
  STATE_IDLE,    // Not doing anything - press to record, hold to sync
  STATE_LOGGING, // Recording IMU data to SD card
  STATE_SYNCING  // WiFi active, serving data
} app_state_t;

static volatile app_state_t current_state = STATE_IDLE;
#define BUTTON_DEBOUNCE_MS 80

// Button hold detection (polled in main loop; volatiles not required but kept)
static volatile uint32_t button_press_start_ms = 0;
static volatile bool button_is_pressed = false;
static volatile bool button_short_press = false;
static volatile bool button_hold_triggered = false;

/* Debounced button level: 1 = released (pull-up), 0 = pressed */
static int btn_debounce_raw = 1;
static uint32_t btn_debounce_since_ms = 0;
static int btn_stable = 1;
#define BUTTON_HOLD_MS 1500
#define BUTTON_SHORT_MAX_MS 800

// Session stats
static uint32_t session_sample_count = 0;
static uint32_t session_start_time = 0;
static bool ap_lingering = false;
static uint32_t ap_linger_start_ms = 0;
/* Disable AP linger auto-shutdown: AP should shut down based on calibration readiness
 * (Gyro+Accel 3/3 in IMUPLUS) rather than a wall-clock timer. */
#define AP_LINGER_TIMEOUT_MS (0)

/* After /api/registration_done we keep the AP up briefly so the app can:
 * - show live calibration status (TIDR 6-1-4),
 * - push user config and ideal stroke (stretch goals),
 * while still auto-shutting down Wi‑Fi for power after a short window.
 * This is separate from ap_lingering (used for "no sessions on SD" idle mode). */
static bool ap_post_reg_keepalive = false;
static uint32_t ap_post_reg_start_ms = 0;
/* AP is kept up after registration until calibration is "swim ready" on this unit.
 * This removes the time limit and avoids confusing users mid-calibration/config. */

// Subsystem status
static bool bno055_available = false;
static bool storage_available = false;
static bool wifi_available = false;
static bool system_has_error = false;

// ============== Error LED Control ==============
static void error_led_set(bool on) {
  gpio_set_level(ERROR_LED_GPIO, on ? 1 : 0);
  if (on) {
    system_has_error = true;
    ESP_LOGW(TAG, "Error LED ON (GPIO%d)", ERROR_LED_GPIO);
  }
}

// LED variables (only used when LED_ENABLED=1)
#if LED_ENABLED
static TaskHandle_t led_blink_task_handle = NULL;
static volatile bool led_blink_stop = false;
static led_strip_handle_t led_strip = NULL;
#endif

static void transition_to_syncing(void);

// ============== Button polling (debounced; runs in main IMU loop) ==========
/* GPIO0 = BOOT strap: holding it low during a brownout/reset forces UART download mode.
 * If Wi‑Fi starts while BOOT is still held, supply spikes can reset the chip with IO0 low.
 * For BUTTON_GPIO==0 we therefore arm sync on hold but call transition_to_syncing() only
 * after release (see main loop). */
static bool pending_sync_after_release = false;

static void button_poll(void) {
  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
  int raw = gpio_get_level(BUTTON_GPIO);

  if (raw != btn_debounce_raw) {
    btn_debounce_raw = raw;
    btn_debounce_since_ms = now;
    return;
  }
  if ((now - btn_debounce_since_ms) < BUTTON_DEBOUNCE_MS) {
    return;
  }
  if (raw == btn_stable) {
    return;
  }

  btn_stable = raw;
  if (btn_stable == 0) {
    button_is_pressed = true;
    button_press_start_ms = now;
    button_hold_triggered = false;
  } else {
    if (button_is_pressed && !button_hold_triggered) {
      uint32_t held = now - button_press_start_ms;
      if (held > 50 && held < BUTTON_SHORT_MAX_MS) {
        button_short_press = true;
      }
    }
    button_is_pressed = false;
    if (pending_sync_after_release) {
      pending_sync_after_release = false;
      ESP_LOGI(TAG, "Button released — starting Wi‑Fi sync");
      transition_to_syncing();
    }
  }
}

// ============== LED Control ==============
// LED control functions (only compiled when LED_ENABLED=1)
#if LED_ENABLED
static void led_set(bool on) {
  if (led_strip == NULL) {
    return;
  }

  if (on) {
    esp_err_t ret = led_strip_set_pixel(led_strip, 0, 255, 0, 0);
    if (ret == ESP_OK) {
      led_strip_refresh(led_strip);
    }
    ESP_LOGI(TAG, "LED ON (GPIO%d - Red)", LED_GPIO);
  } else {
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "LED OFF (GPIO%d)", LED_GPIO);
  }
}
#else
// LED disabled - empty function
static void led_set(bool on) {
  (void)on; // Suppress unused parameter warning
}
#endif

#if LED_ENABLED
static void led_blink_task(void *arg) {
  ESP_LOGI(TAG, "LED blink task started");
  led_blink_stop = false;

  if (led_strip == NULL) {
    led_blink_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  while (!led_blink_stop) {
    led_strip_set_pixel(led_strip, 0, 255, 0, 0);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(250));
    if (led_blink_stop)
      break;

    led_strip_clear(led_strip);
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  led_strip_clear(led_strip);
  led_blink_task_handle = NULL;
  vTaskDelete(NULL);
}

static void led_blink_start(void) {
  if (led_blink_task_handle == NULL) {
    led_blink_stop = false;
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5,
                &led_blink_task_handle);
  }
}

static void led_blink_stop_and_wait(void) {
  if (led_blink_task_handle != NULL) {
    led_blink_stop = true;
    for (int i = 0; i < 20 && led_blink_task_handle != NULL; i++) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  if (led_strip != NULL) {
    led_strip_clear(led_strip);
  }
}
#else
// LED disabled - empty functions
static void led_blink_start(void) {}
static void led_blink_stop_and_wait(void) {}
#endif

// ============== Status LED (GPIO 41) Blink Control for Syncing ==============
static TaskHandle_t status_led_blink_task_handle = NULL;
static volatile bool status_led_blink_stop = false;

static void status_led_blink_task(void *arg) {
  ESP_LOGI(TAG, "Status LED blink task started (GPIO%d)", STATUS_LED_GPIO);
  status_led_blink_stop = false;

  while (!status_led_blink_stop) {
    gpio_set_level(STATUS_LED_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(250));
    if (status_led_blink_stop)
      break;

    gpio_set_level(STATUS_LED_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
  }

  gpio_set_level(STATUS_LED_GPIO, 0);
  status_led_blink_task_handle = NULL;
  vTaskDelete(NULL);
}

static void status_led_blink_start(void) {
  if (status_led_blink_task_handle == NULL) {
    status_led_blink_stop = false;
    xTaskCreate(status_led_blink_task, "status_led_blink", 2048, NULL, 5,
                &status_led_blink_task_handle);
  }
}

static void status_led_blink_stop_and_wait(void) {
  if (status_led_blink_task_handle != NULL) {
    status_led_blink_stop = true;
    for (int i = 0; i < 20 && status_led_blink_task_handle != NULL; i++) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
  gpio_set_level(STATUS_LED_GPIO, 0);
}

/** Deferred from HTTP POST /api/registration_done so the response finishes before the AP/HTTP stack is torn down. */
static void registration_done_worker(void *arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(100));
  /* Registration completion is a UI/account pairing event, not a data-transfer state.
   * The dashboard may call this while we're in SYNCING/LOGGING due to timing.
   * Always stop the registration linger blink. AP stays up until the next recording
   * so the user can replay JSON, review status, and push config/ideal. */
  ap_lingering = false;
  status_led_blink_stop_and_wait();
  if (wifi_available && wifi_server_is_ap_active()) {
    ap_post_reg_keepalive = true;
    ap_post_reg_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(TAG, "registration_done: AP stays alive (until next recording)");
  }
  vTaskDelete(NULL);
}

static void goldenform_registration_done_callback(void) {
  if (xTaskCreate(registration_done_worker, "gf_reg_done", 4096, NULL, 5, NULL) !=
      pdPASS) {
    ESP_LOGW(TAG, "registration_done: could not spawn worker task");
  }
}

// ============== State Transitions ==============
static void transition_to_logging(void) {
#if CONFIG_GOLDENFORM_REQUIRE_SD
  if (!storage_available) {
    ESP_LOGE(TAG, "Recording blocked: SD card required (PDP). Insert card, power-cycle if needed.");
    error_led_set(true);
    return;
  }
#endif
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, ">>> STATE: LOGGING");
  ESP_LOGI(TAG, "Recording IMU data to SD card...");

  if (wifi_available && wifi_server_is_ap_active()) {
    ESP_LOGI(TAG, "Shutting down WiFi AP before recording...");
    wifi_server_stop_ap();
    ap_lingering = false;
    ap_post_reg_keepalive = false;
  }

  session_sample_count = 0;
  session_start_time = esp_timer_get_time() / 1000;

  stroke_detector_reset_session();

  led_blink_stop_and_wait();
  status_led_blink_stop_and_wait();

  if (storage_available) {
    esp_err_t err = storage_start_session();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start storage session: %s",
               esp_err_to_name(err));
      error_led_set(true);
    } else {
      ESP_LOGI(TAG, "Session #%" PRIu32 " started",
               storage_get_session_number());
    }
  }

  gpio_set_level(STATUS_LED_GPIO, 1);
  led_set(true);
  current_state = STATE_LOGGING;

  ESP_LOGI(TAG, "[LED] Status=SOLID Power=ON Error=OFF -> Recording active");
  ESP_LOGI(TAG, "Press to STOP, hold to SYNC.");
  ESP_LOGI(TAG, "========================================");
}

static void transition_to_syncing(void) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, ">>> SYNC requested");

  if (!wifi_available) {
    ESP_LOGW(TAG, "WiFi not available - cannot sync");
    ESP_LOGI(TAG, "========================================");
    return;
  }

  if (current_state == STATE_LOGGING && storage_available) {
    uint32_t dur = (uint32_t)(esp_timer_get_time() / 1000) - session_start_time;
    ESP_LOGI(TAG, "Auto-stopping session (%" PRIu32 " ms, %" PRIu32 " samples)",
             dur, session_sample_count);
    storage_stop_session();
    gpio_set_level(STATUS_LED_GPIO, 0);
    led_set(false);
  }

  if (!wifi_server_is_ap_active()) {
    esp_err_t err = wifi_server_start_ap();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start WiFi AP: %s", esp_err_to_name(err));
      error_led_set(true);
      ESP_LOGI(TAG, "========================================");
      return;
    }
  } else {
    ESP_LOGI(TAG, "WiFi AP already active, reusing connection");
  }
  ap_lingering = false;

  /* Let the AP and TCP stack finish coming up before sync (avoids first-hold
   * ESP_ERR_INVALID_STATE / flaky first transfer right after esp_wifi_start). */
  vTaskDelay(pdMS_TO_TICKS(150));

  esp_err_t err = wifi_server_start_sync();
  if (err == ESP_ERR_INVALID_STATE) {
    vTaskDelay(pdMS_TO_TICKS(200));
    err = wifi_server_start_sync();
  }
  if (err == ESP_ERR_NOT_FOUND) {
    ESP_LOGW(TAG, "No sessions on SD card to sync");
    ESP_LOGI(TAG, "AP stays alive for config/ideal push from app");
    ap_lingering = true;
    ap_linger_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    current_state = STATE_IDLE;
    /* Same visual as active file transfer: blink only during Wi‑Fi sync / setup flows,
     * not after transfer completes (see transition_to_idle — do not restart blink via ap_lingering). */
    status_led_blink_start();
    led_blink_start();
    ESP_LOGI(TAG, "[LED] Status=BLINK — no files; AP open for app link/config");
    ESP_LOGI(TAG, "========================================");
    return;
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start sync: %s", esp_err_to_name(err));
    error_led_set(true);
    wifi_server_stop_ap();
    ESP_LOGI(TAG, "========================================");
    return;
  }

  current_state = STATE_SYNCING;
  status_led_blink_start();
  led_blink_start();
  ESP_LOGI(TAG, ">>> STATE: SYNCING");
  ESP_LOGI(TAG, "[LED] Status=BLINK Power=BLINK -> Wireless sync active");
  ESP_LOGI(TAG, "Connect to WiFi: %s (pw: %s)", WIFI_AP_SSID_BASE, WIFI_AP_PASSWORD);
  ESP_LOGI(TAG, "Blink stops automatically after data transfer.");
  ESP_LOGI(TAG, "========================================");
}

static void transition_to_idle(bool after_sync) {
  ESP_LOGI(TAG, "========================================");
  ESP_LOGI(TAG, ">>> STATE: IDLE");

  if (current_state == STATE_LOGGING && storage_available) {
    storage_stop_session();
  }

  led_blink_stop_and_wait();
  status_led_blink_stop_and_wait();

  if (current_state == STATE_SYNCING && wifi_available) {
    // Stop file transfer state, but keep the AP up for replay/config until next recording.
    wifi_server_stop_sync();
    ap_lingering = true;
    ap_linger_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(TAG, "WiFi AP stays alive — replay JSON and push config/ideal");
  }

  if (after_sync && storage_available) {
    ESP_LOGI(TAG, "Clearing synced data from SD card...");
    storage_delete_all_files();
    storage_reset_session_counter();
  }

  gpio_set_level(STATUS_LED_GPIO, 0);
  led_set(false);
  current_state = STATE_IDLE;
  session_sample_count = 0;

  ESP_LOGI(TAG, "[LED] Status=OFF Power=ON Error=OFF -> Idle");
  ESP_LOGI(TAG, "Press to start recording, hold to sync.");
  ESP_LOGI(TAG, "========================================");
}

// ============== Button Handlers ==============
static void handle_button_press(void) {
  switch (current_state) {
  case STATE_IDLE:
    transition_to_logging();
    break;
  case STATE_LOGGING:
    transition_to_idle(false);
    break;
  case STATE_SYNCING:
    transition_to_idle(true);
    break;
  }
}

static void handle_button_hold(void) {
  switch (current_state) {
  case STATE_IDLE:
  case STATE_LOGGING:
    transition_to_syncing();
    break;
  case STATE_SYNCING:
    break;
  }
}

// ============== Initialization ==============
static void init_gpio(void) {
  // Configure button (active low; no ISR — polled every IMU tick)
  gpio_config_t btn_cfg = {.pin_bit_mask = (1ULL << BUTTON_GPIO),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&btn_cfg);
  btn_debounce_raw = gpio_get_level(BUTTON_GPIO);
  btn_stable = btn_debounce_raw;
  btn_debounce_since_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

  // Configure status LEDs on GPIO 40, 41, 42
  gpio_config_t led_cfg = {.pin_bit_mask = (1ULL << POWER_LED_GPIO) |
                                           (1ULL << STATUS_LED_GPIO) |
                                           (1ULL << ERROR_LED_GPIO),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led_cfg);

  // Power LED (GPIO 40) - always on
  gpio_set_level(POWER_LED_GPIO, 1);
  // Status LED (GPIO 41) - off initially (ON=logging, BLINK=syncing)
  gpio_set_level(STATUS_LED_GPIO, 0);
  // Error LED (GPIO 42) - off initially (turns on when errors occur)
  gpio_set_level(ERROR_LED_GPIO, 0);
  ESP_LOGI(TAG,
           "LEDs initialized: Power=GPIO%d (ON), Status=GPIO%d (OFF), "
           "Error=GPIO%d (OFF)",
           POWER_LED_GPIO, STATUS_LED_GPIO, ERROR_LED_GPIO);

  // Configure LED (only if enabled)
#if LED_ENABLED
  ESP_LOGI(TAG, "Initializing NeoPixel LED strip on GPIO%d...", LED_GPIO);

  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_GPIO,
      .max_leds = 1,
  };

  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000,
      .flags.with_dma = false,
  };

  esp_err_t ret =
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(ret));
    led_strip = NULL;
  } else {
    vTaskDelay(pdMS_TO_TICKS(10));
    led_strip_clear(led_strip);
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "NeoPixel LED strip initialized on GPIO%d", LED_GPIO);
  }
  ESP_LOGI(TAG, "GPIO initialized: Button=GPIO%d, LED=GPIO%d", BUTTON_GPIO,
           LED_GPIO);
#else
  ESP_LOGI(TAG,
           "GPIO initialized: Button=GPIO%d, LED=disabled (GPIO%d reserved "
           "for SD CS)",
           BUTTON_GPIO, LED_GPIO);
#endif
}

static void init_bno055_address_pin(void) {
  gpio_config_t addr_cfg = {.pin_bit_mask = (1ULL << BNO055_ADDR_GPIO),
                            .mode = GPIO_MODE_OUTPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&addr_cfg);
  gpio_set_level(BNO055_ADDR_GPIO, 0); // LOW = address 0x28
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "BNO055 address pin (GPIO%d) set LOW for 0x28",
           BNO055_ADDR_GPIO);
}

// ============== Main Application ==============
void app_main(void) {
  ESP_LOGI(TAG, "GoldenForm Firmware (ESP32-S3-WROOM-1 starting)");

  // Initialize GPIO (button, LED)
  init_gpio();

  // Initialize BNO055 address selection
  init_bno055_address_pin();

  // Initialize I2C
  esp_err_t err = bus_i2c_init(I2C_NUM_0, I2C_SDA_GPIO, I2C_SCL_GPIO, 100000);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "I2C initialized: SDA=GPIO%d, SCL=GPIO%d", I2C_SDA_GPIO,
             I2C_SCL_GPIO);
  }

  // Initialize BNO055 - retry up to 5 times per TIDR 1-3-1
  {
    const int IMU_MAX_RETRIES = 5;
    for (int attempt = 1; attempt <= IMU_MAX_RETRIES; attempt++) {
      ESP_LOGI(TAG, "IMU init attempt %d/%d", attempt, IMU_MAX_RETRIES);
      err = bno055_init(I2C_NUM_0, BNO055_ADDR_A);
      if (err == ESP_OK) {
        bno055_available = true;
        ESP_LOGI(TAG, "BNO055 initialized at 0x%02X (attempt %d)",
                 BNO055_ADDR_A, attempt);
        break;
      }
      ESP_LOGW(TAG, "IMU init attempt %d failed: %s", attempt,
               esp_err_to_name(err));
      if (attempt < IMU_MAX_RETRIES) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }
    }
    if (!bno055_available) {
      ESP_LOGE(
          TAG,
          "IMU initialization failed after %d attempts - entering error state",
          IMU_MAX_RETRIES);
      error_led_set(true);
    }
  }

  // Try to restore BNO055 calibration from NVS (one-shot calibration)
  if (bno055_available) {
    nvs_handle_t nvs;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      nvs_flash_erase();
      err = nvs_flash_init();
    }
    if (err == ESP_OK) {
      err = nvs_open("goldenform", NVS_READWRITE, &nvs);
      if (err == ESP_OK) {
        uint8_t cal_data[22];
        size_t cal_size = sizeof(cal_data);
        err = nvs_get_blob(nvs, "bno055_cal", cal_data, &cal_size);
        if (err == ESP_OK && cal_size == 22) {
          // Load saved calibration offsets into BNO055
          esp_err_t load_err =
              bno055_load_calibration_data(I2C_NUM_0, BNO055_ADDR_A, cal_data);
          if (load_err == ESP_OK) {
            ESP_LOGI(
                TAG,
                "BNO055 calibration restored from NVS (one-shot calibration)");
          } else {
            ESP_LOGW(TAG, "Failed to load calibration data: %s",
                     esp_err_to_name(load_err));
          }
        } else {
          ESP_LOGI(TAG,
                   "No saved calibration found — will calibrate from scratch");
        }
        nvs_close(nvs);
      }
    }
  }

  /* SD: required for PDP sessions (Wi‑Fi sync reads from card). Retry mount — cards often need settle time. */
  {
    const int boot_sd_tries = 5;
    for (int attempt = 1; attempt <= boot_sd_tries; attempt++) {
      err = storage_init();
      if (err == ESP_OK) {
        storage_available = true;
        ESP_LOGI(TAG, "SD card storage initialized (attempt %d/%d)", attempt, boot_sd_tries);
        break;
      }
      ESP_LOGW(TAG, "SD mount failed (%s), attempt %d/%d", esp_err_to_name(err), attempt,
               boot_sd_tries);
      if (attempt < boot_sd_tries) {
        vTaskDelay(pdMS_TO_TICKS(800));
      }
    }
  }
  if (!storage_available) {
#if CONFIG_GOLDENFORM_REQUIRE_SD
    ESP_LOGE(TAG, "SD card required for PDP — insert FAT32 card, check SPI wiring (MOSI/MISO/SCK/CS), then reset.");
#else
    ESP_LOGW(TAG, "SD card not available — continuing without storage (dev mode)");
#endif
    error_led_set(true);
  }

  // Prepare WiFi only; AP starts when user holds the sync button (not at chip boot).
  err = wifi_server_init();
  if (err == ESP_OK) {
    wifi_available = true;
    wifi_server_set_registration_done_callback(goldenform_registration_done_callback);
    ESP_LOGI(TAG,
             "WiFi ready — hold button (GPIO%d) ~1.5s to open GoldenForm AP",
             BUTTON_GPIO);
  } else {
    ESP_LOGW(TAG, "WiFi init failed - continuing without WiFi");
    error_led_set(true);
  }

  // Initialize haptic motor
  err = haptic_init(CONFIG_GOLDENFORM_HAPTIC_GPIO);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Haptic motor initialized on GPIO%d",
             CONFIG_GOLDENFORM_HAPTIC_GPIO);
  } else {
    ESP_LOGW(TAG, "Haptic motor init failed - continuing without haptic");
  }

  // Initialize stroke detector
  stroke_detector_init();

  // Try to load ideal stroke from SD card
  if (storage_available) {
    FILE *ideal_file = fopen("/sdcard/ideal_stroke.bin", "rb");
    if (ideal_file) {
      fseek(ideal_file, 0, SEEK_END);
      long file_size = ftell(ideal_file);
      fseek(ideal_file, 0, SEEK_SET);
      if (file_size > 0 && file_size <= (200 * 3 * sizeof(float))) {
        float *ideal_data = malloc(file_size);
        if (ideal_data) {
          size_t read_bytes = fread(ideal_data, 1, file_size, ideal_file);
          size_t num_samples = read_bytes / (3 * sizeof(float));
          if (num_samples > 0) {
            float ent = 30.0f;
            if (goldenform_ideal_boot_entry_angle(&ent))
              ESP_LOGI(TAG, "Ideal entry angle from NVS: %.1f°", ent);
            stroke_detector_load_ideal(ideal_data, num_samples, ent);
            ESP_LOGI(TAG, "Loaded ideal stroke: %zu samples from SD",
                     num_samples);
          }
          free(ideal_data);
        }
      }
      fclose(ideal_file);
    } else {
      ESP_LOGI(TAG, "No ideal stroke data on SD card (normal for first use)");
    }
  }

  // Load user configuration from NVS
  nvs_handle_t nvs;
  if (nvs_open("goldenform", NVS_READONLY, &nvs) == ESP_OK) {
    float w_cm = 180.0f;
    float h_cm = 180.0f;
    int skill_val = 1; // intermediate

    size_t required_size = sizeof(float);
    if (nvs_get_blob(nvs, "user_cfg_w", &w_cm, &required_size) != ESP_OK) w_cm = 180.0f;
    required_size = sizeof(float);
    if (nvs_get_blob(nvs, "user_cfg_h", &h_cm, &required_size) != ESP_OK) h_cm = 180.0f;
    required_size = sizeof(int);
    if (nvs_get_blob(nvs, "user_cfg_s", &skill_val, &required_size) != ESP_OK) skill_val = 1;

    stroke_detector_set_user_params(w_cm, (haptic_skill_level_t)skill_val);
    nvs_close(nvs);
  } else {
    // defaults
    stroke_detector_set_user_params(180.0f, HAPTIC_SKILL_INTERMEDIATE);
  }

  // Print status
  ESP_LOGI(TAG, "==========================================");
  ESP_LOGI(TAG, "System Status:");
  ESP_LOGI(TAG, "  BNO055 IMU: %s", bno055_available ? "OK" : "NOT FOUND");
  ESP_LOGI(TAG, "  SD Card: %s", storage_available ? "OK" : "NOT FOUND");
  ESP_LOGI(TAG, "  WiFi AP: %s", wifi_available ? "OK" : "FAILED");
  ESP_LOGI(TAG, "  Haptic: %s", haptic_is_available() ? "OK" : "NOT AVAILABLE");
  ESP_LOGI(TAG, "  Stroke Detector: OK (ideal: %s)",
           stroke_detector_has_ideal() ? "LOADED" : "NONE");
  if (system_has_error) {
    ESP_LOGW(TAG, "  Error LED: ON (one or more subsystems failed)");
  }
  ESP_LOGI(TAG, "==========================================");
  ESP_LOGI(TAG, "Button GPIO%d: Press = Start/Stop recording", BUTTON_GPIO);
  if (BUTTON_GPIO == 0) {
    ESP_LOGI(TAG, "              Hold ~1.5s, then RELEASE = Sync (GPIO0 / BOOT)");
  } else {
    ESP_LOGI(TAG, "              Hold ~1.5s = Sync via WiFi");
  }
  ESP_LOGI(TAG, "==========================================");

  // Ensure LED is OFF in IDLE state
  led_set(false);
  current_state = STATE_IDLE;

  // Main loop - uses vTaskDelayUntil() for precise timing-critical IMU sampling
  // This ensures consistent sample intervals even if other tasks run
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_GOLDENFORM_SAMPLE_HZ);

  while (1) {
    button_poll();

    // Detect button hold (while button is still down)
    if (button_is_pressed && !button_hold_triggered &&
        button_press_start_ms > 0) {
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
      if ((now_ms - button_press_start_ms) >= BUTTON_HOLD_MS) {
        button_hold_triggered = true;
        if (BUTTON_GPIO == 0) {
          if (current_state == STATE_IDLE ||
              current_state == STATE_LOGGING) {
            pending_sync_after_release = true;
            ESP_LOGI(TAG,
                     "Sync armed — release BOOT to start Wi‑Fi (GPIO0: avoids "
                     "brownout / UART download mode while held)");
          }
        } else {
          ESP_LOGI(TAG, "Button HOLD detected");
          handle_button_hold();
        }
      }
    }

    // Handle short press (on release)
    if (button_short_press) {
      button_short_press = false;
      ESP_LOGI(TAG, "Button SHORT PRESS detected");
      handle_button_press();
    }

    // Auto-return to IDLE when sync transfer completes (AP stays alive)
    if (current_state == STATE_SYNCING && wifi_available &&
        wifi_server_is_transfer_complete()) {
      ESP_LOGI(TAG, "Transfer complete — returning to IDLE (AP stays alive)");
      vTaskDelay(pdMS_TO_TICKS(1000));
      transition_to_idle(true);
    }

    // Sync timeout: auto-return to IDLE after 5 minutes with no transfer
    {
      static uint32_t sync_timeout_counter = 0;
      if (current_state == STATE_SYNCING && wifi_available &&
          !wifi_server_is_transfer_complete()) {
        sync_timeout_counter++;
        if (sync_timeout_counter >=
            (uint32_t)(300 * CONFIG_GOLDENFORM_SAMPLE_HZ)) {
          ESP_LOGW(TAG, "Sync timeout (5 min) - auto-returning to IDLE");
          sync_timeout_counter = 0;
          transition_to_idle(false);
        }
      } else {
        sync_timeout_counter = 0;
      }
    }

    // AP linger timeout: shut down AP if idle for 10 minutes
    if (AP_LINGER_TIMEOUT_MS > 0 && ap_lingering && wifi_available && current_state == STATE_IDLE &&
        !ap_post_reg_keepalive) {
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
      if ((now_ms - ap_linger_start_ms) > AP_LINGER_TIMEOUT_MS) {
        ESP_LOGI(TAG, "AP linger timeout (10 min) — shutting down WiFi");
        wifi_server_stop_ap();
        ap_lingering = false;
        led_blink_stop_and_wait();
        status_led_blink_stop_and_wait();
      }
    }

    // Keep AP alive after registration/sync; only stop it when recording starts.

    /* IMU samples → protobuf on SD; dashboard integrates via StrokeProcessor after Wi‑Fi sync.
     * (USB/UART is IDF console only — not the PDP data path.) */
    if (bno055_available) {
      bno055_sample_t sample;
      err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &sample);
      if (err == ESP_OK) {
        // Calibration monitoring - log status changes and announce full
        // calibration
        {
          static int8_t prev_sys = -1, prev_gyro = -1, prev_accel = -1,
                        prev_mag = -1;
          static bool fully_calibrated_announced = false;
          bool changed =
              (sample.sys_cal != prev_sys || sample.gyro_cal != prev_gyro ||
               sample.accel_cal != prev_accel || sample.mag_cal != prev_mag);
          if (changed) {
            ESP_LOGI("CAL", "Sys:%d/3  Gyro:%d/3  Accel:%d/3  Mag:%d/3",
                     sample.sys_cal, sample.gyro_cal, sample.accel_cal,
                     sample.mag_cal);
            prev_sys = sample.sys_cal;
            prev_gyro = sample.gyro_cal;
            prev_accel = sample.accel_cal;
            prev_mag = sample.mag_cal;

            if (sample.sys_cal == 3 && sample.gyro_cal == 3 &&
                sample.accel_cal == 3 && sample.mag_cal == 3) {
              if (!fully_calibrated_announced) {
                ESP_LOGI("CAL", "========================================");
                ESP_LOGI("CAL", "  ALL SENSORS FULLY CALIBRATED (3/3)");
                ESP_LOGI("CAL", "  Session data will have best accuracy");
                ESP_LOGI("CAL", "========================================");
                fully_calibrated_announced = true;

                // Save calibration to NVS for one-shot calibration on next boot
                uint8_t cal_data[22];
                esp_err_t save_err = bno055_save_calibration_data(
                    I2C_NUM_0, BNO055_ADDR_A, cal_data);
                if (save_err == ESP_OK) {
                  nvs_handle_t nvs;
                  if (nvs_open("goldenform", NVS_READWRITE, &nvs) == ESP_OK) {
                    nvs_set_blob(nvs, "bno055_cal", cal_data, 22);
                    nvs_commit(nvs);
                    nvs_close(nvs);
                    ESP_LOGI(
                        "CAL",
                        "Calibration saved to NVS — next boot will be instant");
                  }
                }
              }
            } else {
              fully_calibrated_announced = false;
            }
          }
        }

        // --- Stroke Detection & Haptic Feedback ---
        // Feed IMU samples through the on-device stroke detector ONLY during LOGGING.
        // This prevents "ghost strokes" in the logs while the device is IDLE.
        stroke_event_t stroke_event = {0};
        if (current_state == STATE_LOGGING) {
            stroke_event = stroke_detector_feed(&sample);
        }
        
        sample.haptic_fired = stroke_event.haptic_fired ? 1 : 0;
        sample.deviation_score = stroke_event.deviation_score;
        sample.haptic_reason = stroke_event.haptic_reason;
        sample.pull_duration_ms = stroke_event.pull_duration_ms;
        sample.stroke_count = stroke_event.stroke_count;
        sample.turn_count = stroke_event.turn_count;

        // Populate device metadata (wrist side from NVS / user_config, not compile-time)
        sample.device_id = CONFIG_GOLDENFORM_DEVICE_ID;
        sample.device_role = goldenform_device_role_pb();
        sample.entry_angle = stroke_event.entry_angle;
        /* Protobuf field 32: reserved for legacy sessions; always zero in current firmware */
        sample.breath_count = 0;

        if (stroke_event.stroke_detected) {
          ESP_LOGI(TAG, "Stroke #%u detected", (unsigned)stroke_event.stroke_count);
        }
        if (stroke_event.turn_detected) {
          ESP_LOGI(TAG, "Turn #%u detected", (unsigned)stroke_event.turn_count);
        }

        // When logging, also store to SD card
        if (current_state == STATE_LOGGING && storage_available &&
            storage_is_recording()) {
          if (DEBUG_SD_CARD) {
            sample.ax = 1.0f;
            sample.ay = 1.0f;
            sample.az = 1.0f;
            sample.gx = 1.0f;
            sample.gy = 1.0f;
            sample.gz = 1.0f;
            sample.mx = 1.0f;
            sample.my = 1.0f;
            sample.mz = 1.0f;
            sample.roll = 1.0f;
            sample.pitch = 1.0f;
            sample.yaw = 1.0f;
            sample.qw = 1.0f;
            sample.qx = 1.0f;
            sample.qy = 1.0f;
            sample.qz = 1.0f;
            sample.lia_x = 1.0f;
            sample.lia_y = 1.0f;
            sample.lia_z = 1.0f;
            sample.temp = 1.0f;
            sample.sys_cal = 1;
            sample.gyro_cal = 1;
            sample.accel_cal = 1;
            sample.mag_cal = 1;
          }
          err = storage_enqueue_bno_sample(&sample);
          if (err == ESP_OK) {
            session_sample_count++;
            if (session_sample_count % 100 == 0) {
              ESP_LOGI(TAG, "Logging: %u samples", session_sample_count);
            }
          } else {
            ESP_LOGE(TAG, "SD card write error: %s", esp_err_to_name(err));
            error_led_set(true);
          }
        }
      } else if (current_state == STATE_LOGGING) {
        ESP_LOGE(TAG, "IMU read error during logging: %s", esp_err_to_name(err));
        error_led_set(true);
      }
    }

    // Status update every 5 seconds when idle
    static uint32_t status_counter = 0;
    if (current_state == STATE_IDLE) {
      status_counter++;
      if (status_counter >= (uint32_t)(5 * CONFIG_GOLDENFORM_SAMPLE_HZ)) {
        status_counter = 0;
        ESP_LOGI(TAG, "Status: IDLE - Press button to start");
      }
    } else {
      status_counter = 0; // Reset counter when not idle
    }

    vTaskDelayUntil(&last_wake, period);
  }
}
