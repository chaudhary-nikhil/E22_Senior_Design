#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/uart.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "wifi_server.h"

static const char *TAG = "GOLDENFORM";

// ============================================================================
// Configuration Constants
// ============================================================================

// GPS UART configuration (moved to UART1 to avoid USB console conflict)
#define GPS_UART_NUM			UART_NUM_1
#define GPS_TXD_GPIO			17	// ESP32 TX1 (GPIO17) -> SAM-M8Q RX
#define GPS_RXD_GPIO			16	// ESP32 RX1 (GPIO16) <- SAM-M8Q TX
#define GPS_BAUD_RATE			9600	// Default NMEA baud for u-blox unless reconfigured
#define GPS_UART_BUF_SIZE		2048
#define GPS_RX_BUFFER_SIZE		256
#define GPS_LINE_BUFFER_SIZE		256
#define GPS_READ_TIMEOUT_MS		100

// Button GPIO definitions - Single button for start/stop toggle
#define TOGGLE_BUTTON_GPIO		0	// GPIO0 for toggle logging (BOOT button on ESP32 DevKit C)
#define STATUS_LED_GPIO			2	// LED (GPIO2 - standard for ESP32 DevKit C onboard LED)
#define BUTTON_DEBOUNCE_MS		300

// I2C Configuration
#define I2C_NUM				I2C_NUM_0
#define I2C_SDA_GPIO			21
#define I2C_SCL_GPIO			22
#define I2C_FREQ_HZ			100000	// 100kHz for BNO055 compatibility

// IMU Sampling Configuration
#define IMU_SAMPLING_RATE_HZ		50	// 50Hz sampling rate
#define IMU_SAMPLING_PERIOD_MS		(1000 / IMU_SAMPLING_RATE_HZ)	// 20ms

// Status Reporting Configuration
#define STATUS_REPORT_INTERVAL_MS	5000	// Report status every 5 seconds
#define STATUS_REPORT_COUNT_INTERVAL	(STATUS_REPORT_INTERVAL_MS / IMU_SAMPLING_PERIOD_MS)	// 250 at 50Hz

// Data Point Logging Configuration
#define DATA_POINT_LOG_INTERVAL		50	// Log progress every 50 data points

// GPS Reader Task Configuration
#define GPS_TASK_STACK_SIZE		4096
#define GPS_TASK_PRIORITY		5
#define GPS_OVERFLOW_WARN_INTERVAL	100	// Log overflow warning every 100 occurrences

// ============================================================================
// Global State Variables
// ============================================================================

// Logging state
static volatile bool logging_active = false;
static volatile bool button_pressed = false;
static volatile uint32_t last_button_time = 0;

// Session tracking
static uint32_t session_data_count = 0;
static uint32_t session_start_time = 0;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Check if NMEA sentence is a key sentence type (GGA or RMC) that should be
 * logged at INFO level instead of DEBUG level.
 * 
 * @param line NMEA sentence line (null-terminated)
 * @return true if sentence is GGA or RMC (GN or GP prefix), false otherwise
 */
static bool is_important_nmea_sentence(const char *line) {
	return (strstr(line, "$GNGGA") == line ||
		strstr(line, "$GPGGA") == line ||
		strstr(line, "$GNRMC") == line ||
		strstr(line, "$GPRMC") == line);
}

/**
 * Button interrupt handler with debouncing.
 * Sets button_pressed flag when button is pressed (debounced).
 */
static void IRAM_ATTR button_isr_handler(void* arg) {
	uint32_t current_time = esp_timer_get_time() / 1000; // Convert to milliseconds
	if (current_time - last_button_time > BUTTON_DEBOUNCE_MS) {
		button_pressed = true;
		last_button_time = current_time;
	}
}

/**
 * Initialize buttons and LED.
 * Configures toggle button (GPIO0) and status LED (GPIO2).
 * 
 * @return ESP_OK on success
 */
static esp_err_t init_buttons(void) {
	// Configure TOGGLE button (GPIO0) - BOOT button on ESP32 DevKit C
	gpio_config_t toggle_config = {
		.pin_bit_mask = (1ULL << TOGGLE_BUTTON_GPIO),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE  // Trigger on button press (falling edge)
	};
	ESP_ERROR_CHECK(gpio_config(&toggle_config));
	
	// Configure status LED (GPIO2)
	gpio_config_t led_config = {
		.pin_bit_mask = (1ULL << STATUS_LED_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&led_config));
	
	// Install GPIO ISR service
	ESP_ERROR_CHECK(gpio_install_isr_service(0));
	
	// Add ISR handler for toggle button
	ESP_ERROR_CHECK(gpio_isr_handler_add(TOGGLE_BUTTON_GPIO, button_isr_handler, (void*) TOGGLE_BUTTON_GPIO));
	
	// Initialize LED to OFF
	gpio_set_level(STATUS_LED_GPIO, 0);
	
	ESP_LOGI(TAG, "Button initialized: GPIO%d=Toggle Start/Stop (BOOT button), GPIO%d=LED", 
		 TOGGLE_BUTTON_GPIO, STATUS_LED_GPIO);
	return ESP_OK;
}

// ============================================================================
// WiFi Server Callbacks
// ============================================================================

/**
 * WiFi server status callback.
 * Called when WiFi server state changes.
 * 
 * @param state New WiFi server state
 */
static void wifi_status_callback(wifi_server_state_t state) {
	ESP_LOGI(TAG, "WiFi server state changed to: %d", state);
}

/**
 * Session status callback.
 * Called when session state changes.
 * 
 * @param state New session state
 */
static void session_status_callback(session_state_t state) {
	ESP_LOGI(TAG, "Session state changed to: %d", state);
}

// ============================================================================
// GPIO and LED Functions
// ============================================================================

/**
 * Update LED status - ONLY ON during active logging sessions
 */
static void update_led_status(void) {
	gpio_set_level(STATUS_LED_GPIO, logging_active ? 1 : 0);
	ESP_LOGI(TAG, "LED status: %s", logging_active ? "ON (logging)" : "OFF (stopped)");
}

/**
 * Initialize UART for GPS (NMEA from SAM-M8Q).
 * Configures UART1 for GPS communication at 9600 baud.
 * 
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t gps_uart_init(void) {
	uart_config_t uart_config = {
		.baud_rate = GPS_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB
	};
	// Install driver; tolerate already-installed (e.g., console on UART0)
	esp_err_t di_err = uart_driver_install(GPS_UART_NUM, GPS_UART_BUF_SIZE, 0, 0, NULL, 0);
	if (di_err != ESP_OK && di_err != ESP_ERR_INVALID_STATE) {
		ESP_LOGE(TAG, "GPS UART driver install failed: %s", esp_err_to_name(di_err));
		return di_err;
	}
	ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TXD_GPIO, GPS_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_LOGI(TAG, "✅ GPS UART initialized on UART%d: TX=%d RX=%d @ %d baud", GPS_UART_NUM, GPS_TXD_GPIO, GPS_RXD_GPIO, GPS_BAUD_RATE);
	return ESP_OK;
}

/**
 * GPS reader task - reads NMEA sentences from UART and logs them.
 * Key sentences (GGA/RMC) are logged at INFO level, others at DEBUG level.
 */
static void gps_reader_task(void *arg) {
	uint8_t rx_buffer[GPS_RX_BUFFER_SIZE];
	char line_buffer[GPS_LINE_BUFFER_SIZE];
	int line_len = 0;
	uint32_t overflow_count = 0;

	while (1) {
		int len = uart_read_bytes(GPS_UART_NUM, rx_buffer, sizeof(rx_buffer), 
					   pdMS_TO_TICKS(GPS_READ_TIMEOUT_MS));
		if (len > 0) {
			for (int i = 0; i < len; i++) {
				char c = (char)rx_buffer[i];
				
				// Ignore carriage return (CR)
				if (c == '\r') {
					continue;
				}
				
				// Handle line feed (LF) - end of NMEA sentence
				if (c == '\n') {
					if (line_len > 0) {
						line_buffer[line_len] = '\0';
						
						// Log key sentence types at INFO, others at DEBUG
						if (is_important_nmea_sentence(line_buffer)) {
							ESP_LOGI(TAG, "GPS: %s", line_buffer);
						} else {
							ESP_LOGD(TAG, "GPS: %s", line_buffer);
						}
						line_len = 0;
					}
				} else {
					// Add character to line buffer
					if (line_len < (int)sizeof(line_buffer) - 1) {
						line_buffer[line_len++] = c;
					} else {
						// Buffer overflow - reset and log warning
						line_len = 0;
						overflow_count++;
						if (overflow_count % GPS_OVERFLOW_WARN_INTERVAL == 0) {
							ESP_LOGW(TAG, "GPS line buffer overflow (count: %u)", overflow_count);
						}
					}
				}
			}
		}
	}
}

/**
 * Toggle logging session (start/stop with single button).
 * Starts or stops data logging session and updates LED status.
 */
static void toggle_logging_session(void) {
	if (!logging_active) {
		// Start logging
		logging_active = true;
		session_data_count = 0;
		session_start_time = esp_timer_get_time() / 1000; // Convert to milliseconds
		
		ESP_LOGI(TAG, "🟢 LOGGING SESSION STARTED");
		ESP_LOGI(TAG, "📊 Press BOOT button again to STOP logging");
		ESP_LOGI(TAG, "💾 Data will be stored as protobuf on ESP32");
		ESP_LOGI(TAG, "📡 Data will be sent via WiFi to dashboard");
		ESP_LOGI(TAG, "💡 LED is ON - logging is active");
		
		// Start WiFi server logging
		esp_err_t wifi_err = wifi_server_start_logging();
		if (wifi_err != ESP_OK) {
			ESP_LOGW(TAG, "Failed to start WiFi server logging: %s", esp_err_to_name(wifi_err));
		}
		
		update_led_status();
	} else {
		// Stop logging
		logging_active = false;
		uint32_t session_duration = (esp_timer_get_time() / 1000) - session_start_time;
		
		ESP_LOGI(TAG, "🔴 LOGGING SESSION STOPPED");
		ESP_LOGI(TAG, "📊 Session duration: %u ms", session_duration);
		ESP_LOGI(TAG, "📈 Data points collected: %u", session_data_count);
		ESP_LOGI(TAG, "💾 Protobuf data stored on ESP32");
		ESP_LOGI(TAG, "📡 Sending data via WiFi to dashboard...");
		ESP_LOGI(TAG, "💡 LED is OFF - logging stopped");
		
		// Stop WiFi server logging
		esp_err_t wifi_err = wifi_server_stop_logging();
		if (wifi_err != ESP_OK) {
			ESP_LOGW(TAG, "Failed to stop WiFi server logging: %s", esp_err_to_name(wifi_err));
		}
		
		// Session data is available via HTTP GET /session_data endpoint
		ESP_LOGI(TAG, "📤 Session data available via HTTP GET /session_data");
		
		update_led_status();
		
		// Reset session counter
		session_data_count = 0;
	}
}

// ============================================================================
// Main Application
// ============================================================================

void app_main(void) {
	ESP_LOGI(TAG, "🏊 GoldenForm Production Firmware Starting");
	ESP_LOGI(TAG, "==========================================");

	// Init I2C
	esp_err_t i2c_err = bus_i2c_init(I2C_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_FREQ_HZ);
	if (i2c_err != ESP_OK) {
		ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(i2c_err));
		ESP_LOGE(TAG, "Please check I2C wiring");
	} else {
		ESP_LOGI(TAG, "✅ I2C initialized successfully");
	}

	// Init BNO055 IMU (with graceful error handling)
	esp_err_t bno_err = bno055_init(I2C_NUM, BNO055_ADDR_A);
	if (bno_err != ESP_OK) {
		ESP_LOGE(TAG, "BNO055 initialization failed: %s", esp_err_to_name(bno_err));
		ESP_LOGE(TAG, "Please check BNO055 wiring: SDA=%d, SCL=%d, VCC=3.3V, GND=GND", 
			 I2C_SDA_GPIO, I2C_SCL_GPIO);
		ESP_LOGE(TAG, "Continuing without BNO055 - no IMU data will be logged");
		// Don't crash, just continue without IMU
	} else {
		ESP_LOGI(TAG, "✅ BNO055 initialized successfully");
	}

	// Init buttons and LED
	ESP_ERROR_CHECK(init_buttons());

	// Init WiFi server (with error handling)
	esp_err_t wifi_init_status = wifi_server_init(wifi_status_callback, session_status_callback);
	if (wifi_init_status != ESP_OK) {
		ESP_LOGE(TAG, "WiFi server initialization failed: %s", esp_err_to_name(wifi_init_status));
		ESP_LOGE(TAG, "Continuing without WiFi server - logging will work locally only");
		// Don't crash, just continue without WiFi server
	} else {
		ESP_LOGI(TAG, "✅ WiFi server initialized successfully");
		ESP_LOGI(TAG, "🌐 Connect to WiFi: %s (password: %s)", WIFI_AP_SSID, WIFI_AP_PASSWORD);
		ESP_LOGI(TAG, "🌐 Web interface: http://192.168.4.1");
	}

	// Init GPS UART and start reader task
	if (gps_uart_init() == ESP_OK) {
		xTaskCreatePinnedToCore(gps_reader_task, "gps_reader", GPS_TASK_STACK_SIZE, 
					NULL, GPS_TASK_PRIORITY, NULL, tskNO_AFFINITY);
	} else {
		ESP_LOGW(TAG, "GPS UART init failed - GPS data will not be read");
	}

	ESP_LOGI(TAG, "🚀 All systems initialized - Production logging ready");
	ESP_LOGI(TAG, "🔍 UNIQUE TEST ID: ESP32_GOLDENFORM_TEST_2025");
	ESP_LOGI(TAG, "==========================================");
	ESP_LOGI(TAG, "🔄 Press BOOT button (GPIO%d) to TOGGLE logging (Start/Stop)", TOGGLE_BUTTON_GPIO);
	ESP_LOGI(TAG, "💡 LED indicates logging status (ON=logging, OFF=stopped)");
	ESP_LOGI(TAG, "📊 Data stored as protobuf → WiFi → Dashboard → Visualization");
	ESP_LOGI(TAG, "==========================================");

	TickType_t t0 = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(IMU_SAMPLING_PERIOD_MS);

	while (1) {
		// Check for button presses
		if (button_pressed) {
			button_pressed = false;
			toggle_logging_session();
		}

		// Read IMU data if logging is active and BNO055 is available
		if (logging_active && bno_err == ESP_OK) {
			bno055_sample_t s;
			esp_err_t err = bno055_read_sample(I2C_NUM, BNO055_ADDR_A, &s);
			if (err == ESP_OK) {
				// Show first data point for verification
				if (session_data_count == 0) {
					ESP_LOGI(TAG, "🎯 First IMU data: ax=%.2f, ay=%.2f, az=%.2f, qw=%.3f", 
						 s.ax, s.ay, s.az, s.qw);
				}
				
				// Send data to WiFi server for storage (only if WiFi server is working)
				if (wifi_init_status == ESP_OK) {
					esp_err_t add_err = wifi_server_add_data_point(
						s.t_ms,
						s.ax, s.ay, s.az,
						s.gx, s.gy, s.gz,
						s.qw, s.qx, s.qy, s.qz,
						s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal
					);
					if (add_err != ESP_OK) {
						ESP_LOGW(TAG, "Failed to add data point: %s", esp_err_to_name(add_err));
					} else {
						session_data_count++;
						// Log progress periodically
						if (session_data_count % DATA_POINT_LOG_INTERVAL == 0) {
							ESP_LOGI(TAG, "📊 Collected %u data points", session_data_count);
						}
					}
				} else {
					ESP_LOGW(TAG, "WiFi server not available - data not stored");
				}
			} else {
				ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
			}
		}

		// Status message at configured interval
		static int status_count = 0;
		if (status_count % STATUS_REPORT_COUNT_INTERVAL == 0) {
			if (wifi_init_status == ESP_OK) {
				ESP_LOGI(TAG, "Status: Logging=%s, WiFi=%d, Data points=%u", 
					 logging_active ? "ON" : "OFF", 
					 wifi_server_get_state(),
					 wifi_server_get_session_count());
			} else {
				ESP_LOGI(TAG, "Status: Logging=%s, WiFi=FAILED, LED=%s", 
					 logging_active ? "ON" : "OFF",
					 gpio_get_level(STATUS_LED_GPIO) ? "ON" : "OFF");
			}
		}
		status_count++;
		
		vTaskDelayUntil(&t0, period);
	}
}
