 ID | Requirement | File | Lines | Search Term |
|----|-------------|------|-------|------------|
| 1-3-1 | IMU retry ×5 | main/app_main.c | 654–683 | `IMU_MAX_INIT_ATTEMPTS` |
| 1-1-2 | IMU I2C | components/imu_bno055/bno055.c | 67–75, 291–360 | `bus_i2c_wrrd`, `bno055_read_sample` |
| 7-5-1 | SD mount retry | components/storage/storage.c | 785–807 | `max_retries` (first occurrence) |
| 7-4-1 | SPI for SD | components/storage/storage.c | 743–788 | `esp_vfs_fat_sdspi_mount` |
| 7-1-1 | Session storage | components/storage/storage.c | 821–835 | `open_new_data_file` |
| 7-1-2 | Multi-session scan | components/storage/storage.c | 139–156 | `session_number` |
| 5-2-1 | Button sync → AP | main/app_main.c | 406–444 | `transition_to_syncing` |
| 5-2-2 | Orchestrate transfer | components/wifi_server/wifi_server.c | 1690–1766 | `wifi_server_start_sync` |
| 5-4-1 | Data compression/packing (protobuf) | components/protobuf/protobuf_utils.c + components/storage/storage.c | 121–231, 937 | `protobuf_encode_batch`, `protobuf_write_delimited` |
| 8-2-1 | LED patterns | main/app_main.c | 81, 323–368, 437–441 | `STATUS_LED_GPIO` |
| 6-1-1 | Metrics | dashboard/simple_imu_visualizer.py | 808–822 | `vis_data` |
| 6-1-2 | Playback | dashboard/js/gf_playback.js | 13–46 | `togglePlayback` |
| 6-1-3 | Drift correction | dashboard/js/gf_viz_integration.js | 454–527 | `applyPerStrokeOriginOffset` |
| 6-2-2 | HTTP endpoint | components/wifi_server/wifi_server.c | 940+ | `data_json_handler` |
| 6-3-1 | History | dashboard/wifi_session_processor.py | 835–863 | `_get_sessions` |
| 4-3-1 | Voltage regulation | (hardware) | — | Schematic |
| 1-1-1 | Haptic trigger | components/stroke_detector/stroke_detector.c | 381–423 | `haptic_play_pattern` |
| 2-1-1 | Ideal comparison | dashboard/js/gf_ideal_comparison.js | 117–190 | `gfVsIdealMetrics` |
| 3-1-1 | Device registration | dashboard/database.py + dashboard/wifi_session_processor.py | 322–336, 493–511 | `register_device`, `_register_device` |
| 3-2-1 | Merge sessions | dashboard/wifi_session_processor.py | 865–900 | `_merge_sessions` |

---

## By Category

### On-Board Firmware (TIDRs 1-x, 5-x, 7-x, 8-x)
- **1-3-1**: `main/app_main.c:654` — IMU init retry
- **1-1-2**: `components/imu_bno055/bno055.c:67` — I2C read
- **5-2-1**: `main/app_main.c:406` — Button to sync
- **5-2-2**: `components/wifi_server/wifi_server.c:1690` — Orchestrate transfer
- **5-4-1**: `components/protobuf/protobuf_utils.c:121` — Protobuf batch encoding before SD write
- **7-4-1**: `components/storage/storage.c:743` — SPI mount
- **7-5-1**: `components/storage/storage.c:785` — SD retry
- **7-1-1**: `components/storage/storage.c:821` — Session storage
- **7-1-2**: `components/storage/storage.c:139` — Multi-session scan
- **8-2-1**: `main/app_main.c:81` — LED patterns

### Web App (TIDRs 6-x)
- **6-1-1**: `dashboard/simple_imu_visualizer.py:808` — Metrics
- **6-1-2**: `dashboard/js/gf_playback.js:13` — Playback controls
- **6-1-3**: `dashboard/js/gf_viz_integration.js:454` — Drift fix
- **6-2-2**: `components/wifi_server/wifi_server.c:940` — HTTP data
- **6-3-1**: `dashboard/wifi_session_processor.py:835` — History

### Hardware (TIDRs 4-x)
- **4-3-1**: Schematic — Voltage regulation

### Stretch Goals (TIDRs 1-1-1, 2-1-1, 3-x-x)
- **1-1-1**: `components/stroke_detector/stroke_detector.c:381` — Haptic
- **2-1-1**: `dashboard/js/gf_ideal_comparison.js:117` — Coaching
- **3-1-1**: `dashboard/database.py:322` — Device registration
- **3-2-1**: `dashboard/wifi_session_processor.py:865` — Merge sessions

---

## Tips

1. **Use Ctrl+G (or Cmd+G)** in VS Code to jump to a line number
2. **Search for the search term** in the file if line numbers drift
3. **Line comments** in the code mention the TIDR (e.g., "TIDR 1-3-1")
4. **Copy the file path**, then **Cmd+P** (or Ctrl+P) to quick-open it

## Compression Note (TIDR 5-4-1)

- Current implementation uses protobuf binary serialization and length-delimited batching (nanopb), not gzip/deflate.
- Encoding path: `protobuf_encode_batch` and `protobuf_write_delimited` in `components/protobuf/protobuf_utils.c`.
- Write call site from storage flush: `components/storage/storage.c` line ~937.

## Non-TIDR System + Low-Level (Compact)

| Req | Requirement (non-TIDR) | File | Lines | Search Term |
|----|--------------------------|------|-------|------------|
| 7-1 | No overwrite/loss while logging | components/storage/storage.c | 898–951, 869–870 | `flush_psram_to_sd`, `fflush`, `fsync` |
| 7-2 | Read order correct during wireless transfer | components/wifi_server/wifi_server.c | 160–217 | `build_pb_file_list`, `qsort` |
| 1-2,7-3 | Timing correctness IMU read -> SD write | main/app_main.c | 855, 858, 944, 1081 | `CONFIG_GOLDENFORM_SAMPLE_HZ`, `vTaskDelayUntil` |
| 5-3 | Wireless transfer integrity (counts/complete) | components/wifi_server/wifi_server.c | 561–596, 865–877 | `status_handler`, `total_samples_sent` |
| 6-1 | GUI metrics + plots | dashboard/simple_imu_visualizer.py | 808–822 | `vis_data` |
| 5-1,6-2 | Correct web requests for sync | dashboard/wifi_session_processor.py | 28, 1157; 132, 776 | `ESP32_URL`, `url = f'{ESP32_URL}/data.json'`, `/api/device_info` |
| 6-3 | Previous-session history | dashboard/wifi_session_processor.py | 835–863 | `_get_sessions` |
| 8-3 | Start/stop recording via button | main/app_main.c | 515–523 | `handle_button_press` |
| 1-1 | 9-axis + configurable sample rate | components/imu_bno055/bno055.c; main/Kconfig.projbuild; main/app_main.c | 291–428; 121–126; 858 | `bno055_read_sample`, `GOLDENFORM_SAMPLE_HZ` |
| 8-1 | Button debounce (firmware side) | main/app_main.c | 94, 156–172 | `BUTTON_DEBOUNCE_MS`, `button_poll` |
| 6-2-1 | User sees connection success/failure | dashboard/js/gf_wifi_banners.js | 17, 73–78 | `updateWearableConnectionBanner` |
| 6-1-4 | Calibration status displayed in app | dashboard/js/gf_calibration.js | 330–345 | `updateCalibrationDisplay` |
| 5-2-3 | Wi-Fi constrained for certification story | components/wifi_server/wifi_server.c; sdkconfig | 439–441; 2494 | `esp_wifi_set_max_tx_power`, `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER` |
| 5-3-1 | >=1 KB/s transfer-rate requirement (measurement hook) | components/wifi_server/wifi_server.c | 303, 561–596, 875–877 | `/status`, `total_samples_sent` |
| 1-1-1 | IMU provides 9-axis values | components/imu_bno055/bno055.c | 291–428 | `bno055_read_sample` |
| 1-1-3 | IMU calibration across sys/gyro/accel/mag | main/app_main.c; components/imu_bno055/bno055.c | 949–980; 264–274 | `sample.sys_cal`, `bno055_get_calibration_status` |
| 1-1-2 (stretch) | Stroke start/end for haptic compare logic | components/stroke_detector/stroke_detector.c | 71, 320–356 | `stroke_detected`, `stroke_integrating` |
| 1-1-3 (stretch) | Receive/store ideal stroke from off-board | components/wifi_server/wifi_server.c | 1191–1282 | `ideal_stroke_post_handler`, `stroke_detector_load_ideal` |
| 1-2-1 (stretch) | Haptic motor driver path | components/haptic/haptic.c | 155–171, 238 | `haptic_init`, `haptic_play_pattern` |
| 2-1-2 (stretch) | Show haptic-trigger points in session viz | dashboard/js/gf_haptic_timeline.js | 2, 48–66 | `haptic_fired`, `buildHapticTimeline` |
| 3-3-1 (stretch) | Common timestamp alignment across devices | dashboard/simple_imu_visualizer.py; dashboard/wifi_session_processor.py | 877–900; 25, 900 | `align_sessions_by_hop` |

Hardware-only (no firmware line mapping): `2-1`, `2-2`, `3-1`, `3-2`, `4-1`, `4-2`, `4-4`, `4-1-1`, `4-1-2`, `4-2-1`.

---

curl -o /dev/null -s -w "%{speed_download}\n" http://192.168.4.1/data | awk '{if($1>=1024) print "PASS ("$1" B/s)"; else print "FAIL ("$1" B/s)"}'

^ for bit speed test
