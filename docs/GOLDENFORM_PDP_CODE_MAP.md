# GoldenForm — PDP requirement → code map (demo cheat sheet)

Use this during design reviews when someone asks “where is requirement X in the repo?”

- **TIDR** = PDP item explicitly tagged *(TIDR)* (testable implementation detail).
- **non-TIDR** = same subsection of the plan, but **no** *(TIDR)* tag in the PDP text.
- **Line numbers** were checked against the repo on **2026-04-17** (search the **symbol** if your branch has diverged).

---

## Stretch goals (Section 4.2) — what we’re implementing

### Product / system level (mostly non-TIDR)

| PDP item | TIDR? | Where in code | What to say in 10 seconds |
|----------|-------|---------------|---------------------------|
| **4.2 product 1** — haptic when motion deviates | non-TIDR | Firmware: `components/stroke_detector/stroke_detector.c` — `stroke_detector_feed` `177`–`414`; DTW `compare_strokes` `126`–`135`; deviation + `haptic_play_pattern` `349`–`388`; `components/haptic/haptic.c` — file header `1`–`19`, `haptic_init` `155`–`195`, `haptic_play_pattern` `238`–`261`; `main/app_main.c` — copy stroke fields to sample `922`–`942`, enqueue to SD `951`–`989`. | On-device: each logged sample can carry `haptic_fired` / `deviation_score`; motor runs when DTW + thresholds say the stroke is off. |
| **4.2 product 2** — advanced insights vs ideal | non-TIDR | Dashboard: `dashboard/js/gf_ideal_comparison.js` (vs-ideal aggregates, e.g. `gfVsIdealMetrics` from `225`); `dashboard/js/gf_coaching.js` — `_buildStrokeAnalytics` `21`–`64`; optional LLM path `dashboard/wifi_session_processor.py` — `_coaching_insights` `935`–`992`, `dashboard/gemini_coaching.py`. | Local math + ideal overlay in the browser; “AI paragraph” only if `GEMINI_API_KEY` is set. |
| **4.2 product 3** — modular / multi-body | non-TIDR | DB: `dashboard/database.py` — `devices` table `80`–`89`; `dashboard/wifi_session_processor.py` — `_register_device` `465`–`481`, `_get_devices` `514`–`519`; merge + time alignment: `dashboard/simple_imu_visualizer.py` — `align_sessions_by_hop` `886`–`956`; `dashboard/wifi_session_processor.py` — `_merge_sessions` `837`–`933` (calls align at `872`–`873`); viz: `dashboard/js/gf_state.js` — `strokeBoundaries` / `streamKey` comment `38`; second cube + trails `47`–`51`. | SQLite stores per-user devices + `role`; hop-based sync aligns clocks; viewer can show two streams when merged data is loaded. |
| **4.2 on-board 1-1** — detect deviation, trigger haptic | non-TIDR | Same as product 1: `stroke_detector.c` + `haptic.c` + `app_main.c` logging path. | Single pipeline: `stroke_detector_feed` → flags on `bno055_sample_t` → protobuf to SD. |
| **4.2 off-board 2-1** — analyze vs ideal, show insights | non-TIDR | `gf_ideal_comparison.js`, `gf_coaching.js`, metrics in `dashboard/js/gf_analysis.js` (haptic counts / deviation). | Post-session only (browser + optional Gemini). |
| **4.2 off-board 3-1** — register/configure multiple devices | non-TIDR | `database.py` `80`–`89`; `_register_device` / `_get_devices` in `wifi_session_processor.py` `465`–`481`, `514`–`519`; UI: `dashboard/js/gf_wifi_devices.js`, `dashboard/js/gf_wifi_banners.js`. | “Register” = row in `devices` with `role` + optional `wifi_ssid`. |
| **4.2 off-board 3-2** — combined visualization | non-TIDR | Merge endpoint `_merge_sessions` `837`–`933`; viewer `gf_state.js` `38`, `47`–`51`; stroke timeline `dashboard/js/gf_haptic_timeline.js` — `computeStrokeBoundaries` `18`–`53` (`getStreamKey` / `streamKey` at `26`–`35`). | Merged JSON must include distinct streams; viz code keys strokes by band. |
| **4.2 off-board 3-3** — synchronized stroke start across devices | non-TIDR | `align_sessions_by_hop` `886`–`956`; `_merge_sessions` calls it at `872`–`873`. | Demo script: stationary offset start, common hop, then merge. |
| **4.2 hardware 1-2** — vibration motor | non-TIDR | `components/haptic/haptic.c`; Kconfig `main/Kconfig.projbuild` — `GOLDENFORM_HAPTIC_GPIO` `112`–`117`; init in `main/app_main.c` `704`–`711`. | ERM + low-side FET (see `haptic.c` header `1`–`19`); GPIO drive in current build (`haptic_init` `155`–`195`). |

### Component-level stretch — TIDR vs non-TIDR

| ID | Text (short) | TIDR? | File(s) | Symbol / lines |
|----|----------------|-------|---------|----------------|
| **1-1-1** | MCU triggers haptic when stroke deviates beyond threshold | **TIDR** | `stroke_detector.c` | `compare_strokes` `126`–`135`; thresholding + `haptic_play_pattern` `349`–`388` |
| **1-1-2** | MCU identifies stroke start/end | non-TIDR | `stroke_detector.c` | Timing / wall macros `40`–`46`; integration window `327`–`407`; main feed `stroke_detector_feed` `177`–`414` |
| **1-1-3** | MCU receives/stores ideal stroke from off-board | **TIDR** | `main/app_main.c`, `wifi_server.c` | Boot load `/sdcard/ideal_stroke.bin` `716`–`742`; `ideal_stroke_post_handler` `1186`–`1289` (writes SD + `stroke_detector_load_ideal` at `1277`); URIs: `data_json_uri` `313`–`316`, `ideal_stroke_post_uri` `323`–`327`, `device_status_uri` `329`–`332`; registered `463`–`465` |
| **1-2-1** | Motor driven through suitable driver | **TIDR** | `haptic.c` | Hardware note `1`–`19`; `haptic_init` `155`–`195`; `haptic_play_pattern` `238`–`261` |
| **2-1-1** | Web app compares session to ideal | **TIDR** | `gf_ideal_comparison.js` | `gfVsIdealMetrics` build-out from `225`; pairs with `gf_analysis.js` |
| **2-1-2** | Viz shows when haptic fired | non-TIDR | `gf_viz_render.js`, `gf_haptic_timeline.js` | `haptic_fired` flash `163`–`164`, legend `275`; `computeStrokeBoundaries` `18`–`53` (haptic leading-edge `41`–`51`) |
| **3-1-1** | Register multiple wearables + body placement | **TIDR** | `database.py`, `wifi_session_processor.py` | `devices.role` `80`–`89`; `_register_device` `465`–`481` |
| **3-2-1** | Combined visualization merged from multiple devices | **TIDR** | `wifi_session_processor.py`, `gf_state.js` | `_merge_sessions` `837`–`933`; dual trail / cube `47`–`51`, `strokeBoundaries` `38` |
| **3-3-1** | Common timestamp / stroke alignment | **TIDR** | `simple_imu_visualizer.py` | `align_sessions_by_hop` `886`–`956` |

---

## Appendix A — Base PDP TIDRs (quick map)

Only the items that usually come up in the same demo conversation.

| ID | Topic | File(s) | Symbol / lines |
|----|-------|---------|----------------|
| 4-3-1 | Buck-boost rail (hardware) | PCB / power BOM — not expressed as C code | Point to schematic/BOM; firmware assumes stable 3V3 |
| 6-1-1 | Stroke metrics in web app | `simple_imu_visualizer.py` | `StrokeProcessor`: `stroke_count` `255`; stroke decision block `567`–`602`; JSON `stroke_count` `829` |
| 6-1-2 | Replay + playback controls | `dashboard/js/gf_playback.js`, `gf_viz_controls.js`, `integrated_session_viewer.html` | `getPlaybackBounds` `5`–`11`, `togglePlayback` `13`–`47`; speed/stroke UI `gf_viz_controls.js` `5`–`23`; `#play-btn` → `togglePlayback()` in HTML `126` |
| 6-1-3 | Limit long-term position drift | `simple_imu_visualizer.py`, `gf_viz_integration.js` | `_update_uart_motion_gate` `323`–`356` (zeros position/velocity on new motion segment `333`–`337`); `integratePositions` `192`–`204`; `applyPerStrokeOriginOffset` `346`–`426` |
| 6-2-2 | HTTP data from band Wi‑Fi | `components/wifi_server/wifi_server.c` | `data_json_uri` `313`–`316`; `device_status_uri` `329`–`332`; handlers registered with server `460`–`472` (includes `data_json_uri` at `463`) |
| 5-2-1 | AP on sync hold | `main/app_main.c` | `transition_to_syncing` `367`–`438`; `button_poll` `137`–`172` |
| 5-2-2 | Orchestrate transfer SD → app | `components/wifi_server/wifi_server.c`, `components/storage/storage.c` | SD → samples: `storage_enqueue_bno_sample` `324`; flush `flush_psram_to_sd` `880`–`941` (`protobuf_write_delimited` `918`–`919`). HTTP export: `data_json_handler` `937`–`1102` (reads `.pb` via `protobuf_read_delimited` `1026`–`1027`, chunks JSON) |
| 7-4-1 | SD over SPI | `components/storage/storage.c` | `mount_sd_card` `725`–`789` (`esp_vfs_fat_sdspi_mount` at `770`) |
| 7-1-1 / 7-1-2 | NV sessions, multi-session | `storage.c` | Session index scan `133`–`151`; `.pb` naming `open_new_data_file` `803`–`843` |
| 7-5-1 | SD init retry → error | `storage.c`, `app_main.c` | Mount retry loop `767`–`787`; boot `storage_init` retries `665`–`680` |
| 1-3-1 | IMU init retries | `main/app_main.c`, `components/imu_bno055/bno055.c` | Outer loop `IMU_MAX_RETRIES` `602`–`627`; chip-ID probe per I2C address `120`–`138` (`max_retries = 3`) |
| 1-1-2 | IMU on I2C | `main/app_main.c`, `bno055.c` | `bus_i2c_init` `594`–`600`; reads via `bno055_*` I2C helpers |
| 8-2-1 | LEDs reflect power / session / sync / error | `main/app_main.c` | `POWER_LED_GPIO` / `STATUS_LED_GPIO` / `ERROR_LED_GPIO` `60`–`62`; `error_led_set` `113`–`119`; sync blink + state `430`–`436` |
| 5-4-1 | Compact on-wire / on-card format | `components/storage/storage.c`, `components/protobuf/protobuf_utils.c` | `protobuf_write_delimited` call `918`–`919`; encoder `protobuf_write_delimited` `232`–`280` in `protobuf_utils.c` |

---

## Appendix B — Honest gaps / “don’t over-claim”

- **5-3-1 (≥1 KB/s sync)** — verified with Network tab timing, not a named constant in source.
- **5-2-3 (FCC parameters)** — “we use ESP-IDF defaults / menuconfig”; no custom TX boost in repo is the usual story—confirm against your `sdkconfig` before promising specifics.
- **Mechanical / pool safety (sections 2–3)** — enclosure + strap are physical; code only indirectly supports (logging, no exposed UI for conductivity).

---

## Appendix C — One end-to-end sentence per layer

1. **Wearable loop:** `app_main.c` reads BNO055 → `stroke_detector_feed` (stretch) → enrich sample → `storage_enqueue_bno_sample` → `.pb` on SD.
2. **Sync:** hold button → `transition_to_syncing` → ESP HTTP server exposes `/data.json` + APIs.
3. **Laptop:** `wifi_session_processor.py` saves SQLite sessions; merge uses `align_sessions_by_hop`.
4. **Browser:** loads JSON → `StrokeProcessor` / viz integration → charts + 3D + ideal / haptic overlays.
