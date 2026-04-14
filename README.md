# GoldenForm

ESP-IDF firmware and Python dashboard for **BNO055**-based swim stroke analysis: on-device logging, Wi‑Fi sync to the laptop/phone dashboard, 3D visualization, stroke metrics, and haptic coaching.

**PDP data path:** sessions are written to **SD** on the wearable and pulled by the dashboard over **Wi‑Fi** (`/process` / `data.json`). There is **no UART/serial pipeline** for production session transfer (USB is for `idf.py` flash/monitor only).

## Firmware

- **Target**: set in your tree (e.g. `esp32s3`); see `sdkconfig.defaults`.
- **Project name**: `GoldenForm` (`CMakeLists.txt` → `project(GoldenForm)`).
- **Main components**: `imu_bno055`, `stroke_detector`, `haptic`, `storage`, `wifi_server`, `protobuf`, `bus_i2c`, etc. (`main/CMakeLists.txt`).

```bash
source $HOME/esp/esp-idf/export.sh   # or your IDF env
cd /path/to/E22_Senior_Design
idf.py set-target <chip>
idf.py build flash monitor
```

## Dashboard (primary)

Serve the integrated app (handles `/process` sync from the wearable and static `integrated_session_viewer.html`):

```bash
cd dashboard
python3 wifi_session_processor.py
# Open the URL printed (default port 8844, or set `PORT` / `--port`)
```

Key files: `integrated_session_viewer.html`, `app.css`, `app.js`, `js/gf_*.js`, `wifi_session_processor.py` (uses `simple_imu_visualizer.StrokeProcessor` for session processing).

## Optional: USB session logger / legacy playback

`session_logger.py` + `session_visualizer.html` + `visualizer.js` remain for JSON sessions captured over serial; use the integrated dashboard above for the full GoldenForm flow (profile, devices, merge, ideal stroke).

## Data shape

Logged / synced samples include IMU, quaternion, calibration, stroke counts, and haptic fields as produced by the firmware and `StrokeProcessor` (see `dashboard/wifi_session_processor.py` and `dashboard/simple_imu_visualizer.py`).

## Requirements

- ESP-IDF (version aligned with this repo’s `esp-idf` or your install)
- Python 3 for the dashboard
- BNO055 + ESP32 as wired in project configuration
