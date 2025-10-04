# FormSync Firmware Template (ESP-IDF)

Modular layout: **bus → driver → fusion → app**, plus optional BLE, logging, and storage.

## Build
```bash
idf.py set-target esp32
idf.py build flash monitor
```

## Structure
- `components/bus_i2c`: thin I²C helpers
- `components/imu_mpu6050`: IMU-specific register driver
- `components/fusion`: placeholder Madgwick/Mahony API
- `components/ble`: stubs for BLE services (enable later)
- `components/storage`: stubs for SD/FATFS or SPIFFS/LittleFS
- `components/logging`: app-wide logging helpers
- `main`: application tasks and wiring

## Notes
- Keep `sdkconfig` out of git; use `sdkconfig.defaults` to share sane defaults.
- Use feature flags in `Kconfig.projbuild` to toggle BLE/Storage/Fusion choices.
- Prefer `vTaskDelayUntil` for fixed-rate sampling loops.
