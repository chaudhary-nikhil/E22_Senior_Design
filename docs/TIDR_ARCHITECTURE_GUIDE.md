# TIDR Architecture Guide: How Trade-Off Informed Design Requirements Work

## Overview

**TIDRs (Trade-Off Informed Design Requirements)** are 19 critical requirements where GoldenForm made deliberate architectural choices. Each TIDR represents a constraint where the design trades off one factor (e.g., latency, memory, cost, complexity) against another (e.g., reliability, power efficiency, user experience).

The 19 TIDRs span **on-board firmware**, **wireless communication**, **storage**, and **web app visualization**. They interconnect through five key data flows:

1. **IMU → SD (Logging):** Acquire hand position; enforce timing & retry logic
2. **SD → MCU → Web (Sync):** Transfer logged data; compress; track status
3. **Web App Visualization:** Display metrics, playback, drift correction
4. **Device Initialization:** Retry & error handling for IMU, SD, Wi-Fi
5. **Stretch Goal Flows:** Haptic feedback, multi-device sync, coaching

---

## TIDR Inventory (19 Total)

### **Hardware / Power TIDRs (1)**
- **4-3-1**: Buck-Boost ensures 3.1–3.5V across battery discharge  
  *Trade-off: Voltage regulation precision vs cost & PCB complexity*

### **On-Board Firmware TIDRs (8)**

#### Initialization & Retry (3)
- **1-3-1**: IMU retries up to 5 times before error state  
  *Trade-off: Robustness vs startup latency*
- **1-1-2**: IMU communicates via I2C (vs SPI)  
  *Trade-off: Power efficiency & PCB routing vs bandwidth*
- **7-5-1**: SD Card retries at least once before error state  
  *Trade-off: Robustness vs boot time*

#### Wireless & Data Transfer (3)
- **5-2-1**: MCU initiates Wi-Fi network on sync button hold  
  *Trade-off: User-explicit sync (battery) vs always-on convenience*
- **5-2-2**: MCU orchestrates SD → HTTP transfer  
  *Trade-off: MCU complexity vs simplicity of device isolation*
- **8-2-1**: LED indicators show power, session, sync, error states  
  *Trade-off: User feedback granularity vs GPIO count*

#### Storage (2)
- **7-4-1**: SPI (not QSPI or parallel) for SD communication  
  *Trade-off: Simpler PCB & driver vs bandwidth*
- **7-1-1**: SD Card holds non-volatile session data  
  *Trade-off: Offline reliability vs cloud-only simplicity*
- **7-1-2**: Multi-session support without requiring wireless sync  
  *Trade-off: Local storage capacity vs streaming assumptions*

### **Wireless Data Format TIDR (1)**
- **6-2-2**: Web app receives data via Wi-Fi HTTP (not proprietary/BLE)  
  *Trade-off: Standard protocol/browser-native vs latency*

### **Web App Visualization TIDRs (6)**

#### Session Display & Analysis (3)
- **6-1-1**: Display stroke metrics (count, rate, phases, angles)  
  *Trade-off: Analytics granularity vs UI complexity*
- **6-1-2**: Replay window with playback controls & motion visualization  
  *Trade-off: User insight vs rendering overhead*
- **6-1-3**: Prevent long-term drift in stroke position calculation  
  *Trade-off: Calibration vs assumption of stationary reference frame*

#### Session Management (1)
- **6-3-1**: Interactive tool to view ≥1 previous session  
  *Trade-off: History access vs storage/UX simplicity*

### **Stretch Goal TIDRs (3)**

#### Haptic Feedback (1)
- **1-1-1** (Haptic): Trigger haptic motor when stroke deviates beyond threshold  
  *Trade-off: Real-time feedback latency vs alert fatigue*

#### Advanced Insights (1)
- **2-1-1**: Present session vs ideal form comparison  
  *Trade-off: Coaching depth vs computational load*

#### Multi-Device Support (1)
- **3-1-1**: Register & configure multiple devices with body placement  
  *Trade-off: Multi-sensor accuracy vs device coordination complexity*
- **3-2-1**: Merged visualization from multiple devices  
  *Trade-off: Holistic motion capture vs data alignment burden*

---

## Data Flows: How TIDRs Interconnect

### **Flow 1: Logging (IMU → SD)**

```
[IMU Init]
  ↓
  ├─→ TIDR 1-3-1: Retry ×5; if fail → error LED (TIDR 8-2-1)
  ↓
[Main Loop: TIDR 1-2/7-3 timing]
  ├─→ Read BNO055 via TIDR 1-1-2 (I2C)
  ├─→ Enqueue sample to SD writer task
  ├─→ Protobuf encode (TIDR 7-4-1 prep)
  ↓
[SD Mount & Write]
  ├─→ TIDR 7-5-1: Mount retries; if fail → error LED
  ├─→ Session .pb file on FAT (TIDR 7-1-1)
  ├─→ Support multiple sessions (TIDR 7-1-2)
  ↓
[Status via LED: TIDR 8-2-1]
  └─→ Blink pattern indicates "recording"
```

**Key Trade-off**: IMU retry count (5 attempts) vs SD retry count (1 attempt) reflects device priority: IMU failure is fatal to the wearable; SD failure can be detected and reported.

---

### **Flow 2: Wireless Sync (SD → MCU → Web)**

```
[User Action: Hold Sync Button]
  ↓
  └─→ TIDR 5-2-1: Initiate Wi-Fi AP mode
        Trade-off: On-demand sync saves power; delays data availability
  ↓
[MCU Orchestration: TIDR 5-2-2]
  ├─→ Read session list from SD (TIDR 7-1-1/7-1-2)
  ├─→ Sorted by timestamp (for replay integrity)
  ├─→ Stream each .pb file via HTTP handler (TIDR 6-2-2)
  ├─→ Chunked + protobuf-delimited (TIDR 7-4-1 format)
  ↓
[Web App: TIDR 6-2-2 HTTP Handler]
  ├─→ Receive /data_json endpoint
  ├─→ Parse protobuf stream
  ├─→ Store in SQLite (on-laptop DB)
  ↓
[LED Status: TIDR 8-2-1]
  └─→ Blink pattern indicates "syncing"; solid when done
```

**Key Trade-off**: Explicit sync button (TIDR 5-2-1) vs always-on Wi-Fi costs battery & complexity but gives user control.

---

### **Flow 3: Visualization & Metrics (Web App)**

```
[Parse Session Data]
  ↓
  ├─→ TIDR 6-1-1: Compute stroke metrics
  │     ├─ Stroke count (from protobuf phase field)
  │     ├─ Rate (strokes/minute)
  │     ├─ Entry angle, hand depth, phases
  │     └─ Trade-off: Metrics completeness vs calculation overhead
  ↓
[Playback: TIDR 6-1-2]
  ├─→ Renderer spans full session timeline
  ├─→ User can play, pause, scrub
  ├─→ 3D hand motion rendered per frame
  ├─→ Trade-off: Smooth animation vs memory for frame cache
  ↓
[Drift Correction: TIDR 6-1-3]
  ├─→ Per-stroke reference frame reset
  │     (origin at start of each stroke)
  ├─→ Prevents long-term magnetometer drift accumulation
  ├─→ Uses shared phase inference (stroke_phase from StrokeProcessor)
  ├─→ Trade-off: Calibration assumption vs raw IMU trust
  │     (assumes swimmer begins each stroke from stable reference)
  ↓
[History: TIDR 6-3-1]
  ├─→ SQLite session list (timestamp, metadata)
  ├─→ Load previous session on demand
  ├─→ Trade-off: Interactive access vs storage bloat
```

**Critical Link**: TIDR 6-1-3 (drift prevention) depends on accurate stroke boundaries. These come from on-board stroke detection, which ties to Stretch Goal 1-1-1 (haptic trigger timing). **Both must agree on stroke phases** to prevent misalignment.

---

### **Flow 4: Device Initialization & Error Handling**

```
[Boot Sequence]
  ↓
  ├─→ TIDR 1-3-1: IMU retry ×5
  │     └─ If fail: set error LED (TIDR 8-2-1) + halt logging
  ├─→ TIDR 7-5-1: SD mount retry ×5
  │     └─ If fail: error LED + fallback to RAM buffer (if enabled)
  ├─→ TIDR 1-1-2: I2C bus check (IMU presence)
  ├─→ TIDR 7-4-1: SPI bus check (SD presence)
  ↓
[UI Feedback: TIDR 8-2-1]
  ├─→ LED sequence tells user boot status
  │     ├─ Amber: Initializing
  │     ├─ Green: Ready
  │     ├─ Red: Fatal error (IMU or SD failed)
  │     └─ Trade-off: Visual feedback granularity vs GPIO cost
  ↓
[Calibration Status: 6-1-4 (non-TIDR)]
  └─→ Show to user on web app after first sync
```

**Cascade Logic**: If **TIDR 1-3-1** (IMU retry) fails → error LED (TIDR 8-2-1) → user cannot start session. If **TIDR 7-5-1** (SD retry) fails → error → no data logging. Both are critical paths with no bypass.

---

### **Flow 5: Stretch Goals (Haptic + Advanced Insights + Multi-Device)**

#### **Stretch 1-1-1: Haptic on Deviation**
```
[On-Board Stroke Detection]
  ├─→ Compute DTW (dynamic time warping) vs ideal stroke reference
  ├─→ If deviation > threshold: TIDR 1-1-1 trigger haptic
  ├─→ MCU must detect stroke start/end (TIDR 1-1-1 prerequisite)
  └─→ Trade-off: Latency (must compute per-frame) vs accuracy
```

**Link to Base Goals**: Haptic timing **must match** stroke detection fed to visualization (TIDR 6-1-1 metrics). If haptic triggers at sample N but web app thinks stroke starts at sample N+10, user will see misalignment in replay (TIDR 6-1-2).

#### **Stretch 2-1-1: Advanced Insights (Vs Ideal)**
```
[Post-Session Analysis]
  ├─→ Compare session strokes to ideal reference via gfVsIdealMetrics
  ├─→ Present coaching feedback
  └─→ Trade-off: Offline analysis time vs real-time guidance
```

#### **Stretch 3-1-1 & 3-2-1: Multi-Device**
```
[Register Devices]
  ├─→ Store body placement (wrist, ankle, shoulder) in DB
  ├─→ Link sessions to device ID
  ↓
[Merge Sessions: 3-2-1]
  ├─→ Call align_sessions_by_hop (matches timestamps across devices)
  ├─→ Render merged 3D motion
  └─→ Trade-off: Multi-body kinematics complexity vs data handling
```

---

## Architecture Constraints Imposed by TIDRs

### 1. **Timing Coherence (TIDRs 1-2/7-3, 6-1-3, 1-1-1 Haptic)**
- IMU sample rate (SAMPLE_HZ in Kconfig) → SD write period → Web replay frame rate must align
- Drift correction (TIDR 6-1-3) requires stroke boundaries match haptic detection (TIDR 1-1-1)
- **Verification**: Check `components/imu_bno055/Kconfig` SAMPLE_HZ value, confirm consistent in both firmware and web processing

### 2. **SPI/I2C Protocol Stack (TIDRs 7-4-1, 1-1-2)**
- **7-4-1**: SPI-only for SD (simpler vs QSPI) → limits bandwidth
- **1-1-2**: I2C-only for IMU (lower power vs SPI) → possible bus contention if shared
- **Trade-off Impact**: No high-speed concurrent access; firmware must serialize I2C/SPI transactions
- **Verification**: Check `components/imu_bno055/bno055.c` and `components/storage/storage.c` for bus lock patterns

### 3. **On-Demand Wi-Fi (TIDR 5-2-1) vs Always-On**
- Button-hold triggers AP mode → user must explicitly sync
- **Trade-off**: Battery life vs automatic cloud backup
- **Implication**: User data at risk if device battery dies before manual sync
- **Verification**: Look for `transition_to_syncing` in `main/app_main.c` line 371–444; confirm no background sync task

### 4. **Retry Asymmetry (TIDRs 1-3-1 vs 7-5-1)**
- IMU: 5 retries (critical sensor)
- SD: 1 retry (can warn user if failed)
- **Trade-off**: IMU failure → no app startup; SD failure → still can use device but no logging
- **Implication**: Design prioritizes real-time motion capture over offline storage reliability

### 5. **HTTP + Protobuf (TIDR 6-2-2 + format TIDRs)**
- Uses standard HTTP (not BLE or proprietary)
- Protobuf encoding (TIDR 7-4-1 format) adds compression vs raw binary
- **Trade-off**: Standard protocol (easy web integration) vs smaller payload (protobuf overhead)
- **Verification**: Check `components/wifi_server/wifi_server.c` line 941+ for `/data_json` endpoint

### 6. **Session History in Local SQLite (TIDR 6-3-1)**
- All history stored on user's laptop
- **Trade-off**: User privacy vs cloud analytics
- **Implication**: If laptop disk fails, all session history lost (no backup)
- **Verification**: Look for `_get_sessions` in `dashboard/wifi_session_processor.py` line 819–825

---

## Code Map Accuracy Verification

I've reviewed the current [GOLDENFORM_PDP_CODE_MAP.md](GOLDENFORM_PDP_CODE_MAP.md). Here's the status:

### ✅ **Accurate Mappings**

| TIDR | Mapped File | Lines | Status |
|------|-----------|-------|--------|
| 4-3-1 | (hardware) | — | ✓ Correct: hardware constraint |
| 1-3-1 | main/app_main.c | 610–634 | ✓ Verified: 5× retry loop in bno055_init |
| 1-1-2 | components/imu_bno055/bno055.c | 290–360 | ✓ Verified: I2C path in bno055_read_sample |
| 7-5-1 | components/storage/storage.c | 785–807 | ✓ Verified: mount_sd_card retry loop with SPI re-init |
| 7-4-1 | components/storage/storage.c | 743–788 | ✓ Verified: esp_vfs_fat_sdspi_mount (SPI config) |
| 7-1-1 | components/storage/storage.c | 821–835 | ✓ Verified: Session .pb storage logic |
| 7-1-2 | components/storage/storage.c | 139–156 | ✓ Verified: Multi-session index scan |
| 5-2-1 | main/app_main.c | 371–444 | ✓ Verified: transition_to_syncing AP init |
| 5-2-2 | components/wifi_server/wifi_server.c | 1691–1766 | ✓ Verified: wifi_server_start_sync orchestration |
| 6-2-2 | components/wifi_server/wifi_server.c | 941+ | ✓ Verified: data_json_handler HTTP endpoint |
| 8-2-1 | main/app_main.c | 64–69, 323–368, 437–441 | ✓ Verified: LED GPIO patterns for states |
| 6-1-1 | dashboard/simple_imu_visualizer.py | 808–822 | ✓ Verified: vis_data metrics computation |
| 6-1-2 | dashboard/js/gf_playback.js | 5–46 | ✓ Verified: getPlaybackBounds, togglePlayback |
| 6-1-3 | dashboard/js/gf_viz_integration.js | 454–527 | ✓ Verified: applyPerStrokeOriginOffset drift correction |
| 6-3-1 | dashboard/wifi_session_processor.py | 819–825 | ✓ Verified: _get_sessions SQLite query |
| 1-1-1 (Haptic) | components/stroke_detector/stroke_detector.c | 381–416 | ✓ Verified: DTW + haptic_play_pattern |
| 2-1-1 (Insights) | dashboard/js/gf_ideal_comparison.js | 225–298 | ✓ Verified: gfVsIdealMetrics function |
| 3-1-1 (Multi-device) | dashboard/database.py | 80–88 | ✓ Verified: devices table + registration |
| 3-2-1 (Multi-merge) | dashboard/wifi_session_processor.py | 849–884 | ✓ Verified: _merge_sessions combined path |

### ⚠️ **Minor Notes (Not Errors)**

- **5-3-1** (≥1 KB/s throughput): Not a constant in repo; requires runtime measurement. Code map correctly marks "—".
- **5-2-3** (FCC compliance): Code shows only TX power cap in sdkconfig. Compliance is module-level; firmware doesn't enforce all constraints.
- **Line number drift**: Some line ranges may shift after recent edits. Use symbol search (e.g., "transition_to_syncing") if jumping to code.

---

## Summary: How All 19 TIDRs Work Together

| Phase | TIDRs Involved | Flow | Critical Constraint |
|-------|----------------|------|---------------------|
| **Initialization** | 1-3-1, 1-1-2, 7-5-1, 8-2-1 | Boot retries → LED status | Both IMU & SD must init; cascade failure via LED |
| **Logging** | 1-2/7-3, 1-1-2, 7-4-1, 7-1-1, 7-1-2, 8-2-1 | IMU→SD with timing & retries | Timing coherence (sample rate) must match web playback rate |
| **Sync** | 5-2-1, 5-2-2, 7-1-1, 7-1-2, 6-2-2, 8-2-1 | Button hold → AP → HTTP stream | On-demand sync prioritizes battery; user must remember to press button |
| **Visualization** | 6-1-1, 6-1-2, 6-1-3, 6-2-2, 6-3-1 | Parse protobuf → display metrics, replay, history | Drift correction (TIDR 6-1-3) requires accurate stroke boundaries from firmware |
| **Haptic (Stretch)** | 1-1-1, 6-1-1, 6-1-2, 2-1-1 | Detect deviation → trigger feedback → show in replay | Haptic timing must match stroke detection fed to visualization |
| **Multi-Device (Stretch)** | 3-1-1, 3-2-1, 6-1-3 | Register devices → merge sessions → align timestamps | Multi-body kinematics layer on top of single-device drift correction |

---

## Recommendations for Future TIDR Updates

1. **Document measured 5-3-1 throughput** once wireless testing complete.
2. **Verify timing coherence** between IMU sample rate and web frame rate; document expected latency budget.
3. **Test haptic latency** (Stretch 1-1-1) against web replay to confirm alignment within <50ms.
4. **Add multi-device sync test** (Stretch 3-3-1) to validate timestamp alignment across wearables.
5. **Audit LED patterns** (TIDR 8-2-1) for consistency across boot, logging, sync, and error states.

---

## Document Version
**Date**: April 21, 2026  
**Status**: ✓ Code map verified; all 19 TIDRs mapped and linked to source  
**Last Verification**: Line numbers as of main/app_main.c, wifi_server.c, storage.c, and dashboard/ as of latest commit
