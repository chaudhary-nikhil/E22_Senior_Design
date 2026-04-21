================================================================================
§4.1 BASE — PRODUCT LEVEL (top-level)
================================================================================
Req	TIDR	File	Lines	What to look for
1	n	main/app_main.c; dashboard/	887–994; simple_imu_visualizer.py	IMU sample loop + enqueue to SD; StrokeProcessor / viz after sync
2	n	—	—	Comfort / strap / industrial design — not application source
3	n	—	—	Sealed enclosure / pool safety — not application source
4	n	—	—	Battery life / supply — hardware + usage; see Power subsystem
5	n	main/app_main.c; components/wifi_server/	371–444; 1691+	transition_to_syncing; Wi‑Fi + SD → HTTP export
6	n	dashboard/	—	HTML/JS viewer + Python processor (see §6-1 system + component web)
7	n	components/storage/	821–835; 936–937	open_new_data_file; protobuf flush path
8	n	main/app_main.c	79, 141–152; 483–495; 64–69, 323–368, 437–441	Debounce + handle_button_press; LED GPIO + logging/sync patterns

================================================================================
§4.1 BASE — SYSTEM — MECHANICAL SUBSYSTEM
================================================================================
Req	TIDR	File	Lines	What to look for
2-1	n	—	—	Adjustable wearable — CAD / strap hardware
2-2	n	—	—	Form factor — mechanical
3-1	n	—	—	No exposed conductors in water — enclosure BOM
3-2	n	—	—	Thermal — enclosure / battery placement

================================================================================
§4.1 BASE — SYSTEM — STORAGE SUBSYSTEM
================================================================================
Req	TIDR	File	Lines	What to look for
7-1	n	components/storage/storage.c	898–907; 821–835	check_sd_space; session files; see also Comp 7-1-1
7-2	n	components/wifi_server/wifi_server.c	160–220, 217	build_pb_file_list + qsort — transfer read order
7-4	n	components/protobuf/protobuf_utils.c; storage.c	231–278; 936–937	Length-delimited protobuf on card + wire

================================================================================
§4.1 BASE — SYSTEM — SOFTWARE (ON-BOARD)
================================================================================
Req	TIDR	File	Lines	What to look for
1-2, 7-3	n	main/app_main.c	805–806, 887–994, 1023	SAMPLE_HZ period; read BNO055 then storage_enqueue; vTaskDelayUntil
7-5	n	components/storage/storage.c; main/app_main.c	785–807; 673–696	mount_sd_card retries; boot storage_init retries + error LED
1-3	n	main/app_main.c	610–634	bno055_init retry loop (maps to Comp 1-3-1 TIDR)
5-2	n	main/app_main.c; wifi_server.c	371–444; 1691–1766	AP/sync path; wifi_server_start_sync
5-3	n	components/wifi_server/wifi_server.c	160–220, 941+	Sorted file list; chunked data_json export
5-4	n	components/protobuf/protobuf_utils.c	231–278	Protobuf batching (maps to Comp 5-4-1 — not TIDR)
8-2	n	main/app_main.c	64–69, 323–368, 437–441	LED behavior (maps to Comp 8-2-1 TIDR)

================================================================================
§4.1 BASE — SYSTEM — SOFTWARE (OFF-BOARD)
================================================================================
Req	TIDR	File	Lines	What to look for
6-1	n	dashboard/simple_imu_visualizer.py; integrated_session_viewer.html	808–822; (viewer)	vis_data metrics; plots UI
5-1, 6-2	n	dashboard/wifi_session_processor.py; dashboard/js/gf_api.js	274+; gf_api fetch	/api routes; POST/GET to laptop or device proxy
6-3	n	dashboard/wifi_session_processor.py	819–825	_get_sessions SQLite history
8-3	n	main/app_main.c	483–495	handle_button_press — session start/stop (PDP lists under hardware; firmware here)
1-1	n	main/app_main.c; components/imu_bno055/	Kconfig SAMPLE_HZ; 310–344	9-axis in bno055_read_sample; rate from menuconfig
8-1	n	main/app_main.c	79, 141–152	Firmware debounce (PDP wording is “circuitry”; PCB may add RC too)

================================================================================
§4.1 BASE — SYSTEM — POWER SUBSYSTEM
================================================================================
Req	TIDR	File	Lines	What to look for
4-1	n	—	—	Li-Po — BOM / charger
4-2	n	—	—	Protection IC — BOM
4-3	n	—	—	DC-DC 3V3 — BOM (see Comp 4-3-1 TIDR for voltage window)
4-4	n	—	—	Noise protection — BOM / layout

================================================================================
§4.1 BASE — COMPONENT — BATTERY / UVLO (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
4-1-1	n	—	—	1200 mAh / 3.7 V — BOM
4-1-2	n	—	—	Current range — BOM
4-2-1	n	—	—	UVLO threshold — PMIC / supervisor hardware

================================================================================
§4.1 BASE — COMPONENT — BUCK-BOOST (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
4-3-1	y	(hardware)	—	3.1–3.5 V across discharge — schematic; firmware assumes stable 3V3

================================================================================
§4.1 BASE — COMPONENT — WEB APP (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
6-1-1	y	dashboard/simple_imu_visualizer.py	808–822	vis_data stroke_count, phases, entry angle, etc.
6-1-2	y	dashboard/js/gf_playback.js	5–46	getPlaybackBounds, togglePlayback
6-1-3	y	dashboard/js/gf_viz_integration.js	454–527	applyPerStrokeOriginOffset
6-1-4	n	dashboard/js/gf_calibration.js	330–345	updateCalibrationDisplay
6-2-1	n	dashboard/js/gf_wifi_banners.js	117–120	setConnStatus('connected')
6-2-2	y	components/wifi_server/wifi_server.c	941+	data_json_handler HTTP from band
6-3-1	y	dashboard/wifi_session_processor.py	819–825	_get_sessions

================================================================================
§4.1 BASE — COMPONENT — MCU (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
5-2-1	y	main/app_main.c	371–444	transition_to_syncing — AP on sync hold
5-2-2	y	components/wifi_server/wifi_server.c	1691–1766	wifi_server_start_sync — SD → stream
5-2-3	n	components/wifi_server/wifi_server.c	438–443	esp_wifi_set_max_tx_power + sdkconfig / module cert story
5-3-1	n	—	—	≥1 KB/s — measure; not a constant in repo
5-4-1	n	components/protobuf/protobuf_utils.c	231–278	protobuf_write_delimited
8-2-1	y	main/app_main.c	64–69, 323–368, 437–441	Status/power/error GPIO; logging vs sync LED patterns

================================================================================
§4.1 BASE — COMPONENT — SD CARD (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
7-4-1	y	components/storage/storage.c	743–788	esp_vfs_fat_sdspi_mount — SPI SD
7-1-1	y	components/storage/storage.c	821–835	Session .pb on FAT
7-1-2	y	components/storage/storage.c	139–156	Multi-session index scan without requiring sync
7-5-1	y	components/storage/storage.c; main/app_main.c	785–807; 673–696	Mount retries → error; boot storage_init retries

================================================================================
§4.1 BASE — COMPONENT — IMU (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
1-3-1	y	main/app_main.c	610–634	Five bno055_init attempts; error LED
1-1-1	n	components/imu_bno055/bno055.c	310–344	9-axis reads in bno055_read_sample
1-1-2	y	components/imu_bno055/bno055.c	290–360	I2C read path in bno055_read_sample
1-1-3	n	main/app_main.c	891–936	CAL status log + NVS save at full cal

================================================================================
§4.2 STRETCH — PRODUCT LEVEL
================================================================================
Req	TIDR	File	Lines	What to look for
1	n	components/stroke_detector/; components/haptic/	319–416; 238–260	DTW + haptic patterns
2	n	dashboard/js/gf_ideal_comparison.js; gf_coaching.js	225–298; 21–63	Vs-ideal + coaching analytics
3	n	database.py; wifi_session_processor.py; simple_imu_visualizer.py	80–88; 477+; 877+	Multi-device DB; register; align_sessions_by_hop

================================================================================
§4.2 STRETCH — SYSTEM — SOFTWARE (ON-BOARD)
================================================================================
Req	TIDR	File	Lines	What to look for
1-1	n	components/stroke_detector/stroke_detector.c	319–416	stroke_detected + deviation + haptic (ties to Comp Haptic 1-1-x)

================================================================================
§4.2 STRETCH — SYSTEM — SOFTWARE (OFF-BOARD)
================================================================================
Req	TIDR	File	Lines	What to look for
2-1	n	dashboard/js/gf_ideal_comparison.js; gf_coaching.js	225–298; 21–63	Session vs ideal insights
3-1	n	dashboard/wifi_session_processor.py; database.py	477–493; 80–88	Multi-device registration
3-2	n	dashboard/wifi_session_processor.py	849–884	_merge_sessions combined viz data path
3-3	n	dashboard/simple_imu_visualizer.py; wifi_session_processor.py	877+; 883–887	align_sessions_by_hop; called from merge

================================================================================
§4.2 STRETCH — SYSTEM — HARDWARE
================================================================================
Req	TIDR	File	Lines	What to look for
1-2	n	components/haptic/haptic.c	1–20, 155–195	ERM drive — GPIO + haptic_init

================================================================================
§4.2 STRETCH — COMPONENT — HAPTIC FEEDBACK (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
1-1-1	y	components/stroke_detector/stroke_detector.c	381–416	DTW + haptic_play_pattern thresholds
1-1-2	n	components/stroke_detector/stroke_detector.c	319–373	Stroke start + integration window
1-1-3	n	components/wifi_server/wifi_server.c; main/app_main.c	1192–1295; 724–741	ideal_stroke POST + boot load ideal_stroke.bin
1-2-1	n	components/haptic/haptic.c	238–260	haptic_play_pattern; 155–195 haptic_init

================================================================================
§4.2 STRETCH — COMPONENT — ADVANCED INSIGHTS (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
2-1-1	y	dashboard/js/gf_ideal_comparison.js	225–298	gfVsIdealMetrics
2-1-2	n	dashboard/js/gf_haptic_timeline.js	18–46	haptic_fired markers on timeline

================================================================================
§4.2 STRETCH — COMPONENT — MULTI-SENSOR (low-level)
================================================================================
Req	TIDR	File	Lines	What to look for
3-1-1	y	dashboard/database.py; wifi_session_processor.py	80–88; 477–493	devices table + _register_device
3-2-1	y	dashboard/wifi_session_processor.py	849–884	_merge_sessions
3-3-1	n	dashboard/simple_imu_visualizer.py; wifi_session_processor.py	877+; 883–887	align_sessions_by_hop + merge call site

================================================================================
LIMITS
================================================================================
• Mechanical, pool safety, battery/UVLO, buck-boost realization: hardware — not fully represented in C/JS/Python.
• 5-3-1 throughput: verify by measurement.
• 5-2-3 FCC: cite module certification + IDF sdkconfig; firmware shows TX power cap only.
• Line numbers drift: open File and search the symbol in “What to look for”.

HOW TO PASTE INTO GOOGLE DOCS
Copy one block (from ===== through the rows). Paste into Google Sheets to split on tabs, or paste into Docs and convert text to table (separator = Tab).
