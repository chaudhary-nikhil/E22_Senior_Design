/**
 * GoldenForm  --  shared mutable state (cross-module variables).
 * Load after gf_constants.js, before gf_app_nav_profile.js and other gf_*.js features.
 * Split layout: gf_wifi_* / gf_onboarding_journey, gf_session_transfer, gf_haptic_timeline,
 * gf_playback, gf_viz_* (charts/3D/side view), gf_ideal_device (ideal + device push + test buzz).
 */
let currentTab = 'home';
let userProfile = null;
let savedSessions = [];
let activeSessionIdx = -1;
let processedData = [];
let sessionMetrics = null;
let currentIndex = 0;
let isPlaying = false;
let playbackInterval = null;
let positionScale = 6.0;
/** Maps processor world Y to Three.js (−1 = pull reads as −Y / into water). Shared with viz integration. */
window.VIZ_WORLD_Y_SIGN = -1;
let idealStrokeData = null;
let activeWristDeviceRole = 'wrist_right';
let cachedDeviceListLength = 0;
/** Full rows from last /api/devices (device list and setup hints). */
let lastDevicesList = [];
let isDeviceOnline = false;
/** Debounced for UI/playbook  --  avoids Connected/Offline flicker when /device_info flaps. */
let navDisplayOnline = false;
let pendingConfigSync = false;
let pendingIdealSync = false;
let devicePollInterval = null;
let pollOfflineStreak = 0;
const POLL_OFFLINE_AFTER_FAILS = 3;
let lastConnStatusApplied = '';
let syncDeviceCount = 0;
let lastSyncedDeviceInfo = null;
const MAX_HAPTIC_TIMELINE_MARKERS = 400;
const MAX_STROKE_TIMELINE_MARKERS = 250;

let strokeBoundaries = []; // {index, strokeNum, streamKey, label?} per band + stroke for merged sessions
let hapticEvents = [];     // {index, deviation}
let accelChart, gyroChart, magChart;
let scene, camera, renderer, controls;
/** Single rAF loop for Three.js  --  cancel on WebGL context loss to avoid stacked loops / flicker. */
let vizRafId = null;
let imuCube = null;
/** Six face materials (+X,-X,+Y,-Y,+Z,-Z) for haptic flash tinting. */
let cubeMaterials = [];
/** Second wearable (merged timeline): nearest-time sample on the other stream. */
let imuCubeB = null;
let cubeMaterialsB = [];
let trailLine = null;
let trailLineB = null;
let hapticFlashUntil = 0;
/** Throttle live calibration strip updates during playback/scrub (was every frame → layout churn). */
let lastCalStripFrameIdx = -1;
let integratedPositions = [];
let playbackStrokeSegments = [];
/** Euler offsets (rad) applied after quaternion when a gravity axis hint exists. */
let vizCoordinateTransform = { rotationX: 0, rotationY: 0, rotationZ: 0 };
/** Quaternion applied so the first sample renders as a neutral "flat" pose. */
let vizBaseQuatInv = null;
/** Filled when an ideal baseline is loaded: per-stroke deviation vs ideal + aggregates for Analysis. */
let gfVsIdealMetrics = null;
/** Analysis chart: which stroke is compared to the saved ideal (null = auto first stroke). */
let idealCompareStrokeNum = null;
let idealCompareStreamKey = null;
/** Analysis ideal chart: always full-session |LIA| resampled vs baseline (stroke picker removed). */
let idealCompareMode = 'session';

const TRAIL_MAX_POINTS = 2000;
/* Gaussian window used by `resampleTrailFreestyle` before Catmull-Rom.
 * Small window preserves stroke S-curve; larger values convert strokes into ellipses. */
const TRAIL_SMOOTH_WINDOW = 5;
/** Base interval for playback (ms); combined with speed slider. */
const PLAYBACK_BASE_MS = 18;
/** 0 = full session; else play only this stroke #. */
let playbackStrokeFilter = 0;
let loopStrokePlayback = false;
/** 0.25-4; multiplies playback speed. */
let playbackSpeedMultiplier = 0.35;
/** When playing all strokes, loop entire session. */
let loopFullSession = false;
/** Smoothed LIA positions (matches StrokeProcessor world integration, lightly smoothed per stroke). */
let positionStreamPositions = [];
/** High-contrast phase colors (trail + cube tint)  --  distinct underwater vs recovery */
const PHASE_COLOR_HEX = {
    glide: 0x38bdf8, catch: 0x22c55e, pull: 0xef4444,
    recovery: 0xa855f7, idle: 0x64748b
};
const PHASE_COLOR_RGB = {
    glide: [0.22, 0.74, 0.97], catch: [0.13, 0.77, 0.37],
    pull: [0.94, 0.27, 0.27], recovery: [0.66, 0.33, 0.97],
    idle: [0.39, 0.45, 0.55]
};

/** Prefer firmware `strokes` when present so motion matches device logs. */
let useFwStrokesForViz = false;
/**
 * BoxGeometry Y half-extent (m) for createImuCube body — top face at +this from mesh origin.
 * Applied in viz so stroke origin (0,0,0) places the red top face on the water plane (y=0).
 */
const VIZ_IMU_BODY_HALF_H = 0.07;
let rawIntegratedPositions = [];
let velocityArrow = null;
let followDeviceInView = false;
const PHASE_COLOR_SIDE = {
    glide: '#38bdf8', catch: '#22c55e', pull: '#ef4444', recovery: '#a855f7', idle: '#64748b'
};
