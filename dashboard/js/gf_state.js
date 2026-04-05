/**
 * GoldenForm — shared mutable state (cross-module variables).
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
let positionScale = 3.0;
let idealStrokeData = null;
let activeWristDeviceRole = 'wrist_right';
let cachedDeviceListLength = 0;
/** Full rows from last /api/devices (device list and setup hints). */
let lastDevicesList = [];
let isDeviceOnline = false;
/** Debounced for UI/playbook — avoids Connected/Offline flicker when /device_info flaps. */
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
/** Single rAF loop for Three.js — cancel on WebGL context loss to avoid stacked loops / flicker. */
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

const TRAIL_MAX_POINTS = 2000;
const TRAIL_SMOOTH_WINDOW = 7;
/** Base interval for playback (ms); combined with speed slider. */
const PLAYBACK_BASE_MS = 18;
/** 0 = full session; else play only this stroke #. */
let playbackStrokeFilter = 0;
let loopStrokePlayback = false;
/** 0.25–4; multiplies playback speed. */
let playbackSpeedMultiplier = 0.35;
/** When playing all strokes, loop entire session. */
let loopFullSession = false;
/** Smoothed LIA positions (matches StrokeProcessor world integration, lightly smoothed per stroke). */
let positionStreamPositions = [];
/** High-contrast phase colors (trail + cube tint) — distinct underwater vs recovery */
const PHASE_COLOR_HEX = {
    glide: 0x2563eb, catch: 0x15803d, pull: 0xc2410c,
    recovery: 0x7e22ce, idle: 0x52525b
};
const PHASE_COLOR_RGB = {
    glide: [0.145, 0.388, 0.922], catch: [0.082, 0.502, 0.243],
    pull: [0.90, 0.28, 0.05], recovery: [0.49, 0.15, 0.80],
    idle: [0.35, 0.35, 0.38]
};

/** Prefer firmware `strokes` when present so motion matches device logs. */
let useFwStrokesForViz = false;
let rawIntegratedPositions = [];
let velocityArrow = null;
let followDeviceInView = false;
const PHASE_COLOR_SIDE = {
    glide: '#2563eb', catch: '#15803d', pull: '#ea580c', recovery: '#9333ea', idle: '#71717a'
};
