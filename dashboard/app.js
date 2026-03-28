// GoldenForm App — Core Logic v2
// Visualization overhaul: stroke-phase position, haptic markers, ideal overlay,
// multi-device, calibration display, production polish
//
// PDP flow (single spine: Home checklist → tab CTAs):
// Base:   Profile → register wearables (Wi‑Fi) → Session sync handshake (0 sessions OK) →
//         swim + record → sync sessions → Session replay / Analysis / Insights.
// Stretch: Multi-wearable (N>1) one-at-a-time Wi‑Fi + Merge | Ideal stroke + haptic tuning |
//          calibration readout | progress charts — all reachable from Session / Settings / Insights.

const API = '';
/** Cached ideal when server unreachable (mirrors server payload shape). */
const LS_IDEAL_KEY = 'goldenform_ideal_cache';
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
/** True if user registered a head-mounted device in Settings (SQLite). */
let registeredHeadDevice = false;
let activeWristDeviceRole = 'wrist_right';

// ── TAB NAVIGATION ──
function switchTab(tab) {
    if (tab === currentTab) return;
    
    document.querySelectorAll('.page').forEach(p => {
        p.classList.remove('active');
        p.style.display = 'none';
        p.style.opacity = '0';
    });
    document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
    
    const page = document.getElementById('page-' + tab);
    const btn = document.getElementById('tab-' + tab);
    
    if (page) {
        page.style.display = 'block';
        requestAnimationFrame(() => {
            page.classList.add('active');
            page.style.opacity = '1';
            page.style.animation = 'slideUp 0.4s cubic-bezier(0.2, 0.8, 0.2, 1) forwards';
        });
    }
    if (btn) btn.classList.add('active');
    
    currentTab = tab;
    
    // Performance: Only load data when entering the tab
    if (tab === 'insights') {
        loadProgress();
        updateCoachingInsights();
    }
    if (tab === 'home') renderSetupJourney();
    if (tab === 'settings') {
        loadDevices();
        refreshWearableConnectionBanner();
    }
    if (tab === 'analysis' && sessionMetrics) updateAnalysis();
    if (tab === 'session') {
        updateSyncPlaybookMulti();
        updateSyncPlaybookConnectionState();
        const c = document.getElementById('canvas3d');
        if (c && renderer) {
            const w = Math.max(c.clientWidth || 800, 400);
            const h = Math.max(c.clientHeight || 450, 400);
            renderer.setSize(w, h);
            camera.aspect = w / h;
            camera.updateProjectionMatrix();
        }
    }
}

// ── API HELPERS ──
async function apiGet(path) {
    try {
        const r = await fetch(API + path);
        const text = await r.text();
        let data = null;
        try { data = text ? JSON.parse(text) : null; } catch { data = null; }
        if (!r.ok) return data && typeof data === 'object' ? { _httpError: r.status, ...data } : null;
        return data;
    } catch (e) {
        return null;
    }
}
async function apiPost(path, body) {
    try {
        const r = await fetch(API + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        const text = await r.text();
        let data = {};
        try { data = text ? JSON.parse(text) : {}; } catch { data = { error: text || 'Invalid JSON from server' }; }
        if (!r.ok) {
            return {
                status: 'error',
                error: data.error || data.message || ('HTTP ' + r.status),
                httpStatus: r.status
            };
        }
        return data;
    } catch (e) {
        const offline = typeof navigator !== 'undefined' && navigator.onLine === false;
        return { status: 'error', error: offline ? 'Offline — connect to the GoldenForm WiFi / dashboard' : (e.message || 'Failed to fetch') };
    }
}

// ── TOAST NOTIFICATIONS ──
function showToast(msg, type = 'info') {
    const t = document.createElement('div');
    t.className = 'toast toast-' + type;
    t.textContent = msg;
    document.body.appendChild(t);
    requestAnimationFrame(() => t.classList.add('show'));
    setTimeout(() => { t.classList.remove('show'); setTimeout(() => t.remove(), 300); }, 3000);
}

// ── USER PROFILE ──
async function loadUserProfile() {
    userProfile = await apiGet('/api/user');
    const modal = document.getElementById('reg-modal');
    if (userProfile && userProfile.name) {
        updateProfileUI();
        if (modal) modal.classList.remove('show');
    } else {
        if (modal) modal.classList.add('show');
    }
    renderSetupJourney();
}
function updateProfileUI() {
    if (!userProfile) return;
    setText('home-user-name', userProfile.name || 'Swimmer');
    const sk = document.getElementById('home-skill');
    if (sk) sk.textContent = (userProfile.skill_level || 'beginner').charAt(0).toUpperCase() + (userProfile.skill_level || 'beginner').slice(1);
    ['settings-name', 'reg-name'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.name || ''; });
    ['settings-height', 'reg-height'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.height_cm || ''; });
    ['settings-wingspan', 'reg-wingspan'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.wingspan_cm || ''; });
    ['settings-skill', 'reg-skill'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.skill_level || 'beginner'; });
    ['settings-pool-length', 'reg-pool-length'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.pool_length || '25'; });
    const wc = document.getElementById('settings-wearable-count');
    if (wc) wc.value = String(getExpectedWearableCount());
}
async function registerUser(e) {
    if (e) e.preventDefault();
    
    // Determine which form triggered this (modal vs settings page)
    const regModal = document.getElementById('reg-modal');
    const isModal = regModal ? regModal.classList.contains('show') : false;
    const prefix = isModal ? 'reg-' : 'settings-';
    
    const nameInput = document.getElementById(prefix + 'name')?.value.trim();
    if (!nameInput) return;

    const body = {
        name: nameInput,
        height_cm: parseFloat(document.getElementById(prefix + 'height')?.value) || 0,
        wingspan_cm: parseFloat(document.getElementById(prefix + 'wingspan')?.value) || 0,
        skill_level: document.getElementById(prefix + 'skill')?.value || 'beginner',
        pool_length: parseFloat(document.getElementById(prefix + 'pool-length')?.value) || 25
    };
    
    const res = await apiPost('/api/register', body);
    if (res && res.status === 'ok') {
        userProfile = res.profile;
        const wcEl = document.getElementById('settings-wearable-count');
        if (wcEl) {
            try {
                localStorage.setItem(LS_EXPECTED_WEARABLES, String(Math.max(1, Math.min(6, parseInt(wcEl.value, 10) || 1))));
            } catch (e) { /* */ }
        }
        updateProfileUI();
        const rm = document.getElementById('reg-modal');
        if (rm) rm.classList.remove('show');
        showToast('Profile saved!', 'success');
        pushUserConfigToDevice(true); // Auto-sync to device
        renderSetupJourney();
    }
}

// ── DEVICE MANAGEMENT ──
const LS_EXPECTED_WEARABLES = 'goldenform_expected_wearables';
/** Set after a successful Session → Sync Device (/process) — includes 0 sessions (pre–first swim). */
const LS_WIFI_SYNC_OK = 'goldenform_wifi_sync_ok';
/** Mirrors last /api/devices count for guided setup (no extra round-trip). */
let cachedDeviceListLength = 0;
/** Full rows from last /api/devices — used for smart placement and slot copy. */
let lastDevicesList = [];

function escapeHtml(s) {
    if (s == null || s === '') return '';
    return String(s)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;');
}

const ROLE_LABELS = {
    wrist_right: 'Right wrist',
    wrist_left: 'Left wrist',
    head: 'Head',
    ankle_right: 'Right ankle',
    ankle_left: 'Left ankle',
    waist: 'Waist / back'
};

/** Canonical role list — order used for “suggest next placement”. */
const DEVICE_ROLE_ORDER = Object.keys(ROLE_LABELS);

function normalizeDeviceRole(role) {
    const r = String(role || 'wrist_right').toLowerCase();
    return DEVICE_ROLE_ORDER.includes(r) ? r : 'wrist_right';
}

function formatRoleLabel(role) {
    if (!role) return 'Unknown placement';
    const k = String(role).toLowerCase();
    return ROLE_LABELS[k] || String(role).replace(/_/g, ' ');
}

function suggestNextDeviceRole(devices) {
    const used = new Set((devices || []).map(d => String(d.role || '').toLowerCase()));
    for (const r of DEVICE_ROLE_ORDER) {
        if (!used.has(r)) return r;
    }
    return 'wrist_right';
}

/** Live hint: which GoldenForm_* hotspot the phone/browser is routing through (from /api/device_info). */
function updateWearableConnectionBanner(res) {
    const el = document.getElementById('connected-wearable-banner');
    const hid = document.getElementById('dev-wifi-ssid');
    if (!el) return;
    if (!res || res.error || res.status === 'disconnected' || res.device_id === undefined) {
        el.innerHTML =
            '<span class="wearable-banner-inner wearable-banner-inner--offline">' +
            'Not connected to a wearable hotspot. Join <strong>GoldenForm_1</strong>, <strong>GoldenForm_2</strong>, … ' +
            '(SSID matches firmware Device ID — a second board shows a different name than your left‑wrist unit.)</span>';
        if (hid) hid.value = '';
        return;
    }
    const ssid = res.ssid || ('GoldenForm_' + res.device_id);
    const roleL = formatRoleLabel(normalizeDeviceRole(res.device_role));
    if (hid) hid.value = ssid;
    el.innerHTML =
        '<span class="wearable-banner-inner wearable-banner-inner--online">' +
        'On Wi‑Fi <strong>' + escapeHtml(ssid) + '</strong> — firmware ID <strong>' + escapeHtml(String(res.device_id)) + '</strong>, ' +
        'role <strong>' + escapeHtml(roleL) + '</strong>. Use <strong>Load from connected device</strong>, set <strong>Body placement</strong> for <em>this</em> unit, then Register.</span>';
}

async function refreshWearableConnectionBanner() {
    try {
        const res = await apiGet('/api/device_info');
        if (res && !res.error && res.status !== 'disconnected' && res.device_id !== undefined) {
            updateWearableConnectionBanner(res);
        } else {
            updateWearableConnectionBanner(null);
        }
    } catch (e) {
        updateWearableConnectionBanner(null);
    }
}

function refreshDeviceRegistrationHints() {
    const slot = document.getElementById('device-reg-slot');
    if (!slot) return;
    const expected = getExpectedWearableCount();
    const n = cachedDeviceListLength;
    const nextRole = suggestNextDeviceRole(lastDevicesList);
    const nextLabel = formatRoleLabel(nextRole);
    if (n >= expected) {
        slot.innerHTML = '<span class="device-reg-slot-inner device-reg-slot-inner--ok">All <strong>' + expected + '</strong> planned units registered — add or remove below anytime.</span>';
        return;
    }
    const stepNum = n + 1;
    slot.innerHTML =
        '<span class="device-reg-slot-inner">' +
        '<strong>Device ' + stepNum + ' of ' + expected + '</strong> — only this unit’s GoldenForm Wi‑Fi. Suggested: <strong>' + escapeHtml(nextLabel) + '</strong>.</span>';
}

async function fillDeviceFormFromConnectedDevice() {
    try {
        const res = await apiGet('/api/device_info');
        if (!res || res.error || res.status === 'disconnected' || res.device_id === undefined) {
            showToast('No device on GoldenForm Wi‑Fi. Join this wearable’s network, then try again.', 'error');
            return;
        }
        const hw = document.getElementById('dev-hw-id');
        const roleEl = document.getElementById('dev-role');
        const name = document.getElementById('dev-name');
        if (hw) hw.value = String(res.device_id);
        const finalRole = normalizeDeviceRole(res.device_role);
        if (roleEl) roleEl.value = finalRole;
        const ssid = res.ssid || ('GoldenForm_' + res.device_id);
        const hid = document.getElementById('dev-wifi-ssid');
        if (hid) hid.value = ssid;
        if (name) name.value = ssid + ' · ' + formatRoleLabel(finalRole) + ' · HW#' + res.device_id;
        updateWearableConnectionBanner(res);
        showToast('Loaded ' + ssid + ' · HW #' + res.device_id + ' · ' + formatRoleLabel(finalRole) + ' — set placement, then Register.', 'success');
    } catch (e) {
        showToast('Could not read device. Check Wi‑Fi connection.', 'error');
    }
}

function applySmartDeviceDefaults() {
    const hid = document.getElementById('dev-wifi-ssid');
    if (hid) hid.value = '';
    const roleEl = document.getElementById('dev-role');
    const name = document.getElementById('dev-name');
    const next = suggestNextDeviceRole(lastDevicesList);
    if (roleEl) roleEl.value = next;
    if (name) {
        const idx = cachedDeviceListLength + 1;
        const exp = getExpectedWearableCount();
        name.value = 'Wearable ' + idx + '/' + exp + ' · ' + formatRoleLabel(next);
    }
    showToast('Suggested: ' + formatRoleLabel(next) + '. Connect the unit and use Load from connected device to fill the hardware ID.', 'info');
}

function getExpectedWearableCount() {
    try {
        const n = parseInt(localStorage.getItem(LS_EXPECTED_WEARABLES) || '1', 10);
        return Math.max(1, Math.min(6, n > 0 ? n : 1));
    } catch (e) {
        return 1;
    }
}

function hasWifiSyncCompleted() {
    try {
        return !!localStorage.getItem(LS_WIFI_SYNC_OK);
    } catch (e) {
        return false;
    }
}

function recordWifiSyncSuccess() {
    try {
        localStorage.setItem(LS_WIFI_SYNC_OK, String(Date.now()));
    } catch (e) { /* ignore */ }
}

function hasSavedSessions() {
    return Array.isArray(savedSessions) && savedSessions.length > 0;
}

/** Single source for Home journey + nav hint (avoids duplicated profile/device/session math). */
function getOnboardingSnapshot() {
    const profileOk = !!(userProfile && userProfile.name);
    const expected = getExpectedWearableCount();
    const devCount = cachedDeviceListLength;
    const devicesOk = devCount >= expected;
    const hasSession = hasSavedSessions();
    const linkOk = hasWifiSyncCompleted() || hasSession;
    return { profileOk, expected, devCount, devicesOk, hasSession, linkOk };
}

function updateNavNextHint(snap) {
    const el = document.getElementById('nav-next-hint');
    if (!el) return;
    const { profileOk, expected, devCount, devicesOk, hasSession, linkOk } = snap || getOnboardingSnapshot();
    if (!profileOk) el.textContent = 'Next: save your profile';
    else if (!devicesOk) el.textContent = 'Next: register wearables (' + devCount + '/' + expected + ')';
    else if (!linkOk) el.textContent = 'Next: Session → Sync (0 sessions OK before first swim)';
    else if (!hasSession) el.textContent = 'Next: swim → sync again to load sessions';
    else el.textContent = 'Next: replay session & Insights';
}

function updateSyncPlaybookMulti() {
    const el = document.getElementById('sync-playbook-multi');
    if (!el) return;
    const exp = getExpectedWearableCount();
    if (exp > 1) {
        el.textContent = 'You plan to use ' + exp + ' wearables. Connect to each device’s GoldenForm Wi‑Fi one at a time, sync, then use Merge to combine.';
    } else {
        el.textContent = 'Add more wearables anytime in Settings — the home checklist and Merge step update automatically.';
    }
}

function updateSyncPlaybookConnectionState() {
    const wrap = document.getElementById('sync-playbook-wrap');
    if (!wrap) return;
    wrap.classList.toggle('sync-playbook-wrap--online', !!navDisplayOnline);
}

function updateSyncPlaybookFirstTime(snap) {
    const el = document.getElementById('sync-playbook-first');
    if (!el) return;
    const { linkOk } = snap || getOnboardingSnapshot();
    if (linkOk) {
        el.textContent = '';
        el.style.display = 'none';
    } else {
        el.style.display = 'block';
        el.textContent = 'First sync can run before any swim — 0 sessions still links the device and pushes your profile. Sync again after swimming to load data.';
    }
}

/**
 * Home checklist — matches PDP base path; highlights the next incomplete step.
 */
function renderSetupJourney() {
    const host = document.getElementById('setup-journey');
    const phaseEl = document.getElementById('setup-journey-phase');
    if (!host) return;
    const snap = getOnboardingSnapshot();
    const { profileOk, expected, devCount, devicesOk, hasSession, linkOk } = snap;

    let phase = 'Ready — review your data';
    let currentKey = 'viz';
    if (!profileOk) {
        phase = 'Step 1 — Profile';
        currentKey = 'profile';
    } else if (!devicesOk) {
        phase = 'Step 2 — Wearables';
        currentKey = 'devices';
    } else if (!linkOk) {
        phase = 'Step 3 — Wi‑Fi link';
        currentKey = 'sync';
    } else if (!hasSession) {
        phase = 'Step 4 — Swim, then sync';
        currentKey = 'viz';
    } else {
        phase = 'Step 4 — Analyze';
        currentKey = 'viz';
    }
    if (phaseEl) phaseEl.textContent = phase;

    function isStepDone(key) {
        if (key === 'profile') return profileOk;
        if (key === 'devices') return devicesOk;
        if (key === 'sync') return linkOk;
        if (key === 'viz') return hasSession;
        return false;
    }
    function stepClass(key) {
        let c = 'journey-step';
        if (isStepDone(key)) c += ' journey-step--done';
        if (currentKey === key) c += ' journey-step--current';
        return c;
    }

    const step2Text = expected > 1
        ? ('One Wi‑Fi at a time → <strong>Load</strong> HW ID → placement → name → <strong>Register</strong>. Repeat until ' + expected + ' rows appear.')
        : 'GoldenForm Wi‑Fi → load HW ID + placement + name. Lists use <strong>HW #</strong> + placement.';

    const step3Text = !linkOk
        ? ('Session tab → device sync mode → join <strong>GoldenForm</strong> → <strong>Sync Device</strong>. <strong>0 sessions</strong> is OK on the first run.')
        : ('After swimming: same flow, one device at a time. Several units: <strong>Merge</strong> on Session.');

    const step4Label = hasSession ? '4 · Analyze' : '4 · Swim &amp; load data';
    const step4Text = hasSession
        ? 'Replay, charts, form score, ideal comparison, Insights.'
        : 'Record on the wearable → sync mode → <strong>Sync Device</strong> — data shows in Session &amp; Analysis.';
    const step4Tab = hasSession ? 'analysis' : 'session';
    const step4Cta = hasSession ? 'Open Analysis' : 'Go to Session';

    const steps = [
        { key: 'profile', label: '1 · Profile', text: 'Name, wingspan, pool length, and how many wearables you use.', tab: 'settings', cta: 'Open Settings' },
        { key: 'devices', label: '2 · Wearables (' + devCount + '/' + expected + ')', text: step2Text, tab: 'settings', cta: 'Register devices' },
        { key: 'sync', label: '3 · Wi‑Fi sync', text: step3Text, tab: 'session', cta: 'Go to Sync' },
        { key: 'viz', label: step4Label, text: step4Text, tab: step4Tab, cta: step4Cta }
    ];

    host.innerHTML = '<div class="journey-grid">' + steps.map(s =>
        '<div class="' + stepClass(s.key) + '"><div class="journey-step-head"><span>' + s.label + '</span>' +
        (isStepDone(s.key) ? '<span class="journey-check">✓</span>' : '') +
        '</div><p>' + s.text + '</p><button type="button" class="btn btn-outline btn-sm" onclick="switchTab(\'' + s.tab + '\')">' + s.cta + '</button></div>'
    ).join('') + '</div>';

    const cont = document.getElementById('setup-journey-continue');
    if (cont) {
        cont.innerHTML = '<p class="setup-continue">Same GoldenForm Wi‑Fi for Settings and Session. <strong>Stretch:</strong> multi-unit → sync each → Merge · ideal stroke &amp; haptics in Settings · trends in Insights.</p>';
    }

    updateNavNextHint(snap);
    refreshDeviceRegistrationHints();
    updateSyncPlaybookFirstTime(snap);
    updateHomeOnboardingPanels();
}

async function loadDevices() {
    const res = await apiGet('/api/devices');
    const devices = (res && res.devices) || [];
    lastDevicesList = devices;
    cachedDeviceListLength = devices.length;
    registeredHeadDevice = devices.some(d => String(d.role || '').toLowerCase() === 'head');
    const wristDevice = devices.find(d => String(d.role || '').toLowerCase().startsWith('wrist'));
    if (wristDevice) {
        activeWristDeviceRole = wristDevice.role.toLowerCase();
        if (typeof handGroup !== 'undefined' && handGroup) {
            const flip = activeWristDeviceRole === 'wrist_left' ? -2.75 : 2.75;
            handGroup.scale.set(flip, 2.75, 2.75);
            if (ghostArmGroup) ghostArmGroup.scale.set(flip, 2.75, 2.75);
        }
    }
    const list = document.getElementById('device-list');
    if (list) {
        if (!devices.length) {
            list.innerHTML = '<p style="color:var(--text3);font-size:0.85em;">No devices registered yet. Use the steps below — one Wi‑Fi, one registration at a time.</p>';
        } else {
            const roleIcons = { wrist_right: '🤚', wrist_left: '✋', head: '🧠', ankle_right: '🦶', ankle_left: '🦶', waist: '🫁' };
            list.innerHTML = devices.map((d, i) => {
                const idx = i + 1;
                const colorClass = 'device-card--' + ((i % 6) + 1);
                const title = escapeHtml(d.name || ('Unit #' + idx));
                const roleL = formatRoleLabel(d.role);
                const wifiLine = (d.wifi_ssid && String(d.wifi_ssid).trim())
                    ? '<div class="device-card-wifi">Wi‑Fi: <strong>' + escapeHtml(String(d.wifi_ssid).trim()) + '</strong></div>'
                    : '';
                return '<div class="device-card ' + colorClass + '">' +
                    '<div class="device-card-index" title="Order added">' + idx + '</div>' +
                    '<div class="device-card-body">' +
                    '<div class="device-card-title">' + (roleIcons[d.role] || '📱') + ' ' + title + '</div>' +
                    '<div class="device-card-meta">' + escapeHtml(roleL) + ' · <strong>HW #' + escapeHtml(String(d.device_hw_id)) + '</strong></div>' +
                    wifiLine +
                    '<div class="device-card-hint">Sessions and sync tie data to this HW # + placement.</div></div>' +
                    '<button type="button" class="btn btn-sm btn-outline" onclick="deleteDevice(' + d.id + ')" title="Remove from app registry">✕</button></div>';
            }).join('');
        }
    }
    renderSetupJourney();
    updateSyncPlaybookMulti();
}
async function deleteDevice(id) {
    const res = await apiPost('/api/devices/delete', { device_id: id });
    if (res && res.status === 'ok') { showToast('Device removed', 'success'); loadDevices(); }
    else showToast('Failed to remove device', 'error');
}
async function registerDevice(e) {
    if (e) e.preventDefault();
    const wifiEl = document.getElementById('dev-wifi-ssid');
    const wf = (wifiEl && wifiEl.value) ? String(wifiEl.value).trim() : '';
    const body = {
        user_id: userProfile ? userProfile.id : 1,
        device_hw_id: parseInt(document.getElementById('dev-hw-id')?.value) || 0,
        role: document.getElementById('dev-role')?.value || 'wrist_right',
        name: document.getElementById('dev-name')?.value || '',
        wifi_ssid: wf
    };
    const res = await apiPost('/api/devices/register', body);
    if (res && res.status === 'ok') {
        const rl = formatRoleLabel(body.role);
        const nm = (body.name || '').trim();
        showToast(
            'Registered: ' + rl + ' · HW #' + body.device_hw_id +
            (wf ? ' · ' + wf : '') +
            (nm ? ' — ' + nm : ''),
            'success'
        );
    }
    loadDevices();
}

let isDeviceOnline = false;
/** Debounced for UI/playbook — avoids Connected/Offline flicker when /device_info flaps. */
let navDisplayOnline = false;
let pendingConfigSync = false;
let pendingIdealSync = false;
let devicePollInterval = null;
let pollOfflineStreak = 0;
let lastPollAutoSyncAt = 0;
const POLL_OFFLINE_AFTER_FAILS = 2;
const POLL_AUTO_SYNC_COOLDOWN_MS = 90000;
let lastConnStatusApplied = '';

function setConnStatus(state) {
    if (state === lastConnStatusApplied) return;
    lastConnStatusApplied = state;
    navDisplayOnline = state === 'connected' || state === 'syncing';
    const dot = document.querySelector('.status-dot');
    const txt = document.getElementById('conn-text');
    if (dot) dot.className = 'status-dot ' + state;
    if (txt) txt.textContent = state === 'connected' ? 'Connected' : state === 'syncing' ? 'Syncing...' : 'Offline';
}

function startDevicePolling() {
    if (devicePollInterval) clearInterval(devicePollInterval);
    // Poll immediately on start
    pollDevice();
    devicePollInterval = setInterval(pollDevice, 5000);
}

async function pollDevice() {
    try {
        const res = await apiGet('/api/device_info');
        const rawOnline = !!(res && !res.error && res.status !== 'disconnected');
        const wasOnline = isDeviceOnline;
        isDeviceOnline = rawOnline;

        const txt = document.getElementById('conn-text');
        const midSync = txt && txt.textContent === 'Syncing...';

        if (!midSync) {
            if (rawOnline) {
                pollOfflineStreak = 0;
                setConnStatus('connected');
            } else {
                pollOfflineStreak++;
                if (pollOfflineStreak >= POLL_OFFLINE_AFTER_FAILS) {
                    setConnStatus('offline');
                }
            }
        }

        if (res && res.cal) {
            updateCalibrationDisplay(res);
        }

        if (!wasOnline && isDeviceOnline) {
            const now = Date.now();
            if (now - lastPollAutoSyncAt >= POLL_AUTO_SYNC_COOLDOWN_MS) {
                lastPollAutoSyncAt = now;
                showToast('Device detected! Auto-syncing...', 'info');
                await syncFromDevice();
                if (pendingConfigSync) {
                    await pushUserConfigToDevice(true);
                    pendingConfigSync = false;
                }
                if (pendingIdealSync) {
                    await pushIdealToDevice(true);
                    pendingIdealSync = false;
                }
            }
        }
        updateSyncPlaybookConnectionState();
        if (rawOnline && res && res.device_id !== undefined) {
            updateWearableConnectionBanner(res);
        } else {
            updateWearableConnectionBanner(null);
        }
    } catch (e) {
        isDeviceOnline = false;
        updateWearableConnectionBanner(null);
        const txt = document.getElementById('conn-text');
        if (txt && txt.textContent !== 'Syncing...') {
            pollOfflineStreak++;
            if (pollOfflineStreak >= POLL_OFFLINE_AFTER_FAILS) {
                setConnStatus('offline');
            }
        }
        updateSyncPlaybookConnectionState();
    }
}

let syncDeviceCount = 0;
let lastSyncedDeviceInfo = null;
async function syncFromDevice() {
    const statusEl = document.getElementById('sync-status');
    const btn = document.getElementById('sync-btn');
    if (statusEl) { statusEl.style.display = 'block'; statusEl.textContent = 'Connecting to device...'; statusEl.className = 'badge badge-amber'; }
    if (btn) btn.disabled = true;
    setConnStatus('syncing');

    const devInfo = await apiGet('/api/device_info');
    if (devInfo && devInfo.device_id !== undefined) {
        lastSyncedDeviceInfo = devInfo;
        if (statusEl) statusEl.textContent = `Connected to ${devInfo.ssid || 'GoldenForm'} (${devInfo.device_role || 'wrist'})...`;
        if (userProfile && userProfile.id) {
            const role = normalizeDeviceRole(devInfo.device_role);
            const ssid = devInfo.ssid || ('GoldenForm_' + devInfo.device_id);
            const autoName = ssid + ' · ' + formatRoleLabel(role) + ' · HW#' + devInfo.device_id;
            await apiPost('/api/devices/register', {
                user_id: userProfile.id,
                device_hw_id: devInfo.device_id,
                role,
                name: autoName,
                wifi_ssid: devInfo.ssid || ''
            });
        }
    }

    try {
        const res = await apiGet('/process');
        if (!res || res.error) {
            if (statusEl) { statusEl.textContent = res ? res.error : 'No response'; statusEl.className = 'badge badge-red'; }
            setConnStatus('offline');
            showToast('Failed to connect to device', 'error');
            return;
        }
        const sessions = res.sessions || [];
        recordWifiSyncSuccess();
        syncDeviceCount++;
        const regTail = devInfo && devInfo.device_id !== undefined
            ? ` · registry: ${formatRoleLabel(devInfo.device_role || 'wrist_right')} · HW #${devInfo.device_id}`
            : '';
        if (statusEl) {
            statusEl.textContent = sessions.length === 0
                ? `Linked — 0 sessions (OK before first swim) · sync #${syncDeviceCount}`
                : `Synced ${sessions.length} session(s) from device #${syncDeviceCount}`;
            statusEl.className = 'badge badge-green';
        }
        setConnStatus('connected');
        const zeroHint = sessions.length === 0 ? ' After your first swim, sync again to download sessions.' : '';
        showToast(`Synced ${sessions.length} session(s) (sync #${syncDeviceCount})${regTail}${zeroHint ? '. ' + zeroHint : ''}`, 'success');

        for (const s of sessions) {
            addSession({
                name: s.name,
                processed_data: s.processed_data,
                raw_data: s.raw_data,
                metrics: s.metrics,
                duration: s.duration,
                syncedAt: s.syncedAt
            });
        }
        if (sessions.length > 0) {
            const selIdx = savedSessions.length - 1;
            // Defer heavy viz (3D, charts, timeline DOM) so sync toasts/UI stay responsive — avoids post-sync tab freeze.
            setTimeout(() => {
                try {
                    if (selIdx >= 0 && selIdx < savedSessions.length) selectSession(selIdx);
                } catch (e) { /* ignore */ }
            }, 150);
        }

        void Promise.all([pushUserConfigToDevice(true), pushIdealToDevice(true)]).catch(() => {});

        if (syncDeviceCount > 1) {
            showToast(`${syncDeviceCount} devices synced. Use "Merge Devices" to combine.`, 'info');
        }
    } catch (e) {
        if (statusEl) { statusEl.textContent = 'Device unreachable. Using cached data...'; statusEl.className = 'badge badge-amber'; }
        setConnStatus('offline');
        try {
            const cached = await apiGet('/cached');
            if (cached && cached.sessions) {
                for (const s of cached.sessions) addSession({
                    name: s.name || 'Cached',
                    id: s.id,
                    processed_data: s.processed_data,
                    raw_data: s.raw_data,
                    metrics: s.metrics,
                    duration: s.duration,
                    syncedAt: s.syncedAt
                });
                if (cached.sessions.length > 0) {
                    const selIdx = savedSessions.length - 1;
                    setTimeout(() => {
                        try {
                            if (selIdx >= 0 && selIdx < savedSessions.length) selectSession(selIdx);
                        } catch (e) { /* ignore */ }
                    }, 150);
                }
                if (statusEl) statusEl.textContent = 'Loaded from cache';
                showToast('Loaded cached session data', 'info');
            }
        } catch { if (statusEl) { statusEl.textContent = 'No data available'; statusEl.className = 'badge badge-red'; } }
    } finally {
        if (btn) btn.disabled = false;
        try {
            await loadDevices();
        } catch (e) { /* ignore refresh errors */ }
        updateSyncPlaybookConnectionState();
    }
}

const LS_KEY = 'goldenform_sessions';
async function loadSavedSessions() {
    try { savedSessions = JSON.parse(localStorage.getItem(LS_KEY) || '[]'); } catch { savedSessions = []; }
    renderSessionList();
    try {
        const dbSessions = await apiGet('/api/sessions');
        if (dbSessions && dbSessions.sessions) {
            const localIds = new Set(savedSessions.filter(s => s.id).map(s => s.id));
            for (const s of dbSessions.sessions) {
                if (!localIds.has(s.id)) {
                    savedSessions.push({ id: s.id, name: s.name || `Session ${s.id}`, processed_data: s.processed_data || [], metrics: s.metrics || {}, duration: s.duration || 0, syncedAt: s.synced_at });
                }
            }
            while (savedSessions.length > 8) savedSessions.shift();
            persistSessions();
        }
    } catch { }
    renderSessionList();
    renderSetupJourney();
}
function persistSessions() {
    try { localStorage.setItem(LS_KEY, JSON.stringify(savedSessions)); } catch (e) {
        if (savedSessions.length > 3) { savedSessions = savedSessions.slice(-3); try { localStorage.setItem(LS_KEY, JSON.stringify(savedSessions)); } catch { } }
    }
}
function addSession(obj) {
    savedSessions.push(obj);
    while (savedSessions.length > 8) savedSessions.shift();
    persistSessions();
    renderSessionList();
    if (!obj.id) {
        const devIds = [...new Set((obj.processed_data || []).map(d => d.device_id).filter(Boolean))];
        apiPost('/api/sessions/save', { raw_data: obj.raw_data, processed_data: obj.processed_data, metrics: obj.metrics, duration: obj.duration, device_ids: devIds }).then(res => {
            if (res && res.session_id) {
                obj.id = res.session_id;
                persistSessions();
            }
        }).catch(() => { });
    }
}

async function mergeLatestSessions() {
    const btn = document.getElementById('merge-btn');
    if (!btn) return;
    const sessionIdsToMerge = savedSessions.filter(s => s.id).slice(-3).map(s => s.id);
    if (sessionIdsToMerge.length < 2) {
        showToast('Need at least 2 saved sessions (synced with DB) to merge', 'error');
        return;
    }
    btn.disabled = true;
    btn.textContent = 'Merging...';
    try {
        const res = await apiPost('/api/sessions/merge', { session_ids: sessionIdsToMerge });
        if (res.aligned_sessions && res.aligned_sessions.length > 0) {
            let combined = [];
            for (const sess of res.aligned_sessions) {
                for (const p of sess.processed_data) {
                    p._origin_id = sess.id;
                    combined.push(p);
                }
            }
            // Interleave by timestamp
            combined.sort((a, b) => (a.timestamp || 0) - (b.timestamp || 0));
            const obj = {
                name: 'Merged Session (' + sessionIdsToMerge.length + ')',
                processed_data: combined,
                duration: res.aligned_sessions[0].duration,
                metrics: res.aligned_sessions[0].metrics,
                syncedAt: new Date().toISOString()
            };
            addSession(obj);
            selectSession(savedSessions.length - 1);
            showToast('Merged successfully', 'success');
        } else if (res.error) {
            showToast('Merge error: ' + res.error, 'error');
        }
    } catch (e) {
        showToast('Merge failed: ' + e, 'error');
    } finally {
        btn.disabled = false;
        btn.textContent = 'Merge Devices';
        renderSetupJourney();
    }
}
/** Show first-run strip vs quick actions — keeps Home aligned with PDP base path. */
function updateHomeOnboardingPanels() {
    const gettingStarted = document.getElementById('getting-started');
    const quickActions = document.getElementById('quick-actions');
    if (!gettingStarted || !quickActions) return;
    const hasSess = hasSavedSessions();
    gettingStarted.style.display = hasSess ? 'none' : '';
    quickActions.style.display = hasSess ? '' : 'none';
}

function renderSessionList() {
    const c = document.getElementById('session-cards');
    const badge = document.getElementById('session-count-badge');
    if (badge) badge.textContent = savedSessions.length + ' saved';

    updateHomeOnboardingPanels();

    if (!c) return;
    if (!savedSessions.length) {
        c.innerHTML = '<p style="color:var(--text3);font-size:0.85em;padding:16px;">No sessions yet — Session tab, join GoldenForm Wi‑Fi, tap Sync Device.</p>';
        return;
    }
    c.innerHTML = savedSessions.map((s, i) => `<div class="session-item ${i === activeSessionIdx ? 'active' : ''}" onclick="selectSession(${i})"><div><strong>${s.name || 'Session ' + (i + 1)}</strong><div class="meta">${s.metrics ? (s.metrics.stroke_count || 0) + ' strokes · ' + formatTime(s.duration) : ''}</div></div><button class="btn btn-sm btn-outline" onclick="event.stopPropagation();deleteSession(${i})">✕</button></div>`).join('');
}
function deleteSession(i) {
    savedSessions.splice(i, 1);
    persistSessions();
    if (activeSessionIdx === i) { activeSessionIdx = -1; clearViz(); }
    else if (activeSessionIdx > i) activeSessionIdx--;
    renderSessionList();
}
async function selectSession(i) {
    if (i < 0 || i >= savedSessions.length) return;
    activeSessionIdx = i;
    const s = savedSessions[i];
    let pd = s.processed_data || s.processedData || [];
    if ((!pd || !pd.length) && s.id) {
        const detail = await apiGet('/api/sessions/' + s.id);
        if (detail && detail.session) {
            pd = detail.session.processed_data || [];
            s.processed_data = pd;
            s.metrics = detail.session.metrics || s.metrics;
            s.raw_data = detail.session.raw_data;
            persistSessions();
        }
    }
    processedData = pd;
    sessionMetrics = s.metrics || {};
    currentIndex = 0;
    integratedPositions = [];
    rawIntegratedPositions = [];
    lastRenderedStrokeCount = 0;
    lastCalStripFrameIdx = -1;
    followHandInView = true;
    isPlaying = false;
    if (playbackInterval) { clearInterval(playbackInterval); playbackInterval = null; }
    // Pre-compute stroke boundaries and haptic events for timeline
    computeStrokeBoundaries();
    buildPlaybackStrokeSegments();
    renderSessionList();
    updateSessionSummary();
    refreshVizPlaybackUI();
    updateAnalysis();
    updateCoachingInsights();
    // Switch to Session tab first so chart and 3D canvases have real dimensions (not 0x0)
    switchTab('session');
    // Scroll Session tab into view so the 3D canvas and charts are visible
    const sessionPage = document.getElementById('page-session');
    if (sessionPage) sessionPage.scrollIntoView({ behavior: 'smooth', block: 'start' });
    buildHapticTimeline();
    // Defer chart init, 3D init/resize, and first frame so layout is complete
    requestAnimationFrame(() => {
        const c = document.getElementById('canvas3d');
        if (c) {
            const w = Math.max(c.clientWidth || 800, 400);
            const h = Math.max(c.clientHeight || 450, 400);
            // (Re)init 3D if not yet created (e.g. canvas was hidden at load)
            if (!scene || !renderer) init3D();
            if (renderer && camera) {
                renderer.setSize(w, h);
                camera.aspect = w / h;
                camera.updateProjectionMatrix();
            }
        }
        initCharts();
        renderFrame(0);
        if (processedData.length > 0 && accelChart && gyroChart) updateCharts(0);
    });
}

// ── STROKE BOUNDARIES & HAPTIC TIMELINE ──
/** Hard cap — each tick is a DOM node; dense haptic flags used to create 50k+ nodes and crash the tab. */
const MAX_HAPTIC_TIMELINE_MARKERS = 400;
const MAX_STROKE_TIMELINE_MARKERS = 250;

let strokeBoundaries = []; // {index, strokeNum}
let hapticEvents = [];     // {index, deviation}

function limitTimelineEvents(events, max) {
    if (!events || events.length <= max) return events;
    const n = events.length;
    const out = [];
    for (let k = 0; k < max; k++) {
        const i = Math.min(n - 1, Math.floor((k + 0.5) * n / max));
        out.push(events[i]);
    }
    return out;
}

function computeStrokeBoundaries() {
    strokeBoundaries = [];
    hapticEvents = [];
    refreshStrokeFieldMode();
    let lastCount = 0;
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const sn = strokeNumAt(d);
        if (sn > lastCount) {
            strokeBoundaries.push({ index: i, strokeNum: sn });
            lastCount = sn;
        }
        // Leading edge only: firmware may leave haptic_fired true across many consecutive samples.
        // One marker per buzz event keeps the timeline usable and avoids DOM / layout meltdown.
        if (d.haptic_fired && (i === 0 || !processedData[i - 1].haptic_fired)) {
            let dev = d.deviation_score || 0;
            if (dev === 0) {
                for (let j = i; j >= Math.max(0, i - 100); j--) {
                    if (processedData[j].deviation_score > 0) { dev = processedData[j].deviation_score; break; }
                }
            }
            hapticEvents.push({ index: i, deviation: dev });
        }
    }
}

function buildHapticTimeline() {
    const container = document.getElementById('haptic-timeline');
    if (!container || !processedData.length) return;
    container.innerHTML = '';
    const total = processedData.length;
    const strokeTicks = limitTimelineEvents(strokeBoundaries, MAX_STROKE_TIMELINE_MARKERS);
    const hapticTicks = limitTimelineEvents(hapticEvents, MAX_HAPTIC_TIMELINE_MARKERS);
    // Stroke boundaries as blue ticks
    strokeTicks.forEach(sb => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick stroke-tick';
        tick.style.left = (sb.index / total * 100) + '%';
        tick.title = 'Stroke #' + sb.strokeNum;
        container.appendChild(tick);
    });
    // Haptic events as red diamonds
    hapticTicks.forEach(he => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick haptic-tick';
        tick.style.left = (he.index / total * 100) + '%';
        tick.title = 'Haptic fired (deviation: ' + he.deviation.toFixed(3) + ')';
        container.appendChild(tick);
    });
}

// ── PLAYBACK ──
function getPlaybackBounds() {
    if (!processedData.length) return { start: 0, end: 0 };
    if (playbackStrokeFilter <= 0) return { start: 0, end: processedData.length - 1 };
    const seg = playbackStrokeSegments.find(s => s.strokeNum === playbackStrokeFilter);
    if (!seg) return { start: 0, end: processedData.length - 1 };
    return { start: seg.startIdx, end: seg.endIdx };
}

function togglePlayback() {
    if (!processedData.length) return;
    isPlaying = !isPlaying;
    const btn = document.getElementById('play-btn');
    if (isPlaying) {
        followHandInView = true;
        if (btn) btn.textContent = '⏸';
        if (playbackInterval) clearInterval(playbackInterval);
        const tick = () => {
            if (!processedData.length) return;
            const b = getPlaybackBounds();
            if (currentIndex < b.end) {
                currentIndex++;
            } else {
                if (playbackStrokeFilter > 0 && loopStrokePlayback) {
                    currentIndex = b.start;
                } else if (playbackStrokeFilter <= 0 && loopFullSession && processedData.length > 1) {
                    currentIndex = b.start;
                } else {
                    isPlaying = false;
                    if (btn) btn.textContent = '▶';
                    if (playbackInterval) clearInterval(playbackInterval);
                    playbackInterval = null;
                    return;
                }
            }
            renderFrame(currentIndex);
        };
        const spd = Math.max(0.05, Math.min(4, playbackSpeedMultiplier || 1));
        playbackInterval = setInterval(tick, PLAYBACK_BASE_MS / spd);
    } else {
        if (btn) btn.textContent = '▶';
        if (playbackInterval) clearInterval(playbackInterval);
        playbackInterval = null;
    }
}
function resetPlayback() {
    const b = getPlaybackBounds();
    currentIndex = b.start;
    isPlaying = false;
    if (playbackInterval) clearInterval(playbackInterval);
    playbackInterval = null;
    const btn = document.getElementById('play-btn');
    if (btn) btn.textContent = '▶';
    renderFrame(currentIndex);
}
function skipForward() {
    const b = getPlaybackBounds();
    currentIndex = Math.min(currentIndex + 30, b.end);
    renderFrame(currentIndex);
}
function skipBackward() {
    const b = getPlaybackBounds();
    currentIndex = Math.max(currentIndex - 30, b.start);
    renderFrame(currentIndex);
}

/** Seek scrubber from screen X (click or drag). */
function seekPlaybackFromPointer(clientX, trackEl) {
    const b = getPlaybackBounds();
    if (!trackEl || b.end <= b.start) return;
    const rect = trackEl.getBoundingClientRect();
    const w = rect.width || 1;
    const pct = Math.max(0, Math.min(1, (clientX - rect.left) / w));
    currentIndex = Math.round(b.start + pct * (b.end - b.start));
    currentIndex = Math.max(b.start, Math.min(b.end, currentIndex));
    renderFrame(currentIndex);
}

function seekPlayback(e) {
    seekPlaybackFromPointer(e.clientX, e.currentTarget);
}

/** Step by IMU samples (±1 = one timestep). Hold Shift in keyboard for ±10. */
function stepFrame(delta) {
    if (!processedData.length) return;
    const b = getPlaybackBounds();
    currentIndex = Math.max(b.start, Math.min(b.end, currentIndex + delta));
    renderFrame(currentIndex);
}

function initScrubberPointerHandlers() {
    const track = document.getElementById('progress-scrubber');
    if (!track || track.dataset.scrubBound === '1') return;
    track.dataset.scrubBound = '1';
    let activePointer = null;
    track.addEventListener('pointerdown', (e) => {
        if (e.button !== 0) return;
        e.preventDefault();
        track.focus({ preventScroll: true });
        track.setPointerCapture(e.pointerId);
        activePointer = e.pointerId;
        track.classList.add('scrubbing');
        seekPlaybackFromPointer(e.clientX, track);
    });
    track.addEventListener('pointermove', (e) => {
        if (activePointer !== e.pointerId) return;
        seekPlaybackFromPointer(e.clientX, track);
    });
    track.addEventListener('pointerup', (e) => {
        if (activePointer === e.pointerId) {
            try { track.releasePointerCapture(e.pointerId); } catch (err) { /* */ }
            activePointer = null;
        }
        track.classList.remove('scrubbing');
    });
    track.addEventListener('pointercancel', () => {
        activePointer = null;
        track.classList.remove('scrubbing');
    });
    track.addEventListener('keydown', (e) => {
        if (!processedData.length) return;
        const n = e.shiftKey ? 10 : 1;
        if (e.key === 'ArrowLeft' || e.key === 'ArrowDown') {
            stepFrame(-n);
            e.preventDefault();
        } else if (e.key === 'ArrowRight' || e.key === 'ArrowUp') {
            stepFrame(n);
            e.preventDefault();
        } else if (e.key === 'Home') {
            resetPlayback();
            e.preventDefault();
        }
    });
}
function jumpToStroke(n) {
    const sb = strokeBoundaries.find(s => s.strokeNum === n);
    if (sb) { currentIndex = sb.index; renderFrame(currentIndex); }
}
function formatTime(s) {
    if (!s || !isFinite(s)) return '0:00';
    const m = Math.floor(s / 60), sec = Math.floor(s % 60);
    return m + ':' + String(sec).padStart(2, '0');
}

/** Wall-clock style with centiseconds (good for slow scrubbing). */
function formatTimeFine(s) {
    if (!isFinite(s) || s < 0) return '0:00.00';
    const m = Math.floor(s / 60);
    const sec = s - m * 60;
    const secStr = (sec < 10 ? '0' : '') + sec.toFixed(2);
    return m + ':' + secStr;
}

// ── SESSION SUMMARY ──
function updateSessionSummary() {
    if (!sessionMetrics) return;
    const m = sessionMetrics;
    const poolLen = userProfile ? (userProfile.pool_length || 25) : 25;
    const distance = (m.turn_count || 0) * poolLen; // Note: if turn_count=1, it means they finished 1 length of 25m? Or completed 1 turn (2 lengths)?
    // Standard swimming: turn_count = 1 means 1 turn at the wall, so 2 lengths completed.
    // Let's assume turn_count is "number of hits at the wall". 
    const totalDist = (m.turn_count || 0) > 0 ? (m.turn_count + 1) * poolLen : (m.stroke_count > 0 ? poolLen : 0);
    
    const pace = totalDist > 0 ? (m.duration / (totalDist / 100)) : 0; // seconds per 100m

    setText('sum-strokes', m.stroke_count || 0);
    setText('sum-turns', m.turn_count || 0);
    setText('sum-distance', totalDist + 'm');
    setText('sum-pace', pace > 0 ? formatTime(pace) + '/100m' : '--:--');
    setText('sum-duration', formatTime(m.duration));
    setText('sum-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) + '/min' : '--');
    setText('sum-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '--');
    setText('sum-samples', processedData.length);
    
    setText('home-last-strokes', m.stroke_count || 0);
    setText('home-last-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) : '--');
    setText('home-last-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '--');
    setText('home-last-distance', totalDist + 'm');
}
function setText(id, val) { const el = document.getElementById(id); if (el) el.textContent = val; }

// ── CALIBRATION DISPLAY (TIDR 6-1-4) ──
function updateCalibrationDisplay(d) {
    const cal = d?.cal || d?.calibration;
    if (!cal) return;
    const fields = [
        { id: 'cal-sys', val: cal.sys },
        { id: 'cal-gyro', val: cal.gyro },
        { id: 'cal-accel', val: cal.accel },
        { id: 'cal-mag', val: cal.mag }
    ];
    fields.forEach(f => {
        const el = document.getElementById(f.id);
        if (!el) return;
        const v = f.val || 0;
        el.textContent = v + '/3';
        el.className = 'cal-badge ' + (v >= 3 ? 'cal-good' : v >= 1 ? 'cal-warn' : 'cal-bad');
    });
    const sys = cal.sys || 0;
    const gyro = cal.gyro || 0;
    const accel = cal.accel || 0;
    const mag = cal.mag || 0;
    const total = sys + gyro + accel + mag;
    const quality = Math.round(total / 12 * 100);
    setText('cal-quality', quality + '%');

    const hintEl = document.getElementById('cal-hint');
    if (hintEl) {
        if (mag < 2) hintEl.textContent = 'Mag: move in a wide figure‑eight, away from metal, until Mag = 3/3.';
        else if (gyro < 2) hintEl.textContent = 'Gyro: place the wearable still on a table.';
        else if (accel < 2) hintEl.textContent = 'Accel: slowly orient the device through six faces (cube).';
        else if (sys < 2) hintEl.textContent = 'System fusion: walk through gyro + accel + mag steps above.';
        else if (quality >= 75) hintEl.textContent = 'Calibration strong — registers ready for stroke integration.';
        else hintEl.textContent = 'Keep refining motion until all four read 3/3.';
    }
}

// ── ANALYSIS ──
function updateAnalysis() {
    if (!sessionMetrics) return;
    const m = sessionMetrics;
    drawGauge('gauge-aoa', m.avg_entry_angle || 0, 0, 90, 15, 40, '°');
    setText('aoa-value', (m.avg_entry_angle || 0).toFixed(1) + '°');
    setText('aoa-ideal', (m.ideal_entry_angle || 30) + '°');

    const pcts = m.phase_pcts || { glide: 0, catch: 0, pull: 0, recovery: 0 };
    const total = (pcts.glide || 0) + (pcts.catch || 0) + (pcts.pull || 0) + (pcts.recovery || 0) || 1;
    setWidth('phase-glide', (pcts.glide || 0) / total * 100);
    setWidth('phase-catch', (pcts.catch || 0) / total * 100);
    setWidth('phase-pull', (pcts.pull || 0) / total * 100);
    setWidth('phase-recovery', (pcts.recovery || 0) / total * 100);
    setText('phase-glide-pct', (pcts.glide || 0).toFixed(0) + '%');
    setText('phase-catch-pct', (pcts.catch || 0).toFixed(0) + '%');
    setText('phase-pull-pct', (pcts.pull || 0).toFixed(0) + '%');
    setText('phase-recovery-pct', (pcts.recovery || 0).toFixed(0) + '%');

    setText('haptic-count', m.haptic_count || 0);
    setText('avg-deviation', m.avg_deviation ? m.avg_deviation.toFixed(3) : '0.000');

    const score = computeFormScore(m);
    setText('form-score', score.toFixed(1));
    const scoreEl = document.getElementById('form-score');
    if (scoreEl) scoreEl.style.color = score >= 7 ? 'var(--green)' : score >= 5 ? 'var(--amber)' : 'var(--red)';

    const br = m.breathing || {};
    const breathCard = document.getElementById('breathing-card');
    if (breathCard) {
        const headSamples = processedData.some(d => Number(d.device_role) === 3);
        const rightHeavy = (br.right_pct || 0) >= (br.left_pct || 0);
        const showBreathing = registeredHeadDevice && headSamples && (br.breath_count || 0) > 0 && rightHeavy;
        if (showBreathing) {
            breathCard.style.display = '';
            setText('breath-count', br.breath_count);
            setText('breath-rate', br.breaths_per_minute || 0);
            setText('breath-left', (br.left_pct || 0) + '%');
            setText('breath-right', (br.right_pct || 0) + '%');
            setText('breath-roll', (br.avg_roll || 0) + '°');
        } else {
            breathCard.style.display = 'none';
        }
    }

    buildStrokeTable();
    buildIdealComparison();
}

function setWidth(id, pct) { const el = document.getElementById(id); if (el) el.style.width = Math.max(1, pct) + '%'; }

function computeFormScore(m) {
    let s = 5;
    s += Math.min((m.consistency || 0) / 100 * 3, 3);
    const dev = m.avg_deviation || 0;
    if (dev < 0.3) s += 2; else if (dev < 0.7) s += 1; else if (dev > 1) s -= 1;
    const angle = m.avg_entry_angle || 0;
    if (angle >= 15 && angle <= 40) s += 1; else if (angle > 0) s -= 0.5;
    return Math.max(0, Math.min(10, s));
}

function drawGauge(canvasId, value, min, max, idealLow, idealHigh) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h - 10, r = Math.min(cx, cy) - 10;
    ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, 0); ctx.lineWidth = 14; ctx.strokeStyle = '#1a1a2e'; ctx.stroke();
    // Ideal zone
    const a1 = Math.PI + (idealLow - min) / (max - min) * Math.PI;
    const a2 = Math.PI + (idealHigh - min) / (max - min) * Math.PI;
    ctx.beginPath(); ctx.arc(cx, cy, r, a1, a2); ctx.lineWidth = 14; ctx.strokeStyle = 'rgba(34,197,94,0.25)'; ctx.stroke();
    // Value arc
    const pct = Math.max(0, Math.min(1, (value - min) / (max - min)));
    const valAngle = Math.PI + pct * Math.PI;
    ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, valAngle); ctx.lineWidth = 14;
    ctx.strokeStyle = (value >= idealLow && value <= idealHigh) ? '#22c55e' : (value > idealHigh ? '#ef4444' : '#eab308');
    ctx.stroke();
    // Needle
    const nx = cx + (r - 8) * Math.cos(valAngle), ny = cy + (r - 8) * Math.sin(valAngle);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(nx, ny); ctx.lineWidth = 2.5; ctx.strokeStyle = '#fff'; ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fillStyle = '#C5A55A'; ctx.fill();
}

function buildStrokeTable() {
    const tbody = document.getElementById('stroke-table-body');
    if (!tbody || !processedData.length) return;
    let rows = [];
    const startTime = processedData[0].timestamp || 0;

    if (sessionMetrics && sessionMetrics.stroke_breakdown && sessionMetrics.stroke_breakdown.length > 0) {
        rows = sessionMetrics.stroke_breakdown.map(p => ({
            num: p.number, 
            time: Math.max(0, p.timestamp_s - (startTime / 1000)).toFixed(1), 
            angle: p.entry_angle || 0, 
            haptic: p.haptic_fired, 
            deviation: p.deviation || 0
        }));
    } else {
        let lastCount = 0;
        for (let i = 0; i < processedData.length; i++) {
            const p = processedData[i];
            if (p.stroke_count > lastCount) {
                let angle = p.entry_angle || 0;
                let deviation = p.deviation_score || 0;
                let haptic = p.haptic_fired;
                // Scan forward to find deviation/haptic/entry_angle set on integration-complete frame
                const scanLimit = Math.min(i + 120, processedData.length);
                for (let j = i + 1; j < scanLimit; j++) {
                    const q = processedData[j];
                    if (q.stroke_count > p.stroke_count) break;
                    if (q.deviation_score > 0 && deviation === 0) deviation = q.deviation_score;
                    if (q.haptic_fired) haptic = true;
                    if (q.entry_angle > 0 && angle === 0) angle = q.entry_angle;
                }
                rows.push({ 
                    num: p.stroke_count, 
                    time: Math.max(0, (p.timestamp - startTime) / 1000).toFixed(1), 
                    angle: angle, 
                    haptic: haptic, 
                    deviation: deviation 
                });
                lastCount = p.stroke_count;
            }
        }
    }
    tbody.innerHTML = rows.map(r =>
        `<tr class="${r.haptic ? 'row-haptic' : ''}" style="cursor:pointer;" onclick="jumpToStroke(${r.num})"><td>${r.num}</td><td>${r.time}s</td><td>${r.angle.toFixed(1)}°</td><td>${r.haptic ? '<span class="haptic-marker">⚡</span>' : '✓'}</td><td class="${r.deviation > 0.7 ? 'text-red' : r.deviation > 0.3 ? 'text-amber' : 'text-green'}">${r.deviation.toFixed(3)}</td><td onclick="event.stopPropagation(); setSingleStrokeAsIdeal(${r.num})"><button class="btn btn-gold btn-sm" style="padding:2px 6px; font-size:0.7em;">Set Ideal</button></td></tr>`
    ).join('');
}

// ── IDEAL STROKE COMPARISON (TIDR 2-1-1) ──
async function buildIdealComparison() {
    if (!idealStrokeData || !idealStrokeData.length || !processedData.length) {
        const el = document.getElementById('ideal-comparison-content');
        if (el) el.innerHTML = '<p style="color:var(--text3);text-align:center;padding:12px;">No ideal stroke saved. Set one in Settings.</p><div style="height:180px;margin-top:8px;"><canvas id="ideal-compare-chart"></canvas></div>';
        if (window._idealChart) { window._idealChart.destroy(); window._idealChart = null; }
        return;
    }
    const el = document.getElementById('ideal-comparison-content');
    if (!el) return;

    // Compute similarity between ideal and actual LIA profiles
    const actualLIA = processedData.map(d => Math.sqrt(
        (d.acceleration?.ax || 0) ** 2 + (d.acceleration?.ay || 0) ** 2 + (d.acceleration?.az || 0) ** 2
    ));
    const idealLIA = idealStrokeData.map(s => Math.sqrt(
        (s.lia_x || 0) ** 2 + (s.lia_y || 0) ** 2 + (s.lia_z || 0) ** 2
    ));

    // Resample ideal to match actual length for comparison
    const resampled = [];
    for (let i = 0; i < actualLIA.length; i++) {
        const t = idealLIA.length > 1 ? i / (actualLIA.length - 1) * (idealLIA.length - 1) : 0;
        const idx = Math.floor(t);
        const frac = t - idx;
        if (idx >= idealLIA.length - 1) resampled.push(idealLIA[idealLIA.length - 1]);
        else resampled.push(idealLIA[idx] * (1 - frac) + idealLIA[idx + 1] * frac);
    }

    // Compute correlation
    let sumDiff = 0, sumIdeal = 0;
    const len = Math.min(actualLIA.length, resampled.length);
    for (let i = 0; i < len; i++) {
        sumDiff += Math.abs(actualLIA[i] - resampled[i]);
        sumIdeal += resampled[i] || 1;
    }
    const similarity = Math.max(0, Math.min(100, (1 - sumDiff / (sumIdeal || 1)) * 100));
    setText('ideal-similarity', similarity.toFixed(0) + '%');

    // Draw comparison chart
    const ctx = document.getElementById('ideal-compare-chart');
    if (ctx && window.Chart) {
        if (window._idealChart) window._idealChart.destroy();
        const step = Math.max(1, Math.floor(len / 200));
        const labels = [], aData = [], iData = [];
        for (let i = 0; i < len; i += step) {
            labels.push(i);
            aData.push(actualLIA[i]);
            iData.push(resampled[i]);
        }
        window._idealChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels,
                datasets: [
                    { label: 'Your Stroke', data: aData, borderColor: '#fff', borderWidth: 1.5, pointRadius: 0, tension: 0.2 },
                    { label: 'Ideal', data: iData, borderColor: '#C5A55A', borderWidth: 2, borderDash: [5, 3], pointRadius: 0, tension: 0.2 }
                ]
            },
            options: {
                responsive: true, maintainAspectRatio: false, animation: false,
                plugins: { legend: { labels: { color: '#999', font: { size: 11 } } } },
                scales: { x: { display: false }, y: { ticks: { color: '#555' }, grid: { color: '#1a1a25' } } }
            }
        });
    }
}

// ── PROGRESS / INSIGHTS ──
function buildLocalProgressFromSessions() {
    const rows = [];
    for (let i = 0; i < savedSessions.length; i++) {
        const s = savedSessions[i];
        const m = s.metrics;
        if (!m) continue;
        rows.push({
            form_score: computeFormScore(m),
            consistency: m.consistency || 0,
            stroke_rate: m.stroke_rate || 0,
            avg_deviation: m.avg_deviation || 0,
            avg_entry_angle: m.avg_entry_angle || 0,
            stroke_count: m.stroke_count || 0
        });
    }
    return rows;
}

async function loadProgress() {
    const res = await apiGet('/api/progress');
    let data = ((res && res.progress) || []).slice().reverse();
    if (!data.length && hasSavedSessions()) {
        data = buildLocalProgressFromSessions();
    }
    if (!data.length) {
        setText('insights-empty', 'Sync a swim session — progress chart fills from server or saved sessions on this device.');
        return;
    }
    setText('insights-empty', data.length && !((res && res.progress) || []).length
        ? 'Trends from this browser; server sync merges history when available.'
        : '');
    const ctx = document.getElementById('progress-chart');
    if (!ctx || !window.Chart) return;
    if (window._progressChart) window._progressChart.destroy();
    window._progressChart = new Chart(ctx, {
        type: 'line', data: {
            labels: data.map((_, i) => 'Session ' + (i + 1)),
            datasets: [
                { label: 'Form Score', data: data.map(d => d.form_score), borderColor: '#C5A55A', backgroundColor: 'rgba(197,165,90,0.1)', fill: true, tension: 0.4, pointRadius: 5, pointBackgroundColor: '#C5A55A' },
                { label: 'Consistency %', data: data.map(d => d.consistency), borderColor: '#22c55e', tension: 0.4, pointRadius: 4 },
                { label: 'Stroke Rate', data: data.map(d => d.stroke_rate), borderColor: '#3b82f6', tension: 0.4, pointRadius: 3, yAxisID: 'y1' }
            ]
        }, options: {
            responsive: true, maintainAspectRatio: false,
            plugins: { legend: { labels: { color: '#aaa', font: { size: 12 } } } },
            scales: {
                x: { ticks: { color: '#666' }, grid: { color: '#1a1a25' } },
                y: { ticks: { color: '#666' }, grid: { color: '#1a1a25' }, min: 0, max: 10, title: { display: true, text: 'Score', color: '#666' } },
                y1: { position: 'right', ticks: { color: '#666' }, grid: { display: false }, min: 0, title: { display: true, text: 'Strokes/min', color: '#666' } }
            }
        }
    });
}

// ── COACHING INSIGHTS ──
function updateCoachingInsights() {
    let insightPd = processedData;
    let insightMetrics = sessionMetrics;
    if ((!insightPd || !insightPd.length) && hasSavedSessions()) {
        const s = savedSessions[activeSessionIdx >= 0 ? activeSessionIdx : savedSessions.length - 1];
        insightPd = s.processed_data || s.processedData || [];
        insightMetrics = s.metrics || insightMetrics;
    }
    if (!insightMetrics || !insightPd || !insightPd.length) {
        setText('coaching-priorities', 'Sync a session or open a saved session from the Session tab to see coaching insights.');
        setText('stroke-diagnosis-list', 'No issues detected yet.');
        setText('consistency-heatmap', '');
        return;
    }

    const _pdBack = processedData;
    const _smBack = sessionMetrics;
    processedData = insightPd;
    sessionMetrics = insightMetrics;
    refreshStrokeFieldMode();

    const priorityBox = document.getElementById('coaching-priorities');
    const diagBox = document.getElementById('stroke-diagnosis-list');
    const heatBox = document.getElementById('consistency-heatmap');

    let htmlPriorities = '';
    let htmlDiag = '';
    let htmlHeat = '';

    const strokes = [];
    let currentStroke = { num: -1 };

    try {
    processedData.forEach(d => {
        const sNum = strokeNumAt(d);
        if (sNum <= 0) return;
        if (currentStroke.num !== sNum) {
            if (currentStroke.num !== -1) strokes.push(currentStroke);
            currentStroke = { num: sNum, events: [], maxDev: 0, duration: 0 };
        }
        if (d.haptic_fired) {
            currentStroke.events.push({
                reason: d.haptic_reason || 0,
                dev: d.deviation_score || 0,
                dur: d.pull_duration_ms || 0
            });
            if (d.deviation_score > currentStroke.maxDev) currentStroke.maxDev = d.deviation_score;
        }
    });
    if (currentStroke.num !== -1) strokes.push(currentStroke);

    // Heatmap
    if (strokes.length > 0) {
        strokes.forEach(s => {
            let color = 'var(--green)';
            if (s.events.length > 0) {
                if (s.maxDev > 0.8) color = 'var(--red)';
                else color = 'var(--amber)';
            }
            htmlHeat += `<div title="Stroke ${s.num}" style="width:14px;height:14px;border-radius:2px;background-color:${color};cursor:pointer;" onclick="switchTab('session'); setTimeout(()=>jumpToStroke(${s.num}), 50);"></div>`;
        });
    } else {
        htmlHeat = '<p style="color:var(--text3); font-size:12px;">No strokes recorded.</p>';
    }
    if(heatBox) heatBox.innerHTML = htmlHeat;

    // Diagnostics
    let hapticCount = 0;
    let reasonCounts = { 'Pull Too Fast': 0, 'Bad Entry Angle': 0, 'Path Deviation': 0 };

    strokes.forEach(s => {
        if (s.events.length > 0) {
            hapticCount++;
            const ev = s.events[0];
            let msg = '';
            
            // haptic_reason bitfield: 0x01 = DEV_HIGH, 0x02 = ENTRY_BAD, 0x04 = PULL_FAST
            const r = ev.reason;
            if (r & 0x04) { msg = 'Pull phase too fast (' + ev.dur.toFixed(0) + 'ms)'; reasonCounts['Pull Too Fast']++; }
            else if (r & 0x02) { msg = 'Hand entry too shallow/steep'; reasonCounts['Bad Entry Angle']++; }
            else if (r & 0x01) { msg = 'Stroke path deviated highly'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.75) { msg = 'High deviation vs ideal stroke path'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.4) { msg = 'Moderate path deviation — refine pull line'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.15) { msg = 'Entry or timing cue — check angle of attack'; reasonCounts['Bad Entry Angle']++; }
            else { msg = 'Form cue — alignment and rhythm'; reasonCounts['Bad Entry Angle']++; }

            htmlDiag += `
                <div style="background:var(--bg3); padding:8px 10px; border-radius:6px; margin-bottom:6px; font-size:0.85em;">
                    <strong style="color:var(--text1);">Stroke ${s.num}</strong>: <span style="color:var(--amber);">${msg}</span>
                    <button class="btn btn-outline btn-sm" style="float:right; padding:2px 8px; font-size:0.8em;" onclick="switchTab('session'); setTimeout(()=>jumpToStroke(${s.num}), 50)">View</button>
                    <div style="clear:both;"></div>
                </div>`;
        }
    });
    if (!htmlDiag) htmlDiag = '<p style="color:var(--text3); font-size:0.9em;">Perfect! No issues detected in this session.</p>';
    if(diagBox) diagBox.innerHTML = htmlDiag;

    // Priorities
    if (hapticCount === 0) {
        htmlPriorities = `<div style="padding:10px; background:rgba(34,197,94,0.1); color:var(--green); border-radius:6px; font-weight:500;">Your form is looking solid. Keep focusing on consistency!</div>`;
    } else {
        const topIssue = Object.keys(reasonCounts).reduce((a, b) => reasonCounts[a] > reasonCounts[b] ? a : b);
        let advice = '';
        if (topIssue === 'Pull Too Fast') advice = 'Slow down your pull phase to engage more water. Rushing the pull drops your efficiency.';
        else if (topIssue === 'Bad Entry Angle') advice = 'Focus on a clean, fingertips-first entry. Keep your elbow high as you pierce the water.';
        else if (topIssue === 'Path Deviation') advice = 'Your hand is drifting from the ideal straight-line pull under your body. Keep it anchored.';
        else advice = 'Focus on fingertips-first entry and a steady catch–pull line.';

        let numBadStrokes = strokes.filter(s => s.events.length > 0).length;
        htmlPriorities = `
            <div style="padding:10px; background:var(--bg3); border-left:3px solid var(--amber); border-radius:4px; margin-bottom:6px;">
                <h4 style="margin:0 0 4px 0; color:var(--text1); font-size:1em;">1. ${topIssue}</h4>
                <p style="margin:0; font-size:0.85em; color:var(--text2);">${advice}</p>
            </div>
            <div style="font-size:0.85em; color:var(--text3); margin-top:8px;">
                Affected ${numBadStrokes} of ${strokes.length} strokes (${Math.round((numBadStrokes/strokes.length)*100)}%).
            </div>
        `;
    }
    if(priorityBox) priorityBox.innerHTML = htmlPriorities;
    } finally {
        processedData = _pdBack;
        sessionMetrics = _smBack;
        refreshStrokeFieldMode();
    }
}

// ── IDEAL STROKE ──
async function loadIdealStroke() {
    const res = await apiGet('/api/ideal_stroke');
    if (res && res.samples && res.samples.length) {
        idealStrokeData = res.samples;
        setText('ideal-status', `Loaded: ${res.samples.length} samples`);
        cacheIdealLocal(res.samples, sessionMetrics ? sessionMetrics.avg_entry_angle : 30, res.name || 'Ideal');
        return;
    }
    try {
        const raw = localStorage.getItem(LS_IDEAL_KEY);
        if (raw) {
            const o = JSON.parse(raw);
            if (o.samples && o.samples.length) {
                idealStrokeData = o.samples;
                setText('ideal-status', `Loaded: ${o.samples.length} samples (saved on this device)`);
                return;
            }
        }
    } catch (e) { /* ignore */ }
    idealStrokeData = null;
    setText('ideal-status', 'No ideal stroke saved');
}

async function afterIdealSavedServer() {
    await loadIdealStroke();
    buildIdealComparison();
    if (isDeviceOnline) pushIdealToDevice(true);
}

async function setSingleStrokeAsIdeal(strokeNum) {
    if (!processedData || !processedData.length) return;
    refreshStrokeFieldMode();
    const samples = buildIdealLiaSamplesFromStroke(strokeNum);
    if (!samples.length) {
        showToast('No samples found for stroke ' + strokeNum, 'error');
        return;
    }
    const avgAngle = averageEntryAngleForStroke(strokeNum);
    const currentSession = savedSessions[activeSessionIdx];

    if (!currentSession || !currentSession.id) {
        idealStrokeData = samples;
        cacheIdealLocal(samples, avgAngle, 'Stroke ' + strokeNum);
        buildIdealComparison();
        showToast('Ideal saved on this device — use Save session when on dashboard WiFi to store on the server', 'info');
        return;
    }

    let res = await apiPost('/api/ideal_stroke/set_from_stroke', {
        session_id: Number(currentSession.id),
        stroke_num: Number(strokeNum),
        ideal_entry_angle: avgAngle
    });

    if (res && res.status === 'ok') {
        showToast('Stroke ' + strokeNum + ' set as ideal baseline', 'success');
        await afterIdealSavedServer();
        return;
    }

    res = await apiPost('/api/ideal_stroke', {
        name: 'Stroke ' + strokeNum + ' (session ' + currentSession.id + ')',
        samples,
        ideal_entry_angle: avgAngle
    });

    if (res && res.status === 'ok') {
        showToast('Stroke ' + strokeNum + ' saved as ideal (direct upload)', 'success');
        await afterIdealSavedServer();
        return;
    }

    idealStrokeData = samples;
    cacheIdealLocal(samples, avgAngle, 'Stroke ' + strokeNum);
    buildIdealComparison();
    const err = (res && res.error) ? res.error : 'Could not reach dashboard';
    showToast('Saved ideal on this device only: ' + err, 'warn');
}

async function setCurrentAsIdeal() {
    if (!processedData.length || !sessionMetrics || sessionMetrics.stroke_count < 1) { showToast('Record a session with strokes first', 'error'); return; }
    const samples = processedData.map(p => ({ lia_x: p.acceleration?.ax || 0, lia_y: p.acceleration?.ay || 0, lia_z: p.acceleration?.az || 0 }));

    const avgAngle = sessionMetrics ? sessionMetrics.avg_entry_angle : 30.0;

    const res = await apiPost('/api/ideal_stroke', { name: 'From Session', samples, ideal_entry_angle: avgAngle });
    if (res && res.status === 'ok') {
        await loadIdealStroke();
        showToast('Ideal stroke saved! Pushing to device...', 'success');
        await pushIdealToDevice(true);
        return;
    }
    idealStrokeData = samples;
    cacheIdealLocal(samples, avgAngle, 'From Session');
    buildIdealComparison();
    showToast('Saved ideal on this device only: ' + ((res && res.error) || 'offline'), 'warn');
}
async function pushIdealToDevice(silent = false) {
    if (!idealStrokeData || !idealStrokeData.length) {
        if (!silent) showToast('No ideal stroke to push', 'error');
        return;
    }
    if (!isDeviceOnline && !silent) {
        pendingIdealSync = true;
        showToast('Device offline. Ideal stroke queued for next connection.', 'info');
        return;
    } else if (!isDeviceOnline) {
        pendingIdealSync = true;
        return;
    }
    const result = await apiPost('/api/ideal_stroke/push', { 
        samples: idealStrokeData,
        ideal_entry_angle: sessionMetrics ? sessionMetrics.avg_entry_angle : 30.0
    });
    if (!silent) {
        showToast(result && result.status === 'ok' ? 'Pushed ideal to device!' : 'Failed: ' + ((result && result.error) || 'unknown'), result && result.status === 'ok' ? 'success' : 'error');
    }
}

async function deleteIdealStroke() {
    if (!confirm("Delete ideal stroke? This will clear it from the app and device.")) return;
    const result = await apiPost('/api/ideal_stroke/delete', {});
    if (result && result.status === 'ok') {
        idealStrokeData = null;
        try { localStorage.removeItem(LS_IDEAL_KEY); } catch (e) { /* */ }
        setText('ideal-status', 'No ideal stroke saved');
        showToast('Ideal stroke deleted', 'success');
    } else {
        try { localStorage.removeItem(LS_IDEAL_KEY); } catch (e) { /* */ }
        idealStrokeData = null;
        setText('ideal-status', 'No ideal stroke saved');
        showToast('Removed from this device. Server: ' + ((result && result.error) || 'unreachable'), 'warn');
    }
}

async function pushUserConfigToDevice(silent = false) {
    const wingspan = document.getElementById('settings-wingspan') ? parseFloat(document.getElementById('settings-wingspan').value) : 180;
    const height = document.getElementById('settings-height') ? parseFloat(document.getElementById('settings-height').value) : 180;
    const skill = document.getElementById('settings-skill') ? document.getElementById('settings-skill').value : 'beginner';
    const poolLength = document.getElementById('settings-pool-length') ? parseFloat(document.getElementById('settings-pool-length').value) : 25;
    
    if (!isDeviceOnline && !silent) {
        pendingConfigSync = true;
        showToast('Device offline. Profile queued for next connection.', 'info');
        return;
    } else if (!isDeviceOnline) {
        pendingConfigSync = true;
        return;
    }

    const result = await apiPost('/api/user_config/push', {
        wingspan_cm: isNaN(wingspan) ? 180 : wingspan,
        height_cm: isNaN(height) ? 180 : height,
        skill_level: skill,
        pool_length: isNaN(poolLength) ? 25 : poolLength
    });
    
    if (!silent) {
        if (result && result.status === 'ok') {
            showToast('Profile pushed to device', 'success');
        } else {
            showToast('Failed to push profile: ' + ((result && result.error) || 'unknown'), 'error');
        }
    }
}

async function testHapticDevice() {
    const result = await apiPost('/api/test_buzz', {});
    if (result && result.status === 'ok') {
        showToast('Test buzz sent', 'success');
    } else {
        showToast('Test buzz failed: ' + ((result && result.error) || 'unknown'), 'error');
    }
}

// ── CHARTS ──
let accelChart, gyroChart;
function initCharts() {
    if (!window.Chart) return;
    const accelCtx = document.getElementById('accelChart');
    const gyroCtx = document.getElementById('gyroChart');
    if (!accelCtx || !gyroCtx) return;
    if (accelChart) accelChart.destroy();
    if (gyroChart) gyroChart.destroy();
    const chartOpts = {
        responsive: true, maintainAspectRatio: false, animation: false,
        elements: { point: { radius: 0 }, line: { borderWidth: 1.5 } },
        plugins: { legend: { labels: { color: '#888', font: { size: 10 } } } },
        scales: { x: { display: false }, y: { ticks: { color: '#555', font: { size: 10 } }, grid: { color: '#1a1a25' } } }
    };
    accelChart = new Chart(accelCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'X', data: [], borderColor: '#ef4444' },
                { label: 'Y', data: [], borderColor: '#22c55e' },
                { label: 'Z', data: [], borderColor: '#3b82f6' }
            ]
        }, options: chartOpts
    });
    gyroChart = new Chart(gyroCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'X', data: [], borderColor: '#f97316' },
                { label: 'Y', data: [], borderColor: '#a855f7' },
                { label: 'Z', data: [], borderColor: '#06b6d4' }
            ]
        }, options: chartOpts
    });
}

function updateCharts(idx) {
    if (!accelChart || !gyroChart) return;
    const windowSize = 200;
    const start = Math.max(0, idx - windowSize);
    const slice = processedData.slice(start, idx + 1);
    const labels = slice.map((_, i) => i);
    accelChart.data.labels = labels;
    accelChart.data.datasets[0].data = slice.map(d => d.acceleration?.ax || 0);
    accelChart.data.datasets[1].data = slice.map(d => d.acceleration?.ay || 0);
    accelChart.data.datasets[2].data = slice.map(d => d.acceleration?.az || 0);
    accelChart.update('none');
    gyroChart.data.labels = labels;
    gyroChart.data.datasets[0].data = slice.map(d => d.angular_velocity?.gx || 0);
    gyroChart.data.datasets[1].data = slice.map(d => d.angular_velocity?.gy || 0);
    gyroChart.data.datasets[2].data = slice.map(d => d.angular_velocity?.gz || 0);
    gyroChart.update('none');
}

// ══════════════════════════════════════════════════════════════════
//  3D VISUALIZATION — Realistic hand, LIA-integrated path, phase-colored trail
// ══════════════════════════════════════════════════════════════════
let scene, camera, renderer, controls;
/** Single rAF loop for Three.js — cancel on WebGL context loss to avoid stacked loops / flicker. */
let vizRafId = null;
let handGroup = null;
let handMaterials = [];
let trailLine = null;
let strokeMarkerGroup = null;
let hapticFlashUntil = 0;
/** Throttle live calibration strip updates during playback/scrub (was every frame → layout churn). */
let lastCalStripFrameIdx = -1;
let integratedPositions = [];
let lastRenderedStrokeCount = 0;
let playbackStrokeSegments = [];

const SKIN_COLOR = 0xc9a088;
const SKIN_COLOR_SHADOW = 0x9a7359;
const TRAIL_MAX_POINTS = 2000;
const TRAIL_SMOOTH_WINDOW = 7;
/** Base interval for playback (ms); combined with speed slider. */
const PLAYBACK_BASE_MS = 18;
/** 'kinematic' = canonical+LIA blend; 'position' = smoothed IMU position stream (analysis). */
let vizStreamMode = 'kinematic';
/** 0 = full session; else play only this stroke #. */
let playbackStrokeFilter = 0;
let loopStrokePlayback = false;
/** 0.25–4; multiplies playback speed. */
let playbackSpeedMultiplier = 1;
/** When playing all strokes, loop entire session. */
let loopFullSession = false;
/** Smoothed position-only path (pool-framed), parallel to integratedPositions. */
let positionStreamPositions = [];
const PHASE_COLOR_HEX = {
    glide: 0x3b82f6, catch: 0x22c55e, pull: 0xf97316,
    recovery: 0xa855f7, idle: 0x666666
};
const PHASE_COLOR_RGB = {
    glide: [0.23, 0.51, 0.96], catch: [0.13, 0.77, 0.37],
    pull: [0.98, 0.45, 0.09], recovery: [0.66, 0.33, 0.97],
    idle: [0.4, 0.4, 0.4]
};

/** Prefer firmware `strokes` when present so motion matches device logs. */
let useFwStrokesForViz = false;
function refreshStrokeFieldMode() {
    useFwStrokesForViz = processedData.some(d => (d.strokes || 0) > 0);
}
function strokeNumAt(d) {
    return useFwStrokesForViz ? (d.strokes || 0) : (d.stroke_count || 0);
}

function buildIdealLiaSamplesFromStroke(strokeNum) {
    const out = [];
    if (!processedData || !processedData.length) return out;
    refreshStrokeFieldMode();
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        if (strokeNumAt(d) !== strokeNum) continue;
        const acc = d.acceleration || {};
        const q = d.quaternion || {};
        out.push({
            lia_x: acc.ax || 0,
            lia_y: acc.ay || 0,
            lia_z: acc.az || 0,
            qw: q.qw != null ? q.qw : 1,
            qx: q.qx || 0,
            qy: q.qy || 0,
            qz: q.qz || 0,
            entry_angle: d.entry_angle || 0
        });
    }
    return out;
}

function averageEntryAngleForStroke(strokeNum) {
    let sum = 0, n = 0;
    refreshStrokeFieldMode();
    for (const d of processedData) {
        if (strokeNumAt(d) !== strokeNum) continue;
        const ea = Number(d.entry_angle) || 0;
        if (ea > 0.5) {
            sum += ea;
            n++;
        }
    }
    if (n) return sum / n;
    return sessionMetrics ? (sessionMetrics.avg_entry_angle || 30) : 30;
}

function cacheIdealLocal(samples, idealEntryAngle, name) {
    try {
        localStorage.setItem(LS_IDEAL_KEY, JSON.stringify({
            samples,
            ideal_entry_angle: idealEntryAngle,
            name: name || 'Ideal'
        }));
    } catch (e) { /* quota */ }
}

/** Raw LIA / processor positions (session-relative). */
let rawIntegratedPositions = [];

/** Lateral separation between stroke columns (world X). */
const VIZ_STROKE_X_SPREAD = 0.14;
/** How much real LIA integration is mixed in (rest is canonical full-stroke arc). */
const VIZ_CANONICAL_BLEND = 0.88;
/**
 * Pool frame: world +Z is down-lane (away from starting wall). Canonical stroke path is offset so
 * stroke 1 starts near the wall and each stroke advances along the lane (research-style: inertial path
 * blended with body-fixed kinematics, lane-aligned for readability).
 */
const POOL_FRAME = { zWall: -4.25, zPerStroke: 0.52, zAlongU: 0.22 };

/**
 * One freestyle arm cycle in normalized time u ∈ [0,1], keyed in meters before * positionScale.
 * Order: above water → pull under → recovery over → entry at angle → (u=1 meets next stroke reset).
 * Matches coach visualization: full path is visible each stroke, not only IMU rotation.
 */
const STROKE_PATH_KEYS = [
    { u: 0.0,  x: 0.0,  y: 0.38, z: -0.14 },
    { u: 0.14, x: 0.02, y: 0.32, z: -0.10 },
    { u: 0.32, x: 0.04, y: -0.06, z: 0.0 },
    { u: 0.48, x: 0.05, y: -0.22, z: 0.08 },
    { u: 0.62, x: 0.04, y: -0.20, z: 0.12 },
    { u: 0.76, x: 0.0,  y: 0.34, z: 0.06 },
    { u: 0.88, x: -0.02, y: 0.12, z: -0.04 },
    { u: 1.0,  x: 0.0,  y: 0.38, z: -0.14 }
];

function sampleStrokeCanonical(u) {
    const s = positionScale;
    const uu = Math.max(0, Math.min(1, u));
    for (let k = 0; k < STROKE_PATH_KEYS.length - 1; k++) {
        const a = STROKE_PATH_KEYS[k];
        const b = STROKE_PATH_KEYS[k + 1];
        if (uu >= a.u && uu <= b.u) {
            const t = b.u > a.u ? (uu - a.u) / (b.u - a.u) : 0;
            const tt = t * t * (3 - 2 * t);
            const x = THREE.MathUtils.lerp(a.x, b.x, tt);
            const y = THREE.MathUtils.lerp(a.y, b.y, tt);
            const z = THREE.MathUtils.lerp(a.z, b.z, tt);
            return new THREE.Vector3(x * s, y * s, z * s);
        }
    }
    const last = STROKE_PATH_KEYS[STROKE_PATH_KEYS.length - 1];
    return new THREE.Vector3(last.x * s, last.y * s, last.z * s);
}

function getSegEnd(upToIndex) {
    if (!processedData.length || upToIndex < 0) return 0;
    refreshStrokeFieldMode();
    const sc = strokeNumAt(processedData[upToIndex] || {});
    for (let j = upToIndex + 1; j < processedData.length; j++) {
        if (strokeNumAt(processedData[j]) > sc) return j - 1;
    }
    return processedData.length - 1;
}

function firstStrokeSampleIndex() {
    for (let i = 0; i < processedData.length; i++) {
        if (strokeNumAt(processedData[i]) > 0) return i;
    }
    return -1;
}

/** Normalized progress within current stroke [0,1] — drives canonical arc + reset each stroke. */
function strokeProgressU(i) {
    if (!processedData.length) return 0;
    const sc = strokeNumAt(processedData[i]);
    if (sc === 0) {
        const fs = firstStrokeSampleIndex();
        if (fs <= 0) return 0;
        return Math.min(0.1, (i / fs) * 0.1);
    }
    const segStart = getSegStart(i);
    const segEnd = getSegEnd(i);
    const denom = Math.max(1, segEnd - segStart);
    return (i - segStart) / denom;
}

/** Best entry angle (deg) for sample (device reports late; scan backward). */
function entryAngleForSample(i) {
    const d = processedData[i];
    let ea = Number(d.entry_angle) || 0;
    if (ea > 0.5) return ea;
    for (let j = i; j >= Math.max(0, i - 80); j--) {
        const v = Number(processedData[j].entry_angle) || 0;
        if (v > 0.5) return v;
    }
    const ideal = (sessionMetrics && Number(sessionMetrics.ideal_entry_angle)) || 30;
    return ideal;
}

/** Nudge path during entry window using reported angle (deg). */
function entryAnglePathNudge(u, entryDeg) {
    const s = positionScale;
    if (u < 0.74 || u > 0.98) return new THREE.Vector3(0, 0, 0);
    const w = Math.sin((u - 0.74) / 0.24 * Math.PI);
    const delta = (entryDeg - 30) / 45;
    return new THREE.Vector3(
        -delta * 0.04 * w * s,
        delta * 0.06 * w * s,
        delta * 0.03 * w * s
    );
}

function nq(q) {
    if (!q) return new THREE.Quaternion(0, 0, 0, 1);
    const w = q.qw ?? 1, x = q.qx ?? 0, y = q.qy ?? 0, z = q.qz ?? 0;
    const m = Math.hypot(w, x, y, z);
    return m > 1e-8 ? new THREE.Quaternion(x / m, y / m, z / m, w / m) : new THREE.Quaternion(0, 0, 0, 1);
}
function buildRawIntegratedPositions() {
    rawIntegratedPositions = [];
    if (!processedData.length) return;
    refreshStrokeFieldMode();

    const hasProcessorPos = processedData.length > 0 && processedData.every(d =>
        d.position && typeof d.position.px === 'number'
    );
    if (hasProcessorPos) {
        let prevStroke = strokeNumAt(processedData[0]);
        for (let i = 0; i < processedData.length; i++) {
            const d = processedData[i];
            const sc = strokeNumAt(d);
            if (sc > prevStroke) prevStroke = sc;
            const p = d.position;
            rawIntegratedPositions.push(new THREE.Vector3(
                (p.px || 0) * positionScale,
                (p.py || 0) * positionScale,
                (p.pz || 0) * positionScale
            ));
        }
        return;
    }

    let vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
    let prevStroke = strokeNumAt(processedData[0]);
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const sc = strokeNumAt(d);
        if (sc > prevStroke) {
            vx = vy = vz = 0;
            px = py = pz = 0;
            prevStroke = sc;
        }
        let dt = 0.02;
        if (i > 0 && d.timestamp != null && processedData[i - 1].timestamp != null) {
            dt = Math.max(0.001, Math.min(0.1, (d.timestamp - processedData[i - 1].timestamp) / 1000));
        }
        let ax = 0, ay = 0, az = 0;
        if (d.lia) { ax = d.lia.x || 0; ay = d.lia.y || 0; az = d.lia.z || 0; }
        else if (d.acceleration) { ax = d.acceleration.ax || 0; ay = d.acceleration.ay || 0; az = d.acceleration.az || 0; }
        const wA = new THREE.Vector3(ax, ay, az).applyQuaternion(nq(d.quaternion));
        vx += wA.x * dt; vy += wA.y * dt; vz += wA.z * dt;
        vx *= 0.97; vy *= 0.97; vz *= 0.97;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        rawIntegratedPositions.push(new THREE.Vector3(px * positionScale, py * positionScale, pz * positionScale));
    }
}

/** Per-stroke: canonical full stroke arc + small LIA residual + entry-angle nudge. */
function applyStrokeAnchoredDisplay() {
    integratedPositions = [];
    if (!rawIntegratedPositions.length || rawIntegratedPositions.length !== processedData.length) return;
    refreshStrokeFieldMode();
    for (let i = 0; i < processedData.length; i++) {
        const segStart = getSegStart(i);
        const r0 = rawIntegratedPositions[segStart];
        const ri = rawIntegratedPositions[i];
        const strokeN = Math.max(1, strokeNumAt(processedData[segStart]) || 1);
        const spread = ((strokeN - 1) % 7 - 3) * VIZ_STROKE_X_SPREAD * positionScale;
        const u = strokeProgressU(i);
        const canon = sampleStrokeCanonical(u);
        const delta = new THREE.Vector3().subVectors(ri, r0);
        const ea = entryAngleForSample(i);
        const nudge = entryAnglePathNudge(u, ea);
        const pos = canon.clone()
            .multiplyScalar(VIZ_CANONICAL_BLEND)
            .add(delta.multiplyScalar(1.0 - VIZ_CANONICAL_BLEND))
            .add(new THREE.Vector3(spread, 0, 0))
            .add(nudge);
        pos.z += POOL_FRAME.zWall + (strokeN - 1) * POOL_FRAME.zPerStroke + u * POOL_FRAME.zAlongU;
        integratedPositions.push(pos);
    }
}

function integratePositions() {
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    if (!processedData.length) return;
    buildRawIntegratedPositions();
    if (!playbackStrokeSegments.length) buildPlaybackStrokeSegments();
    applyStrokeAnchoredDisplay();
    buildPositionStreamPositions();
}
function getSegStart(upToIndex) {
    if (!processedData.length || upToIndex < 0) return 0;
    refreshStrokeFieldMode();
    const sc = strokeNumAt(processedData[upToIndex] || {});
    for (let i = upToIndex; i >= 0; i--) {
        if (strokeNumAt(processedData[i] || {}) < sc) return i + 1;
    }
    return 0;
}
function smoothTrailPoints(points, windowSize) {
    if (!points.length || windowSize < 2) return points;
    const half = Math.floor(windowSize / 2);
    const out = [];
    for (let i = 0; i < points.length; i++) {
        let x = 0, y = 0, z = 0, n = 0;
        for (let j = Math.max(0, i - half); j <= Math.min(points.length - 1, i + half); j++) {
            x += points[j].x; y += points[j].y; z += points[j].z;
            n++;
        }
        out.push(new THREE.Vector3(x / n, y / n, z / n));
    }
    return out;
}

/** Forward-fill invalid samples (missing / NaN) — common when GPS or pose drops. */
function fillMissingRawPositions(points) {
    const out = points.map(p => (p && p.clone) ? p.clone() : new THREE.Vector3());
    let lastValid = -1;
    for (let i = 0; i < out.length; i++) {
        const p = out[i];
        if (p && isFinite(p.x) && isFinite(p.y) && isFinite(p.z)) {
            lastValid = i;
        } else if (lastValid >= 0) {
            p.copy(out[lastValid]);
        }
    }
    for (let i = out.length - 1; i >= 0; i--) {
        const p = out[i];
        if (p && isFinite(p.x) && isFinite(p.y) && isFinite(p.z)) break;
        let j = i + 1;
        while (j < out.length && out[j] && (!isFinite(out[j].x))) j++;
        if (j < out.length) p.copy(out[j]);
    }
    return out;
}

/** Smooth each stroke segment separately so boundaries stay sharp (freestyle-like arcs). */
function smoothRawPathPerStroke(points) {
    if (!points.length) return points;
    const w = Math.min(11, Math.max(3, Math.floor(points.length / 8) * 2 + 1));
    if (!playbackStrokeSegments.length) {
        return smoothTrailPoints(points, w);
    }
    const out = points.map(p => p.clone());
    for (const seg of playbackStrokeSegments) {
        const { startIdx, endIdx } = seg;
        if (endIdx < startIdx) continue;
        const slice = [];
        for (let i = startIdx; i <= endIdx; i++) slice.push(out[i]);
        const sm = smoothTrailPoints(slice, Math.min(w, slice.length));
        for (let k = 0; k < sm.length; k++) {
            out[startIdx + k].copy(sm[k]);
        }
    }
    return out;
}

function applyPoolFrameOnly(i, vec) {
    const segStart = getSegStart(i);
    const strokeN = Math.max(1, strokeNumAt(processedData[segStart]) || 1);
    const spread = ((strokeN - 1) % 7 - 3) * VIZ_STROKE_X_SPREAD * positionScale;
    const u = strokeProgressU(i);
    const ea = entryAngleForSample(i);
    const nudge = entryAnglePathNudge(u, ea);
    const pos = vec.clone().add(new THREE.Vector3(spread, 0, 0)).add(nudge);
    pos.z += POOL_FRAME.zWall + (strokeN - 1) * POOL_FRAME.zPerStroke + u * POOL_FRAME.zAlongU;
    return pos;
}

function buildPositionStreamPositions() {
    positionStreamPositions = [];
    if (!rawIntegratedPositions.length || rawIntegratedPositions.length !== processedData.length) return;
    refreshStrokeFieldMode();
    let filled = fillMissingRawPositions(rawIntegratedPositions);
    filled = smoothRawPathPerStroke(filled);
    for (let i = 0; i < processedData.length; i++) {
        positionStreamPositions.push(applyPoolFrameOnly(i, filled[i]));
    }
}

/** Positions used for hand, trail, and camera follow. */
function currentAnimationPositions() {
    return vizStreamMode === 'position' ? positionStreamPositions : integratedPositions;
}

/**
 * Freestyle-like trail: moving average + optional Catmull-Rom resample for a continuous curve.
 */
function resampleTrailFreestyle(rawPts, colors) {
    const pre = smoothTrailPoints(rawPts, TRAIL_SMOOTH_WINDOW);
    if (pre.length < 4 || typeof THREE === 'undefined' || !THREE.CatmullRomCurve3) {
        return { points: pre, colors: colors.slice(0, pre.length * 3) };
    }
    try {
        const curve = new THREE.CatmullRomCurve3(pre.map(p => p.clone()));
        const n = Math.min(420, Math.max(pre.length * 5, 48));
        const sampled = curve.getPoints(n);
        const newColors = [];
        for (let i = 0; i < sampled.length; i++) {
            const t = sampled.length > 1 ? i / (sampled.length - 1) : 0;
            const origIdx = t * (pre.length - 1);
            const i0 = Math.floor(origIdx);
            const i1 = Math.min(pre.length - 1, i0 + 1);
            const f = origIdx - i0;
            const o0 = i0 * 3, o1 = i1 * 3;
            newColors.push(
                colors[o0] * (1 - f) + colors[o1] * f,
                colors[o0 + 1] * (1 - f) + colors[o1 + 1] * f,
                colors[o0 + 2] * (1 - f) + colors[o1 + 2] * f
            );
        }
        return { points: sampled, colors: newColors };
    } catch (e) {
        return { points: pre, colors: colors.slice(0, pre.length * 3) };
    }
}

function buildPlaybackStrokeSegments() {
    playbackStrokeSegments = [];
    refreshStrokeFieldMode();
    let last = 0, start = 0;
    for (let i = 0; i < processedData.length; i++) {
        const c = strokeNumAt(processedData[i]);
        if (c > last) {
            if (last > 0 && start < i) playbackStrokeSegments.push({ startIdx: start, endIdx: i - 1, strokeNum: last });
            start = i;
            last = c;
        }
    }
    if (last > 0 && start < processedData.length)
        playbackStrokeSegments.push({ startIdx: start, endIdx: processedData.length - 1, strokeNum: last });
}

let poolEnvironment = null;
let swimmerBody = null;
let ghostArmGroup = null;
let splashGroup = null;
let velocityArrow = null;
/** Orbit target softly follows the hand so translation stays in frame during playback. */
let followHandInView = true;

function createHandModel() {
    if (typeof THREE === 'undefined') return null;
    const group = new THREE.Group();
    handMaterials = [];
    const skinMat = (hex) => {
        const c = hex != null ? hex : SKIN_COLOR;
        let m;
        if (THREE.MeshPhysicalMaterial) {
            m = new THREE.MeshPhysicalMaterial({
                color: c,
                roughness: 0.42,
                metalness: 0,
                emissive: 0x060403,
                clearcoat: 0.08,
                clearcoatRoughness: 0.45
            });
        } else {
            m = new THREE.MeshStandardMaterial({
                color: c,
                roughness: 0.42,
                metalness: 0.06,
                emissive: 0x060403
            });
        }
        m.userData.baseColor = c;
        handMaterials.push(m);
        return m;
    };

    // Palm: main pad + thenar bulge (reads as a hand, not a disk)
    const palmGeo = new THREE.SphereGeometry(0.046, 32, 24);
    palmGeo.scale(1.85, 0.32, 2.05);
    const palm = new THREE.Mesh(palmGeo, skinMat(SKIN_COLOR));
    palm.position.set(0, 0, 0.01);
    group.add(palm);

    const thenar = new THREE.Mesh(new THREE.SphereGeometry(0.024, 20, 14), skinMat(SKIN_COLOR_SHADOW));
    thenar.position.set(0.028, -0.014, 0.055);
    thenar.scale.set(1.1, 0.65, 1.15);
    group.add(thenar);

    // Wrist / forearm stub (tapered)
    const wristGeo = new THREE.CylinderGeometry(0.028, 0.036, 0.055, 14);
    const wrist = new THREE.Mesh(wristGeo, skinMat(SKIN_COLOR));
    wrist.position.set(0, 0, -0.078);
    wrist.rotation.x = Math.PI / 2;
    group.add(wrist);

    const addFinger = (fx, lengths, r0) => {
        let z = 0.052;
        let r = r0;
        for (let k = 0; k < lengths.length; k++) {
            const len = lengths[k];
            const geo = new THREE.CylinderGeometry(r * 0.88, r, len, 12, 2);
            const seg = new THREE.Mesh(geo, skinMat(SKIN_COLOR));
            seg.position.set(fx, 0, z + len / 2);
            seg.rotation.x = Math.PI / 2;
            group.add(seg);
            const kn = new THREE.Mesh(new THREE.SphereGeometry(r * 0.92, 10, 8), skinMat(SKIN_COLOR_SHADOW));
            kn.position.set(fx, 0, z);
            group.add(kn);
            z += len;
            r *= 0.84;
        }
        const tip = new THREE.Mesh(new THREE.SphereGeometry(r * 1.05, 10, 8), skinMat(SKIN_COLOR));
        tip.position.set(fx, 0, z);
        group.add(tip);
    };

    addFinger(-0.024, [0.034, 0.024, 0.018], 0.0078);
    addFinger(-0.008, [0.040, 0.028, 0.021], 0.0082);
    addFinger(0.008, [0.037, 0.026, 0.019], 0.008);
    addFinger(0.024, [0.030, 0.021, 0.016], 0.0068);

    const thumbGrp = new THREE.Group();
    thumbGrp.position.set(-0.05, -0.008, -0.012);
    thumbGrp.rotation.set(-0.35, -0.12, 0.62);
    const tSegs = [
        { rt: 0.011, rb: 0.012, len: 0.028 },
        { rt: 0.008, rb: 0.011, len: 0.026 }
    ];
    let tz = 0;
    for (const ts of tSegs) {
        const seg = new THREE.Mesh(
            new THREE.CylinderGeometry(ts.rt, ts.rb, ts.len, 12, 2),
            skinMat(SKIN_COLOR_SHADOW)
        );
        seg.position.set(0, 0, tz + ts.len / 2);
        seg.rotation.x = Math.PI / 2;
        thumbGrp.add(seg);
        tz += ts.len;
    }
    const tTip = new THREE.Mesh(new THREE.SphereGeometry(0.008, 10, 8), skinMat(SKIN_COLOR));
    tTip.position.set(0, 0, tz);
    thumbGrp.add(tTip);
    group.add(thumbGrp);

    group.scale.setScalar(2.75);
    return group;
}

function createPoolEnvironment() {
    if (typeof THREE === 'undefined') return null;
    const group = new THREE.Group();

    const pLength = userProfile ? (userProfile.pool_length || 25) : 25;
    const half = pLength / 2;

    // Water surface (+Z = down-lane from the starting wall at -Z)
    const waterGeo = new THREE.PlaneGeometry(2.5, pLength, 12, 64);
    const waterMat = new THREE.MeshStandardMaterial({
        color: 0x0a4a72, emissive: 0x001a2a,
        roughness: 0.15, metalness: 0.2,
        transparent: true, opacity: 0.42, side: THREE.DoubleSide
    });
    const water = new THREE.Mesh(waterGeo, waterMat);
    water.rotation.x = -Math.PI / 2;
    group.add(water);

    // Pool floor (tiled)
    const floorGeo = new THREE.PlaneGeometry(2.5, pLength);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x113344, roughness: 0.8 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = -1.2;
    group.add(floor);

    // Starting wall (pool edge / push-off end) — thin slab at -Z end of the lane
    const wallGeo = new THREE.BoxGeometry(2.7, 1.4, 0.12);
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x3a4a58, roughness: 0.85, metalness: 0.05 });
    const wall = new THREE.Mesh(wallGeo, wallMat);
    wall.position.set(0, 0.15, -half);
    group.add(wall);

    // Lane ropes (parallel to +Z swim direction)
    const ropeGeo = new THREE.CylinderGeometry(0.04, 0.04, pLength, 8);
    const ropeMat = new THREE.MeshStandardMaterial({ color: 0xcc3333, roughness: 0.6 });
    const rope1 = new THREE.Mesh(ropeGeo, ropeMat);
    rope1.rotation.x = Math.PI / 2;
    rope1.position.set(-1.25, 0, 0);
    group.add(rope1);

    const rope2 = new THREE.Mesh(ropeGeo, ropeMat);
    rope2.rotation.x = Math.PI / 2;
    rope2.position.set(1.25, 0, 0);
    group.add(rope2);

    // Direction arrow on pool bottom (subtle): +Z = swim direction
    const arr = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, -1.18, -half + 1.2), 1.8,
        0x4488aa, 0.2, 0.12
    );
    arr.line.material.transparent = true;
    arr.line.material.opacity = 0.35;
    arr.cone.material.transparent = true;
    arr.cone.material.opacity = 0.35;
    group.add(arr);

    return group;
}

function createSwimmerBody() {
    if (typeof THREE === 'undefined') return null;
    const group = new THREE.Group();
    const skinMat = new THREE.MeshStandardMaterial({color: 0xd2a18c, roughness: 0.4, metalness:0.05});
    const capMat = new THREE.MeshStandardMaterial({color: 0x222222, roughness: 0.6});
    
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.12, 16, 16), capMat);
    head.position.set(0, 0.05, -0.2);
    head.scale.set(1, 0.9, 1.15);
    group.add(head);

    const torso = new THREE.Mesh(new THREE.CapsuleGeometry(0.18, 0.45, 12, 16), skinMat);
    torso.rotation.x = Math.PI / 2;
    torso.position.set(0, -0.05, 0.15);
    group.add(torso);

    return group;
}

function createSplash(position) {
    if (!splashGroup || typeof THREE === 'undefined') return;
    const count = 6 + Math.floor(Math.random() * 4);
    for (let i = 0; i < count; i++) {
        const geo = new THREE.SphereGeometry(0.02 + Math.random() * 0.03, 4, 3);
        const mat = new THREE.MeshBasicMaterial({
            color: 0x66ccff, transparent: true, opacity: 0.7
        });
        const drop = new THREE.Mesh(geo, mat);
        drop.position.copy(position);
        drop.position.y = 0.05;
        const angle = Math.random() * Math.PI * 2;
        const speed = 0.02 + Math.random() * 0.04;
        drop.userData = {
            vx: Math.cos(angle) * speed,
            vy: 0.03 + Math.random() * 0.05,
            vz: Math.sin(angle) * speed,
            life: 1.0
        };
        splashGroup.add(drop);
    }
}

function init3D() {
    const canvas = document.getElementById('canvas3d');
    if (!canvas || !window.THREE) return;

    if (scene && renderer && camera) {
        const w = Math.max(canvas.clientWidth || 800, 400);
        const h = Math.max(canvas.clientHeight || 450, 400);
        renderer.setSize(w, h);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        return;
    }

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x06060e);
    scene.fog = new THREE.FogExp2(0x06060e, 0.028);

    const w = Math.max(canvas.clientWidth || 800, 400);
    const h = Math.max(canvas.clientHeight || 450, 400);
    camera = new THREE.PerspectiveCamera(55, w / h, 0.05, 200);
    camera.position.set(2.1, 2.15, 3.4);
    camera.lookAt(0, 0.1, 0.5);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    if (!canvas.dataset.gfWebglListeners) {
        canvas.dataset.gfWebglListeners = '1';
        canvas.addEventListener('webglcontextlost', (e) => {
            e.preventDefault();
            if (vizRafId != null) {
                cancelAnimationFrame(vizRafId);
                vizRafId = null;
            }
            try {
                if (renderer) renderer.dispose();
            } catch (err) { /* ignore */ }
            scene = null;
            camera = null;
            renderer = null;
            controls = null;
        }, false);
        canvas.addEventListener('webglcontextrestored', () => {
            init3D();
        }, false);
    }
    if (THREE.sRGBEncoding !== undefined) renderer.outputEncoding = THREE.sRGBEncoding;

    const gh = new THREE.GridHelper(14, 28, 0x1a1a26, 0x0e0e16);
    gh.position.y = -1.21;
    gh.visible = false;
    scene.add(gh);

    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    trailGeo.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.9, linewidth: 2
    }));
    scene.add(trailLine);

    strokeMarkerGroup = new THREE.Group();
    scene.add(strokeMarkerGroup);

    handGroup = createHandModel();
    if (handGroup) scene.add(handGroup);

    ghostArmGroup = createHandModel();
    if (ghostArmGroup) {
        ghostArmGroup.children.forEach(c => {
            if (c.material) c.material = new THREE.MeshStandardMaterial({ color: 0x88ccff, transparent: true, opacity: 0.15, wireframe: true });
            if (c.children) c.children.forEach(cc => {
                if (cc.material) cc.material = new THREE.MeshStandardMaterial({ color: 0x88ccff, transparent: true, opacity: 0.15, wireframe: true });
            });
        });
        scene.add(ghostArmGroup);
    }

    poolEnvironment = createPoolEnvironment();
    if (poolEnvironment) scene.add(poolEnvironment);

    swimmerBody = createSwimmerBody();
    if (swimmerBody) {
        swimmerBody.position.set(0, 0, POOL_FRAME.zWall + 0.35);
        scene.add(swimmerBody);
    }

    splashGroup = new THREE.Group();
    scene.add(splashGroup);

    velocityArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.22,
        0xff8844, 0.07, 0.045
    );
    velocityArrow.visible = false;
    scene.add(velocityArrow);

    scene.add(new THREE.AmbientLight(0x6688aa, 0.45));
    const dl = new THREE.DirectionalLight(0xfff5e6, 1.05);
    dl.position.set(4, 7, 3.5);
    scene.add(dl);
    const dl2 = new THREE.DirectionalLight(0xaabbff, 0.35);
    dl2.position.set(-3.5, 3, -4);
    scene.add(dl2);
    const rim = new THREE.DirectionalLight(0x4488cc, 0.25);
    rim.position.set(0, -2, 6);
    scene.add(rim);

    if (window.THREE && THREE.OrbitControls) {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.enablePan = true;
        controls.target.set(0, 0.1, 0.5);
        controls.minDistance = 0.35;
        controls.maxDistance = 24;
        controls.maxPolarAngle = Math.PI * 0.92;
        controls.minPolarAngle = 0.08;
    }

    vizRafId = requestAnimationFrame(animate);
}

function animate() {
    if (!renderer || !scene || !camera) {
        vizRafId = null;
        return;
    }
    if (strokeMarkerGroup) {
        for (let i = strokeMarkerGroup.children.length - 1; i >= 0; i--) {
            const m = strokeMarkerGroup.children[i];
            if (m.userData.ripple) {
                m.userData.age = (m.userData.age || 0) + 0.032;
                const g = 1 + m.userData.age * 1.15;
                m.scale.set(g, g, 1);
                m.material.opacity = Math.max(0, 0.9 - m.userData.age * 0.48);
                if (m.material.opacity <= 0.02) {
                    strokeMarkerGroup.remove(m);
                    m.geometry.dispose();
                    m.material.dispose();
                }
            } else {
                m.material.opacity -= 0.015;
                m.scale.multiplyScalar(1.02);
                if (m.material.opacity <= 0) {
                    strokeMarkerGroup.remove(m);
                    m.geometry.dispose();
                    m.material.dispose();
                }
            }
        }
    }
    // Animate splash particles
    if (splashGroup) {
        for (let i = splashGroup.children.length - 1; i >= 0; i--) {
            const drop = splashGroup.children[i];
            const u = drop.userData;
            u.vy -= 0.002; // gravity
            drop.position.x += u.vx;
            drop.position.y += u.vy;
            drop.position.z += u.vz;
            u.life -= 0.025;
            drop.material.opacity = Math.max(0, u.life * 0.7);
            if (u.life <= 0) {
                splashGroup.remove(drop);
                drop.geometry.dispose();
                drop.material.dispose();
            }
        }
    }
    if (handGroup && hapticFlashUntil > 0 && Date.now() > hapticFlashUntil) {
        handMaterials.forEach(m => {
            m.color.setHex(m.userData.baseColor != null ? m.userData.baseColor : SKIN_COLOR);
            m.emissive.setHex(0x060403);
        });
        hapticFlashUntil = 0;
    }
    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
    vizRafId = requestAnimationFrame(animate);
}

// ── RENDER FRAME ──
function renderFrame(idx) {
    if (!processedData.length || idx >= processedData.length) return;
    const d = processedData[idx];

    if (integratedPositions.length !== processedData.length) integratePositions();
    const anim = currentAnimationPositions();
    const pos = anim[idx] || new THREE.Vector3();

    if (handGroup) {
        const showHand = document.getElementById('show-hand');
        handGroup.visible = showHand ? showHand.checked : true;
        const uStroke = strokeProgressU(idx);
        const qBase = nq(d.quaternion);
        const qDisplay = qBase.clone();
        const eaLive = entryAngleForSample(idx);
        if (uStroke >= 0.70 && uStroke <= 0.99 && eaLive > 0.5) {
            const rad = (eaLive - 30) * (Math.PI / 180) * 0.6;
            const qPitch = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), rad);
            qDisplay.multiply(qPitch);
        }
        handGroup.setRotationFromQuaternion(qDisplay);
        handGroup.position.copy(pos);
        const phase = d.stroke_phase || d.phase || 'idle';
        const phaseHex = PHASE_COLOR_HEX[phase] || PHASE_COLOR_HEX.idle;
        if (hapticFlashUntil <= Date.now()) {
            handMaterials.forEach(m => {
                const base = new THREE.Color(m.userData.baseColor != null ? m.userData.baseColor : SKIN_COLOR);
                const tint = new THREE.Color(phaseHex);
                m.color.copy(base.lerp(tint, 0.25));
                m.emissive.setHex(0x060403);
            });
        }
    }

    if (d.haptic_fired && hapticFlashUntil <= Date.now()) {
        hapticFlashUntil = Date.now() + 300;
        handMaterials.forEach(m => {
            m.color.setHex(0xff4444);
            m.emissive.setHex(0x880000);
        });
    }

    if (velocityArrow && idx > 0) {
        const p0 = anim[idx - 1];
        const p1 = pos;
        if (p0 && p1) {
            const delta = new THREE.Vector3().subVectors(p1, p0);
            const len = delta.length();
            if (len > 0.00012) {
                velocityArrow.visible = true;
                const dir = delta.multiplyScalar(1 / len);
                velocityArrow.setDirection(dir);
                const L = Math.min(0.7, len * 85);
                velocityArrow.setLength(L, Math.min(0.12, L * 0.2), Math.min(0.08, L * 0.12));
                velocityArrow.position.copy(p0);
            } else {
                velocityArrow.visible = false;
            }
        }
    } else if (velocityArrow) {
        velocityArrow.visible = false;
    }

    if (controls && followHandInView) {
        controls.target.lerp(pos, 0.14);
    }

    if (swimmerBody) {
        const sc = strokeNumAt(d);
        const strokeN = Math.max(1, sc || 1);
        const spread = ((strokeN - 1) % 7 - 3) * VIZ_STROKE_X_SPREAD * positionScale;
        
        // Offset swimmer body so the correct shoulder meets the arm path
        // Shoulder width is approx 0.18m in our model
        const offset = activeWristDeviceRole === 'wrist_left' ? (0.17 * positionScale) : (-0.17 * positionScale);
        swimmerBody.position.x = spread + offset;
    }

    if (ghostArmGroup) {
        if (idealStrokeData && idealStrokeData.length > 0 && vizStreamMode === 'kinematic') {
            ghostArmGroup.visible = true;
            const uStroke = strokeProgressU(idx);
            const idealIdx = Math.floor(uStroke * (idealStrokeData.length - 1));
            const idealSample = idealStrokeData[idealIdx];
            
            const sc = strokeNumAt(d);
            const strokeN = Math.max(1, sc || 1);
            const spread = ((strokeN - 1) % 7 - 3) * VIZ_STROKE_X_SPREAD * positionScale;
            
            const canon = sampleStrokeCanonical(uStroke);
            const idealEA = idealSample.entry_angle || 30;
            const idealNudge = entryAnglePathNudge(uStroke, idealEA);
            
            let qIdeal = nq(idealSample.quaternion || {qw:1,qx:0,qy:0,qz:0}).clone();
            if (uStroke >= 0.70 && uStroke <= 0.99 && idealEA > 0.5) {
                const rad = (idealEA - 30) * (Math.PI / 180) * 0.6;
                const qPitch = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), rad);
                qIdeal.multiply(qPitch);
            }
            ghostArmGroup.setRotationFromQuaternion(qIdeal);
            ghostArmGroup.position.copy(canon.clone().multiplyScalar(VIZ_CANONICAL_BLEND).add(new THREE.Vector3(spread, 0, 0)).add(idealNudge));
        } else {
            ghostArmGroup.visible = false;
        }
    }

    // Splash when hand crosses water surface (y=0 going down)
    if (idx > 0 && anim[idx - 1]) {
        const prevY = anim[idx - 1].y;
        if (prevY > 0 && pos.y <= 0 && strokeNumAt(d) > 0) {
            createSplash(pos);
        }
    }

    // Tint hand blue when below water
    if (handGroup && hapticFlashUntil <= Date.now() && pos.y < 0) {
        handMaterials.forEach(m => {
            const base = new THREE.Color(m.userData.baseColor != null ? m.userData.baseColor : SKIN_COLOR);
            const water = new THREE.Color(0x4488aa);
            const depth = Math.min(1, Math.abs(pos.y) * 2);
            m.color.copy(base.lerp(water, depth * 0.4));
        });
    }

    const sc = strokeNumAt(d);
    if (sc > lastRenderedStrokeCount && strokeMarkerGroup) {
        lastRenderedStrokeCount = sc;
        const rippleGeo = new THREE.RingGeometry(0.05, 0.095, 48);
        const rippleMat = new THREE.MeshBasicMaterial({
            color: 0x7dd3fc, transparent: true, opacity: 0.9, side: THREE.DoubleSide
        });
        const ripple = new THREE.Mesh(rippleGeo, rippleMat);
        ripple.rotation.x = -Math.PI / 2;
        ripple.position.set(pos.x, 0.016, pos.z);
        ripple.userData = { ripple: true, age: 0 };
        strokeMarkerGroup.add(ripple);
    }

    if (trailLine) {
        const seg0 = getSegStart(idx);
        const trailStart = Math.max(seg0, idx - TRAIL_MAX_POINTS);
        const rawPts = [];
        const colors = [];
        for (let i = trailStart; i <= idx; i++) {
            const p = anim[i];
            rawPts.push(p ? p.clone() : new THREE.Vector3());
            const ph = processedData[i]?.stroke_phase || processedData[i]?.phase || 'idle';
            const c = PHASE_COLOR_RGB[ph] || PHASE_COLOR_RGB.idle;
            const age = (idx - i) / Math.max(1, idx - trailStart);
            const fade = 0.3 + 0.7 * (1 - age);
            colors.push(c[0] * fade, c[1] * fade, c[2] * fade);
        }
        const { points: smoothed, colors: colOut } = resampleTrailFreestyle(rawPts, colors);
        if (smoothed.length >= 2) {
            const flat = [];
            smoothed.forEach(p => flat.push(p.x, p.y, p.z));
            trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
            trailLine.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colOut, 3));
            trailLine.geometry.attributes.position.needsUpdate = true;
            trailLine.geometry.attributes.color.needsUpdate = true;
        } else {
            trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        }
    }

    const bPlay = getPlaybackBounds();
    const pct = bPlay.end > bPlay.start ? (idx - bPlay.start) / (bPlay.end - bPlay.start) * 100 : (processedData.length > 1 ? idx / (processedData.length - 1) * 100 : 0);
    const fillEl = document.getElementById('progress-fill');
    if (fillEl) fillEl.style.width = pct + '%';
    const playheadEl = document.getElementById('progress-playhead');
    if (playheadEl) {
        playheadEl.style.left = pct + '%';
        playheadEl.style.display = processedData.length > 1 ? 'block' : 'none';
    }
    const scrubEl = document.getElementById('progress-scrubber');
    if (scrubEl && processedData.length > 1) {
        scrubEl.setAttribute('aria-valuemax', '100');
        scrubEl.setAttribute('aria-valuemin', '0');
        scrubEl.setAttribute('aria-valuenow', String(Math.round(pct)));
    }
    const t = processedData.length > 1 && d.timestamp != null && processedData[0].timestamp != null
        ? (d.timestamp - processedData[0].timestamp) / 1000 : 0;
    const total = processedData.length > 1 && processedData[processedData.length - 1].timestamp != null && processedData[0].timestamp != null
        ? (processedData[processedData.length - 1].timestamp - processedData[0].timestamp) / 1000 : 0;
    setText('play-time', formatTimeFine(t) + ' / ' + formatTimeFine(total));
    let dtMs = 0;
    if (idx > 0 && d.timestamp != null && processedData[idx - 1].timestamp != null) {
        dtMs = d.timestamp - processedData[idx - 1].timestamp;
    }
    const fineParts = [];
    if (dtMs > 0) fineParts.push('Δ ' + dtMs.toFixed(1) + ' ms (IMU)');
    if (d.haptic_fired) fineParts.push('⚡ haptic');
    setText('play-time-fine', fineParts.length ? fineParts.join(' · ') : '');
    const inSel = bPlay.end > bPlay.start ? (idx - bPlay.start + 1) + ' / ' + (bPlay.end - bPlay.start + 1) : '—';
    setText('play-frame', 'Sample ' + (idx + 1) + ' / ' + processedData.length + ' · in range ' + inSel);

    setText('live-strokes', strokeNumAt(d));
    const phase = d.stroke_phase || d.phase || 'idle';
    const phaseEl = document.getElementById('live-phase');
    if (phaseEl) {
        phaseEl.textContent = phase;
        phaseEl.className = 'phase-label phase-label-' + phase;
    }
    // Show latest non-zero entry angle and deviation (they persist per-stroke)
    let liveAngle = 0, liveDeviation = 0;
    for (let i = idx; i >= Math.max(0, idx - 200); i--) {
        const p = processedData[i];
        if (liveAngle === 0 && (p.entry_angle || 0) > 0) liveAngle = p.entry_angle;
        if (liveDeviation === 0 && (p.deviation_score || 0) > 0) liveDeviation = p.deviation_score;
        if (liveAngle > 0 && liveDeviation > 0) break;
    }
    setText('live-angle', liveAngle.toFixed(1) + '°');
    const gx = d.angular_velocity?.gx || 0, gy = d.angular_velocity?.gy || 0, gz = d.angular_velocity?.gz || 0;
    setText('live-gyro', Math.sqrt(gx * gx + gy * gy + gz * gz).toFixed(1));
    setText('live-deviation', liveDeviation.toFixed(3));
    if (idx === 0 || idx === processedData.length - 1 || lastCalStripFrameIdx < 0 || Math.abs(idx - lastCalStripFrameIdx) >= 12) {
        lastCalStripFrameIdx = idx;
        updateCalibrationDisplay(d);
    }
    if (idx % 3 === 0) updateCharts(idx);
}

function refreshVizPlaybackUI() {
    const sel = document.getElementById('viz-stroke-playback');
    if (!sel) return;
    const n = sessionMetrics ? (sessionMetrics.stroke_count || 0) : 0;
    const prev = sel.value;
    let html = '<option value="0">All strokes</option>';
    for (let i = 1; i <= n; i++) html += '<option value="' + i + '">Stroke ' + i + ' only</option>';
    sel.innerHTML = html;
    if (prev && [...sel.options].some(o => o.value === prev)) sel.value = prev;
}

function onVizStreamModeChange() {
    const el = document.getElementById('viz-stream-mode');
    vizStreamMode = el && el.value === 'position' ? 'position' : 'kinematic';
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    integratePositions();
    renderFrame(currentIndex);
}

function focusVizOnHand() {
    if (!camera || !controls || !processedData.length) return;
    if (integratedPositions.length !== processedData.length) integratePositions();
    const anim = currentAnimationPositions();
    const pos = anim[currentIndex];
    if (!pos) return;
    controls.target.copy(pos);
    camera.position.copy(pos.clone().add(new THREE.Vector3(0.7, 0.6, 0.85)));
    followHandInView = false;
    controls.update();
}

function bindVizPlaybackControls() {
    const sp = document.getElementById('viz-playback-speed');
    if (sp) {
        sp.addEventListener('input', () => {
            playbackSpeedMultiplier = Math.max(0.05, Math.min(4, parseFloat(sp.value) || 1));
            const lab = document.getElementById('viz-speed-label');
            if (lab) lab.textContent = playbackSpeedMultiplier.toFixed(2) + '×';
            if (isPlaying) {
                togglePlayback();
                togglePlayback();
            }
        });
    }
    const strokeSel = document.getElementById('viz-stroke-playback');
    if (strokeSel) {
        strokeSel.addEventListener('change', () => {
            playbackStrokeFilter = parseInt(strokeSel.value, 10) || 0;
            resetPlayback();
        });
    }
    const loopEl = document.getElementById('viz-loop-stroke');
    if (loopEl) {
        loopEl.addEventListener('change', () => { loopStrokePlayback = loopEl.checked; });
    }
    const loopFull = document.getElementById('viz-loop-full');
    if (loopFull) {
        loopFull.addEventListener('change', () => { loopFullSession = loopFull.checked; });
    }
}

function clearViz() {
    processedData = [];
    sessionMetrics = null;
    currentIndex = 0;
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    lastRenderedStrokeCount = 0;
    hapticFlashUntil = 0;
    refreshStrokeFieldMode();
    playbackStrokeSegments = [];
    if (trailLine && trailLine.geometry) {
        trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        trailLine.geometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    }
    if (strokeMarkerGroup) {
        while (strokeMarkerGroup.children.length) {
            const c = strokeMarkerGroup.children[0];
            strokeMarkerGroup.remove(c);
            c.geometry?.dispose();
            c.material?.dispose();
        }
    }
    if (handGroup) {
        handGroup.position.set(0, 0, 0);
        handGroup.quaternion.identity();
        handMaterials.forEach(m => {
            m.color.setHex(m.userData.baseColor != null ? m.userData.baseColor : SKIN_COLOR);
            m.emissive.setHex(0x060403);
        });
    }
    if (splashGroup) {
        while (splashGroup.children.length) {
            const c = splashGroup.children[0];
            splashGroup.remove(c);
            c.geometry?.dispose();
            c.material?.dispose();
        }
    }
    setText('play-time', '0:00.00 / 0:00.00');
    setText('play-time-fine', '');
    setText('play-frame', '—');
    const fillEl = document.getElementById('progress-fill');
    if (fillEl) fillEl.style.width = '0%';
    const ph = document.getElementById('progress-playhead');
    if (ph) ph.style.display = 'none';
}
function resetView() {
    followHandInView = false;
    if (camera) {
        camera.position.set(2.1, 2.15, 3.4);
        camera.lookAt(0, 0.1, 0.5);
        if (controls) { controls.target.set(0, 0.1, 0.5); controls.update(); }
    }
}
function updateScale(v) {
    positionScale = parseFloat(v) || 3;
    setText('scale-val', positionScale.toFixed(1) + 'x');
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
}



// Expose handlers for inline HTML onclick usage
Object.assign(window, {
    switchTab,
    syncFromDevice,
    fillDeviceFormFromConnectedDevice,
    applySmartDeviceDefaults,
    mergeLatestSessions,
    togglePlayback,
    resetPlayback,
    skipBackward,
    skipForward,
    seekPlayback,
    stepFrame,
    updateScale,
    resetView,
    onVizStreamModeChange,
    focusVizOnHand,
    clearViz,
    registerUser,
    registerDevice,
    setCurrentAsIdeal,
    setSingleStrokeAsIdeal,
    pushIdealToDevice,
    deleteIdealStroke,
    pushUserConfigToDevice,
    testHapticDevice,
    selectSession,
    deleteSession
});

function bindNavigationButtons() {
    document.querySelectorAll('[data-tab-target]').forEach(btn => {
        btn.addEventListener('click', (e) => {
            e.preventDefault();
            const tab = btn.getAttribute('data-tab-target');
            if (tab) switchTab(tab);
        });
    });
}

// ── INIT ──
window.addEventListener('DOMContentLoaded', () => {
    bindNavigationButtons();
    bindVizPlaybackControls();
    initScrubberPointerHandlers();
    loadUserProfile();
    loadDevices();
    loadSavedSessions();
    loadIdealStroke();
    init3D();
    startDevicePolling();
    window.addEventListener('resize', () => {
        if (renderer && camera) {
            const c = document.getElementById('canvas3d');
            if (c) {
                renderer.setSize(c.clientWidth, c.clientHeight);
                camera.aspect = c.clientWidth / c.clientHeight;
                camera.updateProjectionMatrix();
            }
        }
    });
});
