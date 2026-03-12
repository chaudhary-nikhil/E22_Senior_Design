// GoldenForm App — Core Logic v2
// Visualization overhaul: stroke-phase position, haptic markers, ideal overlay,
// multi-device, calibration display, production polish

const API = '';
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

// ── TAB NAVIGATION ──
function switchTab(tab) {
    document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.nav-tab').forEach(t => t.classList.remove('active'));
    const page = document.getElementById('page-' + tab);
    const btn = document.getElementById('tab-' + tab);
    if (page) { page.classList.add('active'); page.style.animation = 'fadeIn 0.3s ease'; }
    if (btn) btn.classList.add('active');
    currentTab = tab;
    if (tab === 'insights') loadProgress();
    if (tab === 'settings') loadDevices();
    if (tab === 'analysis' && sessionMetrics) updateAnalysis();
}

// ── API HELPERS ──
async function apiGet(path) { try { const r = await fetch(API + path); return r.json(); } catch (e) { return null; } }
async function apiPost(path, body) {
    try {
        const r = await fetch(API + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        return r.json();
    } catch (e) { return { status: 'error', error: e.message }; }
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
    if (userProfile && userProfile.name) {
        updateProfileUI();
        document.getElementById('reg-modal').classList.remove('show');
    } else {
        document.getElementById('reg-modal').classList.add('show');
    }
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
}
async function registerUser(e) {
    if (e) e.preventDefault();
    
    // Determine which form triggered this (modal vs settings page)
    const isModal = document.getElementById('reg-modal').classList.contains('show');
    const prefix = isModal ? 'reg-' : 'settings-';
    
    const nameInput = document.getElementById(prefix + 'name')?.value.trim();
    if (!nameInput) return;

    const body = {
        name: nameInput,
        height_cm: parseFloat(document.getElementById(prefix + 'height')?.value) || 0,
        wingspan_cm: parseFloat(document.getElementById(prefix + 'wingspan')?.value) || 0,
        skill_level: document.getElementById(prefix + 'skill')?.value || 'beginner'
    };
    
    const res = await apiPost('/api/register', body);
    if (res && res.status === 'ok') {
        userProfile = res.profile;
        updateProfileUI();
        document.getElementById('reg-modal').classList.remove('show');
        showToast('Profile saved!', 'success');
        pushUserConfigToDevice(true); // Auto-sync to device
    }
}

// ── DEVICE MANAGEMENT ──
async function loadDevices() {
    const res = await apiGet('/api/devices');
    const list = document.getElementById('device-list');
    if (!list) return;
    const devices = (res && res.devices) || [];
    if (!devices.length) { list.innerHTML = '<p style="color:var(--text3);font-size:0.85em;">No devices registered yet.</p>'; return; }
    const roleIcons = { wrist_right: '🤚', wrist_left: '✋', head: '🧠', ankle_right: '🦶', ankle_left: '🦶', waist: '🫁' };
    list.innerHTML = devices.map(d => `<div class="session-item"><div><strong>${roleIcons[d.role] || '📱'} ${d.name || 'Device ' + d.device_hw_id}</strong><div class="meta">${d.role} · HW ID: ${d.device_hw_id}</div></div><span class="badge badge-gold">${d.role}</span></div>`).join('');
}
async function registerDevice(e) {
    if (e) e.preventDefault();
    const body = {
        user_id: userProfile ? userProfile.id : 1,
        device_hw_id: parseInt(document.getElementById('dev-hw-id').value) || 0,
        role: document.getElementById('dev-role').value,
        name: document.getElementById('dev-name').value || ''
    };
    const res = await apiPost('/api/devices/register', body);
    if (res && res.status === 'ok') showToast('Device registered!', 'success');
    loadDevices();
}

let isDeviceOnline = false;
let pendingConfigSync = false;
let pendingIdealSync = false;
let devicePollInterval = null;

function setConnStatus(state) {
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
        const wasOnline = isDeviceOnline;
        isDeviceOnline = res && !res.error && res.status !== 'disconnected';
        
        // Don't override 'syncing' status if we are currently mid-sync
        const txt = document.getElementById('conn-text');
        if (txt && txt.textContent !== 'Syncing...') {
            setConnStatus(isDeviceOnline ? 'connected' : 'offline');
        }

        if (res && res.cal) {
            updateCalibrationDisplay(res);
        }

        if (!wasOnline && isDeviceOnline) {
            if (pendingConfigSync) {
                await pushUserConfigToDevice(true);
                pendingConfigSync = false;
            }
            if (pendingIdealSync) {
                await pushIdealToDevice(true);
                pendingIdealSync = false;
            }
        }
    } catch(e) {
        isDeviceOnline = false;
        const txt = document.getElementById('conn-text');
        if (txt && txt.textContent !== 'Syncing...') {
            setConnStatus('offline');
        }
    }
}

async function syncFromDevice() {
    const statusEl = document.getElementById('sync-status');
    const btn = document.getElementById('sync-btn');
    statusEl.style.display = 'block';
    statusEl.textContent = 'Connecting to device...';
    statusEl.className = 'badge badge-amber';
    btn.disabled = true;
    setConnStatus('syncing');

    try {
        const res = await apiGet('/process');
        if (!res || res.error) {
            statusEl.textContent = res ? res.error : 'No response';
            statusEl.className = 'badge badge-red';
            setConnStatus('offline');
            showToast('Failed to connect to device', 'error');
            return;
        }
        const sessions = res.sessions || [];
        statusEl.textContent = `✅ Synced ${sessions.length} session(s)`;
        statusEl.className = 'badge badge-green';
        setConnStatus('connected');
        showToast(`Synced ${sessions.length} session(s) from device`, 'success');

        for (const s of sessions) {
            addSession({ name: s.name, processed_data: s.processed_data, metrics: s.metrics, duration: s.duration, syncedAt: s.syncedAt });
        }
        if (sessions.length > 0) selectSession(savedSessions.length - 1);
        
        // Auto-push settings when we know we have an active connection
        pushUserConfigToDevice(true);
        pushIdealToDevice(true);
    } catch (e) {
        statusEl.textContent = 'Device unreachable. Using cached data...';
        statusEl.className = 'badge badge-amber';
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
                if (cached.sessions.length > 0) selectSession(savedSessions.length - 1);
                statusEl.textContent = 'Loaded from cache';
                showToast('Loaded cached session data', 'info');
            }
        } catch { statusEl.textContent = 'No data available'; statusEl.className = 'badge badge-red'; }
    } finally { btn.disabled = false; }
}

const LS_KEY = 'goldenform_sessions';
function loadSavedSessions() {
    try { savedSessions = JSON.parse(localStorage.getItem(LS_KEY) || '[]'); } catch { savedSessions = []; }
    renderSessionList();
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
        apiPost('/api/sessions/save', { raw_data: obj.raw_data, processed_data: obj.processed_data, metrics: obj.metrics, duration: obj.duration, device_ids: [] }).then(res => {
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
    }
}
function renderSessionList() {
    const c = document.getElementById('session-cards');
    const badge = document.getElementById('session-count-badge');
    if (badge) badge.textContent = savedSessions.length + ' saved';
    if (!c) return;
    if (!savedSessions.length) { c.innerHTML = '<p style="color:var(--text3);font-size:0.85em;padding:16px;">No sessions yet. Sync from device.</p>'; return; }
    c.innerHTML = savedSessions.map((s, i) => `<div class="session-item ${i === activeSessionIdx ? 'active' : ''}" onclick="selectSession(${i})"><div><strong>${s.name || 'Session ' + (i + 1)}</strong><div class="meta">${s.metrics ? s.metrics.stroke_count + ' strokes · ' + formatTime(s.duration) : ''}</div></div><button class="btn btn-sm btn-outline" onclick="event.stopPropagation();deleteSession(${i})">✕</button></div>`).join('');
}
function deleteSession(i) {
    savedSessions.splice(i, 1);
    persistSessions();
    if (activeSessionIdx === i) { activeSessionIdx = -1; clearViz(); }
    else if (activeSessionIdx > i) activeSessionIdx--;
    renderSessionList();
}
function selectSession(i) {
    if (i < 0 || i >= savedSessions.length) return;
    activeSessionIdx = i;
    const s = savedSessions[i];
    processedData = s.processed_data || s.processedData || [];
    sessionMetrics = s.metrics || {};
    currentIndex = 0;
    isPlaying = false;
    if (playbackInterval) { clearInterval(playbackInterval); playbackInterval = null; }
    // Pre-compute stroke boundaries and haptic events for timeline
    computeStrokeBoundaries();
    buildPlaybackStrokeSegments();
    renderSessionList();
    updateSessionSummary();
    updateAnalysis();
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
let strokeBoundaries = []; // {index, strokeNum}
let hapticEvents = [];     // {index, deviation}

function computeStrokeBoundaries() {
    strokeBoundaries = [];
    hapticEvents = [];
    let lastCount = 0;
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        if (d.stroke_count > lastCount) {
            strokeBoundaries.push({ index: i, strokeNum: d.stroke_count });
            lastCount = d.stroke_count;
        }
        if (d.haptic_fired) {
            hapticEvents.push({ index: i, deviation: d.deviation_score || 0 });
        }
    }
}

function buildHapticTimeline() {
    const container = document.getElementById('haptic-timeline');
    if (!container || !processedData.length) return;
    container.innerHTML = '';
    const total = processedData.length;
    // Stroke boundaries as blue ticks
    strokeBoundaries.forEach(sb => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick stroke-tick';
        tick.style.left = (sb.index / total * 100) + '%';
        tick.title = 'Stroke #' + sb.strokeNum;
        container.appendChild(tick);
    });
    // Haptic events as red diamonds
    hapticEvents.forEach(he => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick haptic-tick';
        tick.style.left = (he.index / total * 100) + '%';
        tick.title = 'Haptic fired (deviation: ' + he.deviation.toFixed(3) + ')';
        container.appendChild(tick);
    });
}

// ── PLAYBACK ──
function togglePlayback() {
    if (!processedData.length) return;
    isPlaying = !isPlaying;
    const btn = document.getElementById('play-btn');
    if (isPlaying) {
        btn.textContent = '⏸';
        playbackInterval = setInterval(() => {
            if (currentIndex < processedData.length - 1) { currentIndex++; renderFrame(currentIndex); }
            else { isPlaying = false; btn.textContent = '▶'; clearInterval(playbackInterval); }
        }, 20);
    } else { btn.textContent = '▶'; clearInterval(playbackInterval); }
}
function resetPlayback() { currentIndex = 0; isPlaying = false; if (playbackInterval) clearInterval(playbackInterval); document.getElementById('play-btn').textContent = '▶'; renderFrame(0); }
function skipForward() { currentIndex = Math.min(currentIndex + 50, processedData.length - 1); renderFrame(currentIndex); }
function skipBackward() { currentIndex = Math.max(currentIndex - 50, 0); renderFrame(currentIndex); }
function seekPlayback(e) {
    const rect = e.currentTarget.getBoundingClientRect();
    const pct = (e.clientX - rect.left) / rect.width;
    currentIndex = Math.floor(pct * (processedData.length - 1));
    renderFrame(currentIndex);
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

// ── SESSION SUMMARY ──
function updateSessionSummary() {
    if (!sessionMetrics) return;
    const m = sessionMetrics;
    setText('sum-strokes', m.stroke_count || 0);
    setText('sum-turns', m.turn_count || 0);
    setText('sum-duration', formatTime(m.duration));
    setText('sum-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) + '/min' : '--');
    setText('sum-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '--');
    setText('sum-samples', processedData.length);
    setText('home-last-strokes', m.stroke_count || 0);
    setText('home-last-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) : '--');
    setText('home-last-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '--');
}
function setText(id, val) { const el = document.getElementById(id); if (el) el.textContent = val; }

// ── CALIBRATION DISPLAY (TIDR 6-1-4) ──
function updateCalibrationDisplay(d) {
    if (!d || !d.cal) return;
    const fields = [
        { id: 'cal-sys', val: d.cal.sys },
        { id: 'cal-gyro', val: d.cal.gyro },
        { id: 'cal-accel', val: d.cal.accel },
        { id: 'cal-mag', val: d.cal.mag }
    ];
    fields.forEach(f => {
        const el = document.getElementById(f.id);
        if (!el) return;
        const v = f.val || 0;
        el.textContent = v + '/3';
        el.className = 'cal-badge ' + (v >= 3 ? 'cal-good' : v >= 1 ? 'cal-warn' : 'cal-bad');
    });
    // Overall quality - in IMU mode, Mag is 0 and can be ignored for "perfect" score
    const sys = d.cal.sys || 0;
    const gyro = d.cal.gyro || 0;
    const accel = d.cal.accel || 0;
    const total = sys + gyro + accel;
    const quality = Math.round(total / 9 * 100);
    setText('cal-quality', quality + '%');
}

// ── ANALYSIS ──
function updateAnalysis() {
    if (!sessionMetrics) return;
    const m = sessionMetrics;
    drawGauge('gauge-aoa', m.avg_entry_angle || 0, 0, 90, 15, 40, '°');
    setText('aoa-value', (m.avg_entry_angle || 0).toFixed(1) + '°');
    setText('aoa-ideal', (m.ideal_entry_angle || 30) + '°');

    const pcts = m.phase_pcts || { glide: 0, pull: 0, recovery: 0 };
    const total = pcts.glide + pcts.pull + pcts.recovery || 1;
    setWidth('phase-glide', pcts.glide / total * 100);
    setWidth('phase-pull', pcts.pull / total * 100);
    setWidth('phase-recovery', pcts.recovery / total * 100);
    setText('phase-glide-pct', pcts.glide.toFixed(0) + '%');
    setText('phase-pull-pct', pcts.pull.toFixed(0) + '%');
    setText('phase-recovery-pct', pcts.recovery.toFixed(0) + '%');

    setText('haptic-count', m.haptic_count || 0);
    setText('avg-deviation', m.avg_deviation ? m.avg_deviation.toFixed(3) : '0.000');

    const score = computeFormScore(m);
    setText('form-score', score.toFixed(1));
    const scoreEl = document.getElementById('form-score');
    if (scoreEl) scoreEl.style.color = score >= 7 ? 'var(--green)' : score >= 5 ? 'var(--amber)' : 'var(--red)';

    buildStrokeTable();
    buildIdealComparison();
}

function setWidth(id, pct) { const el = document.getElementById(id); if (el) el.style.width = Math.max(2, pct) + '%'; }

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
                rows.push({ 
                    num: p.stroke_count, 
                    time: Math.max(0, (p.timestamp - startTime) / 1000).toFixed(1), 
                    angle: p.entry_angle || 0, 
                    haptic: p.haptic_fired, 
                    deviation: p.deviation_score || 0 
                });
                lastCount = p.stroke_count;
            }
        }
    }
    tbody.innerHTML = rows.map(r =>
        `<tr class="${r.haptic ? 'row-haptic' : ''}"><td>${r.num}</td><td>${r.time}s</td><td>${r.angle.toFixed(1)}°</td><td>${r.haptic ? '<span class="haptic-marker">⚡</span>' : '✓'}</td><td class="${r.deviation > 0.7 ? 'text-red' : r.deviation > 0.3 ? 'text-amber' : 'text-green'}">${r.deviation.toFixed(3)}</td></tr>`
    ).join('');
}

// ── IDEAL STROKE COMPARISON (TIDR 2-1-1) ──
async function buildIdealComparison() {
    if (!idealStrokeData || !idealStrokeData.length || !processedData.length) {
        const el = document.getElementById('ideal-comparison-content');
        if (el) el.innerHTML = '<p style="color:var(--text3);text-align:center;padding:12px;">No ideal stroke saved. Set one in Settings.</p>';
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
async function loadProgress() {
    const res = await apiGet('/api/progress');
    const data = ((res && res.progress) || []).reverse();
    if (!data.length) { setText('insights-empty', 'Complete a session to see your progress over time.'); return; }
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
                y1: { position: 'right', ticks: { color: '#666' }, grid: { display: false }, min: 0, title: { display: true, text: 'Rate (s/min)', color: '#666' } }
            }
        }
    });
}

// ── IDEAL STROKE ──
async function loadIdealStroke() {
    const res = await apiGet('/api/ideal_stroke');
    if (res && res.samples && res.samples.length) {
        idealStrokeData = res.samples;
        setText('ideal-status', `Loaded: ${res.samples.length} samples`);
    } else { idealStrokeData = null; setText('ideal-status', 'No ideal stroke saved'); }
}
async function setCurrentAsIdeal() {
    if (!processedData.length || !sessionMetrics || sessionMetrics.stroke_count < 1) { showToast('Record a session with strokes first', 'error'); return; }
    const samples = processedData.map(p => ({ lia_x: p.acceleration?.ax || 0, lia_y: p.acceleration?.ay || 0, lia_z: p.acceleration?.az || 0 }));
    
    // Compute average entry angle from session if available
    const avgAngle = sessionMetrics ? sessionMetrics.avg_entry_angle : 30.0;
    
    await apiPost('/api/ideal_stroke', { name: 'From Session', samples, ideal_entry_angle: avgAngle });
    await loadIdealStroke();
    showToast('Ideal stroke saved! Pushing to device...', 'success');
    await pushIdealToDevice(true);
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
        setText('ideal-status', 'No ideal stroke saved');
        showToast('Ideal stroke deleted', 'success');
    } else {
        showToast('Failed to delete: ' + ((result && result.error) || 'unknown'), 'error');
    }
}

async function pushUserConfigToDevice(silent = false) {
    const wingspan = document.getElementById('settings-wingspan') ? parseFloat(document.getElementById('settings-wingspan').value) : 180;
    const height = document.getElementById('settings-height') ? parseFloat(document.getElementById('settings-height').value) : 180;
    const skill = document.getElementById('settings-skill') ? document.getElementById('settings-skill').value : 'beginner';
    
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
        skill_level: skill
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
//  3D VISUALIZATION — Ported from integrated session viewer
//  Single cube at path tip, green position line, live stroke arc, colored past strokes
// ══════════════════════════════════════════════════════════════════
let scene, camera, renderer, controls;
let imuCube = null;           // Cube at origin (red/green by tracking)
let handMesh = null;          // 3D hand at path tip (optional)
let positionLine = null;      // Green line from origin to current tip
let liveTrailLine = null;     // White arc for current stroke (quaternion-based)
let strokeTrails = [];        // Colored lines for past strokes (one color per stroke)
let playbackStrokeSegments = []; // { startIdx, endIdx, strokeNum } built from processedData

const TRAIL_SMOOTH_WINDOW = 5;
const STROKE_COLORS = [0x00d4ff, 0xff9500, 0x00ff88, 0xbf5fff, 0xffff00, 0xff4444, 0x4488ff];
const MAX_STROKES_DISPLAYED = 10;

function nq(q) {
    if (!q) return new THREE.Quaternion(0, 0, 0, 1);
    const w = q.qw ?? 1, x = q.qx ?? 0, y = q.qy ?? 0, z = q.qz ?? 0;
    const m = Math.hypot(w, x, y, z);
    return m > 1e-8 ? new THREE.Quaternion(x / m, y / m, z / m, w / m) : new THREE.Quaternion(0, 0, 0, 1);
}
function trailPtDisplacement(dataQ, refQ, scale) {
    const tip = new THREE.Vector3(0, 0, scale);
    tip.applyQuaternion(nq(dataQ));
    const start = new THREE.Vector3(0, 0, scale);
    start.applyQuaternion(nq(refQ));
    return tip.sub(start);
}
function getSegStart(upToIndex) {
    if (!processedData.length || upToIndex < 0) return 0;
    const sc = processedData[upToIndex]?.stroke_count ?? 0;
    for (let i = upToIndex; i >= 0; i--) {
        if ((processedData[i]?.stroke_count ?? 0) < sc) return i + 1;
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

function buildPlaybackStrokeSegments() {
    playbackStrokeSegments = [];
    let last = 0, start = 0;
    for (let i = 0; i < processedData.length; i++) {
        const c = processedData[i].stroke_count ?? 0;
        if (c > last) {
            if (last > 0 && start < i) playbackStrokeSegments.push({ startIdx: start, endIdx: i - 1, strokeNum: last });
            start = i;
            last = c;
        }
    }
    if (last > 0 && start < processedData.length)
        playbackStrokeSegments.push({ startIdx: start, endIdx: processedData.length - 1, strokeNum: last });
}

// 3D hand model (palm + fingers + thumb + wrist), scaled for stroke path
function createHandModel() {
    if (typeof THREE === 'undefined') return null;
    const group = new THREE.Group();
    const mat = new THREE.MeshPhongMaterial({
        color: 0xe8b88b,
        emissive: 0x1a1008,
        specular: 0x442211,
        shininess: 20,
        flatShading: false
    });
    const cyl = (rTop, rBot, h, seg = 10) =>
        new THREE.CylinderGeometry(rTop, rBot, h, seg);

    const palmGeo = new THREE.BoxGeometry(0.09, 0.06, 0.035);
    const palm = new THREE.Mesh(palmGeo, mat.clone());
    palm.position.set(0, 0, 0);
    group.add(palm);

    const fingerSegs = [
        [0.032, 0.028, 0.022], [0.035, 0.031, 0.024], [0.033, 0.029, 0.022], [0.025, 0.022, 0.018]
    ];
    const fingerBaseY = [-0.022, -0.008, 0.008, 0.022];
    for (let f = 0; f < 4; f++) {
        const segs = fingerSegs[f];
        let z = 0.0175, r = 0.012;
        for (let s = 0; s < 3; s++) {
            const seg = new THREE.Mesh(cyl(r, r * 1.08, segs[s], 8), mat.clone());
            seg.position.set(fingerBaseY[f], 0, z + segs[s] / 2);
            seg.rotation.x = Math.PI / 2;
            group.add(seg);
            z += segs[s];
            r *= 0.85;
        }
    }

    const thumbGrp = new THREE.Group();
    thumbGrp.position.set(-0.055, -0.028, -0.01);
    thumbGrp.rotation.z = 0.5;
    thumbGrp.rotation.x = -0.3;
    const t1 = new THREE.Mesh(cyl(0.011, 0.012, 0.028, 8), mat.clone());
    t1.position.set(0, 0, 0.014); t1.rotation.x = Math.PI / 2;
    thumbGrp.add(t1);
    const t2 = new THREE.Mesh(cyl(0.009, 0.011, 0.022, 8), mat.clone());
    t2.position.set(0, 0, 0.039); t2.rotation.x = Math.PI / 2;
    thumbGrp.add(t2);
    group.add(thumbGrp);

    const wrist = new THREE.Mesh(cyl(0.035, 0.04, 0.03, 10), mat.clone());
    wrist.position.set(0, 0, -0.04);
    wrist.rotation.x = Math.PI / 2;
    group.add(wrist);

    group.scale.setScalar(1.4);
    return group;
}

function init3D() {
    const canvas = document.getElementById('canvas3d');
    if (!canvas || !window.THREE) return;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);

    const w = Math.max(canvas.clientWidth || 800, 400);
    const h = Math.max(canvas.clientHeight || 450, 400);
    camera = new THREE.PerspectiveCamera(75, w / h, 0.1, 1000);
    camera.position.set(3, 3, 3);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    scene.add(new THREE.GridHelper(10, 10, 0x333333, 0x1a1a1a));
    scene.add(new THREE.AxesHelper(2));

    const geo = new THREE.BoxGeometry(0.3, 0.3, 0.3);
    imuCube = new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ color: 0xff4444 }));
    imuCube.castShadow = true;
    scene.add(imuCube);

    const plGeo = new THREE.BufferGeometry();
    plGeo.setAttribute('position', new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
    positionLine = new THREE.Line(plGeo, new THREE.LineBasicMaterial({ color: 0x00ff00, opacity: 0.6, transparent: true }));
    scene.add(positionLine);

    const ltGeo = new THREE.BufferGeometry();
    ltGeo.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    liveTrailLine = new THREE.Line(ltGeo, new THREE.LineBasicMaterial({ color: 0xffffff, opacity: 0.95, transparent: true }));
    scene.add(liveTrailLine);

    handMesh = createHandModel();
    if (handMesh) {
        handMesh.visible = true;
        scene.add(handMesh);
    }

    scene.add(new THREE.AmbientLight(0x404040, 0.6));
    const dl = new THREE.DirectionalLight(0xffffff, 0.8);
    dl.position.set(5, 5, 5);
    scene.add(dl);

    if (window.THREE.OrbitControls) {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.target.set(0, 0, 0);
        controls.minDistance = 0.5;
        controls.maxDistance = 15;
    }

    animate();
}

function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
}

// ── RENDER FRAME (integrated session viewer style) ──
function renderFrame(idx) {
    if (!processedData.length || idx >= processedData.length) return;
    const d = processedData[idx];
    const segStart = getSegStart(idx);
    const refQ = processedData[segStart]?.quaternion;
    const pathScale = positionScale;

    // Cube at origin, rotates with quaternion (integrated viewer style); green when tracking
    if (imuCube) {
        imuCube.position.set(0, 0, 0);
        imuCube.setRotationFromQuaternion(nq(d.quaternion));
        if (imuCube.material) imuCube.material.color.setHex(d.tracking_active ? 0x44ff44 : 0xff4444);
        // Cube stays at origin; position line and trail show the path
    }

    // Green position line: origin to current tip (displacement from stroke start)
    let tipVec = new THREE.Vector3(0, 0, 0);
    if (positionLine && refQ) {
        tipVec = trailPtDisplacement(d.quaternion, refQ, pathScale);
        const pa = positionLine.geometry.attributes.position.array;
        pa[3] = tipVec.x; pa[4] = tipVec.y; pa[5] = tipVec.z;
        positionLine.geometry.attributes.position.needsUpdate = true;
    }

    // 3D hand at path tip, oriented with quaternion
    if (handMesh) {
        handMesh.position.copy(tipVec);
        handMesh.setRotationFromQuaternion(nq(d.quaternion));
        const showHand = document.getElementById('show-hand') ? document.getElementById('show-hand').checked : true;
        handMesh.visible = showHand;
    }

    // Live trail: arc from stroke start to current frame (smoothed)
    if (liveTrailLine && refQ) {
        const points = [];
        for (let i = segStart; i <= idx; i++) {
            const pt = trailPtDisplacement(processedData[i]?.quaternion, refQ, pathScale);
            points.push(pt.clone());
        }
        const smoothed = smoothTrailPoints(points, TRAIL_SMOOTH_WINDOW);
        if (smoothed.length >= 2) {
            const flat = [];
            smoothed.forEach(p => { flat.push(p.x, p.y, p.z); });
            liveTrailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
            liveTrailLine.geometry.attributes.position.needsUpdate = true;
        } else {
            liveTrailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        }
    }

    // Past stroke overlays (colored arcs, up to MAX_STROKES_DISPLAYED)
    if (scene) {
        strokeTrails.forEach(t => { scene.remove(t); t.geometry?.dispose(); t.material?.dispose(); });
        strokeTrails = [];
        let shown = 0;
        for (const seg of playbackStrokeSegments) {
            if (seg.endIdx > idx || shown >= MAX_STROKES_DISPLAYED) break;
            const endBound = Math.min(seg.endIdx, idx);
            const ref = processedData[seg.startIdx]?.quaternion;
            const points = [];
            for (let i = seg.startIdx; i <= endBound; i++) {
                const pt = trailPtDisplacement(processedData[i]?.quaternion, ref, pathScale);
                points.push(pt.clone());
            }
            const smoothed = smoothTrailPoints(points, TRAIL_SMOOTH_WINDOW);
            if (smoothed.length >= 2) {
                const flat = [];
                smoothed.forEach(p => { flat.push(p.x, p.y, p.z); });
                const color = STROKE_COLORS[(seg.strokeNum - 1) % STROKE_COLORS.length];
                const geo = new THREE.BufferGeometry();
                geo.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
                const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, opacity: 0.85, transparent: true }));
                scene.add(line);
                strokeTrails.push(line);
                shown++;
            }
        }
    }

    // ── PLAYBACK UI ──
    const pct = processedData.length > 1 ? idx / (processedData.length - 1) * 100 : 0;
    const fillEl = document.getElementById('progress-fill');
    if (fillEl) fillEl.style.width = pct + '%';
    const t = processedData.length > 1 ? (d.timestamp - processedData[0].timestamp) / 1000 : 0;
    const total = processedData.length > 1 ? (processedData[processedData.length - 1].timestamp - processedData[0].timestamp) / 1000 : 0;
    setText('play-time', formatTime(t) + ' / ' + formatTime(total));

    setText('live-strokes', d.stroke_count || 0);
    setText('live-phase', d.stroke_phase || 'idle');
    setText('live-angle', (d.entry_angle || 0).toFixed(1) + '°');
    updateCalibrationDisplay(d);
    if (idx % 3 === 0) updateCharts(idx);
}

function clearViz() {
    processedData = [];
    sessionMetrics = null;
    currentIndex = 0;
    playbackStrokeSegments = [];
    strokeTrails.forEach(t => { if (scene) scene.remove(t); t.geometry?.dispose(); t.material?.dispose(); });
    strokeTrails = [];
    if (positionLine && positionLine.geometry && positionLine.geometry.attributes.position) {
        const pa = positionLine.geometry.attributes.position.array;
        if (pa.length >= 6) { pa[3] = 0; pa[4] = 0; pa[5] = 0; positionLine.geometry.attributes.position.needsUpdate = true; }
    }
    if (liveTrailLine && liveTrailLine.geometry) {
        liveTrailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    }
    if (imuCube) {
        imuCube.position.set(0, 0, 0);
        imuCube.quaternion.identity();
        if (imuCube.material) imuCube.material.color.setHex(0xff4444);
    }
    if (handMesh) handMesh.visible = false;
}
function resetView() {
    if (camera) { camera.position.set(3, 3, 3); camera.lookAt(0, 0, 0); if (controls) controls.target.set(0, 0, 0); }
}
function updateScale(v) { positionScale = parseFloat(v) || 3; setText('scale-val', positionScale.toFixed(1) + 'x'); }



// Expose handlers for inline HTML onclick usage
Object.assign(window, {
    switchTab,
    syncFromDevice,
    mergeLatestSessions,
    togglePlayback,
    resetPlayback,
    skipBackward,
    skipForward,
    seekPlayback,
    updateScale,
    resetView,
    clearViz,
    registerUser,
    registerDevice,
    setCurrentAsIdeal,
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
    loadUserProfile();
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
