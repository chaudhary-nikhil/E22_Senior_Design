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
    const name = (document.getElementById('reg-name') || document.getElementById('settings-name')).value.trim();
    if (!name) return;
    const body = {
        name,
        height_cm: parseFloat((document.getElementById('reg-height') || document.getElementById('settings-height')).value) || 0,
        wingspan_cm: parseFloat((document.getElementById('reg-wingspan') || document.getElementById('settings-wingspan')).value) || 0,
        skill_level: (document.getElementById('reg-skill') || document.getElementById('settings-skill')).value || 'beginner'
    };
    const res = await apiPost('/api/register', body);
    if (res && res.status === 'ok') {
        userProfile = res.profile;
        updateProfileUI();
        document.getElementById('reg-modal').classList.remove('show');
        showToast('Profile saved!', 'success');
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

// ── SYNC & SESSIONS ──
function setConnStatus(state) {
    const dot = document.querySelector('.status-dot');
    const txt = document.getElementById('conn-text');
    if (dot) dot.className = 'status-dot ' + state;
    if (txt) txt.textContent = state === 'connected' ? 'Connected' : state === 'syncing' ? 'Syncing...' : 'Offline';
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
    processedData = s.processed_data || [];
    sessionMetrics = s.metrics || {};
    currentIndex = 0;
    isPlaying = false;
    if (playbackInterval) { clearInterval(playbackInterval); playbackInterval = null; }
    // Pre-compute stroke boundaries and haptic events for timeline
    computeStrokeBoundaries();
    renderSessionList();
    updateSessionSummary();
    updateAnalysis();
    initCharts();
    buildHapticTimeline();
    renderFrame(0);
    switchTab('session');
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
    if (!d) return;
    const fields = [
        { id: 'cal-sys', val: d.sys_cal },
        { id: 'cal-gyro', val: d.gyro_cal },
        { id: 'cal-accel', val: d.accel_cal },
        { id: 'cal-mag', val: d.mag_cal }
    ];
    fields.forEach(f => {
        const el = document.getElementById(f.id);
        if (!el) return;
        const v = f.val || 0;
        el.textContent = v + '/3';
        el.className = 'cal-badge ' + (v >= 3 ? 'cal-good' : v >= 1 ? 'cal-warn' : 'cal-bad');
    });
    // Overall quality
    const total = (d.sys_cal || 0) + (d.gyro_cal || 0) + (d.accel_cal || 0) + (d.mag_cal || 0);
    const quality = Math.round(total / 12 * 100);
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
    let rows = [], lastCount = 0;
    for (let i = 0; i < processedData.length; i++) {
        const p = processedData[i];
        if (p.stroke_count > lastCount) {
            rows.push({ num: p.stroke_count, time: (p.timestamp / 1000).toFixed(1), angle: p.entry_angle || 0, haptic: p.haptic_fired, deviation: p.deviation_score || 0 });
            lastCount = p.stroke_count;
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
    await apiPost('/api/ideal_stroke', { name: 'From Session', samples });
    await loadIdealStroke();
    showToast('Ideal stroke saved!', 'success');
}
async function pushIdealToDevice() {
    if (!idealStrokeData || !idealStrokeData.length) { showToast('No ideal stroke to push', 'error'); return; }
    const result = await apiPost('/api/ideal_stroke/push', { samples: idealStrokeData });
    showToast(result && result.status === 'ok' ? 'Pushed ideal to device!' : 'Failed: ' + ((result && result.error) || 'unknown'), result && result.status === 'ok' ? 'success' : 'error');
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
//  3D VISUALIZATION — Stroke-Phase Position Model
//  Addresses TIDR 6-1-3 (drift prevention) and 2-1-2 (haptic markers)
// ══════════════════════════════════════════════════════════════════
let scene, camera, renderer, controls;
let deviceMeshes = {}; // keyed by dev_role
let waterPlane;
let positionLines = {}, currentStrokePoints = {}, strokeTrails = {};
let hapticMarkers3D = [];
let idealTrailLine = null;
let lastStrokeCounts = {};
let initialQuaternions = {};

// Stroke-phase position model state
let phaseStates = {}; // dev_role -> { vel: {x,y,z}, pos: {x,y,z} }

function getPhaseState(role) {
    if (!phaseStates[role]) phaseStates[role] = { vel: { x: 0, y: 0, z: 0 }, pos: { x: 0, y: 0, z: 0 } };
    return phaseStates[role];
}

function init3D() {
    const canvas = document.getElementById('canvas3d');
    if (!canvas || !window.THREE) return;
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a15);
    scene.fog = new THREE.Fog(0x0a0a15, 8, 20);

    camera = new THREE.PerspectiveCamera(50, canvas.clientWidth / canvas.clientHeight, 0.1, 100);
    camera.position.set(0.8, 0.6, 1.5);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;

    // Devices 1=WristL, 2=WristR, 3=Head, 4=AnkleL, 5=AnkleR
    const wristGeo = new THREE.BoxGeometry(0.12, 0.025, 0.08);
    const boxMatL = new THREE.MeshPhongMaterial({ color: 0x5A85C5, emissive: 0x1a2e4a, specular: 0xffffff, shininess: 60 });
    const boxMatR = new THREE.MeshPhongMaterial({ color: 0xC5A55A, emissive: 0x2a1e08, specular: 0xffffff, shininess: 60 });

    deviceMeshes[1] = new THREE.Mesh(wristGeo, boxMatL);
    deviceMeshes[2] = new THREE.Mesh(wristGeo, boxMatR);

    const headGeo = new THREE.SphereGeometry(0.06, 32, 32);
    const headMat = new THREE.MeshPhongMaterial({ color: 0xef4444, emissive: 0x4a1e1e });
    deviceMeshes[3] = new THREE.Mesh(headGeo, headMat);

    const ankleGeo = new THREE.BoxGeometry(0.1, 0.04, 0.08);
    const ankleMatL = new THREE.MeshPhongMaterial({ color: 0x9333ea, emissive: 0x2e1a4a });
    const ankleMatR = new THREE.MeshPhongMaterial({ color: 0xdb2777, emissive: 0x4a1a2e });
    deviceMeshes[4] = new THREE.Mesh(ankleGeo, ankleMatL);
    deviceMeshes[5] = new THREE.Mesh(ankleGeo, ankleMatR);

    const strapGeo = new THREE.BoxGeometry(0.13, 0.005, 0.015);
    const strapMat = new THREE.MeshPhongMaterial({ color: 0x333333 });
    [1, 2, 4, 5].forEach(role => {
        const s = new THREE.Mesh(strapGeo, strapMat);
        s.position.y = 0.015;
        deviceMeshes[role].add(s);
    });

    [1, 2, 3, 4, 5].forEach(role => {
        deviceMeshes[role].castShadow = true;
        deviceMeshes[role].visible = false;
        scene.add(deviceMeshes[role]);
    });

    // Default fallback
    deviceMeshes[0] = new THREE.Mesh(wristGeo, boxMatR);
    deviceMeshes[0].castShadow = true;
    deviceMeshes[0].visible = false;
    scene.add(deviceMeshes[0]);

    // Lights
    const ambient = new THREE.AmbientLight(0x404060, 0.5);
    scene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.9);
    dirLight.position.set(3, 5, 3);
    dirLight.castShadow = true;
    scene.add(dirLight);
    const rimLight = new THREE.DirectionalLight(0x3366ff, 0.3);
    rimLight.position.set(-2, 1, -3);
    scene.add(rimLight);

    // Water surface
    const waterGeo = new THREE.PlaneGeometry(8, 8, 32, 32);
    const waterMat = new THREE.MeshPhongMaterial({
        color: 0x006699, transparent: true, opacity: 0.2,
        side: THREE.DoubleSide, flatShading: true
    });
    waterPlane = new THREE.Mesh(waterGeo, waterMat);
    waterPlane.rotation.x = -Math.PI / 2;
    waterPlane.position.y = -0.15;
    waterPlane.receiveShadow = true;
    scene.add(waterPlane);

    // Grid
    const grid = new THREE.GridHelper(6, 30, 0x1a1a30, 0x111120);
    grid.position.y = -0.16;
    scene.add(grid);

    // Trail line for current stroke
    const trailGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]);
    positionLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 2 }));
    scene.add(positionLine);

    // Controls
    if (window.THREE.OrbitControls) {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.target.set(0, 0, 0);
        controls.minDistance = 0.5;
        controls.maxDistance = 10;
    }

    animate();
}

function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    // Gentle water wave animation
    if (waterPlane) {
        const t = Date.now() * 0.001;
        const pos = waterPlane.geometry.attributes.position;
        for (let i = 0; i < pos.count; i++) {
            const x = pos.getX(i), z = pos.getZ(i);
            pos.setY(i, Math.sin(x * 2 + t) * 0.005 + Math.cos(z * 1.5 + t * 0.7) * 0.003);
        }
        pos.needsUpdate = true;
    }
    if (renderer && scene && camera) renderer.render(scene, camera);
}

// ── STROKE-PHASE POSITION MODEL ──
// Instead of raw double-integration (which drifts), we model freestyle stroke
// as a constrained arc using the stroke phase + LIA magnitude.
// Phase model for freestyle:
//   Glide:    hand forward, slight descent (negative Z, steady X)
//   Pull:     hand pulls backward through water (positive X sweep, Z dips)
//   Recovery: hand exits water, sweeps forward over surface (arc up and forward)
function computePhasePosition(d, dt, role) {
    const phase = d.stroke_phase || 'idle';
    const liaMag = Math.sqrt(
        (d.acceleration?.ax || 0) ** 2 +
        (d.acceleration?.ay || 0) ** 2 +
        (d.acceleration?.az || 0) ** 2
    );

    const state = getPhaseState(role);
    const decay = 0.92;
    state.vel.x *= decay;
    state.vel.y *= decay;
    state.vel.z *= decay;

    const speed = Math.min(liaMag * 0.02, 0.15) * positionScale * 0.3;

    if (phase === 'pull') {
        state.vel.x += speed * 0.8;
        state.vel.z -= speed * 0.3;
    } else if (phase === 'recovery') {
        state.vel.x -= speed * 0.6;
        state.vel.z += speed * 0.5;
        state.vel.y += speed * 0.2;
    } else {
        state.vel.x -= speed * 0.2;
        state.vel.z -= speed * 0.1;
    }

    state.pos.x += state.vel.x * dt;
    state.pos.y += state.vel.y * dt;
    state.pos.z += state.vel.z * dt;

    return { x: state.pos.x, y: state.pos.z, z: -state.pos.y };
}

function renderFrame(idx) {
    if (!processedData.length || idx >= processedData.length) return;
    const d = processedData[idx];
    const role = d.dev_role || 0;
    const mesh = deviceMeshes[role] || deviceMeshes[0];
    if (mesh) mesh.visible = true;

    let dt = 0.01;
    for (let i = idx - 1; i >= 0 && i >= idx - 50; i--) {
        if ((processedData[i].dev_role || 0) === role) {
            dt = (d.timestamp - processedData[i].timestamp) / 1000;
            break;
        }
    }
    if (dt <= 0 || dt > 1) dt = 0.01;

    // ── ORIENTATION ──
    if (mesh && d.quaternion) {
        const q = d.quaternion;
        if (!initialQuaternions[role]) initialQuaternions[role] = new THREE.Quaternion(q.qx, q.qy, q.qz, q.qw).invert();
        const rawQ = new THREE.Quaternion(q.qx, q.qy, q.qz, q.qw);
        const corrected = initialQuaternions[role].clone().multiply(rawQ);
        mesh.quaternion.copy(corrected);
    }

    // ── POSITION ──
    if (!currentStrokePoints[role]) currentStrokePoints[role] = [];
    if (d.tracking_active && dt > 0 && dt < 0.5) {
        const pos = computePhasePosition(d, dt, role);
        let baseY = 0, baseX = 0, baseZ = 0;
        if (role === 3) baseY = 0.4;
        else if (role === 4) { baseY = -0.5; baseX = -0.2; }
        else if (role === 5) { baseY = -0.5; baseX = 0.2; }
        else if (role === 1) baseX = -0.3;
        else if (role === 2) baseX = 0.3;

        if (mesh) mesh.position.set(pos.x + baseX, pos.y + baseY, pos.z + baseZ);
        currentStrokePoints[role].push(new THREE.Vector3(pos.x + baseX, pos.y + baseY, pos.z + baseZ));

        if (!positionLines[role]) {
            const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3()]);
            const colors = { 1: 0x5A85C5, 2: 0xC5A55A, 3: 0xef4444, 4: 0x9333ea, 5: 0xdb2777, 0: 0xffffff };
            positionLines[role] = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: colors[role] || 0xffffff, linewidth: 2 }));
            scene.add(positionLines[role]);
        }
        positionLines[role].geometry.dispose();
        positionLines[role].geometry = new THREE.BufferGeometry().setFromPoints(currentStrokePoints[role]);
    }

    // ── STROKE BOUNDARY ──
    const lastCount = lastStrokeCounts[role] || 0;
    if (d.stroke_count > lastCount) {
        if (currentStrokePoints[role].length > 3) {
            const colors = [0x3b82f6, 0x22c55e, 0xf59e0b, 0xa855f7, 0xef4444, 0x06b6d4, 0xec4899, 0x84cc16];
            const tg = new THREE.BufferGeometry().setFromPoints([...currentStrokePoints[role]]);
            const tl = new THREE.Line(tg, new THREE.LineBasicMaterial({
                color: colors[Object.keys(strokeTrails).length % colors.length],
                transparent: true, opacity: 0.6
            }));
            scene.add(tl);
            if (!strokeTrails[role]) strokeTrails[role] = [];
            strokeTrails[role].push(tl);
            while (strokeTrails[role].length > 5) { const old = strokeTrails[role].shift(); scene.remove(old); old.geometry.dispose(); }
        }
        currentStrokePoints[role] = [];
        getPhaseState(role).pos = { x: 0, y: 0, z: 0 };
        getPhaseState(role).vel = { x: 0, y: 0, z: 0 };
        lastStrokeCounts[role] = d.stroke_count;
    }

    // ── HAPTIC MARKER ──
    if (d.haptic_fired && mesh) {
        mesh.material.emissive.setHex(0xff2200);
        setTimeout(() => { if (mesh) mesh.material.emissive.setHex(0x2a1e08); }, 200);
        const markerGeo = new THREE.OctahedronGeometry(0.02, 0);
        const markerMat = new THREE.MeshBasicMaterial({ color: 0xff4444 });
        const marker = new THREE.Mesh(markerGeo, markerMat);
        marker.position.copy(mesh.position);
        scene.add(marker);
        hapticMarkers3D.push(marker);
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
    processedData = []; sessionMetrics = null; currentIndex = 0;
    lastStrokeCounts = {}; currentStrokePoints = {};
    phaseStates = {};
    Object.values(strokeTrails).forEach(arr => arr.forEach(t => { if (scene) scene.remove(t); t.geometry.dispose(); }));
    strokeTrails = {};
    Object.values(positionLines).forEach(l => { if (scene) scene.remove(l); l.geometry.dispose(); });
    positionLines = {};
    hapticMarkers3D.forEach(m => { if (scene) scene.remove(m); m.geometry.dispose(); }); hapticMarkers3D = [];
    initialQuaternions = {};
    Object.values(deviceMeshes).forEach(m => { if (m) m.visible = false; });
}
function resetView() {
    if (camera) { camera.position.set(0.8, 0.6, 1.5); if (controls) controls.target.set(0, 0, 0); }
}
function updateScale(v) { positionScale = parseFloat(v) || 3; setText('scale-val', positionScale.toFixed(1) + 'x'); }

// ── INIT ──
window.addEventListener('DOMContentLoaded', () => {
    loadUserProfile();
    loadSavedSessions();
    loadIdealStroke();
    init3D();
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
