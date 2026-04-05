/**
 * GoldenForm: calibration snapshot (browser) and live fusion status from the device.
 */
// Persistence: LS_CAL_KEY in gf_constants.js

function normalizeCal(c) {
    if (!c || typeof c !== 'object') return null;
    return {
        sys: Math.min(3, Math.max(0, Number(c.sys) || 0)),
        gyro: Math.min(3, Math.max(0, Number(c.gyro) || 0)),
        accel: Math.min(3, Math.max(0, Number(c.accel) || 0)),
        mag: Math.min(3, Math.max(0, Number(c.mag) || 0))
    };
}

function calQualityPercentFromNormalized(n) {
    if (!n) return 0;
    return Math.round((n.sys + n.gyro + n.accel + n.mag) / 12 * 100);
}

/** True when fusion is strong enough to trust before a swim (all channels at least 2, quality at least 75%). */
function isCalibrationSwimReady(cal) {
    const n = normalizeCal(cal);
    if (!n) return false;
    if (n.sys < 2 || n.gyro < 2 || n.accel < 2 || n.mag < 2) return false;
    return calQualityPercentFromNormalized(n) >= 75;
}

/** Setup journey: step complete when saved snapshot qualifies (stable after you disconnect). */
function calibrationStepOk() {
    try {
        const raw = localStorage.getItem(LS_CAL_KEY);
        if (!raw) return false;
        const o = JSON.parse(raw);
        return isCalibrationSwimReady(o);
    } catch {
        return false;
    }
}

/** Nav pill: live fusion from /api/device_info when connected; offline uses last browser snapshot (LS_CAL_KEY); no snapshot shows Cal …. */
function updateNavCalPill(res) {
    const el = document.getElementById('nav-cal-pill');
    if (!el) return;
    let cal = null;
    if (res && res.cal) cal = normalizeCal(res.cal);
    if (!cal) {
        try {
            const raw = localStorage.getItem(LS_CAL_KEY);
            if (raw) cal = normalizeCal(JSON.parse(raw));
        } catch { /* ignore */ }
    }
    if (!cal) {
        el.textContent = 'Cal …';
        el.className = 'nav-cal-pill nav-cal-pill--muted';
        el.title = 'Connect wearable. Calibration updates live here.';
        return;
    }
    const pct = calQualityPercentFromNormalized(cal);
    const weak = cal.sys < 2 || cal.gyro < 2 || cal.accel < 2 || cal.mag < 2;
    el.textContent = 'Cal ' + pct + '%';
    el.className = 'nav-cal-pill ' + (weak ? 'nav-cal-pill--warn' : 'nav-cal-pill--ok');
    el.title = 'S' + cal.sys + ' G' + cal.gyro + ' A' + cal.accel + ' M' + cal.mag + ' (tap → Session)';
}

/**
 * Session tab: show whether recorded samples include strong fusion (per-sample calibration).
 * Dims viz slightly when recording had weak cal; nudge to fix IMU before trusting 3D.
 */
function updateSessionCalBanner() {
    const el = document.getElementById('session-cal-banner');
    const page = document.getElementById('page-session');
    if (!el || !page) return;
    const pd = typeof processedData !== 'undefined' && processedData ? processedData : [];
    if (!pd.length) {
        el.hidden = true;
        el.innerHTML = '';
        page.classList.remove('page-session--weak-cal');
        return;
    }
    let n = 0;
    let mins = { sys: 3, gyro: 3, accel: 3, mag: 3 };
    for (const p of pd) {
        const c = p.calibration || p.cal;
        if (!c || typeof c !== 'object') continue;
        const s = Math.min(3, Math.max(0, Number(c.sys) || 0));
        const g = Math.min(3, Math.max(0, Number(c.gyro) || 0));
        const a = Math.min(3, Math.max(0, Number(c.accel) || 0));
        const m = Math.min(3, Math.max(0, Number(c.mag) || 0));
        mins.sys = Math.min(mins.sys, s);
        mins.gyro = Math.min(mins.gyro, g);
        mins.accel = Math.min(mins.accel, a);
        mins.mag = Math.min(mins.mag, m);
        n++;
    }
    el.hidden = false;
    if (n === 0) {
        el.className = 'session-cal-banner session-cal-banner--muted';
        el.innerHTML = '<span class="session-cal-banner__icon">○</span><span>Recording has no per-sample cal. Check live <strong>Cal</strong> in the nav before the next swim.</span>';
        page.classList.remove('page-session--weak-cal');
        return;
    }
    const weak = mins.sys < 2 || mins.gyro < 2 || mins.accel < 2 || mins.mag < 2;
    page.classList.toggle('page-session--weak-cal', weak);
    const minTag = 'min S' + mins.sys + ' G' + mins.gyro + ' A' + mins.accel + ' M' + mins.mag;
    if (weak) {
        el.className = 'session-cal-banner session-cal-banner--warn';
        el.innerHTML = '<span class="session-cal-banner__icon">!</span><span>Weak fusion in this file (' + minTag + '). 3D is indicative only. Calibrate the wearable, then re-record.</span>';
    } else {
        el.className = 'session-cal-banner session-cal-banner--ok';
        el.innerHTML = '<span class="session-cal-banner__icon">✓</span><span>Fusion OK in recording (' + minTag + ').</span>';
    }
}

function refreshCalibrationSavedStrip() {
    const el = document.getElementById('cal-saved-strip');
    if (!el) return;
    try {
        const raw = localStorage.getItem(LS_CAL_KEY);
        if (!raw) {
            el.textContent = 'No snapshot yet. Connect, move through the guide, and we auto-save when quality rises.';
            return;
        }
        const o = JSON.parse(raw);
        const ageMin = o.savedAt ? Math.round((Date.now() - o.savedAt) / 60000) : '?';
        el.textContent = 'Saved ' + calQualityPercentFromNormalized(normalizeCal(o)) + '% · ' + ageMin + 'm' + (o.device_id != null ? ' · #' + o.device_id : '');
    } catch {
        el.textContent = '';
    }
}

function persistCalibrationSnapshot(cal, deviceId) {
    const n = normalizeCal(cal);
    if (!n) return;
    try {
        const prev = JSON.parse(localStorage.getItem(LS_CAL_KEY) || '{}');
        localStorage.setItem(LS_CAL_KEY, JSON.stringify({
            ...n,
            device_id: deviceId != null ? deviceId : prev.device_id,
            savedAt: Date.now()
        }));
    } catch (e) { /* quota */ }
    refreshCalibrationSavedStrip();
    updateNavCalPill({ cal: n });
    if (typeof renderSetupJourney === 'function') renderSetupJourney();
}

/** Auto-save when total calibration quality improves (no extra taps). */
function maybeAutoSaveCalibrationFromPoll(res) {
    if (!res || res.error || res.status === 'disconnected' || !res.cal) return;
    const n = normalizeCal(res.cal);
    if (!n) return;
    let prev = null;
    try { prev = JSON.parse(localStorage.getItem(LS_CAL_KEY) || 'null'); } catch { prev = null; }
    const score = n.sys + n.gyro + n.accel + n.mag;
    const prevScore = prev && typeof prev.sys === 'number' ? prev.sys + prev.gyro + prev.accel + prev.mag : -1;
    const wasReady = prev && typeof prev === 'object' && isCalibrationSwimReady(prev);
    const nowReady = isCalibrationSwimReady(n);
    if (score > prevScore || (nowReady && !wasReady)) {
        persistCalibrationSnapshot(n, res.device_id);
    }
}

function openCalibrationGuide() {
    const m = document.getElementById('cal-guide-modal');
    if (!m) return;
    m.classList.add('show');
}
function closeCalibrationGuide() {
    const m = document.getElementById('cal-guide-modal');
    if (!m) return;
    m.classList.remove('show');
}

async function saveCalibrationSnapshotManual() {
    const res = await apiGet('/api/device_info');
    if (!res || res.error || res.status === 'disconnected' || res.device_id === undefined) {
        showToast('Device not reachable. Join the GoldenForm Wi‑Fi first.', 'error');
        return;
    }
    const cal = res.cal;
    if (!cal) {
        showToast('No calibration data from device yet.', 'error');
        return;
    }
    persistCalibrationSnapshot(cal, res.device_id);
    updateCalibrationDisplay({ cal: normalizeCal(cal) });
    showToast('Calibration snapshot saved in this browser', 'success');
}

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
        if (isCalibrationSwimReady(cal)) {
            hintEl.textContent = 'Strong enough for fusion. You can head to the pool when you are ready.';
        } else if (mag < 2) hintEl.textContent = 'Mag: wide figure 8, away from metal, aim for 3/3.';
        else if (gyro < 2) hintEl.textContent = 'Gyro: rest flat on the table, aim for 3/3.';
        else if (accel < 2) hintEl.textContent = 'Accel: six faces, slow and flat, aim for 3/3.';
        else if (sys < 2) hintEl.textContent = 'System: finish gyro, accel, and mag first.';
        else if (quality >= 75) hintEl.textContent = 'Close. Push any yellow channel to 3/3.';
        else hintEl.textContent = 'Bring each channel toward 3/3 using the guide.';
    }

    const nCal = normalizeCal(cal);
    const readyEl = document.getElementById('cal-ready-banner');
    const cardEl = document.getElementById('card-imu-calibration');
    const liveReady = isCalibrationSwimReady(cal);
    if (readyEl) {
        if (liveReady) {
            readyEl.hidden = false;
            readyEl.innerHTML =
                '<div class="cal-ready-banner__inner">' +
                '<span class="cal-ready-banner__title">You are good to swim</span>' +
                '<span class="cal-ready-banner__sub">Fusion looks solid. Start recording on the band in the water. The radio stays off during the swim; feedback comes from the motor on your wrist, not over Wi‑Fi.</span>' +
                '</div>';
        } else {
            readyEl.hidden = true;
            readyEl.innerHTML = '';
        }
    }
    if (cardEl) cardEl.classList.toggle('card--calibration-ready', liveReady);

    updateNavCalPill({ cal: nCal });
}

/** When /api/device_info has no cal (offline), clear the Settings card so we never show stale nibbles that disagree with the device. */
function clearCalibrationLiveDisplay() {
    ['cal-sys', 'cal-gyro', 'cal-accel', 'cal-mag'].forEach((id) => {
        const el = document.getElementById(id);
        if (el) {
            el.textContent = '--';
            el.className = 'cal-badge cal-warn';
        }
    });
    const q = document.getElementById('cal-quality');
    if (q) q.textContent = '--';
    const hintEl = document.getElementById('cal-hint');
    if (hintEl) {
        hintEl.textContent = 'Join the band Wi‑Fi on this computer to refresh live fusion numbers.';
    }
    const readyEl = document.getElementById('cal-ready-banner');
    if (readyEl) {
        readyEl.hidden = true;
        readyEl.innerHTML = '';
    }
    const cardEl = document.getElementById('card-imu-calibration');
    if (cardEl) cardEl.classList.remove('card--calibration-ready');
}

