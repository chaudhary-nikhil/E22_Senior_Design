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

function _opModeFromPayload(payload) {
    // Firmware /api/device_info includes `bno_opmode` (BNO055 OPR_MODE).
    const v = payload && payload.bno_opmode != null ? Number(payload.bno_opmode) : null;
    return Number.isFinite(v) ? v : null;
}

function _isImuPlusMode(payloadOrMode) {
    const m = (typeof payloadOrMode === 'number') ? payloadOrMode : _opModeFromPayload(payloadOrMode);
    return m === 0x08; // BNO055_OPERATION_MODE_IMUPLUS
}

function calQualityPercentFromNormalized(n, payloadOrMode = null) {
    if (!n) return 0;
    const imuPlus = _isImuPlusMode(payloadOrMode);
    if (imuPlus) {
        // IMUPLUS (0x08) ignores magnetometer fusion; treat Gyro+Accel as the full scale (6 points).
        return Math.round((n.gyro + n.accel) / 6 * 100);
    }
    /* NDOF (0x0C) system-cal nibble is volatile and commonly drops during motion even when
     * fusion is practically stable. For user-facing readiness and the “shut AP off” policy,
     * treat Gyro+Accel+Mag as the relevant channels (9 points). */
    return Math.round((n.sys + n.gyro + n.accel + n.mag) / 12 * 100);
}

/** True when fusion is strong enough to trust before a swim.
 * Note: many GoldenForm builds use BNO055 IMUPLUS mode (0x08) which does not
 * rely on magnetometer fusion. In that mode Sys/Mag can remain 0 in-water and
 * on some benches; we treat Gyro+Accel as the swim-critical channels. */
function isCalibrationSwimReady(cal, payload = null) {
    const n = normalizeCal(cal);
    if (!n) return false;
    const imuPlus = _isImuPlusMode(payload);
    const imuPlusLikely = (n.mag === 0 && n.sys === 0 && (n.gyro >= 2 || n.accel >= 2));
    if (imuPlus || (!payload && imuPlusLikely)) {
        /* Match firmware/AP policy: full 3/3 on gyro + accel before “ready to log”. */
        return (n.gyro >= 3 && n.accel >= 3);
    }
    /* NDOF / full fusion: require all four registers at 3/3 to match firmware NVS save policy. */
    return (n.sys >= 3 && n.gyro >= 3 && n.accel >= 3 && n.mag >= 3);
}

/** Setup journey: step complete when saved snapshot qualifies (stable after you disconnect). */
function calibrationStepOk() {
    try {
        const expected = (typeof getExpectedWearableCount === 'function') ? getExpectedWearableCount() : 1;

        // Multi-device: require a swim-ready snapshot for each expected device_id.
        if (expected > 1) {
            const rawMap = localStorage.getItem(LS_CAL_MAP_KEY);
            if (!rawMap) return false;
            const map = JSON.parse(rawMap) || {};
            const ids = Object.keys(map).map((k) => Number(k)).filter((n) => Number.isFinite(n) && n > 0);
            let okCount = 0;
            for (const id of ids) {
                if (isCalibrationSwimReady(map[String(id)])) okCount++;
            }
            return okCount >= expected;
        }

        // Single device: legacy snapshot key.
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
    let source = 'saved';
    if (res && res.cal) {
        cal = normalizeCal(res.cal);
        source = 'live';
    }
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

    let sessionMinNote = '';
    const pd = typeof processedData !== 'undefined' && processedData ? processedData : [];
    if (pd.length > 10) {
        let mins = { sys: 3, gyro: 3, accel: 3, mag: 3 };
        let n = 0;
        for (const p of pd) {
            const c = p.calibration || p.cal;
            if (!c || typeof c !== 'object') continue;
            mins.sys = Math.min(mins.sys, Math.min(3, Math.max(0, Number(c.sys) || 0)));
            mins.gyro = Math.min(mins.gyro, Math.min(3, Math.max(0, Number(c.gyro) || 0)));
            mins.accel = Math.min(mins.accel, Math.min(3, Math.max(0, Number(c.accel) || 0)));
            mins.mag = Math.min(mins.mag, Math.min(3, Math.max(0, Number(c.mag) || 0)));
            n++;
        }
        if (n > 0) {
            sessionMinNote = ' | Rec min: S' + mins.sys + ' G' + mins.gyro + ' A' + mins.accel + ' M' + mins.mag;
        }
    }

    const pct = calQualityPercentFromNormalized(cal, res || cal);
    const imuPlus = _isImuPlusMode(res);
    const imuPlusLikely = (cal.mag === 0 && cal.sys === 0 && (cal.gyro >= 2 || cal.accel >= 2));
    const weak = (imuPlus || (!res && imuPlusLikely))
        ? (cal.gyro < 3 || cal.accel < 3)
        : (cal.gyro < 2 || cal.accel < 2 || cal.mag < 2);
    el.textContent = 'Cal ' + pct + '%';
    el.className = 'nav-cal-pill ' + (weak ? 'nav-cal-pill--warn' : 'nav-cal-pill--ok');
    const modeHint = (imuPlus || (!res && imuPlusLikely)) ? 'IMUPLUS' : 'NDOF';
    const srcLabel = source === 'live' ? 'Live' : 'Saved';
    const pillExplain = 'Nav Cal = your wearable bench right now (live) or the last snapshot saved in this browser—not the worst moment inside an open recording.';
    el.title = pillExplain + ' ' + srcLabel + ' · ' + modeHint + ' · S' + cal.sys + ' G' + cal.gyro + ' A' + cal.accel + ' M' + cal.mag + sessionMinNote;
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
    let lowSys = 0;
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
        if (s < 3) lowSys++;
        n++;
    }
    el.hidden = false;
    if (n === 0) {
        el.className = 'session-cal-banner session-cal-banner--muted';
        el.innerHTML = '<span class="session-cal-banner__icon">○</span><span>Recording has no per-sample cal. The nav <strong>Cal</strong> pill shows live fusion or your saved browser snapshot; it can look better than this file if cal dipped during the swim.</span>';
        page.classList.remove('page-session--weak-cal');
        return;
    }
    const imuPlusLikely = (mins.mag === 0 && mins.sys === 0 && (mins.gyro >= 2 || mins.accel >= 2));
    const weak = imuPlusLikely
        ? (mins.gyro < 3 || mins.accel < 3)
        : (mins.sys < 3 || mins.gyro < 2 || mins.accel < 2 || mins.mag < 2);
    page.classList.toggle('page-session--weak-cal', weak);
    const minTag = 'min S' + mins.sys + ' G' + mins.gyro + ' A' + mins.accel + ' M' + mins.mag;
    const rareLowSys = n > 0 && mins.sys === 0 && lowSys <= Math.max(1, Math.floor(n * 0.03));
    if (weak) {
        el.className = 'session-cal-banner session-cal-banner--warn';
        let extra = '';
        if (rareLowSys) {
            extra = ' Only ' + lowSys + ' of ' + n + ' samples had S&lt;3 (often the first moments after power‑up). The minimum is from that tail, not the whole swim.';
        }
        el.innerHTML = '<span class="session-cal-banner__icon">!</span><span><strong>Weak fusion in this file</strong> (' + minTag + ' = lowest per-sample quality in the recording). Use this for how trustworthy this swim’s orientation is. <strong>Nav Cal</strong> is separate: live bench or your last browser snapshot, often higher than the worst moment in the file.' + extra + '</span>';
    } else {
        el.className = 'session-cal-banner session-cal-banner--ok';
        el.innerHTML = '<span class="session-cal-banner__icon">✓</span><span>Fusion OK in this recording (' + minTag + '). Nav <strong>Cal</strong> stays live/saved bench; this banner reflects samples in this file only.</span>';
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
        el.textContent = 'Saved ' + calQualityPercentFromNormalized(normalizeCal(o), o) + '% · ' + ageMin + 'm' + (o.device_id != null ? ' · #' + o.device_id : '');
    } catch {
        el.textContent = '';
    }
}

function persistCalibrationSnapshot(cal, deviceId) {
    const n = normalizeCal(cal);
    if (!n) return;
    const opmode = _opModeFromPayload(cal);
    const mergeMax = (a, b) => {
        const A = normalizeCal(a) || { sys: 0, gyro: 0, accel: 0, mag: 0 };
        const B = normalizeCal(b) || { sys: 0, gyro: 0, accel: 0, mag: 0 };
        return {
            sys: Math.max(A.sys || 0, B.sys || 0),
            gyro: Math.max(A.gyro || 0, B.gyro || 0),
            accel: Math.max(A.accel || 0, B.accel || 0),
            mag: Math.max(A.mag || 0, B.mag || 0),
        };
    };
    try {
        const prev = JSON.parse(localStorage.getItem(LS_CAL_KEY) || '{}');
        const merged = mergeMax(prev, n);
        localStorage.setItem(LS_CAL_KEY, JSON.stringify({
            ...merged,
            bno_opmode: opmode != null ? opmode : prev.bno_opmode,
            device_id: deviceId != null ? deviceId : prev.device_id,
            savedAt: Date.now()
        }));
    } catch (e) { /* quota */ }

    // Also store per-device snapshots for multi-wearable setups.
    try {
        if (deviceId != null) {
            const id = Number(deviceId);
            if (Number.isFinite(id) && id > 0) {
                const rawMap = localStorage.getItem(LS_CAL_MAP_KEY);
                const map = rawMap ? (JSON.parse(rawMap) || {}) : {};
                const prevDev = map[String(id)] || null;
                const mergedDev = mergeMax(prevDev, n);
                map[String(id)] = {
                    ...mergedDev,
                    bno_opmode: opmode,
                    device_id: id,
                    savedAt: Date.now(),
                };
                localStorage.setItem(LS_CAL_MAP_KEY, JSON.stringify(map));
            }
        }
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
    const imuPlus = _isImuPlusMode(res);
    const score = imuPlus ? (n.gyro + n.accel) : (n.sys + n.gyro + n.accel + n.mag);
    const prevImuPlus = _isImuPlusMode(prev);
    const prevScore = prev && typeof prev === 'object'
        ? (prevImuPlus ? ((prev.gyro || 0) + (prev.accel || 0)) : ((prev.sys || 0) + (prev.gyro || 0) + (prev.accel || 0) + (prev.mag || 0)))
        : -1;
    const scoreFinal = score;
    const wasReady = prev && typeof prev === 'object' && isCalibrationSwimReady(prev, prev);
    const nowReady = isCalibrationSwimReady(n, res);
    if (scoreFinal > prevScore || (nowReady && !wasReady)) {
        persistCalibrationSnapshot({ ...n, bno_opmode: res.bno_opmode }, res.device_id);
        if (nowReady && !wasReady) {
            try { if (typeof showToast === 'function') showToast('Calibration ready — saved snapshot', 'success'); } catch (e) { /* ignore */ }
            if (typeof gfNotifyBandRegistrationQuiet === 'function') {
                gfNotifyBandRegistrationQuiet().catch(() => {});
            }
        }
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
    let res = null;
    if (typeof fetchDeviceInfoMerged === 'function') {
        const merged = await fetchDeviceInfoMerged();
        res = merged.res;
    } else {
        res = await apiGet('/api/device_info');
    }
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
    if (typeof gfNotifyBandRegistrationQuiet === 'function') {
        gfNotifyBandRegistrationQuiet().catch(() => {});
    }
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
    const imuPlus = _isImuPlusMode(d);
    const quality = calQualityPercentFromNormalized({ sys, gyro, accel, mag }, d);
    setText('cal-quality', quality + '%');

    const hintEl = document.getElementById('cal-hint');
    if (hintEl) {
        const imuPlus = _isImuPlusMode(d);
        const imuPlusLikely = (mag === 0 && sys === 0 && (gyro >= 2 || accel >= 2));
        if (isCalibrationSwimReady(cal, d)) {
            hintEl.textContent = (imuPlus || imuPlusLikely)
                ? 'Good to swim (IMUPLUS): gyro + accel are solid. Mag/System are not used in this mode.'
                : 'Good to swim (NDOF): all registers at 3/3. Calibration saved to device.';
        } else if ((imuPlus || imuPlusLikely) && (gyro < 3 || accel < 3)) {
            hintEl.textContent = 'IMUPLUS mode: bring Gyro and Accel to 3/3 (then you are ready to log).';
        } else if (sys < 3 && gyro >= 3 && accel >= 3 && mag >= 3) {
            hintEl.textContent = 'NDOF: System still at ' + sys + '/3. Slow rotations in all axes to lock in System calibration.';
        } else if (mag < 2) hintEl.textContent = 'Mag: wide figure 8, away from metal, aim for 3/3.';
        else if (gyro < 2) hintEl.textContent = 'Gyro: rest flat on the table, aim for 3/3.';
        else if (accel < 2) hintEl.textContent = 'Accel: six faces, slow and flat, aim for 3/3.';
        else if (!imuPlus && sys < 3) hintEl.textContent = 'System at ' + sys + '/3. Slow rotations to bring System to 3/3 — device saves calibration only when all registers reach 3/3.';
        else hintEl.textContent = 'Bring each channel toward 3/3 using the guide.';
    }

    const nCal = normalizeCal(cal);
    const readyEl = document.getElementById('cal-ready-banner');
    const cardEl = document.getElementById('card-imu-calibration');
    const liveReady = isCalibrationSwimReady(cal, d);
    if (readyEl) {
        if (liveReady) {
            readyEl.hidden = false;
            readyEl.innerHTML =
                '<div class="cal-ready-banner__inner">' +
                '<span class="cal-ready-banner__title">You are good to swim</span>' +
                '<span class="cal-ready-banner__sub">Start logging on the band. Wi‑Fi will shut off automatically after calibration to save power. After your swim, use Sync to import.</span>' +
                '<div class="cal-ready-banner__actions" style="margin-top:10px;display:flex;gap:10px;flex-wrap:wrap;">' +
                '<button type="button" class="btn btn-gold btn-sm" onclick="switchTab(\'session\')">Next: Session</button>' +
                '<button type="button" class="btn btn-outline btn-sm" onclick="switchTab(\'analysis\')">View Analysis</button>' +
                '</div>' +
                '</div>';
        } else {
            readyEl.hidden = true;
            readyEl.innerHTML = '';
        }
    }
    if (cardEl) cardEl.classList.toggle('card--calibration-ready', liveReady);

    const devParams = document.getElementById('cal-device-params');
    if (devParams) {
        const sk = d.skill_level != null ? String(d.skill_level) : '';
        const ws = d.wingspan_cm != null && Number.isFinite(Number(d.wingspan_cm)) ? Number(d.wingspan_cm) : null;
        const ht = d.height_cm != null && Number.isFinite(Number(d.height_cm)) ? Number(d.height_cm) : null;
        const hp = d.haptic_profile && typeof d.haptic_profile === 'object' ? d.haptic_profile : null;
        const hasDevice = !!(sk || ws != null || ht != null || hp);
        /* Only refresh when payload is full device_info — session scrub passes samples without these fields. */
        if (hasDevice) {
            const parts = [];
            if (sk) {
                parts.push('Skill on band: <strong>' + sk.charAt(0).toUpperCase() + sk.slice(1) + '</strong>');
            }
            if (ws != null) {
                parts.push('wingspan ' + ws.toFixed(1) + ' cm');
            }
            if (ht != null) {
                parts.push('height ' + ht.toFixed(1) + ' cm');
            }
            if (hp && hp.threshold != null) {
                const th = Number(hp.threshold);
                const et = hp.entry_tol_deg != null ? Number(hp.entry_tol_deg) : NaN;
                parts.push(
                    'haptic DTW threshold ' + (Number.isFinite(th) ? th.toFixed(2) : '—') +
                    ', entry ±' + (Number.isFinite(et) ? et.toFixed(0) : '—') + '°'
                );
            }
            devParams.innerHTML = '<span style="color:var(--text3)">Live from firmware</span> · ' + parts.join(' · ');
            devParams.hidden = false;
        }
    }

    updateNavCalPill({ cal: nCal });
}

/** When /api/device_info has no cal (offline), clear the Settings card so we never show stale nibbles that disagree with the device. */
function clearCalibrationLiveDisplay() {
    // Offline: show the last saved snapshot if available (so users still see numbers).
    try {
        const raw = localStorage.getItem(LS_CAL_KEY);
        if (raw) {
            const o = JSON.parse(raw);
            const n = normalizeCal(o);
            if (n) {
                updateCalibrationDisplay({ cal: n });
                const hintEl = document.getElementById('cal-hint');
                if (hintEl) {
                    hintEl.textContent = 'Offline — showing last saved calibration snapshot. Join band Wi‑Fi to refresh live values.';
                }
                return;
            }
        }
    } catch { /* ignore */ }

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
    const devParams = document.getElementById('cal-device-params');
    if (devParams) {
        devParams.hidden = true;
        devParams.textContent = '';
    }
}

