/**
 * GoldenForm: Home setup journey and sync playbook copy.
 */
function hasSavedSessions() {
    return Array.isArray(savedSessions) && savedSessions.length > 0;
}

/** Funnel position for setup cards (single source for render + lightweight refresh). */
function journeyStepIsDone(key, s) {
    const snap = s || getOnboardingSnapshot();
    if (key === 'profile') return snap.profileOk;
    if (key === 'devices') return snap.devicesOk;
    if (key === 'cal') return snap.calOk;
    if (key === 'sync') return snap.linkOkForJourney;
    if (key === 'viz') return snap.hasSessionForJourney;
    return false;
}

function getSetupJourneyFunnelState(snap) {
    const s = snap || getOnboardingSnapshot();
    const {
        profileOk, devicesOk, calOk, linkOkForJourney, hasSessionForJourney
    } = s;
    let phase = 'Ready: review your data';
    let currentKey = 'viz';
    if (!profileOk) {
        phase = 'Step 1: Profile';
        currentKey = 'profile';
    } else if (!devicesOk) {
        phase = 'Step 2: Wearables';
        currentKey = 'devices';
    } else if (!calOk) {
        phase = 'Step 3: IMU calibration';
        currentKey = 'cal';
    } else if (!linkOkForJourney) {
        phase = 'Step 4: Check Session sync';
        currentKey = 'sync';
    } else if (!hasSessionForJourney) {
        phase = 'Step 5: Import a recording';
        currentKey = 'viz';
    } else {
        phase = 'Step 5: Analyze';
        currentKey = 'viz';
    }
    return { phase, currentKey, snap: s };
}

function getOnboardingSnapshot() {
    const profileOk = !!(userProfile && userProfile.name);
    const expected = getExpectedWearableCount();
    const devCount = cachedDeviceListLength;
    const devicesOk = devCount >= expected;
    const calOk = typeof calibrationStepOk === 'function' ? calibrationStepOk() : false;
    const hasSession = hasSavedSessions();
    const linkOk = hasWifiSyncCompleted();
    /* Raw LS flags can survive across server/demo resets or first bootstrap without an instance
     * transition — do not show Sync/Analyze as done unless the funnel up to that point is satisfied. */
    const prereq123 = profileOk && devicesOk && calOk;
    const linkOkForJourney = !!linkOk && prereq123;
    const hasSessionForJourney = hasSession && linkOkForJourney;
    return {
        profileOk, expected, devCount, devicesOk, calOk, hasSession, linkOk,
        linkOkForJourney, hasSessionForJourney
    };
}

function _chip(icon, text, variant = '') {
    const v = variant ? (' setup-chip--' + variant) : '';
    return `<span class="setup-chip${v}"><i data-lucide="${icon}"></i><span>${text}</span></span>`;
}

function renderSetupChips(snap) {
    const el = document.getElementById('setup-chips');
    if (!el) return;
    const s = snap || getOnboardingSnapshot();
    const chips = [];

    chips.push(_chip('watch', `${s.devCount}/${s.expected} band${s.expected > 1 ? 's' : ''}`, s.devicesOk ? 'ok' : 'warn'));

    // Calibration: single-device shows %, multi-device shows count ok/expected.
    if (s.expected > 1) {
        let okCount = 0;
        try {
            const rawMap = localStorage.getItem(LS_CAL_MAP_KEY);
            const map = rawMap ? (JSON.parse(rawMap) || {}) : {};
            for (const k of Object.keys(map)) {
                if (isCalibrationSwimReady(map[k])) okCount++;
            }
        } catch { /* ignore */ }
        chips.push(_chip('activity', `Cal ${okCount}/${s.expected}`, s.calOk ? 'ok' : 'warn'));
    } else {
        let calPct = null;
        try {
            const raw = localStorage.getItem(LS_CAL_KEY);
            if (raw) {
                const o = JSON.parse(raw);
                const n = normalizeCal(o);
                if (n) calPct = calQualityPercentFromNormalized(n, o);
            }
        } catch { /* ignore */ }
        chips.push(_chip('activity', calPct != null ? `Cal ${calPct}%` : 'Cal —', s.calOk ? 'ok' : 'warn'));
    }

    const conn = (typeof navDisplayOnline !== 'undefined' && navDisplayOnline) ? 'Wi‑Fi linked' : 'Offline';
    chips.push(_chip('wifi', conn, (typeof navDisplayOnline !== 'undefined' && navDisplayOnline) ? 'ok' : 'muted'));

    if (s.hasSessionForJourney) chips.push(_chip('sparkles', 'Replay ready', 'ok'));
    else chips.push(_chip('database', 'No sessions yet', 'muted'));

    el.innerHTML = chips.join('');
    try { if (typeof window.refreshIcons === 'function') window.refreshIcons(); } catch (e) { /* ignore */ }
}

function renderSetupPrimaryCta(snap) {
    const btn = document.getElementById('setup-primary-cta');
    const label = document.getElementById('setup-primary-cta-label');
    if (!btn || !label) return;
    const s = snap || getOnboardingSnapshot();

    let key = 'profile';
    let text = 'Save profile';
    if (!s.profileOk) { key = 'profile'; text = 'Save your profile'; }
    else if (!s.devicesOk) { key = 'devices'; text = 'Add your band'; }
    else if (!s.calOk) { key = 'cal'; text = (s.expected > 1 ? 'Calibrate both bands' : 'Calibrate on Wi‑Fi'); }
    else if (!s.linkOkForJourney) { key = 'sync'; text = 'Check Session sync'; }
    else if (!s.hasSessionForJourney) { key = 'sync'; text = 'Start logging on the band'; }
    else { key = 'viz'; text = 'Open Analysis'; }

    btn.setAttribute('onclick', `setupJourneyGo('${key}')`);
    label.textContent = text;
}

function maybeShowReadyToLogOnce(snap) {
    const s = snap || getOnboardingSnapshot();
    // Only after all planned devices are registered + calibrated, before first import.
    if (!s.profileOk || !s.devicesOk || !s.calOk || s.hasSessionForJourney) return;
    try {
        const key = `${s.expected}:${(userProfile && userProfile.id != null) ? userProfile.id : 'nouser'}`;
        const shown = localStorage.getItem(LS_READY_TO_LOG_SHOWN);
        if (shown === key) return;
        localStorage.setItem(LS_READY_TO_LOG_SHOWN, key);
    } catch (e) { /* ignore */ }
    try {
        if (typeof showToast === 'function') {
            showToast('Ready to swim. Start logging on the band, then come back and Sync to import.', 'success');
        }
    } catch (e) { /* ignore */ }
}

function renderSetupProgressBar(snap) {
    const fill = document.getElementById('setup-progress-fill');
    const label = document.getElementById('setup-progress-label');
    if (!fill || !label) return;
    const s = snap || getOnboardingSnapshot();
    // 5-stage product flow: Profile, Wearables, Calibration, Link check, Has a session.
    const done =
        (s.profileOk ? 1 : 0) +
        (s.devicesOk ? 1 : 0) +
        (s.calOk ? 1 : 0) +
        (s.linkOkForJourney ? 1 : 0) +
        (s.hasSessionForJourney ? 1 : 0);
    const pct = Math.round((done / 5) * 100);
    fill.style.width = pct + '%';
    label.textContent = `${done}/5`;
}

function updateNavNextHint(snap) {
    const el = document.getElementById('nav-next-hint');
    if (!el) return;
    const { profileOk, expected, devCount, devicesOk, calOk, linkOkForJourney, hasSessionForJourney } = snap || getOnboardingSnapshot();
    if (!profileOk) el.textContent = 'Next: Profile';
    else if (!devicesOk) el.textContent = 'Next: Add bands (' + devCount + '/' + expected + ')';
    else if (!calOk) el.textContent = 'Next: Calibrate (Wi‑Fi)';
    else if (!linkOkForJourney) el.textContent = 'Next: Sync link check';
    else if (!hasSessionForJourney) el.textContent = 'Next: Swim → Sync';
    else el.textContent = 'Next: Replay + Insights';
}

function updateSyncPlaybookMulti() {
    const el = document.getElementById('sync-playbook-multi');
    if (!el) return;
    const exp = getExpectedWearableCount();
    if (exp > 1) {
        el.textContent = 'Two bands: sync each one → then tap Merge.';
    } else {
        el.textContent = 'Optional: add a second band in Settings anytime.';
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
    const { linkOkForJourney } = snap || getOnboardingSnapshot();
    if (linkOkForJourney) {
        el.textContent = '';
        el.style.display = 'none';
    } else {
        el.style.display = 'block';
        el.textContent = 'First Sync now only proves the app reached the wearable. After you swim, Sync now again to pull that recording.';
    }
}

function refreshDeviceRegistrationHints() {
    const slot = document.getElementById('device-reg-slot');
    if (!slot) return;
    const expected = getExpectedWearableCount();
    const n = cachedDeviceListLength;
    if (n >= expected) {
        slot.innerHTML = '<span class="device-reg-slot-inner device-reg-slot-inner--ok">All <strong>' + expected + '</strong> planned wearables are added. You can change the list below anytime.</span>';
        return;
    }
    const stepNum = n + 1;
    slot.innerHTML =
        '<span class="device-reg-slot-inner">' +
        '<strong>Wearable ' + stepNum + ' of ' + expected + '</strong>. Connect to its Wi‑Fi, then tap <strong>Add this wearable</strong>.</span>';
}

/**
 * Update journey step rings / aria without rebuilding the grid (safe with tab switches).
 * Call after navigation or when snapshot changes without a full renderSetupJourney().
 */
function refreshSetupJourneyStepVisuals() {
    const grid = document.querySelector('#setup-journey .journey-grid');
    if (!grid) return;
    const { phase, currentKey, snap } = getSetupJourneyFunnelState();
    const phaseEl = document.getElementById('setup-journey-phase');
    if (phaseEl) phaseEl.textContent = phase;
    grid.querySelectorAll('[data-journey-key]').forEach((el) => {
        const key = el.getAttribute('data-journey-key');
        if (!key) return;
        el.className = 'journey-step';
        if (journeyStepIsDone(key, snap)) el.classList.add('journey-step--done');
        if (currentKey === key) {
            el.classList.add('journey-step--current');
            el.setAttribute('aria-current', 'step');
        } else {
            el.removeAttribute('aria-current');
        }
    });
    try { updateNavNextHint(snap); } catch (e) { /* ignore */ }
}

function renderSetupJourney() {
    const host = document.getElementById('setup-journey');
    const phaseEl = document.getElementById('setup-journey-phase');
    if (!host) return;
    const snap = getOnboardingSnapshot();
    const {
        expected, devCount, hasSessionForJourney, calOk, linkOkForJourney
    } = snap;

    const { phase, currentKey } = getSetupJourneyFunnelState(snap);
    if (phaseEl) phaseEl.textContent = phase;

    function stepClass(key) {
        let c = 'journey-step';
        if (journeyStepIsDone(key, snap)) c += ' journey-step--done';
        if (currentKey === key) c += ' journey-step--current';
        return c;
    }

    const step2Text = expected > 1
        ? ('Add both wrists (one Wi‑Fi at a time).')
        : 'Add your band (right or left wrist).';

    const step3Text = !calOk
        ? ('On band Wi‑Fi: get all four channels to green (≥2).')
        : ('Calibration saved. You’re good to swim.');

    const step4Text = !linkOkForJourney
        ? ('Join GoldenForm Wi‑Fi → Session → Sync now (0 sessions is OK).')
        : ('Link confirmed. After a swim, Sync now pulls the file.');

    const step5Label = hasSessionForJourney ? '5 · Analyze' : '5 · Record, then import';
    const step5Text = hasSessionForJourney
        ? 'Replay 3D, charts, haptics, Insights.'
        : ('Swim offline → then Sync now to import.');

    const steps = [
        { key: 'profile', label: '1 · Profile', text: 'Name + body measurements.', tab: 'settings', cta: 'Profile' },
        { key: 'devices', label: '2 · Wearables (' + devCount + '/' + expected + ')', text: step2Text, tab: 'settings', cta: 'Wearables' },
        { key: 'cal', label: '3 · Calibrate', text: step3Text, tab: 'settings', cta: 'Calibration' },
        { key: 'sync', label: '4 · Sync', text: step4Text, tab: 'session', cta: 'Session' },
        { key: 'viz', label: step5Label, text: step5Text, tab: hasSessionForJourney ? 'analysis' : 'session', cta: hasSessionForJourney ? 'Analysis' : 'Session' }
    ];

    host.innerHTML = '<div class="journey-grid" role="list">' + steps.map(s =>
        '<div class="' + stepClass(s.key) + '" data-journey-key="' + s.key + '" role="listitem"' +
        (currentKey === s.key ? ' aria-current="step"' : '') +
        '><div class="journey-step-head"><span>' + s.label + '</span>' +
        (journeyStepIsDone(s.key, snap) ? '<span class="journey-check">✓</span>' : '') +
        '</div><p>' + s.text + '</p><button type="button" class="btn btn-outline btn-sm" onclick="setupJourneyGo(\'' + s.key + '\')">' + s.cta + '</button></div>'
    ).join('') + '</div>';
    try { if (typeof window.refreshIcons === 'function') window.refreshIcons(); } catch (e) { /* ignore */ }

    const cont = document.getElementById('setup-journey-continue');
    if (cont) {
        cont.innerHTML = '<p class="setup-continue"><strong>Flow:</strong> Register → Calibrate (Wi‑Fi) → Swim (offline) → Sync → Replay + Insights.</p>';
    }

    updateNavNextHint(snap);
    renderSetupChips(snap);
    renderSetupPrimaryCta(snap);
    renderSetupProgressBar(snap);
    maybeShowReadyToLogOnce(snap);
    refreshDeviceRegistrationHints();
    updateSyncPlaybookFirstTime(snap);
    updateHomeOnboardingPanels();
}

// Ensure icons render for static playbook markup too.
try { if (typeof window.refreshIcons === 'function') window.refreshIcons(); } catch (e) { /* ignore */ }

/** Journey buttons: scroll to calibration card on Settings when key is cal. */
function setupJourneyGo(key) {
    if (key === 'cal') {
        switchTab('settings', { force: true });
        requestAnimationFrame(() => {
            const el = document.getElementById('card-imu-calibration');
            if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
        return;
    }
    if (key === 'devices') {
        switchTab('settings', { force: true });
        requestAnimationFrame(() => {
            const el = document.querySelector('.device-registration-card');
            if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
        return;
    }
    if (key === 'viz') {
        if (typeof hasSavedSessions === 'function' && hasSavedSessions()) switchTab('analysis', { force: true });
        else switchTab('session', { force: true });
        return;
    }
    if (key === 'sync') {
        switchTab('session', { force: true });
        return;
    }
    if (key === 'profile') {
        switchTab('settings', { force: true });
    }
}
