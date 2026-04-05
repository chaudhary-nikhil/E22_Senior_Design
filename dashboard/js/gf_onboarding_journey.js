/**
 * GoldenForm: Home setup journey and sync playbook copy.
 */
function hasSavedSessions() {
    return Array.isArray(savedSessions) && savedSessions.length > 0;
}

function getOnboardingSnapshot() {
    const profileOk = !!(userProfile && userProfile.name);
    const expected = getExpectedWearableCount();
    const devCount = cachedDeviceListLength;
    const devicesOk = devCount >= expected;
    const calOk = typeof calibrationStepOk === 'function' ? calibrationStepOk() : false;
    const hasSession = hasSavedSessions();
    const linkOk = hasWifiSyncCompleted();
    return { profileOk, expected, devCount, devicesOk, calOk, hasSession, linkOk };
}

function updateNavNextHint(snap) {
    const el = document.getElementById('nav-next-hint');
    if (!el) return;
    const { profileOk, expected, devCount, devicesOk, calOk, hasSession, linkOk } = snap || getOnboardingSnapshot();
    if (!profileOk) el.textContent = 'Next: save your profile';
    else if (!devicesOk) el.textContent = 'Next: add wearables (' + devCount + '/' + expected + ')';
    else if (!calOk) el.textContent = 'Next: Settings, finish IMU calibration on Wi‑Fi';
    else if (!linkOk) el.textContent = 'Next: Session, first Sync now (link check; 0 files OK)';
    else if (!hasSession) el.textContent = 'Next: record a swim, then Sync now to import it';
    else el.textContent = 'Next: replay session and Insights';
}

function updateSyncPlaybookMulti() {
    const el = document.getElementById('sync-playbook-multi');
    if (!el) return;
    const exp = getExpectedWearableCount();
    if (exp > 1) {
        el.textContent = 'You plan to use ' + exp + ' wearables. Connect one Wi‑Fi at a time, sync each, then use Merge to combine.';
    } else {
        el.textContent = 'You can add another wearable anytime in Settings. The checklist updates automatically.';
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

function renderSetupJourney() {
    const host = document.getElementById('setup-journey');
    const phaseEl = document.getElementById('setup-journey-phase');
    if (!host) return;
    const snap = getOnboardingSnapshot();
    const { profileOk, expected, devCount, devicesOk, calOk, hasSession, linkOk } = snap;

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
    } else if (!linkOk) {
        phase = 'Step 4: Check Session sync';
        currentKey = 'sync';
    } else if (!hasSession) {
        phase = 'Step 5: Import a recording';
        currentKey = 'viz';
    } else {
        phase = 'Step 5: Analyze';
        currentKey = 'viz';
    }
    if (phaseEl) phaseEl.textContent = phase;

    function isStepDone(key) {
        if (key === 'profile') return profileOk;
        if (key === 'devices') return devicesOk;
        if (key === 'cal') return calOk;
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
        ? ('Join each band’s Wi‑Fi once, tap <strong>Add this wearable</strong>, repeat until ' + expected + ' show in the list.')
        : 'Join the band’s Wi‑Fi, tap <strong>Add this wearable</strong>.';

    const step3Text = !calOk
        ? ('On the same Wi‑Fi, open the calibration card below. Watch the 0 to 3 badges move live, then get to green on all four. Do this before your first swim.')
        : ('Saved snapshot meets the bar. You can still open Settings anytime to re-check fusion.');

    const step4Text = !linkOk
        ? ('Goal: prove Session can talk to the band. Put the wearable in <strong>sync mode</strong>, join its <strong>GoldenForm</strong> Wi‑Fi on this computer, open Session, tap <strong>Sync now</strong>. <em>No swim file yet is normal.</em> You are only checking the link.')
        : ('A Session sync finished while you were on the band’s Wi‑Fi. More than one unit: repeat per band, then <strong>Merge</strong>.');

    const step5Label = hasSession ? '5 · Analyze' : '5 · Record, then import';
    const step5Text = hasSession
        ? 'Replay, charts, form score, ideal comparison, Insights.'
        : ('After a swim: sync mode on the band, join Wi‑Fi, <strong>Sync now</strong> to <em>import</em> that recording. Step 4 had no file on the card yet; this step does.');

    const steps = [
        { key: 'profile', label: '1 · Profile', text: 'Name, wingspan, pool length, and how many wearables you use.', tab: 'settings', cta: 'Open Settings' },
        { key: 'devices', label: '2 · Wearables (' + devCount + '/' + expected + ')', text: step2Text, tab: 'settings', cta: 'Wearables' },
        { key: 'cal', label: '3 · IMU calibration', text: step3Text, tab: 'settings', cta: 'Open calibration' },
        { key: 'sync', label: '4 · Check Session sync', text: step4Text, tab: 'session', cta: 'Go to Session' },
        { key: 'viz', label: step5Label, text: step5Text, tab: hasSession ? 'analysis' : 'session', cta: hasSession ? 'Open Analysis' : 'Go to Session' }
    ];

    host.innerHTML = '<div class="journey-grid">' + steps.map(s =>
        '<div class="' + stepClass(s.key) + '"><div class="journey-step-head"><span>' + s.label + '</span>' +
        (isStepDone(s.key) ? '<span class="journey-check">✓</span>' : '') +
        '</div><p>' + s.text + '</p><button type="button" class="btn btn-outline btn-sm" onclick="setupJourneyGo(\'' + s.key + '\')">' + s.cta + '</button></div>'
    ).join('') + '</div>';

    const cont = document.getElementById('setup-journey-continue');
    if (cont) {
        cont.innerHTML = '<p class="setup-continue">Profile and calibration use Settings; Session is for Sync. Calibrate after you register a band, before the first swim. Step 4 is the empty link test; step 5 pulls real files. Several bands: sync each, then Merge. Ideal stroke in Settings; trends in Insights.</p>';
    }

    updateNavNextHint(snap);
    refreshDeviceRegistrationHints();
    updateSyncPlaybookFirstTime(snap);
    updateHomeOnboardingPanels();
}

/** Journey buttons: scroll to calibration card on Settings when key is cal. */
function setupJourneyGo(key) {
    if (key === 'cal') {
        switchTab('settings');
        requestAnimationFrame(() => {
            const el = document.getElementById('card-imu-calibration');
            if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
        return;
    }
    if (key === 'devices') {
        switchTab('settings');
        requestAnimationFrame(() => {
            const el = document.querySelector('.device-registration-card');
            if (el) el.scrollIntoView({ behavior: 'smooth', block: 'start' });
        });
        return;
    }
    if (key === 'viz') {
        if (typeof hasSavedSessions === 'function' && hasSavedSessions()) switchTab('analysis');
        else switchTab('session');
        return;
    }
    if (key === 'sync') {
        switchTab('session');
        return;
    }
    if (key === 'profile') {
        switchTab('settings');
    }
}
