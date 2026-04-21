/**
 * GoldenForm — UI: tab switching + user profile / registration.
 */
// ── TAB NAVIGATION ──
function switchTab(tab, opts = {}) {
    // Programmatic navigation should pass { force: true }. User-driven clicks may come from
    // addEventListener (we pass force there) or inline onclick — window.event is unreliable
    // in many browsers, so do not gate on it.
    /* Re-entering Session must run layout/sync-playbook hooks (e.g. switching saved sessions). */
    if (tab === currentTab) {
        // Avoid tearing down/re-showing the same page (visible "blink").
        // Still run per-tab refresh hooks as needed.
        if (tab === 'session') {
            updateSyncPlaybookMulti();
            updateSyncPlaybookConnectionState();
            try {
                if (typeof init3D === 'function') init3D();
                if (renderer && scene && camera) renderer.render(scene, camera);
            } catch (e) { /* ignore */ }
        } else if (tab === 'insights') {
            loadProgress();
            updateCoachingInsights();
            try { if (typeof gfEnsureInsightsTabAiCoaching === 'function') gfEnsureInsightsTabAiCoaching(); } catch (e) { /* ignore */ }
        } else if (tab === 'settings') {
            loadDevices();
            refreshWearableConnectionBanner();
            if (typeof refreshIdealViewerContext === 'function') refreshIdealViewerContext();
        } else if (tab === 'analysis') {
            if (sessionMetrics && activeSessionIdx >= 0 && processedData && processedData.length) updateAnalysis();
            else if (typeof resetAnalysisPlaceholders === 'function') resetAnalysisPlaceholders();
        } else if (tab === 'home') {
            renderSetupJourney();
        }
        try { if (typeof refreshSetupJourneyStepVisuals === 'function') refreshSetupJourneyStepVisuals(); } catch (e) { /* ignore */ }
        return;
    }
    
    // Pause heavy 3D rendering when leaving Session to avoid GPU churn.
    if (typeof stopVizLoop === 'function' && currentTab === 'session' && tab !== 'session') {
        try { stopVizLoop(); } catch (e) { /* ignore */ }
    }

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
        page.style.opacity = '1';
        page.classList.add('active');
    }
    if (btn) btn.classList.add('active');
    
    currentTab = tab;
    
    // Performance: Only load data when entering the tab
    if (tab === 'insights') {
        try { loadProgress(); } catch (e) { /* ignore */ }
        try { updateCoachingInsights(); } catch (e) { /* ignore */ }
        try { if (typeof gfEnsureInsightsTabAiCoaching === 'function') gfEnsureInsightsTabAiCoaching(); } catch (e) { /* ignore */ }
    }
    if (tab === 'home') { try { renderSetupJourney(); } catch (e) { /* ignore */ } }
    if (tab === 'settings') {
        try { loadDevices(); } catch (e) { /* ignore */ }
        try { refreshWearableConnectionBanner(); } catch (e) { /* ignore */ }
        if (typeof refreshIdealViewerContext === 'function') { try { refreshIdealViewerContext(); } catch (e) { /* ignore */ } }
    }
    if (tab === 'analysis') {
        try {
            if (sessionMetrics && activeSessionIdx >= 0 && processedData && processedData.length) updateAnalysis();
            else if (typeof resetAnalysisPlaceholders === 'function') resetAnalysisPlaceholders();
        } catch (e) { /* ignore */ }
    }
    if (tab === 'session') {
        try { updateSyncPlaybookMulti(); } catch (e) { /* ignore */ }
        try { updateSyncPlaybookConnectionState(); } catch (e) { /* ignore */ }
        /* WebGL often stays black until the canvas is visible — re-init/resize then force one frame. */
        try {
            if (typeof init3D === 'function') init3D();
            if (renderer && scene && camera) renderer.render(scene, camera);
        } catch (e) { /* ignore */ }
        // Resume 3D loop only when Session is visible.
        if (typeof startVizLoop === 'function') {
            try { startVizLoop(); } catch (e) { /* ignore */ }
        }
    }
    try { if (typeof refreshSetupJourneyStepVisuals === 'function') refreshSetupJourneyStepVisuals(); } catch (e) { /* ignore */ }
}

function togglePasswordVisibility(inputId, btn) {
    const el = document.getElementById(inputId);
    if (!el) return;
    const show = el.type === 'password';
    el.type = show ? 'text' : 'password';
    if (btn) {
        btn.textContent = show ? '🙈' : '👁';
        btn.title = show ? 'Hide password' : 'Show password';
    }
}

// apiGet, apiPost, showToast — dashboard/js/gf_api.js

// ── USER PROFILE ──
function _cmToIn(cm) { return (Number(cm) || 0) / 2.54; }
function _inToCm(inches) { return (Number(inches) || 0) * 2.54; }

function _getUnits(prefix) {
    const u = document.getElementById(prefix + 'units');
    return (u && (u.value === 'cm' || u.value === 'in')) ? u.value : 'in';
}

function _parseLenToCm(input, units) {
    if (input == null) return 0;
    let raw = String(input).trim().toLowerCase();
    if (!raw) return 0;

    // Normalize common quote variants: 5’10” / 5'10'' / 5'10"
    raw = raw
        .replace(/[’′]/g, "'")
        .replace(/[“”″]/g, '"')
        .replace(/''/g, '"');

    // Accept explicit cm
    const cmMatch = raw.match(/^([0-9]+(\.[0-9]+)?)\s*cm$/);
    if (cmMatch) return Number(cmMatch[1]) || 0;

    // Accept feet+inches (5'10", 5ft 10in, 5 10)
    const ftIn = raw.match(/^(\d+)\s*(?:ft|')\s*(\d{1,2})?\s*(?:in|")?\s*$/);
    if (ftIn) {
        const ft = Number(ftIn[1]) || 0;
        const inch = Number(ftIn[2] || 0) || 0;
        return _inToCm(ft * 12 + inch);
    }

    // Plain number: interpret based on selected units
    const n = Number(raw.replace(/[^\d.]/g, ''));
    if (!isFinite(n) || n <= 0) return 0;
    return units === 'cm' ? n : _inToCm(n);
}

function _formatCmForUI(cm, units, mode) {
    const v = Number(cm) || 0;
    if (!v) return '';
    if (units === 'cm') return String(Math.round(v));
    const inches = _cmToIn(v);
    if (mode === 'height') {
        const ft = Math.floor(inches / 12);
        const inch = Math.round(inches - ft * 12);
        return ft > 0 ? `${ft}'${inch}"` : String(Math.round(inches));
    }
    return String(Math.round(inches));
}

// Expose for other modules (e.g. pushUserConfigToDevice).
window.gfParseLenToCm = _parseLenToCm;
window.gfGetUnits = _getUnits;
window.gfFormatCmForUI = _formatCmForUI;

function persistUserProfileCache() {
    if (!userProfile || typeof userProfile.name !== 'string' || !userProfile.name.trim()) return;
    try {
        localStorage.setItem(LS_USER_PROFILE_CACHE, JSON.stringify({
            id: userProfile.id,
            email: userProfile.email,
            name: userProfile.name,
            height_cm: userProfile.height_cm,
            wingspan_cm: userProfile.wingspan_cm,
            skill_level: userProfile.skill_level
        }));
    } catch (e) { /* quota */ }
}

function showAuthModal(preferTab) {
    const m = document.getElementById('auth-modal');
    if (!m) return;
    const demo = !!window.GF_DEMO_MODE;
    const tab = preferTab || (demo ? 'register' : 'login');
    setAuthTab(tab);
    m.classList.add('show');
}

function hideAuthModal() {
    const m = document.getElementById('auth-modal');
    if (m) m.classList.remove('show');
}

function setAuthTab(tab) {
    const loginP = document.getElementById('auth-panel-login');
    const regP = document.getElementById('auth-panel-register');
    const tLogin = document.getElementById('auth-tab-login');
    const tReg = document.getElementById('auth-tab-register');
    if (tab === 'register') {
        if (loginP) loginP.hidden = true;
        if (regP) regP.hidden = false;
        if (tLogin) { tLogin.classList.remove('auth-tab--active'); }
        if (tReg) { tReg.classList.add('auth-tab--active'); }
    } else {
        if (loginP) loginP.hidden = false;
        if (regP) regP.hidden = true;
        if (tLogin) { tLogin.classList.add('auth-tab--active'); }
        if (tReg) { tReg.classList.remove('auth-tab--active'); }
    }
}

async function loginUser(e) {
    if (e) e.preventDefault();
    const emailEl = document.getElementById('login-email');
    const passEl = document.getElementById('login-password');
    const email = (emailEl && emailEl.value) ? emailEl.value.trim() : '';
    const password = (passEl && passEl.value) ? passEl.value : '';
    if (!email || !password) {
        showToast('Enter email and password', 'error');
        return;
    }
    const res = await apiPost('/api/auth/login', { email, password });
    if (res && res.status === 'ok' && res.token) {
        setAuthToken(res.token);
        userProfile = res.profile;
        persistUserProfileCache();
        updateProfileUI();
        hideAuthModal();
        updateLogoutVisibility();
        showToast('Signed in', 'success');
        if (typeof loadSavedSessions === 'function') await loadSavedSessions();
        if (typeof loadDevices === 'function') await loadDevices();
        if (typeof renderSetupJourney === 'function') renderSetupJourney();
        try { pushUserConfigToDevice(true); } catch (err) { /* offline */ }
        return;
    }
    showToast((res && res.error) ? res.error : 'Sign-in failed', 'error');
}

async function createAccountFromModal(e) {
    if (e) e.preventDefault();
    const nameInput = document.getElementById('reg-name')?.value.trim();
    const email = document.getElementById('reg-email')?.value.trim();
    const password = document.getElementById('reg-password')?.value || '';
    if (!nameInput || !email || !password) {
        showToast('Name, email, and password are required', 'error');
        return;
    }
    const units = _getUnits('reg-');
    const heightRaw = document.getElementById('reg-height')?.value;
    const wingspanRaw = document.getElementById('reg-wingspan')?.value;
    const body = {
        name: nameInput,
        email,
        password,
        height_cm: _parseLenToCm(heightRaw, units),
        wingspan_cm: _parseLenToCm(wingspanRaw, units),
        skill_level: document.getElementById('reg-skill')?.value || 'beginner'
    };
    const res = await apiPost('/api/auth/register', body);
    if (res && res.status === 'ok' && res.token) {
        setAuthToken(res.token);
        userProfile = res.profile;
        const wcEl = document.getElementById('reg-wearable-count');
        if (wcEl) {
            try {
                localStorage.setItem(LS_EXPECTED_WEARABLES, String(Math.max(1, Math.min(2, parseInt(wcEl.value, 10) || 1))));
            } catch (err) { /* ignore */ }
        }
        updateProfileUI();
        persistUserProfileCache();
        hideAuthModal();
        updateLogoutVisibility();
        showToast('Account created', 'success');
        try { pushUserConfigToDevice(true); } catch (err) { /* ignore */ }
        if (typeof loadSavedSessions === 'function') await loadSavedSessions();
        if (typeof loadDevices === 'function') await loadDevices();
        if (typeof renderSetupJourney === 'function') renderSetupJourney();
        return;
    }
    showToast((res && res.error) ? res.error : 'Could not create account', 'error');
}

async function saveProfileFromSettings(e) {
    if (e) e.preventDefault();
    if (!userProfile || !userProfile.id) {
        showAuthModal('login');
        return;
    }
    const emailInput = document.getElementById('settings-email-display')?.value.trim();
    const nameInput = document.getElementById('settings-name')?.value.trim();
    if (!nameInput) return;
    const units = _getUnits('settings-');
    const heightRaw = document.getElementById('settings-height')?.value;
    const wingspanRaw = document.getElementById('settings-wingspan')?.value;
    const body = {
        email: emailInput,
        name: nameInput,
        height_cm: _parseLenToCm(heightRaw, units),
        wingspan_cm: _parseLenToCm(wingspanRaw, units),
        skill_level: document.getElementById('settings-skill')?.value || 'beginner'
    };
    const res = await apiPost('/api/profile', body);
    if (res && res.status === 'ok' && res.profile) {
        userProfile = res.profile;
        const wcEl = document.getElementById('settings-wearable-count');
        if (wcEl) {
            try {
                localStorage.setItem(LS_EXPECTED_WEARABLES, String(Math.max(1, Math.min(2, parseInt(wcEl.value, 10) || 1))));
            } catch (err) { /* ignore */ }
        }
        updateProfileUI();
        persistUserProfileCache();
        showToast('Profile saved', 'success');
        try { pushUserConfigToDevice(true); } catch (err) { /* ignore */ }
        if (typeof renderSetupJourney === 'function') renderSetupJourney();
        return;
    }
    if (res && res.httpStatus === 401) {
        showAuthModal('login');
        return;
    }
    showToast((res && res.error) ? res.error : 'Could not save profile', 'error');
}

async function logoutUser() {
    try {
        await apiPost('/api/auth/logout', {});
    } catch (e) { /* ignore */ }
    setAuthToken(null);
    userProfile = null;
    try { localStorage.removeItem(LS_USER_PROFILE_CACHE); } catch (e) { /* ignore */ }
    try { if (typeof stopRealtimeEvents === 'function') stopRealtimeEvents(); } catch (e) { /* ignore */ }

    ['login-email', 'login-password', 'reg-name', 'reg-email', 'reg-password',
     'reg-height', 'reg-wingspan'].forEach(id => {
        const el = document.getElementById(id);
        if (el) el.value = '';
    });
    const regSkill = document.getElementById('reg-skill');
    if (regSkill) regSkill.value = 'beginner';
    const regWc = document.getElementById('reg-wearable-count');
    if (regWc) regWc.value = '1';

    updateLogoutVisibility();
    showAuthModal('login');
    showToast('Signed out', 'info');
    if (typeof loadSavedSessions === 'function') await loadSavedSessions();
    if (typeof loadDevices === 'function') await loadDevices();
    if (typeof renderSetupJourney === 'function') renderSetupJourney();
}

function updateLogoutVisibility() {
    const btn = document.getElementById('nav-logout-btn');
    if (!btn) return;
    const authed = !!userProfile && !!userProfile.id;
    btn.style.display = authed ? '' : 'none';
}

async function loadUserProfile() {
    let token = null;
    try { token = localStorage.getItem('gf_session_token'); } catch (e) { token = null; }

    if (!token) {
        userProfile = null;
        showAuthModal(window.GF_DEMO_MODE ? 'register' : 'login');
        updateLogoutVisibility();
        return;
    }

    try {
        userProfile = await apiGet('/api/user', { timeoutMs: 15000 });
    } catch (e) {
        userProfile = null;
    }
    if (!userProfile || typeof userProfile !== 'object') userProfile = null;
    if (userProfile && userProfile._httpError === 401) {
        setAuthToken(null);
        userProfile = null;
    }
    if (userProfile && (userProfile.error === 'Unauthorized' || userProfile.auth_required)) {
        setAuthToken(null);
        userProfile = null;
    }

    const name = (userProfile && typeof userProfile.name === 'string') ? userProfile.name.trim() : '';
    const isBad = !userProfile || !name;
    if (isBad) {
        userProfile = null;
        setAuthToken(null);
        showAuthModal(window.GF_DEMO_MODE ? 'register' : 'login');
        updateLogoutVisibility();
        return;
    }

    updateProfileUI();
    persistUserProfileCache();
    hideAuthModal();
    updateLogoutVisibility();
}
function updateProfileUI() {
    if (!userProfile) return;
    setText('home-user-name', userProfile.name || 'Swimmer');
    const sk = document.getElementById('home-skill');
    if (sk) sk.textContent = (userProfile.skill_level || 'beginner').charAt(0).toUpperCase() + (userProfile.skill_level || 'beginner').slice(1);
    const emailDisp = document.getElementById('settings-email-display');
    if (emailDisp) emailDisp.value = userProfile.email || '';
    ['settings-name', 'reg-name'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.name || ''; });
    // Default units: inches/feet (US-friendly). Keep internal storage in cm.
    ['settings-', 'reg-'].forEach(prefix => {
        const unitsEl = document.getElementById(prefix + 'units');
        if (unitsEl && !unitsEl.value) unitsEl.value = 'in';
        const units = unitsEl ? unitsEl.value : 'in';
        const hEl = document.getElementById(prefix + 'height');
        const wEl = document.getElementById(prefix + 'wingspan');
        if (hEl) hEl.value = _formatCmForUI(userProfile.height_cm || 0, units, 'height');
        if (wEl) wEl.value = _formatCmForUI(userProfile.wingspan_cm || 0, units, 'wingspan');
    });
    ['settings-skill', 'reg-skill'].forEach(id => { const el = document.getElementById(id); if (el) el.value = userProfile.skill_level || 'beginner'; });
    ['settings-wearable-count', 'reg-wearable-count'].forEach((id) => {
        const wc = document.getElementById(id);
        if (wc) wc.value = String(getExpectedWearableCount());
    });
    persistUserProfileCache();
}

/** @deprecated Use createAccountFromModal or saveProfileFromSettings */
async function registerUser(e) {
    if (e) e.preventDefault();
    const authModal = document.getElementById('auth-modal');
    if (authModal && authModal.classList.contains('show') && !document.getElementById('auth-panel-register')?.hidden) {
        await createAccountFromModal(e);
        return;
    }
    await saveProfileFromSettings(e);
}
