/**
 * GoldenForm — UI: tab switching + user profile / registration.
 */
// ── TAB NAVIGATION ──
function switchTab(tab) {
    /* Re-entering Session must run layout/sync-playbook hooks (e.g. switching saved sessions). */
    if (tab === currentTab && tab !== 'session') return;
    
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
        loadProgress();
        updateCoachingInsights();
    }
    if (tab === 'home') renderSetupJourney();
    if (tab === 'settings') {
        loadDevices();
        refreshWearableConnectionBanner();
        if (typeof refreshIdealViewerContext === 'function') refreshIdealViewerContext();
    }
    if (tab === 'analysis') {
        if (sessionMetrics && activeSessionIdx >= 0 && processedData && processedData.length) updateAnalysis();
        else if (typeof resetAnalysisPlaceholders === 'function') resetAnalysisPlaceholders();
    }
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

// apiGet, apiPost, showToast — dashboard/js/gf_api.js

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
    /* renderSetupJourney: run after loadDevices in app.js init so device count / sessions are current */
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
                localStorage.setItem(LS_EXPECTED_WEARABLES, String(Math.max(1, Math.min(2, parseInt(wcEl.value, 10) || 1))));
            } catch (e) { /* */ }
        }
        updateProfileUI();
        const rm = document.getElementById('reg-modal');
        if (rm) rm.classList.remove('show');
        showToast('Profile saved!', 'success');
        pushUserConfigToDevice(true); // Auto-sync to device
        if (typeof loadSavedSessions === 'function') await loadSavedSessions();
        renderSetupJourney();
    }
}

