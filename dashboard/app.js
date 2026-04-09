/**
 * GoldenForm — bootstrap (entry point).
 * Loads last; registers window handlers and DOMContentLoaded.
 * Feature code lives under dashboard/js/gf_*.js (viz split: gf_viz_*, Wi‑Fi: gf_wifi_* / gf_onboarding_journey).
 */

// Expose handlers for inline HTML onclick usage
Object.assign(window, {
    switchTab,
    syncFromDevice,
    addWearableFromConnection,
    mergeLatestSessions,
    togglePlayback,
    resetPlayback,
    replayFromStart,
    skipBackward,
    skipForward,
    seekPlayback,
    stepFrame,
    updateScale,
    resetView,
    focusVizOnDevice,
    frameVizOnTrail,
    clearViz,
    resetLivePlaybackHud,
    registerUser,
    loginUser,
    createAccountFromModal,
    saveProfileFromSettings,
    logoutUser,
    setAuthTab,
    registerDevice,
    setCurrentAsIdeal,
    setSingleStrokeAsIdeal,
    pushIdealToDevice,
    deleteIdealStroke,
    pushUserConfigToDevice,
    testHapticDevice,
    selectSession,
    deleteSession,
    clearLocalSessions,
    saveCalibrationSnapshotManual,
    openCalibrationGuide,
    closeCalibrationGuide,
    jumpToStroke,
    setupJourneyGo,
    refreshAiCoachingInsights,
    analysisStrokeRowClick,
    setIdealCompareStroke,
    onIdealCompareModeChange,
    exportSessionRawJson,
    replayApJsonToSession
});

function bindNavigationButtons() {
    document.querySelectorAll('[data-tab-target]').forEach(btn => {
        btn.addEventListener('click', (e) => {
            e.preventDefault();
            const tab = btn.getAttribute('data-tab-target');
            /* force: real click from addEventListener does not set window.event; switchTab's guard
             * would block. Avoid duplicate onclick + this listener (would double init3D / Session work). */
            if (tab) switchTab(tab, { force: true });
        });
    });
}

// ── INIT ──
window.addEventListener('DOMContentLoaded', async () => {
    if (await syncBootstrapInstance()) return;
    const safe = async (fn) => {
        try { return await fn(); } catch (e) { return null; }
    };
    try { bindNavigationButtons(); } catch (e) { /* ignore */ }
    /* Run before loadUserProfile: that await can be slow; otherwise setup badges stay "Loading…"
       and the nav hint never updates until the whole chain completes. */
    try { if (typeof renderSetupJourney === 'function') renderSetupJourney(); } catch (e) { /* ignore */ }
    try { bindVizPlaybackControls(); } catch (e) { /* ignore */ }
    try { initScrubberPointerHandlers(); } catch (e) { /* ignore */ }
    try { refreshCalibrationSavedStrip(); } catch (e) { /* ignore */ }
    try { updateNavCalPill(null); } catch (e) { /* ignore */ }
    try { resetSessionSummaryPlaceholders(); } catch (e) { /* ignore */ }
    try { resetHomeStatsPlaceholders(); } catch (e) { /* ignore */ }
    /* Order matters: profile → local sessions → devices, then renderSetupJourney (inside loadDevices)
       sees correct userProfile, savedSessions, and cachedDeviceListLength. */
    await safe(() => loadUserProfile());
    /* Demo startup must always show registration (create account) so you can present the full flow.
     * If any earlier JS path fails to open the modal, force it here. */
    try {
        const authed = !!(userProfile && userProfile.id);
        const demo = !!window.GF_DEMO_MODE;
        if (!authed) {
            if (typeof showAuthModal === 'function') showAuthModal(demo ? 'register' : 'login');
        }
    } catch (e) { /* ignore */ }
    await safe(() => loadSavedSessions());
    await safe(() => loadDevices());
    await safe(() => loadIdealStroke());
    // Ensure Home stepper renders even if backend calls failed.
    try { if (typeof renderSetupJourney === 'function') renderSetupJourney(); } catch (e) { /* ignore */ }

    // Realtime updates (SSE) + lightweight refresh loop fallback.
    try { if (typeof startRealtimeEvents === 'function') startRealtimeEvents(); } catch (e) { /* ignore */ }
    try { if (typeof startLiveRefreshLoop === 'function') startLiveRefreshLoop(); } catch (e) { /* ignore */ }

    try {
        if (typeof updateLogoutVisibility === 'function') updateLogoutVisibility();
    } catch (e) { /* ignore */ }
    if (activeSessionIdx < 0) {
        try { resetAnalysisPlaceholders(); } catch (e) { /* ignore */ }
        if (typeof buildIdealComparison === 'function') await safe(() => buildIdealComparison());
        try { updateCoachingInsights(); } catch (e) { /* ignore */ }
    }
    try { init3D(); } catch (e) { /* ignore */ }
    /* If the default tab were ever Session, ensure the viz loop runs after init3D (init3D only
     * auto-starts RAF when currentTab === 'session'). */
    try {
        if (typeof currentTab !== 'undefined' && currentTab === 'session' && typeof startVizLoop === 'function') {
            startVizLoop();
        }
    } catch (e) { /* ignore */ }
    if (activeSessionIdx < 0 && (!processedData || !processedData.length) && typeof resetLivePlaybackHud === 'function') {
        resetLivePlaybackHud();
    }
    try { startDevicePolling(); } catch (e) { /* ignore */ }
    try {
        if (window.lucide && typeof window.lucide.createIcons === 'function') {
            window.lucide.createIcons();
            window.refreshIcons = () => {
                try { window.lucide.createIcons(); } catch (e) { /* ignore */ }
            };
        } else {
            window.refreshIcons = () => {};
        }
    } catch (e) { /* ignore */ }
    window.addEventListener('resize', () => {
        resizeSideViewCanvas();
        if (processedData.length) drawSideViewViz(currentIndex);
        if (renderer && camera) {
            const c = document.getElementById('canvas3d');
            if (c) {
                const w = Math.min(Math.max(c.clientWidth || 800, 400), 1400);
                const h = Math.min(Math.max(c.clientHeight || 450, 400), 900);
                renderer.setSize(w, h, false);
                camera.aspect = w / h;
                camera.updateProjectionMatrix();
            }
        }
    });
});
