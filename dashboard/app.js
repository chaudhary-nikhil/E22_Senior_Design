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
    skipBackward,
    skipForward,
    seekPlayback,
    stepFrame,
    updateScale,
    resetView,
    focusVizOnDevice,
    clearViz,
    resetLivePlaybackHud,
    registerUser,
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
    setupJourneyGo
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
window.addEventListener('DOMContentLoaded', async () => {
    if (await syncBootstrapInstance()) return;
    bindNavigationButtons();
    bindVizPlaybackControls();
    initScrubberPointerHandlers();
    refreshCalibrationSavedStrip();
    updateNavCalPill(null);
    resetSessionSummaryPlaceholders();
    resetHomeStatsPlaceholders();
    /* Order matters: profile → local sessions → devices, then renderSetupJourney (inside loadDevices)
       sees correct userProfile, savedSessions, and cachedDeviceListLength. */
    await loadUserProfile();
    await loadSavedSessions();
    await loadDevices();
    await loadIdealStroke();
    if (activeSessionIdx < 0) {
        resetAnalysisPlaceholders();
        if (typeof buildIdealComparison === 'function') await buildIdealComparison();
        updateCoachingInsights();
    }
    init3D();
    if (activeSessionIdx < 0 && (!processedData || !processedData.length) && typeof resetLivePlaybackHud === 'function') {
        resetLivePlaybackHud();
    }
    startDevicePolling();
    window.addEventListener('resize', () => {
        resizeSideViewCanvas();
        if (processedData.length) drawSideViewViz(currentIndex);
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
