/**
 * GoldenForm — realtime updates: SSE + fallback refresh loop.
 *
 * SSE is used for instant updates (device_info, sessions/devices/progress invalidations).
 * Fallback loop keeps the UI fresh even if EventSource is blocked.
 */

let gfEventSource = null;
let gfLiveInterval = null;
let gfLastLiveTick = 0;
let gfRefreshQueued = false;

function _isTabVisible() {
    try { return document.visibilityState === 'visible'; } catch { return true; }
}

function _authTokenForSse() {
    try { return localStorage.getItem('gf_session_token') || ''; } catch { return ''; }
}

function _scheduleRefresh() {
    if (gfRefreshQueued) return;
    gfRefreshQueued = true;
    setTimeout(async () => {
        gfRefreshQueued = false;
        const authed = !!(typeof userProfile !== 'undefined' && userProfile && userProfile.id);
        if (!authed) return;
        // Keep it light: only hit endpoints if the tab is visible.
        if (!_isTabVisible()) return;
        try { if (typeof loadDevices === 'function') await loadDevices(); } catch (e) { /* ignore */ }
        try { if (typeof loadProgress === 'function') await loadProgress(); } catch (e) { /* ignore */ }
        // Sessions list is from localStorage by design; keep it consistent for this account.
        try { if (typeof loadSavedSessions === 'function') await loadSavedSessions(); } catch (e) { /* ignore */ }
        try { if (typeof renderSetupJourney === 'function') renderSetupJourney(); } catch (e) { /* ignore */ }
    }, 180);
}

function startLiveRefreshLoop() {
    if (gfLiveInterval) clearInterval(gfLiveInterval);
    gfLiveInterval = setInterval(() => {
        const now = Date.now();
        if (now - gfLastLiveTick < 8000) return;
        gfLastLiveTick = now;
        _scheduleRefresh();
    }, 5000);
}

function stopLiveRefreshLoop() {
    if (gfLiveInterval) clearInterval(gfLiveInterval);
    gfLiveInterval = null;
}

function startRealtimeEvents() {
    try {
        if (gfEventSource) {
            gfEventSource.close();
            gfEventSource = null;
        }
        const t = _authTokenForSse();
        if (!t) return;
        gfEventSource = new EventSource('/api/events?token=' + encodeURIComponent(t));

        gfEventSource.addEventListener('hello', (ev) => {
            // no-op for now; demo flag is already set by /api/bootstrap.
        });

        gfEventSource.addEventListener('device_info', (ev) => {
            try {
                const payload = JSON.parse(ev.data || '{}');
                // Reuse the existing poll UI functions.
                if (typeof setConnStatus === 'function') {
                    const online = payload && payload.status !== 'disconnected';
                    setConnStatus(online ? 'connected' : 'offline');
                }
                if (typeof updateWearableConnectionBanner === 'function') {
                    updateWearableConnectionBanner(payload);
                }
                if (typeof normalizeDeviceInfoCal === 'function' && typeof updateCalibrationDisplay === 'function') {
                    const cal = normalizeDeviceInfoCal(payload);
                    if (cal) {
                        payload.cal = cal;
                        updateCalibrationDisplay(payload);
                    }
                }
            } catch (e) { /* ignore */ }
        });

        const invalidate = () => _scheduleRefresh();
        gfEventSource.addEventListener('sessions', invalidate);
        gfEventSource.addEventListener('devices', invalidate);
        gfEventSource.addEventListener('progress', invalidate);
        gfEventSource.addEventListener('profile', invalidate);
        gfEventSource.addEventListener('auth', invalidate);

        gfEventSource.onerror = () => {
            // fall back to polling loop only
            try { gfEventSource.close(); } catch { /* ignore */ }
            gfEventSource = null;
        };
    } catch (e) {
        gfEventSource = null;
    }
}

function stopRealtimeEvents() {
    try { if (gfEventSource) gfEventSource.close(); } catch { /* ignore */ }
    gfEventSource = null;
}

// Expose
Object.assign(window, {
    startRealtimeEvents,
    stopRealtimeEvents,
    startLiveRefreshLoop,
    stopLiveRefreshLoop
});

