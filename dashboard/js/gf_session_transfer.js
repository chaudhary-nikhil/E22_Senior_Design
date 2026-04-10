/**
 * GoldenForm  --  Session data: localStorage + SQLite list, merge, load sample into viewer.
 */
async function loadSavedSessions() {
    try { savedSessions = JSON.parse(localStorage.getItem(LS_KEY) || '[]'); } catch { savedSessions = []; }
    /* Keep only sessions saved for the current account. */
    const uid = typeof userProfile !== 'undefined' && userProfile && userProfile.id != null
        ? Number(userProfile.id)
        : null;
    if (uid != null) {
        const raw = savedSessions;
        let migrated = false;
        for (let i = 0; i < raw.length; i++) {
            if (raw[i].user_id == null || raw[i].user_id === undefined) {
                raw[i].user_id = uid;
                migrated = true;
            }
        }
        if (migrated) {
            persistSessions();
        }
        const filtered = raw.filter((s) => Number(s.user_id) === uid);
        if (filtered.length !== raw.length) {
            savedSessions = filtered;
            persistSessions();
            activeSessionIdx = -1;
            try {
                localStorage.removeItem(LS_WIFI_SYNC_OK);
                localStorage.removeItem(LS_CAL_KEY);
            } catch (e) { /* ignore */ }
            if (typeof clearViz === 'function') clearViz();
            if (typeof refreshCalibrationSavedStrip === 'function') refreshCalibrationSavedStrip();
            if (typeof updateNavCalPill === 'function') updateNavCalPill(null);
        }
    }
    /* Do not auto-merge /api/sessions on load  --  that repopulated the list from SQLite and broke "empty first open".
       Sessions appear after Sync (/process) or merge; server DB is still used for save/merge APIs. */
    renderSessionList();
    /* renderSetupJourney: defer to loadDevices() in bootstrap so profile + sessions + device counts agree */
}
function persistSessions() {
    try { localStorage.setItem(LS_KEY, JSON.stringify(savedSessions)); } catch (e) {
        if (savedSessions.length > 3) { savedSessions = savedSessions.slice(-3); try { localStorage.setItem(LS_KEY, JSON.stringify(savedSessions)); } catch { } }
    }
}
function addSession(obj) {
    if (typeof userProfile !== 'undefined' && userProfile && userProfile.id != null) {
        obj.user_id = userProfile.id;
    }
    savedSessions.push(obj);
    while (savedSessions.length > 8) savedSessions.shift();
    persistSessions();
    renderSessionList();
    renderSetupJourney();
    if (!obj.id) {
        const devIds = [...new Set((obj.processed_data || []).map(d => d.device_id).filter(Boolean))];
        apiPost('/api/sessions/save', { raw_data: obj.raw_data, processed_data: obj.processed_data, metrics: obj.metrics, duration: obj.duration, device_ids: devIds }).then(res => {
            if (res && res.session_id) {
                obj.id = res.session_id;
                persistSessions();
            }
        }).catch(() => { });
    }
}

function mergeAlignedSessionMetrics(alignedSessions) {
    let strokeCount = 0;
    let hapticCount = 0;
    let devSum = 0;
    let devN = 0;
    let angleSum = 0;
    let angleN = 0;
    for (const s of alignedSessions) {
        const m = s.metrics || {};
        strokeCount += (m.stroke_count || 0);
        hapticCount += (m.haptic_count || 0);
        if (m.avg_deviation != null && !isNaN(Number(m.avg_deviation))) {
            devSum += Number(m.avg_deviation);
            devN++;
        }
        if (m.avg_entry_angle != null && !isNaN(Number(m.avg_entry_angle))) {
            angleSum += Number(m.avg_entry_angle);
            angleN++;
        }
    }
    return {
        stroke_count: strokeCount,
        haptic_count: hapticCount,
        avg_deviation: devN ? devSum / devN : 0,
        avg_entry_angle: angleN ? angleSum / angleN : 0,
        merged_from_sessions: alignedSessions.length
    };
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
            let tmin = Infinity;
            let tmax = -Infinity;
            for (const p of combined) {
                const t = p.timestamp || 0;
                if (t < tmin) tmin = t;
                if (t > tmax) tmax = t;
            }
            const wallDuration = (tmin !== Infinity && tmax > tmin) ? (tmax - tmin) / 1000 : 0;
            const obj = {
                name: 'Merged Session (' + sessionIdsToMerge.length + ')',
                processed_data: combined,
                duration: wallDuration,
                metrics: mergeAlignedSessionMetrics(res.aligned_sessions),
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
        renderSetupJourney();
    }
}
/** Show first-run strip vs quick actions  --  keeps Home aligned with PDP base path. */
function updateHomeOnboardingPanels() {
    const quickActions = document.getElementById('quick-actions');
    if (!quickActions) return;
    const showQuick = typeof getOnboardingSnapshot === 'function'
        ? getOnboardingSnapshot().hasSessionForJourney
        : hasSavedSessions();
    quickActions.style.display = showQuick ? '' : 'none';
}

function renderSessionList() {
    const c = document.getElementById('session-cards');
    const badge = document.getElementById('session-count-badge');
    if (badge) badge.textContent = savedSessions.length + ' saved';

    updateHomeOnboardingPanels();

    if (!c) return;
    if (!savedSessions.length) {
        c.innerHTML = '<p style="color:var(--text3);font-size:0.85em;padding:16px;">No sessions yet. Join the band\u2019s <strong>GoldenForm_</strong> Wi-Fi, then open Session and tap <strong>Sync now</strong> to download swims.</p>';
        return;
    }
    c.innerHTML = savedSessions.map((s, i) => {
        const label = s.name || ('Session ' + (s.id != null ? s.id : (i + 1)));
        return `<div class="session-item ${i === activeSessionIdx ? 'active' : ''}" onclick="selectSession(${i})"><div><strong>${label}</strong><div class="meta">${s.metrics ? (s.metrics.stroke_count || 0) + ' strokes · ' + formatTime(s.duration) : ''}</div></div><button class="btn btn-sm btn-outline" onclick="event.stopPropagation();deleteSession(${i})">✕</button></div>`;
    }).join('');
}
function deleteSession(i) {
    savedSessions.splice(i, 1);
    persistSessions();
    if (activeSessionIdx === i) { activeSessionIdx = -1; clearViz(); }
    else if (activeSessionIdx > i) activeSessionIdx--;
    renderSessionList();
    renderSetupJourney();
}

/** Clear this browser's session list (localStorage only). SQLite copies remain until deleted server-side. */
function clearLocalSessions() {
    if (!savedSessions.length) {
        showToast('No sessions in this browser', 'info');
        return;
    }
    if (!confirm('Remove all sessions from this browser? (Server database is unchanged.)')) return;
    savedSessions = [];
    activeSessionIdx = -1;
    try { localStorage.removeItem(LS_KEY); } catch (e) { /* ignore */ }
    clearViz();
    renderSessionList();
    renderSetupJourney();
    resetSessionSummaryPlaceholders();
    if (typeof resetAnalysisPlaceholders === 'function') resetAnalysisPlaceholders();
    if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
    showToast('Local session list cleared', 'success');
}
async function selectSession(i) {
    if (i < 0 || i >= savedSessions.length) return;
    activeSessionIdx = i;
    const s = savedSessions[i];
    let pd = s.processed_data || s.processedData || [];
    if ((!pd || !pd.length) && s.id) {
        const detail = await apiGet('/api/sessions/' + s.id);
        if (detail && detail.session) {
            pd = detail.session.processed_data || [];
            s.processed_data = pd;
            s.metrics = detail.session.metrics || s.metrics;
            s.raw_data = detail.session.raw_data;
            persistSessions();
        }
    }
    processedData = pd;
    sessionMetrics = s.metrics || {};
    currentIndex = 0;
    integratedPositions = [];
    rawIntegratedPositions = [];
    lastCalStripFrameIdx = -1;
    refreshVizCoordinateTransform();
    isPlaying = false;
    if (playbackInterval) { clearInterval(playbackInterval); playbackInterval = null; }
    // Pre-compute stroke boundaries and haptic events for timeline
    computeStrokeBoundaries();
    buildPlaybackStrokeSegments();
    renderSessionList();
    updateSessionSummary();
    refreshVizPlaybackUI();
    updateAnalysis();
    updateCoachingInsights();
    // Switch to Session tab first so chart and 3D canvases have real dimensions (not 0x0)
    switchTab('session', { force: true });
    // Scroll Session tab into view so the 3D canvas and charts are visible
    const sessionPage = document.getElementById('page-session');
    if (sessionPage) sessionPage.scrollIntoView({ behavior: 'smooth', block: 'start' });
    buildHapticTimeline();
    updateSessionCalBanner();
    // Defer chart init, 3D init/resize, and first frame so layout is complete
    requestAnimationFrame(() => {
        const c = document.getElementById('canvas3d');
        if (c) {
            const w = Math.max(c.clientWidth || 800, 400);
            const h = Math.max(c.clientHeight || 450, 400);
            // (Re)init 3D if not yet created (e.g. canvas was hidden at load)
            if (!scene || !renderer) init3D();
            if (renderer && camera) {
                renderer.setSize(Math.min(w, 1400), Math.min(h, 900), false);
                camera.aspect = w / h;
                camera.updateProjectionMatrix();
            }
        }
        initCharts();
        resizeSideViewCanvas();
        renderFrame(0);
        if (processedData.length > 0 && accelChart && gyroChart) updateCharts(0);
        if (typeof frameVizOnTrail === 'function') frameVizOnTrail();
        if (typeof refreshIdealViewerContext === 'function') refreshIdealViewerContext();
    });
}

