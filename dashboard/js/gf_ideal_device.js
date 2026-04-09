/**
 * GoldenForm — Ideal stroke API, push to device, test buzz (diagnostics).
 */
let idealDisplayInfo = null;

function parseSqliteUtc(s) {
    if (s == null || s === '') return null;
    const t = String(s).trim();
    if (!t) return null;
    return new Date(t.includes('T') ? t : t.replace(' ', 'T') + 'Z');
}

function refreshIdealViewerContext() {
    const el = document.getElementById('ideal-viewer-context');
    if (!el) return;
    if (typeof savedSessions === 'undefined' || typeof activeSessionIdx === 'undefined') {
        el.textContent = '';
        return;
    }
    if (activeSessionIdx < 0 || !savedSessions.length) {
        el.textContent = 'Load a session on the Session tab first, then use the button below.';
        return;
    }
    const s = savedSessions[activeSessionIdx];
    const label = s.name || ('Session #' + (s.id != null ? s.id : (activeSessionIdx + 1)));
    const esc = typeof escapeHtml === 'function' ? escapeHtml : (t) => String(t);
    el.innerHTML = 'Uses the session open in the viewer: <strong>' + esc(label) + '</strong>' +
        (s.id != null ? ' <span class="ideal-id-tag">id ' + esc(String(s.id)) + '</span>' : '');
}

function updateIdealStrokePanel() {
    const main = document.getElementById('ideal-status');
    const meta = document.getElementById('ideal-meta');
    if (!main) return;
    if (!idealStrokeData || !idealStrokeData.length || !idealDisplayInfo) {
        main.textContent = 'No ideal baseline saved';
        if (meta) meta.textContent = '';
        refreshIdealViewerContext();
        return;
    }
    const d = idealDisplayInfo;
    main.textContent = d.numSamples + ' samples · ' + (d.name || 'Baseline');
    if (meta) {
        const parts = [];
        if (d.createdAt) {
            const dt = parseSqliteUtc(d.createdAt);
            if (dt && !isNaN(dt.getTime())) parts.push('Saved ' + formatEasternDateTime(dt.getTime()));
        } else if (d.savedAt) parts.push('Saved ' + formatEasternDateTime(d.savedAt));
        if (d.sourceSessionName) parts.push('Source: ' + d.sourceSessionName);
        meta.textContent = parts.join(' · ');
    }
    refreshIdealViewerContext();
}

// ── IDEAL STROKE ──
async function loadIdealStroke() {
    idealDisplayInfo = null;
    const res = await apiGet('/api/ideal_stroke');
    if (res && res.samples && res.samples.length) {
        idealStrokeData = res.samples;
        idealDisplayInfo = {
            source: 'server',
            name: res.name || 'Baseline',
            numSamples: res.samples.length,
            createdAt: res.created_at || null,
            id: res.id
        };
        cacheIdealLocal(
            res.samples,
            typeof sessionMetrics !== 'undefined' && sessionMetrics ? sessionMetrics.avg_entry_angle : 30,
            res.name || 'Ideal',
            { serverCreatedAt: res.created_at }
        );
        updateIdealStrokePanel();
        if (typeof processedData !== 'undefined' && processedData && processedData.length && typeof updateAnalysis === 'function' && sessionMetrics) {
            updateAnalysis();
        }
        return;
    }
    try {
        const raw = localStorage.getItem(LS_IDEAL_KEY);
        if (raw) {
            const o = JSON.parse(raw);
            if (o.samples && o.samples.length) {
                idealStrokeData = o.samples;
                idealDisplayInfo = {
                    source: 'local',
                    name: o.name || 'Baseline',
                    numSamples: o.samples.length,
                    savedAt: o.savedAt,
                    sourceSessionName: o.sourceSessionName,
                    sourceSessionId: o.sourceSessionId,
                    createdAt: o.serverCreatedAt || null
                };
                updateIdealStrokePanel();
                if (typeof processedData !== 'undefined' && processedData && processedData.length && typeof updateAnalysis === 'function' && sessionMetrics) {
                    updateAnalysis();
                }
                return;
            }
        }
    } catch (e) { /* ignore */ }
    idealStrokeData = null;
    idealDisplayInfo = null;
    updateIdealStrokePanel();
    if (typeof processedData !== 'undefined' && processedData && processedData.length && typeof updateAnalysis === 'function' && sessionMetrics) {
        updateAnalysis();
    }
}

async function afterIdealSavedServer() {
    await loadIdealStroke();
    if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
    if (typeof updateAnalysis === 'function' && sessionMetrics) updateAnalysis();
    if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
    if (isDeviceOnline) pushIdealToDevice(true);
}

/** SQLite-backed session id for /api/ideal_stroke/set_from_stroke (excludes apjson: / local-only ids). */
function gfNumericSessionIdForApi(sess) {
    if (!sess || sess.id == null || sess.id === '') return null;
    const raw = String(sess.id);
    if (raw.includes(':') || raw.includes('apjson')) return null;
    const n = Number(sess.id);
    return (Number.isFinite(n) && n > 0) ? n : null;
}

async function setSingleStrokeAsIdeal(strokeNum, streamKey) {
    if (!processedData || !processedData.length) return;
    refreshStrokeFieldMode();
    const samples = buildIdealLiaSamplesFromStroke(strokeNum, streamKey);
    if (!samples.length) {
        showToast('No samples found for that stroke on this band.', 'error');
        return;
    }
    const avgAngle = averageEntryAngleForStroke(strokeNum, streamKey);
    const currentSession = savedSessions[activeSessionIdx];
    const meta = {};
    if (currentSession) {
        if (currentSession.id != null) meta.sourceSessionId = currentSession.id;
        meta.sourceSessionName = currentSession.name || ('Session #' + (currentSession.id || strokeNum));
    }

    const sidApi = gfNumericSessionIdForApi(currentSession);
    if (!currentSession || sidApi == null) {
        idealCompareStrokeNum = Number(strokeNum);
        idealCompareStreamKey = (streamKey != null && streamKey !== '') ? String(streamKey) : null;
        idealStrokeData = samples;
        cacheIdealLocal(samples, avgAngle, 'Stroke ' + strokeNum, meta);
        idealDisplayInfo = {
            source: 'local',
            name: 'Stroke ' + strokeNum,
            numSamples: samples.length,
            savedAt: Date.now(),
            sourceSessionName: meta.sourceSessionName
        };
        updateIdealStrokePanel();
        if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
        buildIdealComparison();
        if (typeof updateAnalysis === 'function' && sessionMetrics) updateAnalysis();
        if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
        if (isDeviceOnline) {
            pushIdealToDevice(true);
            showToast('Stroke ' + strokeNum + ' set as ideal — pushing to wearable', 'success');
        } else {
            showToast('Ideal saved on this device. Sync on dashboard Wi‑Fi to store on the server.', 'info');
        }
        return;
    }

    let res = await apiPost('/api/ideal_stroke/set_from_stroke', {
        session_id: sidApi,
        stroke_num: Number(strokeNum),
        ideal_entry_angle: avgAngle
    });

    if (res && res.status === 'ok') {
        idealCompareStrokeNum = Number(strokeNum);
        idealCompareStreamKey = (streamKey != null && streamKey !== '') ? String(streamKey) : null;
        showToast('Stroke ' + strokeNum + ' set as ideal baseline', 'success');
        await afterIdealSavedServer();
        return;
    }

    res = await apiPost('/api/ideal_stroke', {
        name: 'Stroke ' + strokeNum + ' (session ' + currentSession.id + ')',
        samples,
        ideal_entry_angle: avgAngle
    });

    if (res && res.status === 'ok') {
        idealCompareStrokeNum = Number(strokeNum);
        idealCompareStreamKey = (streamKey != null && streamKey !== '') ? String(streamKey) : null;
        showToast('Stroke ' + strokeNum + ' saved as ideal (direct upload)', 'success');
        await afterIdealSavedServer();
        return;
    }

    idealCompareStrokeNum = Number(strokeNum);
    idealCompareStreamKey = (streamKey != null && streamKey !== '') ? String(streamKey) : null;
    idealStrokeData = samples;
    cacheIdealLocal(samples, avgAngle, 'Stroke ' + strokeNum, meta);
    idealDisplayInfo = {
        source: 'local',
        name: 'Stroke ' + strokeNum,
        numSamples: samples.length,
        savedAt: Date.now(),
        sourceSessionName: meta.sourceSessionName,
        sourceSessionId: meta.sourceSessionId
    };
    updateIdealStrokePanel();
    buildIdealComparison();
    if (typeof updateAnalysis === 'function' && sessionMetrics) updateAnalysis();
    if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
    const err = (res && res.error) ? res.error : 'Could not reach dashboard';
    showToast('Saved ideal on this device only: ' + err, 'warn');
}

async function setCurrentAsIdeal() {
    if (!processedData.length || !sessionMetrics || sessionMetrics.stroke_count < 1) {
        showToast('Open a session with strokes on the Session tab first', 'error');
        return;
    }
    const samples = typeof buildIdealLiaSamplesFromFullSession === 'function'
        ? buildIdealLiaSamplesFromFullSession()
        : processedData.map((p) => {
            const L = p.lia || null;
            const acc = p.acceleration || {};
            return {
                lia_x: (L && L.x != null) ? L.x : (acc.ax || 0),
                lia_y: (L && L.y != null) ? L.y : (acc.ay || 0),
                lia_z: (L && L.z != null) ? L.z : (acc.az || 0)
            };
        });
    const avgAngle = sessionMetrics ? sessionMetrics.avg_entry_angle : 30.0;
    const sess = typeof savedSessions !== 'undefined' ? savedSessions[activeSessionIdx] : null;
    const meta = {};
    if (sess) {
        if (sess.id != null) meta.sourceSessionId = sess.id;
        meta.sourceSessionName = sess.name || ('Session #' + (sess.id != null ? sess.id : ''));
    }
    const idealName = sess ? (sess.name || ('Session ' + (sess.id || 'current'))) : 'From Session';

    const res = await apiPost('/api/ideal_stroke', { name: idealName, samples, ideal_entry_angle: avgAngle });
    if (res && res.status === 'ok') {
        await loadIdealStroke();
        showToast('Ideal baseline saved. Pushing to wearable…', 'success');
        await pushIdealToDevice(true);
        return;
    }
    idealStrokeData = samples;
    cacheIdealLocal(samples, avgAngle, idealName, meta);
    idealDisplayInfo = {
        source: 'local',
        name: idealName,
        numSamples: samples.length,
        savedAt: Date.now(),
        sourceSessionName: meta.sourceSessionName,
        sourceSessionId: meta.sourceSessionId
    };
    updateIdealStrokePanel();
    buildIdealComparison();
    if (typeof updateAnalysis === 'function' && sessionMetrics) updateAnalysis();
    if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
    showToast('Saved on this device only: ' + ((res && res.error) || 'offline'), 'warn');
}

async function pushIdealToDevice(silent = false) {
    if (!idealStrokeData || !idealStrokeData.length) {
        if (!silent) showToast('No ideal stroke to push', 'error');
        return;
    }
    if (!isDeviceOnline && !silent) {
        pendingIdealSync = true;
        showToast('Device offline. Ideal queued for next connection.', 'info');
        return;
    } else if (!isDeviceOnline) {
        pendingIdealSync = true;
        return;
    }
    const result = await apiPost('/api/ideal_stroke/push', {
        samples: idealStrokeData,
        ideal_entry_angle: sessionMetrics ? sessionMetrics.avg_entry_angle : 30.0
    });
    if (!silent) {
        showToast(result && result.status === 'ok' ? 'Pushed to wearable' : 'Failed: ' + ((result && result.error) || 'unknown'), result && result.status === 'ok' ? 'success' : 'error');
    }
}

async function deleteIdealStroke() {
    if (!confirm('Delete ideal baseline from app and wearable?')) return;
    const result = await apiPost('/api/ideal_stroke/delete', {});
    if (result && result.status === 'ok') {
        idealStrokeData = null;
        idealDisplayInfo = null;
        try { localStorage.removeItem(LS_IDEAL_KEY); } catch (e) { /* */ }
        updateIdealStrokePanel();
        showToast('Ideal baseline removed', 'success');
    } else {
        try { localStorage.removeItem(LS_IDEAL_KEY); } catch (e) { /* */ }
        idealStrokeData = null;
        idealDisplayInfo = null;
        updateIdealStrokePanel();
        showToast('Removed locally. Server: ' + ((result && result.error) || 'unreachable'), 'warn');
    }
}

async function pushUserConfigToDevice(silent = false) {
    const units = (typeof window.gfGetUnits === 'function') ? window.gfGetUnits('settings-') : 'cm';
    const wingspanRaw = document.getElementById('settings-wingspan') ? document.getElementById('settings-wingspan').value : '';
    const heightRaw = document.getElementById('settings-height') ? document.getElementById('settings-height').value : '';
    const parseLenToCm = (typeof window.gfParseLenToCm === 'function')
        ? window.gfParseLenToCm
        : ((v) => parseFloat(v) || 0);
    const wingspan = parseLenToCm(wingspanRaw, units) || 180;
    const height = parseLenToCm(heightRaw, units) || 180;
    const skill = document.getElementById('settings-skill') ? document.getElementById('settings-skill').value : 'beginner';
    const poolLength = document.getElementById('settings-pool-length') ? parseFloat(document.getElementById('settings-pool-length').value) : 25;
    const devRoleEl = document.getElementById('dev-role');
    const device_role = devRoleEl ? normalizeDeviceRole(devRoleEl.value) : 'wrist_right';

    if (!isDeviceOnline && !silent) {
        pendingConfigSync = true;
        showToast('Device offline. Profile queued for next connection.', 'info');
        return;
    } else if (!isDeviceOnline) {
        pendingConfigSync = true;
        return;
    }

    const result = await apiPost('/api/user_config/push', {
        wingspan_cm: isNaN(wingspan) ? 180 : wingspan,
        height_cm: isNaN(height) ? 180 : height,
        skill_level: skill,
        pool_length: isNaN(poolLength) ? 25 : poolLength,
        device_role
    });

    if (!silent) {
        if (result && result.status === 'ok') {
            showToast('Profile pushed to device', 'success');
        } else {
            showToast('Failed to push profile: ' + ((result && result.error) || 'unknown'), 'error');
        }
    }
}

async function testHapticDevice() {
    const result = await apiPost('/api/test_buzz', {});
    if (result && result.status === 'ok') {
        showToast('Motor test sent', 'success');
    } else {
        showToast('Motor test failed: ' + ((result && result.error) || 'unknown'), 'error');
    }
}

window.refreshIdealViewerContext = refreshIdealViewerContext;
