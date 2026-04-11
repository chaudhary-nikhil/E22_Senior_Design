/**
 * GoldenForm — Coaching insight text generation.
 * Sections: Session Consistency (heatmap), Per-Stroke Issues (diagnosis), Top Coaching Priorities.
 */
// ── COACHING INSIGHTS ──
let _coachingDataHash = '';

function setInsightsSourceHint(mode) {
    const el = document.getElementById('insights-source-hint');
    if (!el) return;
    if (mode === 'local') {
        el.hidden = false;
        el.textContent = 'Insights use sessions saved on this device. Sync your band to add new swims; replay on Session.';
    } else {
        el.hidden = true;
        el.textContent = '';
    }
}

/** Extract rich per-stroke analytics from processedData + strokeBoundaries. */
function _buildStrokeAnalytics() {
    const analytics = [];
    if (!strokeBoundaries || !strokeBoundaries.length) return analytics;
    const startTime = processedData[0] ? (processedData[0].timestamp || 0) : 0;
    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length) ? strokeBoundaries[b + 1].index - 1 : processedData.length - 1;
        let entryAngle = 0, maxLia = 0, hapticFired = false, hapticReason = 0;
        let devScore = 0, pullDur = 0, liaMags = [];
        for (let i = start; i <= end; i++) {
            const d = processedData[i];
            const lia = typeof gfLiaMagFromProcessed === 'function' ? gfLiaMagFromProcessed(d) : 0;
            liaMags.push(lia);
            if (lia > maxLia) maxLia = lia;
            if (d.entry_angle > 0 && entryAngle === 0) entryAngle = d.entry_angle;
            if (d.haptic_fired) {
                hapticFired = true;
                hapticReason = d.haptic_reason || 0;
                if (d.deviation_score > devScore) devScore = d.deviation_score;
                if (d.pull_duration_ms > 0) pullDur = d.pull_duration_ms;
            }
            if (d.deviation_score > devScore && !hapticFired) devScore = d.deviation_score;
        }
        const tStart = processedData[start] ? (processedData[start].timestamp || 0) : 0;
        const tEnd = processedData[end] ? (processedData[end].timestamp || 0) : 0;
        const durationMs = tEnd - tStart;
        analytics.push({
            num: sb.strokeNum,
            streamKey: sb.streamKey,
            label: sb.label || ('#' + sb.strokeNum),
            entryAngle,
            maxLia,
            durationMs,
            hapticFired,
            hapticReason,
            devScore,
            pullDur,
            sampleCount: end - start + 1,
            liaMean: liaMags.length ? liaMags.reduce((a, b) => a + b, 0) / liaMags.length : 0
        });
    }
    return analytics;
}

function _mean(arr) { return arr.length ? arr.reduce((a, b) => a + b, 0) / arr.length : 0; }
function _stddev(arr) {
    if (arr.length < 2) return 0;
    const m = _mean(arr);
    return Math.sqrt(arr.reduce((s, v) => s + (v - m) * (v - m), 0) / arr.length);
}

function updateCoachingInsights() {
    const viewerHasData = typeof processedData !== 'undefined' && processedData && processedData.length;
    let insightPd = processedData;
    let insightMetrics = sessionMetrics;
    let usedLocalFallback = false;
    if ((!insightPd || !insightPd.length) && hasSavedSessions() && activeSessionIdx >= 0) {
        const s = savedSessions[activeSessionIdx];
        insightPd = s.processed_data || s.processedData || [];
        insightMetrics = s.metrics || insightMetrics;
        usedLocalFallback = !!(insightPd && insightPd.length);
    }
    if (!insightMetrics || !insightPd || !insightPd.length) {
        setInsightsSourceHint('off');
        setText('coaching-priorities', 'Sync a session or open a saved session from the Session tab to see coaching insights.');
        const diag = document.getElementById('stroke-diagnosis-list');
        if (diag) diag.innerHTML = '<p style="color:var(--text3);font-size:0.9em;margin:0;">No session loaded. Select one on the Session tab after sync.</p>';
        setText('consistency-heatmap', '');
        return;
    }

    const idealFp = (typeof idealStrokeData !== 'undefined' && idealStrokeData && idealStrokeData.length)
        ? String(idealStrokeData.length) + ':' + String(idealStrokeData[0] && idealStrokeData[0].lia_x != null ? idealStrokeData[0].lia_x : '0')
        : '0';
    const selfBl = (typeof gfSessionIsIdealBaselineForUi === 'function' && gfSessionIsIdealBaselineForUi()) ? '1' : '0';
    const hash = String(insightPd.length) + ':' + String(insightMetrics.stroke_count || 0) + ':' +
        String(insightMetrics.avg_deviation || 0) + ':' + String(insightMetrics.avg_entry_angle || 0) + ':' +
        idealFp + ':' + selfBl;
    if (hash === _coachingDataHash) return;
    _coachingDataHash = hash;

    setInsightsSourceHint((viewerHasData || !usedLocalFallback) ? 'off' : 'local');

    const _pdBack = processedData;
    const _smBack = sessionMetrics;
    processedData = insightPd;
    sessionMetrics = insightMetrics;
    refreshStrokeFieldMode();

    const priorityBox = document.getElementById('coaching-priorities');
    const diagBox = document.getElementById('stroke-diagnosis-list');
    const heatBox = document.getElementById('consistency-heatmap');

    const _sbBack = strokeBoundaries.map(b => ({ index: b.index, strokeNum: b.strokeNum, streamKey: b.streamKey, label: b.label }));

    try {
        if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
        computeStrokeBoundaries();
        const strokes = _buildStrokeAnalytics();

        _renderConsistencyHeatmap(heatBox, strokes);
        _renderPerStrokeIssues(diagBox, strokes, insightMetrics);
        _renderCoachingPriorities(priorityBox, strokes, insightMetrics);

        _autoTriggerAiInsights();
    } finally {
        processedData = _pdBack;
        sessionMetrics = _smBack;
        strokeBoundaries = _sbBack;
        refreshStrokeFieldMode();
    }
}

// ── SESSION CONSISTENCY HEATMAP ──
// Compares each stroke to the session mean across entry angle, duration, and LIA peak.
function _renderConsistencyHeatmap(heatBox, strokes) {
    if (!heatBox) return;
    if (!strokes.length) {
        heatBox.innerHTML = '<p style="color:var(--text3); font-size:12px;">No strokes recorded.</p>';
        return;
    }
    const angles = strokes.map(s => s.entryAngle).filter(a => a > 0);
    const durations = strokes.map(s => s.durationMs).filter(d => d > 0);
    const peaks = strokes.map(s => s.maxLia);

    const mAngle = _mean(angles), sdAngle = _stddev(angles) || 1;
    const mDur = _mean(durations), sdDur = _stddev(durations) || 1;
    const mPeak = _mean(peaks), sdPeak = _stddev(peaks) || 0.1;

    let html = '';
    let goodCount = 0;
    strokes.forEach(s => {
        const zAngle = s.entryAngle > 0 ? Math.abs(s.entryAngle - mAngle) / sdAngle : 0;
        const zDur = s.durationMs > 0 ? Math.abs(s.durationMs - mDur) / sdDur : 0;
        const zPeak = Math.abs(s.maxLia - mPeak) / sdPeak;
        const zMax = Math.max(zAngle, zDur, zPeak);

        let color = 'var(--green)';
        if (zMax > 2.0) color = 'var(--red)';
        else if (zMax > 1.0) color = 'var(--amber)';
        else goodCount++;

        let tooltipParts = [(s.label || ('Stroke ' + s.num))];
        if (zAngle > 1) tooltipParts.push('angle ' + s.entryAngle.toFixed(0) + '° (avg ' + mAngle.toFixed(0) + '°)');
        if (zDur > 1) tooltipParts.push('timing ' + (s.durationMs / 1000).toFixed(1) + 's (avg ' + (mDur / 1000).toFixed(1) + 's)');
        if (zPeak > 1) tooltipParts.push('peak accel ' + s.maxLia.toFixed(1) + ' (avg ' + mPeak.toFixed(1) + ')');
        if (s.hapticFired) tooltipParts.push('⚡ band cued');
        if (tooltipParts.length === 1) tooltipParts.push('consistent');
        const tooltip = tooltipParts.join(' · ');

        const jk = s.streamKey ? `, ${JSON.stringify(s.streamKey)}` : '';
        html += `<div title="${tooltip.replace(/"/g, '&quot;')}" style="width:14px;height:14px;border-radius:2px;background-color:${color};cursor:pointer;" onclick="switchTab('session'); setTimeout(()=>jumpToStroke(${s.num}${jk}), 50);"></div>`;
    });

    const pctGood = Math.round(goodCount / strokes.length * 100);
    html += `<div style="width:100%;font-size:0.75em;color:var(--text3);margin-top:4px;">${goodCount}/${strokes.length} strokes consistent (${pctGood}%). Green=consistent, Yellow=minor variation, Red=outlier vs session average.</div>`;
    heatBox.innerHTML = html;
}

// ── PER-STROKE PATTERNS ──
// Summarizes stroke-level patterns for coaching; individual stroke breakdown lives in Analysis tab.
function _renderPerStrokeIssues(diagBox, strokes, metrics) {
    if (!diagBox) return;
    if (!strokes.length) {
        diagBox.innerHTML = '<p style="color:var(--text3);font-size:0.9em;">No strokes to analyze.</p>';
        return;
    }

    const hapticStrokes = strokes.filter(s => s.hapticFired);
    const durations = strokes.map(s => s.durationMs).filter(d => d > 0);
    const meanDur = _mean(durations);
    const durSd = _stddev(durations);
    const angles = strokes.map(s => s.entryAngle).filter(a => a > 0);
    const avgAngle = _mean(angles);
    const angleSd = _stddev(angles);
    const peaks = strokes.map(s => s.maxLia);
    const peakSd = _stddev(peaks);
    const rushed = strokes.filter(s => meanDur > 0 && s.durationMs > 0 && (s.durationMs - meanDur) / meanDur < -0.30);
    const slow = strokes.filter(s => meanDur > 0 && s.durationMs > 0 && (s.durationMs - meanDur) / meanDur > 0.30);

    let html = '';
    const patterns = [];

    if (hapticStrokes.length > 0) {
        const consecutive = _findConsecutiveRuns(hapticStrokes.map(s => s.num));
        const runDesc = consecutive.length ? consecutive.map(r => r.length > 1 ? (r[0] + '–' + r[r.length - 1]) : String(r[0])).join(', ') : hapticStrokes.map(s => s.num).join(', ');
        patterns.push({
            color: 'var(--red)',
            title: 'Band cued on ' + hapticStrokes.length + ' of ' + strokes.length + ' strokes',
            detail: 'Strokes: ' + runDesc + '. Avg firmware deviation on cued strokes: ' + _mean(hapticStrokes.map(s => s.devScore)).toFixed(2) + '. Open <strong>Analysis</strong> for the full |LIA| chart and stroke table.'
        });
    }

    if (angleSd > 8 && angles.length >= 3) {
        patterns.push({
            color: 'var(--amber)',
            title: 'Entry angle varies widely (±' + angleSd.toFixed(0) + '°)',
            detail: 'Range: ' + Math.min(...angles).toFixed(0) + '°–' + Math.max(...angles).toFixed(0) + '°. A consistent entry angle sets up a better catch.'
        });
    }

    if (rushed.length > 0 || slow.length > 0) {
        const parts = [];
        if (rushed.length) parts.push(rushed.length + ' rushed (>' + (meanDur * 0.7 / 1000).toFixed(1) + 's faster)');
        if (slow.length) parts.push(slow.length + ' slow');
        patterns.push({
            color: 'var(--amber)',
            title: 'Timing inconsistency: ' + parts.join(', '),
            detail: 'Avg stroke: ' + (meanDur / 1000).toFixed(1) + 's ± ' + (durSd / 1000).toFixed(1) + 's. Consistent rhythm improves efficiency.'
        });
    }

    if (peakSd > 3 && peaks.length >= 3) {
        patterns.push({
            color: 'var(--amber)',
            title: 'Power output varies between strokes',
            detail: 'Peak acceleration ranges from ' + Math.min(...peaks).toFixed(1) + ' to ' + Math.max(...peaks).toFixed(1) + ' m/s². Aim for even effort across all strokes.'
        });
    }

    if (patterns.length === 0) {
        if (hapticStrokes.length === 0) {
            html = '<p style="color:var(--green);font-size:0.9em;margin:0;">No band cues fired and stroke patterns are consistent. Solid session.</p>';
        } else {
            html = '<p style="color:var(--green);font-size:0.9em;margin:0;">No major patterns detected. Check the Analysis tab for stroke-by-stroke details.</p>';
        }
    } else {
        html = patterns.map(p =>
            `<div style="background:var(--bg3);padding:8px 10px;border-left:3px solid ${p.color};border-radius:4px;margin-bottom:6px;font-size:0.85em;">
                <strong style="color:var(--text1);">${p.title}</strong>
                <div style="color:var(--text2);margin-top:3px;font-size:0.9em;">${p.detail}</div>
            </div>`
        ).join('');
    }
    diagBox.innerHTML = html;
}

function _findConsecutiveRuns(nums) {
    if (!nums.length) return [];
    const sorted = [...nums].sort((a, b) => a - b);
    const runs = [[sorted[0]]];
    for (let i = 1; i < sorted.length; i++) {
        if (sorted[i] === sorted[i - 1] + 1) runs[runs.length - 1].push(sorted[i]);
        else runs.push([sorted[i]]);
    }
    return runs;
}

// ── TOP COACHING PRIORITIES ──
// Ranked, quantified priorities that synthesize across all strokes.
function _renderCoachingPriorities(priorityBox, strokes, metrics) {
    if (!priorityBox) return;
    if (!strokes.length) {
        priorityBox.innerHTML = '<p style="color:var(--text3);">No strokes to analyze.</p>';
        return;
    }

    const m = metrics || {};
    const priorities = [];

    const angles = strokes.map(s => s.entryAngle).filter(a => a > 0);
    const avgAngle = _mean(angles);
    const angleSd = _stddev(angles);
    if (avgAngle > 0 && (avgAngle < 15 || avgAngle > 40)) {
        const dir = avgAngle < 15 ? 'too flat' : 'too steep';
        priorities.push({
            severity: Math.abs(avgAngle - 27.5) / 12.5,
            title: 'Entry Angle',
            detail: `Your average entry angle is ${avgAngle.toFixed(0)}° (${dir}). The target range is 15–40°, with 25–35° being optimal for freestyle. ${angleSd > 8 ? 'Your angle also varies a lot (±' + angleSd.toFixed(0) + '°) — work on a repeatable fingertips-first entry.' : 'Focus on a cleaner catch with a higher elbow.'}`
        });
    } else if (angleSd > 10 && angles.length >= 3) {
        priorities.push({
            severity: angleSd / 10,
            title: 'Entry Angle Consistency',
            detail: `Your entry angle varies widely (±${angleSd.toFixed(0)}°, range ${Math.min(...angles).toFixed(0)}–${Math.max(...angles).toFixed(0)}°). A consistent entry sets up a better catch. Practice a repeatable arm extension.`
        });
    }

    const fwDevs = strokes.map(s => s.devScore).filter(d => d > 0);
    const avgFwDev = _mean(fwDevs);
    const hapticCount = strokes.filter(s => s.hapticFired).length;
    if (hapticCount > 0) {
        const pctHaptic = Math.round(hapticCount / strokes.length * 100);
        priorities.push({
            severity: avgFwDev > 0 ? avgFwDev : 1.0,
            title: 'Stroke Form Deviation',
            detail: `Band buzzed on ${hapticCount} of ${strokes.length} strokes (${pctHaptic}%). Average firmware deviation: ${avgFwDev.toFixed(2)}. Focus on matching the acceleration pattern you set as your baseline — smooth catch, steady pull, clean recovery.`
        });
    }

    const durations = strokes.map(s => s.durationMs).filter(d => d > 0);
    const meanDur = _mean(durations);
    const durSd = _stddev(durations);
    if (meanDur > 0 && durSd / meanDur > 0.25 && durations.length >= 3) {
        priorities.push({
            severity: durSd / meanDur * 2,
            title: 'Stroke Timing',
            detail: `Your stroke-to-stroke timing varies by ${(durSd / meanDur * 100).toFixed(0)}% (avg ${(meanDur / 1000).toFixed(1)}s ± ${(durSd / 1000).toFixed(1)}s). Consistent rhythm improves efficiency. Try counting or pacing with breathing.`
        });
    }

    const sr = m.stroke_rate || 0;
    if (sr > 0 && (sr < 10 || sr > 50)) {
        priorities.push({
            severity: 0.6,
            title: 'Stroke Rate',
            detail: sr < 10
                ? `Your stroke rate of ${sr.toFixed(0)} strokes/min is quite low. For distance freestyle, 20–30 is typical. Increase your tempo without sacrificing form.`
                : `Your stroke rate of ${sr.toFixed(0)} strokes/min is very high. Rushing reduces distance per stroke. Slow down and extend your glide.`
        });
    }

    const pcts = m.phase_pcts || {};
    const pullPct = pcts.pull || 0, recovPct = pcts.recovery || 0, glidePct = pcts.glide || 0;
    if (pullPct > 0 && recovPct > 0) {
        if (pullPct < 15) {
            priorities.push({
                severity: 0.7,
                title: 'Pull Phase Too Short',
                detail: `Your pull phase is only ${pullPct.toFixed(0)}% of the stroke cycle (typical is 20–30%). A longer, more engaged pull generates more propulsion. Focus on a full S-curve pull through.`
            });
        }
        if (recovPct > 35) {
            priorities.push({
                severity: 0.5,
                title: 'Recovery Phase Long',
                detail: `Recovery takes ${recovPct.toFixed(0)}% of your cycle (typical 15–25%). A quicker arm recovery above water lets you spend more time in propulsive phases.`
            });
        }
        if (glidePct > 60) {
            priorities.push({
                severity: 0.5,
                title: 'Excessive Glide',
                detail: `Glide is ${glidePct.toFixed(0)}% of your cycle. Some glide is good, but over 50% can mean you are decelerating too much between strokes. Initiate the catch earlier.`
            });
        }
    }

    priorities.sort((a, b) => b.severity - a.severity);
    const top = priorities.slice(0, 3);

    let selfBaseNote = '';
    if (typeof gfSessionIsIdealBaselineForUi === 'function' && gfSessionIsIdealBaselineForUi()) {
        selfBaseNote = '<p style="margin:0 0 10px 0;font-size:0.82em;color:var(--text3);line-height:1.45;">This swim is your saved baseline template—vs-baseline scores are hidden so you are not judged against yourself. Load another session to see gaps vs this template.</p>';
    }

    if (top.length === 0) {
        const cons = m.consistency || 0;
        const noHaptics = hapticCount === 0;
        if (cons >= 80 && noHaptics) {
            priorityBox.innerHTML = selfBaseNote + '<div style="padding:10px; background:rgba(34,197,94,0.1); color:var(--green); border-radius:6px; font-weight:500;">Session looks solid across all metrics — no band cues fired. Keep this consistency and consider setting a more challenging ideal baseline.</div>';
        } else {
            priorityBox.innerHTML = selfBaseNote + '<div style="padding:10px; background:rgba(34,197,94,0.1); color:var(--green); border-radius:6px; font-weight:500;">No major issues. ' + (cons < 80 ? 'Work on stroke-to-stroke consistency (' + cons.toFixed(0) + '%).' : 'Maintain this form.') + '</div>';
        }
        return;
    }

    let html = selfBaseNote;
    top.forEach((p, i) => {
        const borderColor = i === 0 ? 'var(--red)' : 'var(--amber)';
        html += `<div style="padding:10px; background:var(--bg3); border-left:3px solid ${borderColor}; border-radius:4px; margin-bottom:8px;">
            <h4 style="margin:0 0 4px 0; color:var(--text1); font-size:0.95em;">${i + 1}. ${p.title}</h4>
            <p style="margin:0; font-size:0.84em; color:var(--text2); line-height:1.45;">${p.detail}</p>
        </div>`;
    });
    priorityBox.innerHTML = html;
}

let _aiInsightsSessionKey = '';
function _aiInsightsKeyForAuto() {
    const idealFp = (typeof idealStrokeData !== 'undefined' && idealStrokeData && idealStrokeData.length)
        ? String(idealStrokeData.length) + ':' + String(idealStrokeData[0] && idealStrokeData[0].lia_x != null ? idealStrokeData[0].lia_x : '0')
        : '0';
    const selfBl = (typeof gfSessionIsIdealBaselineForUi === 'function' && gfSessionIsIdealBaselineForUi()) ? '1' : '0';
    return String(sessionMetrics ? (sessionMetrics.stroke_count || 0) : 0) + ':' +
        String(processedData ? processedData.length : 0) + ':' + idealFp + ':' + selfBl;
}
function _autoTriggerAiInsights() {
    const key = _aiInsightsKeyForAuto();
    if (key === _aiInsightsSessionKey) return;
    if (!processedData || !processedData.length) return;
    _aiInsightsSessionKey = key;
    const aiBody = document.getElementById('ai-coaching-body');
    if (aiBody) {
        aiBody.innerHTML = '';
        aiBody.textContent = 'Generating…';
    }
    setTimeout(() => refreshAiCoachingInsights(), 200);
}

/** Call when entering Insights so AI runs even if coaching hash short-circuited (tab-only navigation). */
function gfEnsureInsightsTabAiCoaching() {
    const bodyEl = document.getElementById('ai-coaching-body');
    const key = _aiInsightsKeyForAuto();
    const t = bodyEl ? String(bodyEl.textContent || '').trim() : '';
    if (t === 'Generating…' && key === _aiInsightsSessionKey) return;
    const looksEmpty = !t || t.startsWith('Load a session') || t.startsWith('No response') ||
        t.startsWith('Unexpected response') || t.startsWith('AI request failed') || t.startsWith('Gemini unreachable') ||
        t.startsWith('Network error') || t.startsWith('Too many AI') || t.startsWith('Offline.');
    if (!sessionMetrics || !processedData || !processedData.length) {
        if (bodyEl && looksEmpty) bodyEl.textContent = 'Load a session on the Session tab first.';
        return;
    }
    if (key === _aiInsightsSessionKey && !looksEmpty) return;
    _aiInsightsSessionKey = '';
    _autoTriggerAiInsights();
}

function _gfCoachingIdealBaselineKind() {
    if (typeof idealStrokeData === 'undefined' || !idealStrokeData || !idealStrokeData.length) return 'none';
    const name = (typeof idealDisplayInfo !== 'undefined' && idealDisplayInfo && idealDisplayInfo.name)
        ? String(idealDisplayInfo.name) : '';
    if (/stroke\s+\d+/i.test(name)) return 'single_stroke';
    const pd = typeof processedData !== 'undefined' && processedData ? processedData : [];
    if (pd.length > 40 && idealStrokeData.length >= Math.floor(pd.length * 0.65)) return 'full_session';
    return 'library_or_custom';
}

function _gfCoachingSameSessionAsIdeal(insightPd) {
    try {
        if (typeof idealDisplayInfo === 'undefined' || !idealDisplayInfo || idealDisplayInfo.sourceSessionId == null) return false;
        if (typeof savedSessions === 'undefined' || typeof activeSessionIdx === 'undefined' || activeSessionIdx < 0) return false;
        const s = savedSessions[activeSessionIdx];
        if (!s || s.id == null) return false;
        return Number(idealDisplayInfo.sourceSessionId) === Number(s.id);
    } catch (e) {
        return false;
    }
}

/**
 * Server-side Gemini coaching (requires GEMINI_API_KEY on dashboard server).
 */
function gfEscapeHtmlInsight(s) {
    return String(s)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;');
}

function gfFormatInsightInline(s) {
    let x = gfEscapeHtmlInsight(s);
    x = x.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
    return x;
}

function gfFormatInsightMarkdown(text) {
    if (!text) return '';
    const lines = String(text).split(/\r?\n/);
    const parts = [];
    let inList = false;
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i];
        const t = line.trim();
        if (/^###\s+/.test(t)) {
            if (inList) { parts.push('</ul>'); inList = false; }
            parts.push('<h4 class="coaching-md-h">' + gfEscapeHtmlInsight(t.replace(/^###\s+/, '')) + '</h4>');
        } else if (/^[-*]\s+/.test(t)) {
            if (!inList) { parts.push('<ul class="coaching-md-ul">'); inList = true; }
            parts.push('<li>' + gfFormatInsightInline(t.replace(/^[-*]\s+/, '')) + '</li>');
        } else if (t === '') {
            if (inList) { parts.push('</ul>'); inList = false; }
            parts.push('<br/>');
        } else {
            if (inList) { parts.push('</ul>'); inList = false; }
            parts.push('<p class="coaching-md-p">' + gfFormatInsightInline(t) + '</p>');
        }
    }
    if (inList) parts.push('</ul>');
    return '<div class="coaching-md">' + parts.join('') + '</div>';
}

async function refreshAiCoachingInsights() {
    const bodyEl = document.getElementById('ai-coaching-body');
    const metaEl = document.getElementById('ai-coaching-meta');
    if (!bodyEl) return;
    bodyEl.innerHTML = '';
    bodyEl.textContent = 'Generating…';
    if (metaEl) metaEl.textContent = '';

    const insightPd = (typeof processedData !== 'undefined' && processedData && processedData.length)
        ? processedData : null;
    const insightMetrics = sessionMetrics;
    if (!insightMetrics || !insightPd || !insightPd.length) {
        bodyEl.textContent = 'Load a session on the Session tab first.';
        return;
    }

    if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
    if (typeof computeStrokeBoundaries === 'function') computeStrokeBoundaries();

    const strokes = [];
    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length) ? strokeBoundaries[b + 1].index - 1 : insightPd.length - 1;
        const p = insightPd[start] || {};
        let entryAngle = Number(p.entry_angle) || 0;
        let devScore = Number(p.deviation_score) || 0;
        let hapticFired = !!p.haptic_fired;
        for (let j = start + 1; j <= Math.min(end, start + 120); j++) {
            const q = insightPd[j] || {};
            if (q.entry_angle > 0 && entryAngle === 0) entryAngle = Number(q.entry_angle);
            if (q.deviation_score > devScore) devScore = Number(q.deviation_score);
            if (q.haptic_fired) hapticFired = true;
        }
        let deviationVsIdeal = 0;
        let vsIdealAlert = false;
        if (typeof gfLookupVsIdealForStroke === 'function') {
            const vi = gfLookupVsIdealForStroke(sb.strokeNum, sb.streamKey);
            if (vi) {
                deviationVsIdeal = Number(vi.deviation) || 0;
                vsIdealAlert = !!vi.vsIdealAlert;
            }
        }
        strokes.push({
            stroke_num: sb.strokeNum,
            entry_angle: entryAngle,
            firmware_deviation: devScore,
            deviation_vs_ideal: deviationVsIdeal,
            vs_ideal_alert: vsIdealAlert,
            device_haptic: hapticFired
        });
    }

    const hasIdeal = !!(typeof idealStrokeData !== 'undefined' && idealStrokeData && idealStrokeData.length);
    const vsIdealAvg = (typeof gfVsIdealMetrics !== 'undefined' && gfVsIdealMetrics && gfVsIdealMetrics.hasIdeal &&
        typeof gfVsIdealMetrics.avgDeviation === 'number') ? gfVsIdealMetrics.avgDeviation : null;
    const payload = {
        session_metrics: {
            ...insightMetrics,
            avg_deviation_vs_ideal: vsIdealAvg
        },
        strokes,
        ideal_loaded: hasIdeal,
        ideal_baseline_kind: _gfCoachingIdealBaselineKind(),
        ideal_compare_mode: (typeof idealCompareMode !== 'undefined' && idealCompareMode) ? idealCompareMode : null,
        ideal_compare_stroke_num: null,
        same_session_as_ideal: _gfCoachingSameSessionAsIdeal(insightPd),
        is_self_baseline_template: (typeof gfSessionIsIdealBaselineForUi === 'function' && gfSessionIsIdealBaselineForUi())
    };

    const extraHeaders = {};
    try {
        if (localStorage.getItem('gf_coaching_privacy_nolog') === '1') {
            extraHeaders['X-GoldenForm-Coaching-NoLog'] = '1';
        }
    } catch (e) { /* ignore */ }
    const res = await apiPost('/api/coaching/insights', payload, { headers: extraHeaders });
    if (!res) {
        bodyEl.textContent = 'No response from server. Start the dashboard app, and connect this computer to normal Wi‑Fi with internet (not only the band hotspot).';
        return;
    }
    if (res.status === 'unconfigured') { bodyEl.textContent = res.message || 'Set GEMINI_API_KEY in dashboard/.env on the server.'; return; }
    if (res.httpStatus === 429 || res.rate_limited) { bodyEl.textContent = res.error || 'Too many AI requests. Wait a few minutes and try again.'; return; }
    if (res.status === 'error' || res._httpError) {
        const hint = res.hint ? String(res.hint) : '';
        const core = res.error || res.detail || 'AI request failed.';
        bodyEl.textContent = hint ? (core + ' ' + hint) : core;
        return;
    }
    if (res.status === 'ok' && res.text) {
        bodyEl.innerHTML = gfFormatInsightMarkdown(res.text);
        if (metaEl) metaEl.textContent = res.model ? ('Model: ' + res.model) : '';
        return;
    }
    bodyEl.textContent = 'Unexpected response.';
}
