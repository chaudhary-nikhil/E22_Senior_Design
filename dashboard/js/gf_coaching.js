/**
 * GoldenForm — Coaching insight text generation.
 */
// ── COACHING INSIGHTS ──
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
    setInsightsSourceHint((viewerHasData || !usedLocalFallback) ? 'off' : 'local');

    const _pdBack = processedData;
    const _smBack = sessionMetrics;
    processedData = insightPd;
    sessionMetrics = insightMetrics;
    refreshStrokeFieldMode();

    const priorityBox = document.getElementById('coaching-priorities');
    const diagBox = document.getElementById('stroke-diagnosis-list');
    const heatBox = document.getElementById('consistency-heatmap');

    let htmlPriorities = '';
    let htmlDiag = '';
    let htmlHeat = '';

    const strokes = [];
    const _sbBack = strokeBoundaries.map(b => ({ index: b.index, strokeNum: b.strokeNum, streamKey: b.streamKey, label: b.label }));

    try {
    if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
    computeStrokeBoundaries();
    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length) ? strokeBoundaries[b + 1].index - 1 : processedData.length - 1;
        const seg = { num: sb.strokeNum, streamKey: sb.streamKey, label: sb.label || ('#' + sb.strokeNum), events: [], maxDev: 0 };
        for (let i = start; i <= end; i++) {
            const d = processedData[i];
            if (d.haptic_fired) {
                seg.events.push({
                    reason: d.haptic_reason || 0,
                    dev: d.deviation_score || 0,
                    dur: d.pull_duration_ms || 0
                });
                if (d.deviation_score > seg.maxDev) seg.maxDev = d.deviation_score;
            }
        }
        strokes.push(seg);
    }

    // Heatmap (prefer vs-ideal when baseline loaded; else firmware haptic deviation)
    if (strokes.length > 0) {
        strokes.forEach(s => {
            let color = 'var(--green)';
            const jk = s.streamKey ? `, ${JSON.stringify(s.streamKey)}` : '';
            const key = String(s.num) + '::' + String(s.streamKey || '');
            const vi = (gfVsIdealMetrics && gfVsIdealMetrics.hasIdeal && gfVsIdealMetrics.byStroke)
                ? gfVsIdealMetrics.byStroke.get(key)
                : null;
            if (vi) {
                if (vi.vsIdealAlert) {
                    color = vi.deviation > 0.8 ? 'var(--red)' : 'var(--amber)';
                }
            } else if (s.events.length > 0) {
                if (s.maxDev > 0.8) color = 'var(--red)';
                else color = 'var(--amber)';
            }
            htmlHeat += `<div title="${(s.label || ('Stroke ' + s.num)).replace(/"/g, '&quot;')}" style="width:14px;height:14px;border-radius:2px;background-color:${color};cursor:pointer;" onclick="switchTab('session'); setTimeout(()=>jumpToStroke(${s.num}${jk}), 50);"></div>`;
        });
    } else {
        htmlHeat = '<p style="color:var(--text3); font-size:12px;">No strokes recorded.</p>';
    }
    if(heatBox) heatBox.innerHTML = htmlHeat;

    // Diagnostics
    let hapticCount = 0;
    let reasonCounts = { 'Pull Too Fast': 0, 'Bad Entry Angle': 0, 'Path Deviation': 0 };

    strokes.forEach(s => {
        if (s.events.length > 0) {
            hapticCount++;
            const ev = s.events[0];
            let msg = '';
            
            // haptic_reason bitfield: 0x01 = DEV_HIGH, 0x02 = ENTRY_BAD, 0x04 = PULL_FAST
            const r = ev.reason;
            if (r & 0x04) { msg = 'Pull phase too fast (' + ev.dur.toFixed(0) + 'ms)'; reasonCounts['Pull Too Fast']++; }
            else if (r & 0x02) { msg = 'Hand entry too shallow/steep'; reasonCounts['Bad Entry Angle']++; }
            else if (r & 0x01) { msg = 'Stroke path deviated highly'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.75) { msg = 'High deviation vs ideal stroke path'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.4) { msg = 'Moderate path deviation. Refine pull line.'; reasonCounts['Path Deviation']++; }
            else if (ev.dev > 0.15) { msg = 'Entry or timing cue. Check angle of attack.'; reasonCounts['Bad Entry Angle']++; }
            else { msg = 'Form cue: alignment and rhythm.'; reasonCounts['Bad Entry Angle']++; }

            const jk2 = s.streamKey ? `, ${JSON.stringify(s.streamKey)}` : '';
            htmlDiag += `
                <div style="background:var(--bg3); padding:8px 10px; border-radius:6px; margin-bottom:6px; font-size:0.85em;">
                    <strong style="color:var(--text1);">${s.label || ('Stroke ' + s.num)}</strong>: <span style="color:var(--amber);">${msg}</span>
                    <button class="btn btn-outline btn-sm" style="float:right; padding:2px 8px; font-size:0.8em;" onclick="switchTab('session'); setTimeout(()=>jumpToStroke(${s.num}${jk2}), 50)">View</button>
                    <div style="clear:both;"></div>
                </div>`;
        }
    });
    if (!htmlDiag) htmlDiag = '<p style="color:var(--text3); font-size:0.9em;">Perfect! No issues detected in this session.</p>';
    if(diagBox) diagBox.innerHTML = htmlDiag;

    // Priorities
    if (hapticCount === 0) {
        htmlPriorities = `<div style="padding:10px; background:rgba(34,197,94,0.1); color:var(--green); border-radius:6px; font-weight:500;">Your form is looking solid. Keep focusing on consistency!</div>`;
    } else {
        const topIssue = Object.keys(reasonCounts).reduce((a, b) => reasonCounts[a] > reasonCounts[b] ? a : b);
        let advice = '';
        if (topIssue === 'Pull Too Fast') advice = 'Slow down your pull phase to engage more water. Rushing the pull drops your efficiency.';
        else if (topIssue === 'Bad Entry Angle') advice = 'Focus on a clean, fingertips-first entry. Keep your elbow high as you pierce the water.';
        else if (topIssue === 'Path Deviation') advice = 'Your hand is drifting from the ideal straight-line pull under your body. Keep it anchored.';
        else advice = 'Focus on fingertips-first entry and a steady catch–pull line.';

        let numBadStrokes = strokes.filter(s => s.events.length > 0).length;
        htmlPriorities = `
            <div style="padding:10px; background:var(--bg3); border-left:3px solid var(--amber); border-radius:4px; margin-bottom:6px;">
                <h4 style="margin:0 0 4px 0; color:var(--text1); font-size:1em;">1. ${topIssue}</h4>
                <p style="margin:0; font-size:0.85em; color:var(--text2);">${advice}</p>
            </div>
            <div style="font-size:0.85em; color:var(--text3); margin-top:8px;">
                Affected ${numBadStrokes} of ${strokes.length} strokes (${Math.round((numBadStrokes/strokes.length)*100)}%).
            </div>
        `;
    }
    if(priorityBox) priorityBox.innerHTML = htmlPriorities;
    } finally {
    processedData = _pdBack;
    sessionMetrics = _smBack;
    strokeBoundaries = _sbBack;
    refreshStrokeFieldMode();
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

/** Turn Gemini markdown-style output into readable HTML (headings, lists, bold). */
function gfFormatInsightMarkdown(text) {
    if (!text) return '';
    const lines = String(text).split(/\r?\n/);
    const parts = [];
    let inList = false;
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i];
        const t = line.trim();
        if (/^###\s+/.test(t)) {
            if (inList) {
                parts.push('</ul>');
                inList = false;
            }
            parts.push('<h4 class="coaching-md-h">' + gfEscapeHtmlInsight(t.replace(/^###\s+/, '')) + '</h4>');
        } else if (/^[-*]\s+/.test(t)) {
            if (!inList) {
                parts.push('<ul class="coaching-md-ul">');
                inList = true;
            }
            parts.push('<li>' + gfFormatInsightInline(t.replace(/^[-*]\s+/, '')) + '</li>');
        } else if (t === '') {
            if (inList) {
                parts.push('</ul>');
                inList = false;
            }
            parts.push('<br/>');
        } else {
            if (inList) {
                parts.push('</ul>');
                inList = false;
            }
            parts.push('<p class="coaching-md-p">' + gfFormatInsightInline(t) + '</p>');
        }
    }
    if (inList) parts.push('</ul>');
    return '<div class="coaching-md">' + parts.join('') + '</div>';
}

async function refreshAiCoachingInsights() {
    const bodyEl = document.getElementById('ai-coaching-body');
    const metaEl = document.getElementById('ai-coaching-meta');
    const btn = document.getElementById('ai-coaching-btn');
    if (!bodyEl) return;
    if (btn) {
        btn.disabled = true;
        btn.textContent = '…';
    }
    bodyEl.innerHTML = '';
    bodyEl.textContent = 'Generating…';
    if (metaEl) metaEl.textContent = '';

    const insightPd = (typeof processedData !== 'undefined' && processedData && processedData.length)
        ? processedData
        : null;
    const insightMetrics = sessionMetrics;
    if (!insightMetrics || !insightPd || !insightPd.length) {
        bodyEl.textContent = 'Load a session on the Session tab first.';
        if (btn) { btn.disabled = false; btn.textContent = 'Generate insights'; }
        return;
    }

    if (typeof gfComputeVsIdealMetrics === 'function') gfComputeVsIdealMetrics();
    if (typeof computeStrokeBoundaries === 'function') computeStrokeBoundaries();

    const strokes = [];
    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const p = insightPd[start] || {};
        const key = String(sb.strokeNum) + '::' + String(sb.streamKey || '');
        const vi = (gfVsIdealMetrics && gfVsIdealMetrics.byStroke)
            ? gfVsIdealMetrics.byStroke.get(key)
            : null;
        strokes.push({
            stroke_num: sb.strokeNum,
            entry_angle: Number(p.entry_angle) || 0,
            deviation_vs_ideal: vi ? vi.deviation : 0,
            device_haptic: vi ? vi.deviceHaptic : false,
            vs_ideal_alert: vi ? vi.vsIdealAlert : false
        });
    }

    const payload = {
        session_metrics: insightMetrics,
        strokes,
        ideal_loaded: !!(typeof idealStrokeData !== 'undefined' && idealStrokeData && idealStrokeData.length)
    };

    const extraHeaders = {};
    try {
        if (localStorage.getItem('gf_coaching_privacy_nolog') === '1') {
            extraHeaders['X-GoldenForm-Coaching-NoLog'] = '1';
        }
    } catch (e) { /* ignore */ }
    const res = await apiPost('/api/coaching/insights', payload, { headers: extraHeaders });
    if (btn) {
        btn.disabled = false;
        btn.textContent = 'Generate insights';
    }
    if (!res) {
        bodyEl.textContent = 'No response from server. Is the dashboard running?';
        return;
    }
    if (res.status === 'unconfigured') {
        bodyEl.textContent = res.message || 'Set GEMINI_API_KEY in dashboard/.env on the server.';
        return;
    }
    if (res.httpStatus === 429 || res.rate_limited) {
        bodyEl.textContent = res.error || 'Too many AI requests. Wait a few minutes and try again.';
        return;
    }
    if (res.status === 'error' || res._httpError) {
        bodyEl.textContent = res.error || res.detail || 'AI request failed.';
        return;
    }
    if (res.status === 'ok' && res.text) {
        bodyEl.innerHTML = gfFormatInsightMarkdown(res.text);
        if (metaEl) metaEl.textContent = res.model ? ('Model: ' + res.model) : '';
        return;
    }
    bodyEl.textContent = 'Unexpected response.';
}

