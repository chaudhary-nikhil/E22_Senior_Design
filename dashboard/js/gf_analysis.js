/**
 * GoldenForm — Analysis tab: gauges, phases, stroke table helpers.
 */
// ── ANALYSIS ──
function resetAnalysisPlaceholders() {
    const dash = '-';
    drawGauge('gauge-aoa', 0, 0, 90, 15, 40);
    setText('aoa-value', dash);
    setText('aoa-ideal', '30°');
    ['phase-glide', 'phase-catch', 'phase-pull', 'phase-recovery'].forEach((id) => {
        const el = document.getElementById(id);
        if (el) el.style.width = '0%';
    });
    setText('phase-glide-pct', dash);
    setText('phase-catch-pct', dash);
    setText('phase-pull-pct', dash);
    setText('phase-recovery-pct', dash);
    setText('haptic-count', dash);
    const hh = document.getElementById('haptic-card-hint');
    if (hh) hh.textContent = 'Band buzzes during swim';
    setText('avg-deviation', dash);
    setText('form-score', dash);
    const fs = document.getElementById('form-score');
    if (fs) fs.style.color = 'var(--text2)';
    const tbody = document.getElementById('stroke-table-body');
    if (tbody) tbody.innerHTML = '';
    idealCompareStrokeNum = null;
    idealCompareStreamKey = null;
    if (typeof idealCompareMode !== 'undefined') idealCompareMode = 'stroke';
    const icm = document.getElementById('ideal-compare-mode');
    if (icm) icm.value = 'stroke';
}

function updateAnalysis() {
    if (!sessionMetrics) return;
    const m = sessionMetrics;
    drawGauge('gauge-aoa', m.avg_entry_angle || 0, 0, 90, 15, 40, '°');
    setText('aoa-value', (m.avg_entry_angle || 0).toFixed(1) + '°');
    setText('aoa-ideal', (m.ideal_entry_angle || 30) + '°');

    const pcts = m.phase_pcts || { glide: 0, catch: 0, pull: 0, recovery: 0 };
    const total = (pcts.glide || 0) + (pcts.catch || 0) + (pcts.pull || 0) + (pcts.recovery || 0) || 1;
    setWidth('phase-glide', (pcts.glide || 0) / total * 100);
    setWidth('phase-catch', (pcts.catch || 0) / total * 100);
    setWidth('phase-pull', (pcts.pull || 0) / total * 100);
    setWidth('phase-recovery', (pcts.recovery || 0) / total * 100);
    setText('phase-glide-pct', (pcts.glide || 0).toFixed(0) + '%');
    setText('phase-catch-pct', (pcts.catch || 0).toFixed(0) + '%');
    setText('phase-pull-pct', (pcts.pull || 0).toFixed(0) + '%');
    setText('phase-recovery-pct', (pcts.recovery || 0).toFixed(0) + '%');

    const hintEl = document.getElementById('haptic-card-hint');
    setText('haptic-count', m.haptic_count || 0);
    if (hintEl) hintEl.textContent = 'Band buzzes during swim';

    let avgDev = m.avg_deviation != null ? m.avg_deviation : 0;
    setText('avg-deviation', (typeof avgDev === 'number' ? avgDev : 0).toFixed(3));

    const score = computeFormScore(m, { avgDeviationOverride: avgDev });
    setText('form-score', score.toFixed(1));
    const scoreEl = document.getElementById('form-score');
    if (scoreEl) scoreEl.style.color = score >= 7 ? 'var(--green)' : score >= 5 ? 'var(--amber)' : 'var(--red)';

    buildIdealComparison();
    buildStrokeTable();
}

function setWidth(id, pct) { const el = document.getElementById(id); if (el) el.style.width = Math.max(1, pct) + '%'; }

/** Resolve vs-ideal row when stream keys differ: metrics use "4::" but boundaries use "4::hw2". */
function gfLookupVsIdealForStroke(num, streamKey) {
    if (!gfVsIdealMetrics || !gfVsIdealMetrics.hasIdeal || !gfVsIdealMetrics.byStroke) return null;
    const n = String(num);
    const sk = streamKey != null ? String(streamKey) : '';
    let vi = gfVsIdealMetrics.byStroke.get(n + '::' + sk);
    if (vi) return vi;
    vi = gfVsIdealMetrics.byStroke.get(n + '::');
    if (vi) return vi;
    for (const [k, v] of gfVsIdealMetrics.byStroke) {
        if (k.startsWith(n + '::')) return v;
    }
    return null;
}

function computeFormScore(m, opts = null) {
    let s = 4;
    s += Math.min((m.consistency || 0) / 100 * 2.5, 2.5);
    const dev = (opts && typeof opts.avgDeviationOverride === 'number')
        ? opts.avgDeviationOverride
        : (m.avg_deviation || 0);
    if (dev < 0.2) s += 2;
    else if (dev < 0.4) s += 1.5;
    else if (dev < 0.7) s += 0.5;
    else if (dev > 1) s -= 1;

    const angle = m.avg_entry_angle || 0;
    if (angle >= 20 && angle <= 35) s += 1;
    else if (angle >= 15 && angle <= 40) s += 0.5;
    else if (angle > 0) s -= 0.5;

    const sr = m.stroke_rate || 0;
    if (sr >= 15 && sr <= 40) s += 0.5;
    else if (sr > 0 && (sr < 10 || sr > 50)) s -= 0.5;

    const pcts = m.phase_pcts || {};
    const pull = pcts.pull || 0, glide = pcts.glide || 0;
    if (pull >= 18 && pull <= 35 && glide >= 20 && glide <= 55) s += 0.5;
    else if (pull < 10 || glide > 65) s -= 0.5;

    return Math.max(0, Math.min(10, Math.round(s * 10) / 10));
}

function drawGauge(canvasId, value, min, max, idealLow, idealHigh) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h - 10, r = Math.min(cx, cy) - 10;
    ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, 0); ctx.lineWidth = 14; ctx.strokeStyle = '#1a1a2e'; ctx.stroke();
    // Ideal zone
    const a1 = Math.PI + (idealLow - min) / (max - min) * Math.PI;
    const a2 = Math.PI + (idealHigh - min) / (max - min) * Math.PI;
    ctx.beginPath(); ctx.arc(cx, cy, r, a1, a2); ctx.lineWidth = 14; ctx.strokeStyle = 'rgba(34,197,94,0.25)'; ctx.stroke();
    // Value arc
    const pct = Math.max(0, Math.min(1, (value - min) / (max - min)));
    const valAngle = Math.PI + pct * Math.PI;
    ctx.beginPath(); ctx.arc(cx, cy, r, Math.PI, valAngle); ctx.lineWidth = 14;
    ctx.strokeStyle = (value >= idealLow && value <= idealHigh) ? '#22c55e' : (value > idealHigh ? '#ef4444' : '#eab308');
    ctx.stroke();
    // Needle
    const nx = cx + (r - 8) * Math.cos(valAngle), ny = cy + (r - 8) * Math.sin(valAngle);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(nx, ny); ctx.lineWidth = 2.5; ctx.strokeStyle = '#fff'; ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fillStyle = '#C5A55A'; ctx.fill();
}

function buildStrokeTable() {
    const tbody = document.getElementById('stroke-table-body');
    if (!tbody || !processedData.length) return;
    if (typeof gfComputeVsIdealMetrics === 'function' && typeof idealStrokeData !== 'undefined' && idealStrokeData && idealStrokeData.length) {
        gfComputeVsIdealMetrics();
    }
    if (typeof computeStrokeBoundaries === 'function') computeStrokeBoundaries();

    let rows = [];
    const startTime = processedData[0].timestamp || 0;

    if (sessionMetrics && sessionMetrics.stroke_breakdown && sessionMetrics.stroke_breakdown.length > 0) {
        rows = sessionMetrics.stroke_breakdown.map((p, idx) => {
            let streamKey = '';
            if (typeof strokeBoundaries !== 'undefined' && strokeBoundaries && strokeBoundaries.length) {
                const b = strokeBoundaries[idx];
                if (b && b.strokeNum === p.number) streamKey = b.streamKey || '';
                else {
                    const hit = strokeBoundaries.find((sb) => sb.strokeNum === p.number);
                    if (hit) streamKey = hit.streamKey || '';
                }
            }
            return {
                num: p.number,
                streamKey,
                time: Math.max(0, p.timestamp_s - (startTime / 1000)).toFixed(1),
                angle: p.entry_angle || 0,
                haptic: p.haptic_fired,
                deviation: p.deviation || 0
            };
        });
    } else if (typeof strokeBoundaries !== 'undefined' && strokeBoundaries && strokeBoundaries.length > 0) {
        for (let b = 0; b < strokeBoundaries.length; b++) {
            const sb = strokeBoundaries[b];
            const start = sb.index;
            const end = (b + 1 < strokeBoundaries.length) ? strokeBoundaries[b + 1].index - 1 : processedData.length - 1;
            const p = processedData[start];
            let angle = p.entry_angle || 0;
            let deviation = p.deviation_score || 0;
            let haptic = p.haptic_fired;
            const scanLimit = Math.min(start + 120, end + 120, processedData.length);
            for (let j = start + 1; j <= Math.min(end, scanLimit - 1); j++) {
                const q = processedData[j];
                if (q.deviation_score > 0 && deviation === 0) deviation = q.deviation_score;
                if (q.haptic_fired) haptic = true;
                if (q.entry_angle > 0 && angle === 0) angle = q.entry_angle;
            }
            const sk = sb.streamKey || '';
            rows.push({
                num: sb.strokeNum,
                streamKey: sk,
                label: sb.label || String(sb.strokeNum),
                time: Math.max(0, (p.timestamp - startTime) / 1000).toFixed(1),
                angle,
                haptic,
                deviation
            });
        }
    } else {
        let lastCount = 0;
        for (let i = 0; i < processedData.length; i++) {
            const p = processedData[i];
            if (p.stroke_count > lastCount) {
                let angle = p.entry_angle || 0;
                let deviation = p.deviation_score || 0;
                let haptic = p.haptic_fired;
                const scanLimit = Math.min(i + 120, processedData.length);
                for (let j = i + 1; j < scanLimit; j++) {
                    const q = processedData[j];
                    if (q.stroke_count > p.stroke_count) break;
                    if (q.deviation_score > 0 && deviation === 0) deviation = q.deviation_score;
                    if (q.haptic_fired) haptic = true;
                    if (q.entry_angle > 0 && angle === 0) angle = q.entry_angle;
                }
                rows.push({
                    num: p.stroke_count,
                    streamKey: '',
                    label: String(p.stroke_count),
                    time: Math.max(0, (p.timestamp - startTime) / 1000).toFixed(1),
                    angle,
                    haptic,
                    deviation
                });
                lastCount = p.stroke_count;
            }
        }
    }
    tbody.innerHTML = rows.map(r => {
        const firstCol = r.label && r.label !== String(r.num) ? r.label : String(r.num);
        const vi = (gfVsIdealMetrics && gfVsIdealMetrics.hasIdeal)
            ? gfLookupVsIdealForStroke(r.num, r.streamKey)
            : null;
        const fwDev = r.deviation || 0;
        const fwHaptic = !!r.haptic;
        const clientDev = vi ? vi.deviation : 0;
        const devShow = fwDev > 0 ? fwDev : (vi ? clientDev : 0);
        const cueFire = fwHaptic;
        const cueTitle = fwHaptic
            ? ('Band haptic fired · dev ' + devShow.toFixed(3))
            : (vi ? ('vs ideal ' + devShow.toFixed(3) + ' · no band buzz') : 'No band haptic');
        const rowClass = (cueFire ? 'row-haptic' : '') +
            (idealCompareStrokeNum === r.num && (idealCompareStreamKey || '') === (r.streamKey || '')
                ? ' analysis-row-selected'
                : '');
        const cueCell = cueFire
            ? `<span class="haptic-marker" title="${String(cueTitle).replace(/"/g, '&quot;')}">⚡</span>`
            : `<span title="${String(cueTitle).replace(/"/g, '&quot;')}">✓</span>`;
        const devClass = devShow > 0.7 ? 'text-red' : devShow > 0.3 ? 'text-amber' : 'text-green';
        const skJson = (r.streamKey != null && r.streamKey !== '') ? JSON.stringify(r.streamKey) : 'null';
        return `<tr class="${rowClass}" style="cursor:pointer;" onclick="analysisStrokeRowClick(event, ${r.num}, ${skJson})"><td>${firstCol}</td><td>${r.time}s</td><td>${r.angle.toFixed(1)}°</td><td>${cueCell}</td><td class="${devClass}">${devShow.toFixed(3)}</td><td onclick="event.stopPropagation(); setSingleStrokeAsIdeal(${r.num}, ${r.streamKey ? JSON.stringify(r.streamKey) : 'undefined'})"><button class="btn btn-gold btn-sm" style="padding:2px 6px; font-size:0.7em;">Set Ideal</button></td></tr>`;
    }).join('');
}

