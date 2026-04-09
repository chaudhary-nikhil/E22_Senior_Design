/**
 * GoldenForm — Ideal stroke comparison (LIA vs session) + per-stroke deviation vs baseline.
 */

function gfLiaMagFromProcessed(d) {
    if (!d) return 0;
    const L = d.lia;
    if (L && (L.x != null || L.y != null || L.z != null)) {
        const x = Number(L.x) || 0, y = Number(L.y) || 0, z = Number(L.z) || 0;
        return Math.sqrt(x * x + y * y + z * z);
    }
    const acc = d.acceleration || {};
    const ax = Number(acc.ax) || 0, ay = Number(acc.ay) || 0, az = Number(acc.az) || 0;
    return Math.sqrt(ax * ax + ay * ay + az * az);
}

function gfLiaMagFromIdealSample(s) {
    if (!s) return 0;
    const x = Number(s.lia_x) || 0, y = Number(s.lia_y) || 0, z = Number(s.lia_z) || 0;
    return Math.sqrt(x * x + y * y + z * z);
}

/** Linear resample `arr` to length `outLen` (same rule as full-session chart). */
function gfResampleSeries(arr, outLen) {
    if (!arr || !arr.length || outLen < 1) return [];
    if (outLen === 1) return [arr[0]];
    const out = [];
    const n = arr.length;
    for (let i = 0; i < outLen; i++) {
        const t = n > 1 ? (i / (outLen - 1)) * (n - 1) : 0;
        const idx = Math.floor(t);
        const frac = t - idx;
        if (idx >= n - 1) out.push(arr[n - 1]);
        else out.push(arr[idx] * (1 - frac) + arr[idx + 1] * frac);
    }
    return out;
}

/**
 * When the saved ideal is a full session (long) and we compare one stroke, take the proportional
 * |LIA| segment from the ideal session so the comparison matches that stroke’s place in the lap.
 */
function gfIdealMagsSliceForStroke(idealFullMags, strokeIdx0, totalStrokes) {
    const n = idealFullMags.length;
    if (n < 2 || totalStrokes < 1) return idealFullMags;
    const t0 = strokeIdx0 / totalStrokes;
    const t1 = (strokeIdx0 + 1) / totalStrokes;
    let i0 = Math.floor(t0 * (n - 1));
    let i1 = Math.max(i0 + 1, Math.min(n - 1, Math.ceil(t1 * (n - 1))));
    if (i1 <= i0) i1 = Math.min(n - 1, i0 + 1);
    const out = [];
    for (let j = i0; j <= i1; j++) out.push(idealFullMags[j]);
    return out.length >= 2 ? out : idealFullMags;
}

function gfIdealMagsForStrokeCompare(actualMags, idealFullMags, strokeBoundaryIdx, totalStrokeSegments) {
    if (!idealFullMags || !idealFullMags.length || !actualMags || actualMags.length < 2) return idealFullMags;
    if (idealFullMags.length <= actualMags.length * 1.75) return idealFullMags;
    return gfIdealMagsSliceForStroke(idealFullMags, strokeBoundaryIdx, totalStrokeSegments);
}

/**
 * Same resampling and error model as the ideal comparison chart (both series → targetLen).
 * Returns resampled arrays so callers can avoid double work.
 */
function gfIdealCompareChartStats(actualLIA, idealLIA) {
    const L0 = actualLIA ? actualLIA.length : 0;
    const empty = { similarityPct: 0, deviation: 0, targetLen: 0, actualRs: [], idealRs: [] };
    if (L0 < 2 || !idealLIA || !idealLIA.length) return empty;
    const targetLen = Math.min(320, Math.max(48, L0));
    const actualRs = gfResampleSeries(actualLIA, targetLen);
    const idealRs = gfResampleSeries(idealLIA, targetLen);
    let sumDiff = 0;
    let sumIdeal = 0;
    for (let i = 0; i < targetLen; i++) {
        sumDiff += Math.abs(actualRs[i] - idealRs[i]);
        sumIdeal += Math.abs(idealRs[i]) || 0;
    }
    const similarityPct = Math.max(0, Math.min(100, (1 - sumDiff / (sumIdeal || 1)) * 100));
    const meanIdeal = sumIdeal / targetLen;
    const mae = sumDiff / targetLen;
    const denom = meanIdeal * 0.28 + 0.04;
    const deviation = Math.min(1.5, mae / denom);
    return { similarityPct, deviation, targetLen, actualRs, idealRs };
}

/**
 * Normalized ~0–1.5 deviation vs ideal; aligned with the Analysis chart (dual resample to targetLen).
 */
function gfDeviationVsIdealMae(actualMags, idealTemplateMags) {
    if (!actualMags || actualMags.length < 2 || !idealTemplateMags || !idealTemplateMags.length) return 0;
    return gfIdealCompareChartStats(actualMags, idealTemplateMags).deviation;
}

/** Sample index range for one stroke (merged sessions: pass streamKey when needed). */
function gfGetStrokeSampleRange(strokeNum, streamKey) {
    if (typeof processedData === 'undefined' || !processedData || !processedData.length) return null;
    refreshStrokeFieldMode();
    computeStrokeBoundaries();
    if (!strokeBoundaries || !strokeBoundaries.length) return null;
    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        if (sb.strokeNum !== strokeNum) continue;
        if (streamKey != null && streamKey !== '') {
            if (sb.streamKey !== streamKey) continue;
        }
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length)
            ? strokeBoundaries[b + 1].index - 1
            : processedData.length - 1;
        return { start, end, label: sb.label || ('#' + sb.strokeNum), sb };
    }
    return null;
}

/**
 * Prefer exact (strokeNum + streamKey); if streamKey mismatches merged-session rows, fall back to
 * the first segment with that stroke number so the chart still updates when you click a row.
 */
function gfGetStrokeSampleRangeLoose(strokeNum, streamKey) {
    if (typeof processedData === 'undefined' || !processedData || !processedData.length) return null;
    refreshStrokeFieldMode();
    computeStrokeBoundaries();
    if (!strokeBoundaries || !strokeBoundaries.length) return null;

    const pick = (b) => {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length)
            ? strokeBoundaries[b + 1].index - 1
            : processedData.length - 1;
        return { start, end, label: sb.label || ('#' + sb.strokeNum), sb };
    };

    if (streamKey != null && streamKey !== '') {
        for (let b = 0; b < strokeBoundaries.length; b++) {
            const sb = strokeBoundaries[b];
            if (sb.strokeNum === strokeNum && sb.streamKey === streamKey) return pick(b);
        }
    }
    for (let b = 0; b < strokeBoundaries.length; b++) {
        if (strokeBoundaries[b].strokeNum === strokeNum) return pick(b);
    }
    return null;
}

function gfValidateIdealCompareSelection() {
    if (typeof idealCompareStrokeNum === 'undefined') return;
    refreshStrokeFieldMode();
    computeStrokeBoundaries();
    if (!strokeBoundaries.length) return;
    const range = gfGetStrokeSampleRangeLoose(idealCompareStrokeNum, idealCompareStreamKey);
    if (range && range.sb) {
        idealCompareStrokeNum = range.sb.strokeNum;
        idealCompareStreamKey = range.sb.streamKey || null;
        return;
    }
    idealCompareStrokeNum = strokeBoundaries[0].strokeNum;
    idealCompareStreamKey = strokeBoundaries[0].streamKey || null;
}

/**
 * Recompute per-stroke deviation vs saved ideal (client-side). Device-reported deviation
 * stays 0 when no ideal was on the band at record time — this fixes Analysis when you Set Ideal in-app.
 */
function gfComputeVsIdealMetrics() {
    const thresh = (typeof GF_VS_IDEAL_HAPTIC_THRESH !== 'undefined')
        ? GF_VS_IDEAL_HAPTIC_THRESH
        : 0.32;
    gfVsIdealMetrics = {
        hasIdeal: false,
        byStroke: new Map(),
        avgDeviation: 0,
        vsIdealAlertCount: 0,
        deviceHapticByKey: new Map(),
        sessionChartDeviation: null,
        sessionVsIdealAlert: false,
        selectedStrokeDeviation: null
    };
    if (typeof idealStrokeData === 'undefined' || !idealStrokeData || !idealStrokeData.length) {
        return;
    }
    if (typeof processedData === 'undefined' || !processedData || !processedData.length) {
        return;
    }

    const idealTemplateMags = idealStrokeData.map(gfLiaMagFromIdealSample);
    const fullSessionMags = processedData.map((row) => gfLiaMagFromProcessed(row));
    if (fullSessionMags.length >= 2) {
        const sess = gfIdealCompareChartStats(fullSessionMags, idealTemplateMags);
        gfVsIdealMetrics.sessionChartDeviation = sess.deviation;
        gfVsIdealMetrics.sessionVsIdealAlert = sess.deviation > thresh;
    }

    refreshStrokeFieldMode();
    computeStrokeBoundaries();

    if (!strokeBoundaries || !strokeBoundaries.length) {
        gfVsIdealMetrics.hasIdeal = true;
        gfVsIdealMetrics.avgDeviation = gfVsIdealMetrics.sessionChartDeviation != null
            ? gfVsIdealMetrics.sessionChartDeviation
            : 0;
        gfVsIdealMetrics.vsIdealAlertCount = gfVsIdealMetrics.sessionVsIdealAlert ? 1 : 0;
        return;
    }

    gfVsIdealMetrics.hasIdeal = true;
    const devs = [];

    for (let b = 0; b < strokeBoundaries.length; b++) {
        const sb = strokeBoundaries[b];
        const start = sb.index;
        const end = (b + 1 < strokeBoundaries.length)
            ? strokeBoundaries[b + 1].index - 1
            : processedData.length - 1;
        const actualMags = [];
        let deviceHaptic = false;
        for (let i = start; i <= end; i++) {
            const d = processedData[i];
            actualMags.push(gfLiaMagFromProcessed(d));
            if (d.haptic_fired) deviceHaptic = true;
        }
        const idealForStroke = gfIdealMagsForStrokeCompare(
            actualMags, idealTemplateMags, b, strokeBoundaries.length
        );
        const st = gfIdealCompareChartStats(actualMags, idealForStroke);
        const dev = st.deviation;
        const key = String(sb.strokeNum) + '::' + String(sb.streamKey || '');
        const vsIdealAlert = dev > thresh;
        gfVsIdealMetrics.byStroke.set(key, {
            deviation: dev,
            deviceHaptic,
            vsIdealAlert
        });
        gfVsIdealMetrics.deviceHapticByKey.set(key, deviceHaptic);
        devs.push(dev);
        if (vsIdealAlert) gfVsIdealMetrics.vsIdealAlertCount++;
    }

    if (devs.length) {
        gfVsIdealMetrics.avgDeviation = devs.reduce((a, b) => a + b, 0) / devs.length;
    }

    let selDev = null;
    if (typeof idealCompareStrokeNum !== 'undefined' && idealCompareStrokeNum != null && strokeBoundaries.length) {
        const wantSk = (typeof idealCompareStreamKey !== 'undefined' && idealCompareStreamKey != null && idealCompareStreamKey !== '')
            ? String(idealCompareStreamKey)
            : '';
        for (let b = 0; b < strokeBoundaries.length; b++) {
            const sb = strokeBoundaries[b];
            if (sb.strokeNum !== idealCompareStrokeNum) continue;
            if (wantSk && String(sb.streamKey || '') !== wantSk) continue;
            const key = String(sb.strokeNum) + '::' + String(sb.streamKey || '');
            const row = gfVsIdealMetrics.byStroke.get(key);
            if (row) {
                selDev = row.deviation;
                break;
            }
        }
        if (selDev == null) {
            for (let b = 0; b < strokeBoundaries.length; b++) {
                if (strokeBoundaries[b].strokeNum !== idealCompareStrokeNum) continue;
                const key = String(strokeBoundaries[b].strokeNum) + '::' + String(strokeBoundaries[b].streamKey || '');
                const row = gfVsIdealMetrics.byStroke.get(key);
                if (row) {
                    selDev = row.deviation;
                    break;
                }
            }
        }
    }
    gfVsIdealMetrics.selectedStrokeDeviation = selDev;
}

function setIdealCompareStroke(num, streamKey) {
    idealCompareStrokeNum = num != null ? Number(num) : null;
    idealCompareStreamKey = (streamKey != null && streamKey !== '') ? String(streamKey) : null;
    if (typeof idealCompareMode !== 'undefined') idealCompareMode = 'stroke';
    const sel = document.getElementById('ideal-compare-mode');
    if (sel) sel.value = 'stroke';
    if (typeof buildIdealComparison === 'function') buildIdealComparison();
}

function onIdealCompareModeChange(v) {
    if (typeof idealCompareMode !== 'undefined') {
        idealCompareMode = (v === 'session') ? 'session' : 'stroke';
    }
    if (typeof buildIdealComparison === 'function') buildIdealComparison();
}

/** Row click on Analysis per-stroke table: compare this stroke + sync playback. */
function analysisStrokeRowClick(e, num, streamKey) {
    if (e && e.target && e.target.closest && e.target.closest('button')) return;
    const sk = (streamKey != null && streamKey !== '') ? streamKey : undefined;
    setIdealCompareStroke(num, sk);
    if (typeof jumpToStroke === 'function') jumpToStroke(num, sk);
}

// ── IDEAL STROKE COMPARISON (one stroke at a time vs saved ideal template) ──
function buildIdealComparison() {
    const host = document.getElementById('ideal-compare-chart-host');
    const cap = document.getElementById('ideal-compare-caption');
    if (!host) return;

    const lead = document.getElementById('ideal-compare-lead');
    if (lead) {
        lead.textContent = 'Pick a comparison mode, then click a row for a single stroke. Y = |LIA| (m/s²). X = progress through the stroke or through the full session when “Full session” is selected (both series resampled to the same length).';
    }

    const sel = document.getElementById('ideal-compare-mode');
    if (sel && typeof idealCompareMode !== 'undefined') sel.value = idealCompareMode;

    const destroyChart = () => {
        if (window._idealChart) {
            try { window._idealChart.destroy(); } catch (err) { /* ignore */ }
            window._idealChart = null;
        }
    };

    const showError = (msg) => {
        destroyChart();
        if (cap) {
            cap.style.display = 'none';
            cap.innerHTML = '';
        }
        host.innerHTML = '<p style="color:var(--text3);text-align:center;padding:16px;">' + msg + '</p>';
    };

    if (!processedData || !processedData.length) {
        showError('Load a session on the Session tab first.');
        setText('ideal-similarity', '--');
        return;
    }

    if (!idealStrokeData || !idealStrokeData.length) {
        showError('No ideal stroke saved. Set one in Settings or use <strong>Set Ideal</strong> on a stroke below.');
        setText('ideal-similarity', '--');
        gfComputeVsIdealMetrics();
        return;
    }

    refreshStrokeFieldMode();
    computeStrokeBoundaries();
    gfValidateIdealCompareSelection();

    if (idealCompareStrokeNum == null && strokeBoundaries && strokeBoundaries.length) {
        idealCompareStrokeNum = strokeBoundaries[0].strokeNum;
        idealCompareStreamKey = strokeBoundaries[0].streamKey || null;
    }

    const mode = (typeof idealCompareMode !== 'undefined' && idealCompareMode === 'session') ? 'session' : 'stroke';

    let actualLIA = [];
    let captionTitle = '';
    let xAxisTitle = 'Progress through this stroke (%)';
    let actualLabel = 'Actual (|LIA|)';

    if (mode === 'session') {
        actualLIA = processedData.map((row) => gfLiaMagFromProcessed(row));
        captionTitle = 'Full session';
        xAxisTitle = 'Progress through session (%)';
        actualLabel = 'Session (|LIA|)';
    } else {
        const range = gfGetStrokeSampleRangeLoose(idealCompareStrokeNum, idealCompareStreamKey);
        if (!range || range.end < range.start) {
            showError('No strokes found in this session.');
            setText('ideal-similarity', '--');
            gfComputeVsIdealMetrics();
            return;
        }
        for (let i = range.start; i <= range.end; i++) {
            actualLIA.push(gfLiaMagFromProcessed(processedData[i]));
        }
        const skLabel = (idealCompareStreamKey && String(idealCompareStreamKey).length)
            ? (' · ' + (typeof streamLabelShort === 'function'
                ? streamLabelShort(processedData[range.start])
                : idealCompareStreamKey))
            : '';
        captionTitle = 'Stroke ' + idealCompareStrokeNum + skLabel;
    }

    const idealMagsFull = idealStrokeData.map(gfLiaMagFromIdealSample);
    let idealLIA = idealMagsFull;
    if (mode === 'stroke' && strokeBoundaries && strokeBoundaries.length) {
        let bi = -1;
        for (let k = 0; k < strokeBoundaries.length; k++) {
            const sb = strokeBoundaries[k];
            if (sb.strokeNum !== idealCompareStrokeNum) continue;
            if (idealCompareStreamKey != null && idealCompareStreamKey !== '' &&
                String(sb.streamKey || '') !== String(idealCompareStreamKey)) continue;
            bi = k;
            break;
        }
        if (bi < 0) {
            for (let k = 0; k < strokeBoundaries.length; k++) {
                if (strokeBoundaries[k].strokeNum === idealCompareStrokeNum) {
                    bi = k;
                    break;
                }
            }
        }
        if (bi >= 0) {
            idealLIA = gfIdealMagsForStrokeCompare(actualLIA, idealMagsFull, bi, strokeBoundaries.length);
        }
    }

    let L = actualLIA.length;
    if (L < 2) {
        showError('Not enough samples to plot.');
        setText('ideal-similarity', '--');
        gfComputeVsIdealMetrics();
        return;
    }

    host.innerHTML = '<canvas id="ideal-compare-chart"></canvas>';

    const chartStats = gfIdealCompareChartStats(actualLIA, idealLIA);
    L = chartStats.targetLen;
    const actualRs = chartStats.actualRs;
    const resampled = chartStats.idealRs;
    const similarity = chartStats.similarityPct;
    setText('ideal-similarity', similarity.toFixed(0) + '%');

    if (cap) {
        cap.style.display = 'block';
        cap.innerHTML =
            '<strong>' + captionTitle + '</strong> — white = |LIA| (resampled). Gold = ideal baseline (resampled to the same length). Similarity ' +
            similarity.toFixed(0) + '% (MAE vs mean ideal magnitude).';
    }

    const ctx = document.getElementById('ideal-compare-chart');
    if (!ctx || !window.Chart) return;

    destroyChart();

    const maxPts = 220;
    const step = Math.max(1, Math.floor(L / maxPts));
    const labels = [];
    const aData = [];
    const iData = [];
    const progressLabel = mode === 'session' ? 'Through session' : 'Through stroke';
    for (let i = 0; i < L; i += step) {
        labels.push(Math.round((L > 1 ? i / (L - 1) : 0) * 100));
        aData.push(actualRs[i]);
        iData.push(resampled[i]);
    }

    const chartCommon = {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: 'index', intersect: false },
        plugins: {
            legend: {
                position: 'bottom',
                labels: { color: '#a1a1b5', font: { size: 11 }, boxWidth: 12 }
            },
            tooltip: {
                callbacks: {
                    title: (items) => (items && items.length ? progressLabel + ': ' + items[0].label + '%' : ''),
                    label: (item) => {
                        const v = item.parsed.y != null ? item.parsed.y : item.raw;
                        return (item.dataset.label || '') + ': ' + (typeof v === 'number' ? v.toFixed(2) : v) + ' m/s²';
                    }
                }
            }
        },
        scales: {
            x: {
                display: true,
                title: {
                    display: true,
                    text: xAxisTitle,
                    color: '#7a7a90',
                    font: { size: 11 }
                },
                ticks: { color: '#6b6b80', maxTicksLimit: 10 },
                grid: { color: 'rgba(255,255,255,0.06)' }
            },
            y: {
                display: true,
                title: {
                    display: true,
                    text: '|Linear acceleration| (m/s²)',
                    color: '#7a7a90',
                    font: { size: 11 }
                },
                ticks: { color: '#6b6b80' },
                grid: { color: '#1a1a25' }
            }
        }
    };

    window._idealChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels,
            datasets: [
                {
                    label: actualLabel,
                    data: aData,
                    borderColor: '#f4f4f8',
                    borderWidth: 1.8,
                    pointRadius: 0,
                    tension: 0.18,
                    fill: false
                },
                {
                    label: 'Ideal baseline (resampled)',
                    data: iData,
                    borderColor: '#E8C878',
                    borderWidth: 2.6,
                    borderDash: [7, 5],
                    pointRadius: 0,
                    tension: 0.18,
                    fill: false,
                    order: 1
                }
            ]
        },
        options: chartCommon
    });

    gfComputeVsIdealMetrics();
}
