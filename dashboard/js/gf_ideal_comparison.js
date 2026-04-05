/**
 * GoldenForm — Ideal stroke comparison (LIA vs session).
 */
// ── IDEAL STROKE COMPARISON ──
async function buildIdealComparison() {
    if (!idealStrokeData || !idealStrokeData.length || !processedData.length) {
        const el = document.getElementById('ideal-comparison-content');
        if (el) el.innerHTML = '<p style="color:var(--text3);text-align:center;padding:12px;">No ideal stroke saved. Set one in Settings.</p><div style="height:180px;margin-top:8px;"><canvas id="ideal-compare-chart"></canvas></div>';
        if (window._idealChart) { window._idealChart.destroy(); window._idealChart = null; }
        return;
    }
    const el = document.getElementById('ideal-comparison-content');
    if (!el) return;

    // Compute similarity between ideal and actual LIA profiles
    const actualLIA = processedData.map(d => Math.sqrt(
        (d.acceleration?.ax || 0) ** 2 + (d.acceleration?.ay || 0) ** 2 + (d.acceleration?.az || 0) ** 2
    ));
    const idealLIA = idealStrokeData.map(s => Math.sqrt(
        (s.lia_x || 0) ** 2 + (s.lia_y || 0) ** 2 + (s.lia_z || 0) ** 2
    ));

    // Resample ideal to match actual length for comparison
    const resampled = [];
    for (let i = 0; i < actualLIA.length; i++) {
        const t = idealLIA.length > 1 ? i / (actualLIA.length - 1) * (idealLIA.length - 1) : 0;
        const idx = Math.floor(t);
        const frac = t - idx;
        if (idx >= idealLIA.length - 1) resampled.push(idealLIA[idealLIA.length - 1]);
        else resampled.push(idealLIA[idx] * (1 - frac) + idealLIA[idx + 1] * frac);
    }

    // Compute correlation
    let sumDiff = 0, sumIdeal = 0;
    const len = Math.min(actualLIA.length, resampled.length);
    for (let i = 0; i < len; i++) {
        sumDiff += Math.abs(actualLIA[i] - resampled[i]);
        sumIdeal += resampled[i] || 1;
    }
    const similarity = Math.max(0, Math.min(100, (1 - sumDiff / (sumIdeal || 1)) * 100));
    setText('ideal-similarity', similarity.toFixed(0) + '%');

    // Draw comparison chart
    const ctx = document.getElementById('ideal-compare-chart');
    if (ctx && window.Chart) {
        if (window._idealChart) window._idealChart.destroy();
        const step = Math.max(1, Math.floor(len / 200));
        const labels = [], aData = [], iData = [];
        for (let i = 0; i < len; i += step) {
            labels.push(i);
            aData.push(actualLIA[i]);
            iData.push(resampled[i]);
        }
        window._idealChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels,
                datasets: [
                    { label: 'Your Stroke', data: aData, borderColor: '#fff', borderWidth: 1.5, pointRadius: 0, tension: 0.2 },
                    { label: 'Ideal', data: iData, borderColor: '#C5A55A', borderWidth: 2, borderDash: [5, 3], pointRadius: 0, tension: 0.2 }
                ]
            },
            options: {
                responsive: true, maintainAspectRatio: false, animation: false,
                plugins: { legend: { labels: { color: '#999', font: { size: 11 } } } },
                scales: { x: { display: false }, y: { ticks: { color: '#555' }, grid: { color: '#1a1a25' } } }
            }
        });
    }
}

