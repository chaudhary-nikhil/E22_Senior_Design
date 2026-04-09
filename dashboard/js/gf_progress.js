/**
 * GoldenForm — Progress / Insights charts from stored sessions.
 */
// ── PROGRESS / INSIGHTS ──
function buildLocalProgressFromSessions() {
    const rows = [];
    for (let i = 0; i < savedSessions.length; i++) {
        const s = savedSessions[i];
        const m = s.metrics;
        if (!m) continue;
        rows.push({
            form_score: (typeof m.form_score === 'number' && isFinite(m.form_score)) ? m.form_score : computeFormScore(m),
            consistency: m.consistency || 0,
            stroke_rate: m.stroke_rate || 0,
            avg_deviation: m.avg_deviation || 0,
            avg_entry_angle: m.avg_entry_angle || 0,
            stroke_count: m.stroke_count || 0
        });
    }
    return rows;
}

async function loadProgress() {
    const res = await apiGet('/api/progress');
    let data = ((res && res.progress) || []).slice().reverse();
    if (!data.length && hasSavedSessions()) {
        data = buildLocalProgressFromSessions();
    }
    // Sanitize so charts never explode (NaN/undefined) and keep points in-bounds.
    data = (data || []).map((d) => {
        const num = (x, def = 0) => {
            const v = Number(x);
            return Number.isFinite(v) ? v : def;
        };
        return {
            ...d,
            form_score: Math.max(0, Math.min(10, num(d.form_score, 0))),
            consistency: Math.max(0, Math.min(100, num(d.consistency, 0))),
            stroke_rate: Math.max(0, num(d.stroke_rate, 0)),
        };
    });
    if (!data.length) {
        if (window._progressChart) {
            try { window._progressChart.destroy(); } catch (e) { /* ignore */ }
            window._progressChart = null;
        }
        setText('insights-empty', 'Sync a swim session. The progress chart fills from the server or saved sessions on this device.');
        return;
    }
    setText('insights-empty', data.length && !((res && res.progress) || []).length
        ? 'Trends from this browser; server sync merges history when available.'
        : '');
    const ctx = document.getElementById('progress-chart');
    if (!ctx || !window.Chart) return;
    if (window._progressChart) window._progressChart.destroy();
    window._progressChart = new Chart(ctx, {
        type: 'line', data: {
            labels: data.map((_, i) => 'Session ' + (i + 1)),
            datasets: [
                { label: 'Form Score', data: data.map(d => d.form_score), borderColor: '#C5A55A', backgroundColor: 'rgba(197,165,90,0.1)', fill: true, tension: 0.4, pointRadius: 5, pointBackgroundColor: '#C5A55A' },
                { label: 'Consistency %', data: data.map(d => d.consistency), borderColor: '#22c55e', tension: 0.35, pointRadius: 4, yAxisID: 'y2' },
                { label: 'Stroke Rate', data: data.map(d => d.stroke_rate), borderColor: '#3b82f6', tension: 0.4, pointRadius: 3, yAxisID: 'y1' }
            ]
        }, options: {
            responsive: true, maintainAspectRatio: false,
            plugins: { legend: { labels: { color: '#aaa', font: { size: 12 } } } },
            scales: {
                x: { ticks: { color: '#666' }, grid: { color: '#1a1a25' } },
                y: { ticks: { color: '#666' }, grid: { color: '#1a1a25' }, min: 0, max: 10, title: { display: true, text: 'Score', color: '#666' } },
                y1: { position: 'right', ticks: { color: '#666' }, grid: { display: false }, min: 0, title: { display: true, text: 'Strokes/min', color: '#666' } },
                y2: { position: 'left', ticks: { color: '#666' }, grid: { display: false }, min: 0, max: 100, title: { display: true, text: 'Consistency %', color: '#666' }, display: false }
            }
        }
    });
}

