/**
 * GoldenForm — Session IMU charts (accel, gyro, magnetometer).
 */

const _chartStrokePlugin = {
    id: 'gfStrokeLines',
    afterDraw(chart) {
        const meta = chart.options?.plugins?.gfStrokeLines;
        if (!meta || !meta.indices || !meta.indices.length) return;
        const ctx = chart.ctx;
        const xScale = chart.scales.x;
        if (!xScale) return;
        ctx.save();
        ctx.strokeStyle = 'rgba(250,204,21,0.45)';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        const area = chart.chartArea;
        for (const localIdx of meta.indices) {
            const px = xScale.getPixelForValue(localIdx);
            if (px >= area.left && px <= area.right) {
                ctx.beginPath();
                ctx.moveTo(px, area.top);
                ctx.lineTo(px, area.bottom);
                ctx.stroke();
            }
        }
        ctx.restore();
    }
};
if (window.Chart) Chart.register(_chartStrokePlugin);

function initCharts() {
    if (!window.Chart) return;
    const accelCtx = document.getElementById('accelChart');
    const gyroCtx = document.getElementById('gyroChart');
    const magCtx = document.getElementById('magChart');
    if (!accelCtx || !gyroCtx) return;
    if (accelChart) accelChart.destroy();
    if (gyroChart) gyroChart.destroy();
    if (magChart) magChart.destroy();
    magChart = null;
    const chartOpts = {
        responsive: true, maintainAspectRatio: false, animation: false,
        elements: { point: { radius: 0 }, line: { borderWidth: 1.8, tension: 0.15 } },
        plugins: {
            legend: { labels: { color: '#94a3b8', font: { size: 10 }, boxWidth: 12, padding: 8 } },
            gfStrokeLines: { indices: [] },
        },
        scales: {
            x: { display: false },
            y: {
                ticks: { color: '#64748b', font: { size: 10 }, maxTicksLimit: 6 },
                grid: { color: 'rgba(51,65,85,0.35)' },
            },
        },
    };
    accelChart = new Chart(accelCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'Ax', data: [], borderColor: '#ef4444', backgroundColor: 'rgba(239,68,68,0.08)', fill: true },
                { label: 'Ay', data: [], borderColor: '#22c55e', backgroundColor: 'rgba(34,197,94,0.08)', fill: true },
                { label: 'Az', data: [], borderColor: '#3b82f6', backgroundColor: 'rgba(59,130,246,0.08)', fill: true },
            ]
        }, options: { ...chartOpts, plugins: { ...chartOpts.plugins, title: { display: true, text: 'Linear Acceleration (m/s\u00B2)', color: '#94a3b8', font: { size: 11, weight: 'normal' } } } }
    });
    gyroChart = new Chart(gyroCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'Gx', data: [], borderColor: '#f97316', backgroundColor: 'rgba(249,115,22,0.08)', fill: true },
                { label: 'Gy', data: [], borderColor: '#a855f7', backgroundColor: 'rgba(168,85,247,0.08)', fill: true },
                { label: 'Gz', data: [], borderColor: '#06b6d4', backgroundColor: 'rgba(6,182,212,0.08)', fill: true },
            ]
        }, options: { ...chartOpts, plugins: { ...chartOpts.plugins, title: { display: true, text: 'Angular Velocity (rad/s)', color: '#94a3b8', font: { size: 11, weight: 'normal' } } } }
    });
    /* Product UI: magnetometer is not a useful live graph for users (often constant/flat). */
    try {
        if (magCtx) {
            const card = magCtx.closest?.('.card') || magCtx.parentElement;
            if (card) card.style.display = 'none';
        }
    } catch (e) { /* ignore */ }
}

function _strokeIndicesInWindow(start, end) {
    const out = [];
    if (!strokeBoundaries || !strokeBoundaries.length) return out;
    for (const sb of strokeBoundaries) {
        if (sb.index >= start && sb.index <= end) out.push(sb.index - start);
    }
    return out;
}

function updateCharts(idx) {
    if (!accelChart || !gyroChart) return;
    const windowSize = 200;
    const start = Math.max(0, idx - windowSize);
    const slice = processedData.slice(start, idx + 1);
    const labels = slice.map((_, i) => i);
    const mag = (d) => d.magnetometer || {};
    const strokeIdx = _strokeIndicesInWindow(start, idx);

    accelChart.data.labels = labels;
    accelChart.data.datasets[0].data = slice.map(d => d.acceleration?.ax || 0);
    accelChart.data.datasets[1].data = slice.map(d => d.acceleration?.ay || 0);
    accelChart.data.datasets[2].data = slice.map(d => d.acceleration?.az || 0);
    accelChart.options.plugins.gfStrokeLines = { indices: strokeIdx };
    accelChart.update('none');

    gyroChart.data.labels = labels;
    gyroChart.data.datasets[0].data = slice.map(d => d.angular_velocity?.gx || 0);
    gyroChart.data.datasets[1].data = slice.map(d => d.angular_velocity?.gy || 0);
    gyroChart.data.datasets[2].data = slice.map(d => d.angular_velocity?.gz || 0);
    gyroChart.options.plugins.gfStrokeLines = { indices: strokeIdx };
    gyroChart.update('none');

    // Magnetometer chart intentionally omitted.
}
