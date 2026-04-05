/**
 * GoldenForm — Session IMU charts (accel, gyro, magnetometer).
 */
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
        elements: { point: { radius: 0 }, line: { borderWidth: 1.5 } },
        plugins: { legend: { labels: { color: '#888', font: { size: 10 } } } },
        scales: { x: { display: false }, y: { ticks: { color: '#555', font: { size: 10 } }, grid: { color: '#1a1a25' } } }
    };
    accelChart = new Chart(accelCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'X', data: [], borderColor: '#ef4444' },
                { label: 'Y', data: [], borderColor: '#22c55e' },
                { label: 'Z', data: [], borderColor: '#3b82f6' }
            ]
        }, options: chartOpts
    });
    gyroChart = new Chart(gyroCtx, {
        type: 'line', data: {
            labels: [], datasets: [
                { label: 'X', data: [], borderColor: '#f97316' },
                { label: 'Y', data: [], borderColor: '#a855f7' },
                { label: 'Z', data: [], borderColor: '#06b6d4' }
            ]
        }, options: chartOpts
    });
    if (magCtx) {
        magChart = new Chart(magCtx, {
            type: 'line', data: {
                labels: [], datasets: [
                    { label: 'X', data: [], borderColor: '#f87171' },
                    { label: 'Y', data: [], borderColor: '#38bdf8' },
                    { label: 'Z', data: [], borderColor: '#a3e635' }
                ]
            }, options: chartOpts
        });
    }
}

function updateCharts(idx) {
    if (!accelChart || !gyroChart) return;
    const windowSize = 200;
    const start = Math.max(0, idx - windowSize);
    const slice = processedData.slice(start, idx + 1);
    const labels = slice.map((_, i) => i);
    const mag = (d) => d.magnetometer || {};
    accelChart.data.labels = labels;
    accelChart.data.datasets[0].data = slice.map(d => d.acceleration?.ax || 0);
    accelChart.data.datasets[1].data = slice.map(d => d.acceleration?.ay || 0);
    accelChart.data.datasets[2].data = slice.map(d => d.acceleration?.az || 0);
    accelChart.update('none');
    gyroChart.data.labels = labels;
    gyroChart.data.datasets[0].data = slice.map(d => d.angular_velocity?.gx || 0);
    gyroChart.data.datasets[1].data = slice.map(d => d.angular_velocity?.gy || 0);
    gyroChart.data.datasets[2].data = slice.map(d => d.angular_velocity?.gz || 0);
    gyroChart.update('none');
    if (magChart) {
        magChart.data.labels = labels;
        magChart.data.datasets[0].data = slice.map(d => mag(d).mx || 0);
        magChart.data.datasets[1].data = slice.map(d => mag(d).my || 0);
        magChart.data.datasets[2].data = slice.map(d => mag(d).mz || 0);
        magChart.update('none');
    }
}
