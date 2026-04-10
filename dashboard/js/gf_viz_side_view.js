/**
 * GoldenForm  --  2D side canvas: LIA path per stroke, phase-colored segments, pitch / gyro overlay.
 */
function resizeSideViewCanvas() {
    const canvas = document.getElementById('canvas-side-view');
    const wrap = canvas && canvas.closest('.viz-side-canvas-wrap');
    if (!canvas || !wrap) return;
    const w = Math.max(280, Math.min(960, wrap.clientWidth || 880));
    const h = Math.round(Math.max(200, w * 0.38));
    if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
    }
}

function getLiaPositionSample(i) {
    const d = processedData[i];
    if (!d) return { px: 0, py: 0, pz: 0 };
    /* Prefer StrokeProcessor positions after per-stroke smoothing (same as 3D trail).
     * Raw JSON positions are noisier; IMU+ world frame also drifts in yaw  --  smoothing helps visuals. */
    const s = (typeof positionScale !== 'undefined' && positionScale > 0) ? positionScale : 3;
    if (typeof positionStreamPositions !== 'undefined' && positionStreamPositions &&
        positionStreamPositions.length === processedData.length && positionStreamPositions[i]) {
        const v = positionStreamPositions[i];
        return {
            px: v.x / s,
            py: v.y / s,
            pz: v.z / s
        };
    }
    const pos = d.position || {};
    return {
        px: Number(pos.px) || 0,
        py: Number(pos.py) || 0,
        pz: Number(pos.pz) || 0
    };
}

function strokeBoundsForIndex(idx) {
    refreshStrokeFieldMode();
    if (!processedData.length) return { start: 0, end: 0, strokeNum: 0 };
    const d0 = processedData[idx] || {};
    const sk = getStreamKey(d0);
    const sc = strokeNumAt(d0);
    if (sc <= 0) {
        return { start: 0, end: Math.max(0, processedData.length - 1), strokeNum: 0 };
    }
    let start = idx;
    for (let i = idx - 1; i >= 0; i--) {
        const di = processedData[i];
        if (strokeNumAt(di) === sc && getStreamKey(di) === sk) start = i;
        else break;
    }
    let end = idx;
    for (let j = idx + 1; j < processedData.length; j++) {
        const dj = processedData[j];
        if (strokeNumAt(dj) === sc && getStreamKey(dj) === sk) end = j;
        else break;
    }
    return { start, end, strokeNum: sc };
}

function pitchDegFromQuaternion(q) {
    if (!q) return 0;
    const qw = q.qw != null ? q.qw : 1;
    const qx = q.qx || 0;
    const qy = q.qy || 0;
    const qz = q.qz || 0;
    const sinp = 2 * (qw * qy - qz * qx);
    return Math.asin(Math.max(-1, Math.min(1, sinp))) * 180 / Math.PI;
}

function gyroSagittalDeg(g) {
    if (!g) return 0;
    const gx = g.gx || 0;
    const gy = g.gy || 0;
    const gz = g.gz || 0;
    return Math.atan2(gy, Math.sqrt(gx * gx + gz * gz)) * 180 / Math.PI;
}

function clearSideViewCanvas() {
    const canvas = document.getElementById('canvas-side-view');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#0b1422';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawSideViewViz(idx) {
    const canvas = document.getElementById('canvas-side-view');
    if (!canvas || !processedData.length) return;
    if (typeof integratePositions === 'function' &&
        (!positionStreamPositions || positionStreamPositions.length !== processedData.length)) {
        integratePositions();
    }
    resizeSideViewCanvas();
    const ctx = canvas.getContext('2d');
    const W = canvas.width;
    const H = canvas.height;
    ctx.fillStyle = '#0b1422';
    ctx.fillRect(0, 0, W, H);

    const b = strokeBoundsForIndex(idx);
    if (b.strokeNum <= 0) {
        ctx.fillStyle = '#888';
        ctx.font = '13px system-ui, sans-serif';
        ctx.fillText('Move to a stroke (stroke # ≥ 1) to see LIA path reset per stroke.', 16, H / 2);
        return;
    }

    const p0 = getLiaPositionSample(b.start);
    const multi = typeof hasMultipleDeviceStreams === 'function' && hasMultipleDeviceStreams();
    const skCur = getStreamKey(processedData[idx]);
    const t0 = processedData[b.start].timestamp;
    const t1 = processedData[b.end].timestamp;

    const allPts = [];
    for (let i = b.start; i <= b.end; i++) {
        const p = getLiaPositionSample(i);
        allPts.push({
            x: p.pz - p0.pz,
            y: p.py - p0.py,
            i
        });
    }

    const otherPathPts = [];
    if (multi && t0 != null && t1 != null) {
        for (let i = 0; i < processedData.length; i++) {
            if (getStreamKey(processedData[i]) === skCur) continue;
            const ts = processedData[i].timestamp;
            if (ts == null || ts < t0 || ts > t1) continue;
            const p = getLiaPositionSample(i);
            otherPathPts.push({
                x: p.pz - p0.pz,
                y: p.py - p0.py,
                i
            });
        }
        otherPathPts.sort((a, b) =>
            (processedData[a.i].timestamp || 0) - (processedData[b.i].timestamp || 0));
        for (const p of otherPathPts) {
            allPts.push(p);
        }
    }

    const upto = Math.min(idx, b.end);
    const pathPts = [];
    for (let i = b.start; i <= upto; i++) {
        const p = getLiaPositionSample(i);
        pathPts.push({ x: p.pz - p0.pz, y: p.py - p0.py, i });
    }

    let minX = 0, maxX = 0.01, minY = 0, maxY = 0.01;
    for (const p of allPts) {
        minX = Math.min(minX, p.x);
        maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y);
        maxY = Math.max(maxY, p.y);
    }
    const pad = 44;
    const rangeX = Math.max(maxX - minX, 0.04);
    const rangeY = Math.max(maxY - minY, 0.04);
    const mapX = (x) => pad + (x - minX) / rangeX * (W - 2 * pad);
    const mapY = (y) => H - pad - (y - minY) / rangeY * (H - 2 * pad);

    ctx.strokeStyle = 'rgba(120, 170, 255, 0.35)';
    ctx.lineWidth = 1;
    const wy = mapY(0);
    if (wy > pad && wy < H - pad) {
        ctx.beginPath();
        ctx.moveTo(pad, wy);
        ctx.lineTo(W - pad, wy);
        ctx.stroke();
        ctx.fillStyle = 'rgba(120, 170, 255, 0.7)';
        ctx.font = '10px system-ui';
        ctx.fillText('water / ref', pad, wy - 4);
    }

    function rgbCss(rgb) {
        return 'rgba(' + Math.round(rgb[0] * 255) + ',' + Math.round(rgb[1] * 255) + ',' + Math.round(rgb[2] * 255) + ',0.94)';
    }

    if (multi && otherPathPts.length > 1) {
        const skO = getStreamKey(processedData[otherPathPts[0].i]);
        ctx.strokeStyle = rgbCss(streamColorRgbForKey(skO));
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.globalAlpha = 0.75;
        ctx.beginPath();
        ctx.moveTo(mapX(otherPathPts[0].x), mapY(otherPathPts[0].y));
        for (let k = 1; k < otherPathPts.length; k++) {
            ctx.lineTo(mapX(otherPathPts[k].x), mapY(otherPathPts[k].y));
        }
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
    }

    for (let k = 1; k < pathPts.length; k++) {
        const d0 = processedData[pathPts[k - 1].i];
        const ph = (d0 && (d0.stroke_phase || d0.phase)) || 'idle';
        if (multi) {
            const rgb = streamColorRgbForKey(getStreamKey(d0));
            const cPhase = PHASE_COLOR_RGB[ph] || PHASE_COLOR_RGB.idle;
            ctx.strokeStyle = rgbCss([
                rgb[0] * 0.55 + cPhase[0] * 0.45,
                rgb[1] * 0.55 + cPhase[1] * 0.45,
                rgb[2] * 0.55 + cPhase[2] * 0.45
            ]);
        } else {
            ctx.strokeStyle = PHASE_COLOR_SIDE[ph] || PHASE_COLOR_SIDE.idle;
        }
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(mapX(pathPts[k - 1].x), mapY(pathPts[k - 1].y));
        ctx.lineTo(mapX(pathPts[k].x), mapY(pathPts[k].y));
        ctx.stroke();
    }

    const d = processedData[idx];
    const phase = (d && (d.stroke_phase || d.phase)) || 'idle';
    ctx.fillStyle = '#e8e8f0';
    ctx.font = '12px system-ui, sans-serif';
    const cap = multi
        ? ('Stroke #' + b.strokeNum + ' · ' + streamLabelShort(d) + ' (solid) · other band (dashed) · colors = band + phase')
        : ('Stroke #' + b.strokeNum + ' · phase: ' + phase + ' · path colors = phase (catch/pull/recovery/glide)');
    ctx.fillText(cap, 12, 18);

    ctx.fillStyle = '#9ca3af';
    ctx.font = '10px system-ui';
    ctx.fillText('Forward Δpz →  ·  ↑py = vertical (m), origin = stroke start · path uses smoothed trail (matches 3D)', 12, 32);

    if (pathPts.length === 0) return;
    const last = pathPts[pathPts.length - 1];
    const mx = mapX(last.x);
    const my = mapY(last.y);
    ctx.fillStyle = '#38bdf8';
    ctx.beginPath();
    ctx.arc(mx, my, 6, 0, Math.PI * 2);
    ctx.fill();

    let cueFlash = !!(d && d.haptic_fired);
    if (!cueFlash && typeof hapticFlashUntil !== 'undefined' && hapticFlashUntil > Date.now()) {
        cueFlash = true;
    }
    if (!cueFlash) {
        for (let hi = Math.max(b.start, idx - 12); hi <= idx; hi++) {
            const dd = processedData[hi];
            if (!dd || !dd.haptic_fired) continue;
            if (hi > b.start && processedData[hi - 1].haptic_fired) continue;
            cueFlash = true;
            break;
        }
    }
    if (cueFlash) {
        ctx.strokeStyle = 'rgba(232, 200, 120, 0.95)';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.arc(mx, my, 14, 0, Math.PI * 2);
        ctx.stroke();
        ctx.fillStyle = 'rgba(232, 200, 120, 0.22)';
        ctx.beginPath();
        ctx.arc(mx, my, 18, 0, Math.PI * 2);
        ctx.fill();
    }

    const q = d.quaternion || {};
    const pitch = pitchDegFromQuaternion(q);
    const gdeg = gyroSagittalDeg(d.angular_velocity);
    ctx.strokeStyle = '#4ade80';
    ctx.lineWidth = 3;
    const len = Math.min(48, 24 + W * 0.02);
    const ang = pitch * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(mx, my);
    ctx.lineTo(mx + len * Math.sin(ang), my - len * Math.cos(ang));
    ctx.stroke();

    ctx.fillStyle = '#d1d5db';
    ctx.font = '11px system-ui, sans-serif';
    ctx.font = '10px system-ui, sans-serif';
    ctx.fillText(
        'AoA: pitch (quat) ' + pitch.toFixed(0) + '° · gyro sagittal ' + gdeg.toFixed(0) + '°  --  IMU+ has no mag; yaw drifts; path is indicative',
        12,
        H - 12
    );
}
