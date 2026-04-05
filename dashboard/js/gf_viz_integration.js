/**
 * GoldenForm — LIA / processor position integration and stroke-segmented trail prep.
 *
 * Authoritative path: use `position` from Python `StrokeProcessor` (wifi_session_processor /
 * simple_imu_visualizer) — Kalman-smoothed LIA → world frame → integrate only while
 * `stroke_integrating` (see `tracking_active` in JSON). This matches the UART / replay tuning
 * described at the reference tree:
 * https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/0401fbde9e7a69887208db1df3e5feff27011ba6
 *
 * Fallback (no per-sample `position`): same order — 1D Kalman per axis (same coeffs as
 * SimpleKalmanFilter in simple_imu_visualizer.py), then world-frame rotation, then integration
 * gated by `tracking_active` when present.
 */
function gfKalman1D(Q, R, P0, x0) {
    let p = P0;
    let x = x0;
    return function (measurement) {
        p = p + Q;
        const k = p / (p + R);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
        return x;
    };
}

function buildRawIntegratedPositions() {
    rawIntegratedPositions = [];
    if (!processedData.length) return;
    refreshStrokeFieldMode();

    const hasProcessorPos = processedData.length > 0 && processedData.every(d =>
        d.position && typeof d.position.px === 'number'
    );
    if (hasProcessorPos) {
        let prevStroke = strokeNumAt(processedData[0]);
        for (let i = 0; i < processedData.length; i++) {
            const d = processedData[i];
            const sc = strokeNumAt(d);
            if (sc > prevStroke) prevStroke = sc;
            const p = d.position;
            rawIntegratedPositions.push(new THREE.Vector3(
                (p.px || 0) * positionScale,
                (p.py || 0) * positionScale,
                (p.pz || 0) * positionScale
            ));
        }
        return;
    }

    const hasTracking = processedData.some(d => d.tracking_active !== undefined);
    const kx = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const ky = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const kz = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    let vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
    let prevStroke = strokeNumAt(processedData[0]);
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const sc = strokeNumAt(d);
        if (sc > prevStroke) {
            vx = vy = vz = 0;
            px = py = pz = 0;
            prevStroke = sc;
        }
        let dt = 0.02;
        if (i > 0 && d.timestamp != null && processedData[i - 1].timestamp != null) {
            dt = Math.max(0.001, Math.min(0.1, (d.timestamp - processedData[i - 1].timestamp) / 1000));
        }
        /* Match StrokeProcessor: no integration when not in pull-through (no idle drift). */
        if (hasTracking && d.tracking_active === false) {
            rawIntegratedPositions.push(new THREE.Vector3(px * positionScale, py * positionScale, pz * positionScale));
            vx = vy = vz = 0;
            continue;
        }
        let ax = 0, ay = 0, az = 0;
        if (d.lia) { ax = d.lia.x || 0; ay = d.lia.y || 0; az = d.lia.z || 0; }
        else if (d.acceleration) { ax = d.acceleration.ax || 0; ay = d.acceleration.ay || 0; az = d.acceleration.az || 0; }
        const sx = kx(ax);
        const sy = ky(ay);
        const sz = kz(az);
        const wA = new THREE.Vector3(sx, sy, sz).applyQuaternion(nq(d.quaternion));
        vx += wA.x * dt; vy += wA.y * dt; vz += wA.z * dt;
        vx *= 0.97; vy *= 0.97; vz *= 0.97;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        rawIntegratedPositions.push(new THREE.Vector3(px * positionScale, py * positionScale, pz * positionScale));
    }
}

function integratePositions() {
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    if (!processedData.length) return;
    buildRawIntegratedPositions();
    if (!playbackStrokeSegments.length) buildPlaybackStrokeSegments();
    let filled = fillMissingRawPositions(rawIntegratedPositions);
    filled = smoothRawPathPerStroke(filled);
    positionStreamPositions = filled;
    integratedPositions = filled;
}

function getSegStart(upToIndex) {
    if (!processedData.length || upToIndex < 0) return 0;
    refreshStrokeFieldMode();
    const sc = strokeNumAt(processedData[upToIndex] || {});
    for (let i = upToIndex; i >= 0; i--) {
        if (strokeNumAt(processedData[i] || {}) < sc) return i + 1;
    }
    return 0;
}

function smoothTrailPoints(points, windowSize) {
    if (!points.length || windowSize < 2) return points;
    const half = Math.floor(windowSize / 2);
    const out = [];
    for (let i = 0; i < points.length; i++) {
        let x = 0, y = 0, z = 0, n = 0;
        for (let j = Math.max(0, i - half); j <= Math.min(points.length - 1, i + half); j++) {
            x += points[j].x; y += points[j].y; z += points[j].z;
            n++;
        }
        out.push(new THREE.Vector3(x / n, y / n, z / n));
    }
    return out;
}

function fillMissingRawPositions(points) {
    const out = points.map(p => (p && p.clone) ? p.clone() : new THREE.Vector3());
    let lastValid = -1;
    for (let i = 0; i < out.length; i++) {
        const p = out[i];
        if (p && isFinite(p.x) && isFinite(p.y) && isFinite(p.z)) {
            lastValid = i;
        } else if (lastValid >= 0) {
            p.copy(out[lastValid]);
        }
    }
    for (let i = out.length - 1; i >= 0; i--) {
        const p = out[i];
        if (p && isFinite(p.x) && isFinite(p.y) && isFinite(p.z)) break;
        let j = i + 1;
        while (j < out.length && out[j] && (!isFinite(out[j].x))) j++;
        if (j < out.length) p.copy(out[j]);
    }
    return out;
}

function smoothRawPathPerStroke(points) {
    if (!points.length) return points;
    const w = Math.min(11, Math.max(3, Math.floor(points.length / 8) * 2 + 1));
    if (!playbackStrokeSegments.length) {
        return smoothTrailPoints(points, w);
    }
    const out = points.map(p => p.clone());
    for (const seg of playbackStrokeSegments) {
        const { startIdx, endIdx } = seg;
        if (endIdx < startIdx) continue;
        const slice = [];
        for (let i = startIdx; i <= endIdx; i++) slice.push(out[i]);
        const sm = smoothTrailPoints(slice, Math.min(w, slice.length));
        for (let k = 0; k < sm.length; k++) {
            out[startIdx + k].copy(sm[k]);
        }
    }
    return out;
}

function currentAnimationPositions() {
    return positionStreamPositions;
}

function resampleTrailFreestyle(rawPts, colors) {
    const pre = smoothTrailPoints(rawPts, TRAIL_SMOOTH_WINDOW);
    if (pre.length < 4 || typeof THREE === 'undefined' || !THREE.CatmullRomCurve3) {
        return { points: pre, colors: colors.slice(0, pre.length * 3) };
    }
    try {
        const curve = new THREE.CatmullRomCurve3(pre.map(p => p.clone()));
        const n = Math.min(420, Math.max(pre.length * 5, 48));
        const sampled = curve.getPoints(n);
        const newColors = [];
        for (let i = 0; i < sampled.length; i++) {
            const t = sampled.length > 1 ? i / (sampled.length - 1) : 0;
            const origIdx = t * (pre.length - 1);
            const i0 = Math.floor(origIdx);
            const i1 = Math.min(pre.length - 1, i0 + 1);
            const f = origIdx - i0;
            const o0 = i0 * 3, o1 = i1 * 3;
            newColors.push(
                colors[o0] * (1 - f) + colors[o1] * f,
                colors[o0 + 1] * (1 - f) + colors[o1 + 1] * f,
                colors[o0 + 2] * (1 - f) + colors[o1 + 2] * f
            );
        }
        return { points: sampled, colors: newColors };
    } catch (e) {
        return { points: pre, colors: colors.slice(0, pre.length * 3) };
    }
}

function buildPlaybackStrokeSegments() {
    playbackStrokeSegments = [];
    refreshStrokeFieldMode();
    let prevSk = null;
    let prevSn = -1;
    let start = 0;
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const sk = getStreamKey(d);
        const c = strokeNumAt(d);
        if (c <= 0) continue;
        const isNewStroke = (prevSn < 0) || (sk !== prevSk) || (sk === prevSk && c > prevSn);
        if (isNewStroke) {
            if (prevSn > 0 && start < i) {
                playbackStrokeSegments.push({
                    startIdx: start,
                    endIdx: i - 1,
                    strokeNum: prevSn,
                    streamKey: prevSk
                });
            }
            start = i;
            prevSk = sk;
            prevSn = c;
        }
    }
    if (prevSn > 0 && start < processedData.length) {
        playbackStrokeSegments.push({
            startIdx: start,
            endIdx: processedData.length - 1,
            strokeNum: prevSn,
            streamKey: prevSk
        });
    }
}
