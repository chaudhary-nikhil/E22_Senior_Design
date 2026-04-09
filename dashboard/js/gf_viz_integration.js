/**
 * GoldenForm — LIA / processor position integration and stroke-segmented trail prep.
 *
 * Authoritative path: use `position` from Python `StrokeProcessor` (wifi_session_processor /
 * simple_imu_visualizer) — Kalman-smoothed LIA → world frame → integrate while `tracking_active`.
 * Live Wi‑Fi uses a 0401fbde-style motion gate (|gy| + |LIA|); batch replay uses pull-through /
 * pos_tracking. Reference:
 * https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/0401fbde9e7a69887208db1df3e5feff27011ba6
 *
 * Fallback (no per-sample `position`): same order — 1D Kalman per axis (same coeffs as
 * SimpleKalmanFilter in simple_imu_visualizer.py), then world-frame rotation, then integration
 * gated by `tracking_active` when present.
 */
/**
 * Map processor world position (px,py,pz) to Three.js scene coordinates.
 * Scene: Y up, pool grid in XZ. StrokeProcessor integrates in a world frame where the pull
 * often grows py; negating Y matches 0401fbde-style “down into the water” vs screen-up intuition.
 */
function vizMapProcessorPosition(px, py, pz) {
    const s = typeof positionScale !== 'undefined' ? positionScale : 1;
    return new THREE.Vector3(
        (Number(px) || 0) * s,
        (Number(py) || 0) * s,
        (Number(pz) || 0) * s
    );
}

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

/** Phase-adaptive integration (0401fbde / StrokeProcessor-style) — replay path without per-sample `tracking_active`. */
const GF_PHASE_PARAMS = {
    catch: { deadzone: 0.2, decay: 0.98 },
    pull: { deadzone: 0.2, decay: 0.98 },
    exit: { deadzone: 0.25, decay: 0.96 },
    recovery: { deadzone: 0.5, decay: 0.92 },
    glide: { deadzone: 0.4, decay: 0.90 },
    idle: { deadzone: 0.3, decay: 0.95 }
};

function gfNormalizePhaseName(s) {
    if (!s) return '';
    const t = String(s).toLowerCase().trim();
    if (GF_PHASE_PARAMS[t]) return t;
    return '';
}

/**
 * Prefer `stroke_phase` / `phase` on the sample (from Python batch / processor).
 * Else infer from time-in-stroke + |LIA| + ‖ω‖ (same spirit as simple_imu_visualizer timeline).
 */
function gfInferReplayPhase(d, dtStrokeMs, liaBodyMag, gyroMag) {
    const fromField = gfNormalizePhaseName(d && (d.stroke_phase || d.phase));
    if (fromField) return fromField;
    const t = dtStrokeMs;
    const CATCH_MS = 150;
    const PULL_MS = 500;
    const EXIT_MS = 700;
    if (t < CATCH_MS) return 'catch';
    if (t < PULL_MS) return 'pull';
    if (t < EXIT_MS) return 'exit';
    if (t >= EXIT_MS && liaBodyMag < 1.5 && gyroMag < 0.55) return 'glide';
    if (gyroMag > 1.5) return 'recovery';
    if (liaBodyMag < 3.0) return 'recovery';
    return 'recovery';
}

function gfPhaseParamsForReplay(d, seg, i, ts, ax, ay, az, gyro) {
    const def = GF_PHASE_PARAMS.pull;
    if (!seg || i < seg.start) return def;
    const d0 = processedData[seg.start];
    const t0 = (d0 && d0.timestamp != null) ? Number(d0.timestamp) : (seg.start * 20);
    const dtStroke = Math.max(0, ts - t0);
    const liaBodyMag = Math.hypot(ax, ay, az);
    const gx = Number(gyro.gx || gyro.x || 0) || 0;
    const gy = Number(gyro.gy || gyro.y || 0) || 0;
    const gz = Number(gyro.gz || gyro.z || 0) || 0;
    const gyroMag = Math.hypot(gx, gy, gz);
    const name = gfInferReplayPhase(d, dtStroke, liaBodyMag, gyroMag);
    return GF_PHASE_PARAMS[name] || def;
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
            rawIntegratedPositions.push(vizMapProcessorPosition(p.px, p.py, p.pz));
        }
        return;
    }

    const hasTracking = processedData.some(d => d.tracking_active !== undefined);
    computeStrokeBoundaries();
    const useStrokeSegments = !hasTracking && strokeBoundaries && strokeBoundaries.length > 0;
    const strokeSegAt = (idx) => {
        if (!strokeBoundaries || !strokeBoundaries.length) return null;
        let b = -1;
        for (let k = 0; k < strokeBoundaries.length; k++) {
            if (strokeBoundaries[k].index <= idx) b = k;
            else break;
        }
        if (b < 0) return null;
        const start = strokeBoundaries[b].index;
        const end = (b + 1 < strokeBoundaries.length)
            ? strokeBoundaries[b + 1].index - 1
            : processedData.length - 1;
        return { start, end };
    };
    /* Fallback when no stroke indices in file: short motion gate (older behavior). */
    let gateOn = false;
    let gateStartMs = 0;
    const START_GY = 0.52;
    const START_A = 0.16;
    const END_GY = 0.18;
    const END_A = 0.2;
    const MIN_ON_MS = 420;

    const kx = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const ky = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const kz = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    let vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
    let prevStroke = strokeNumAt(processedData[0]);
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const sc = strokeNumAt(d);
        if (hasTracking && sc > prevStroke) {
            vx = vy = vz = 0;
            px = py = pz = 0;
            prevStroke = sc;
        }
        let dt = 0.02;
        if (i > 0 && d.timestamp != null && processedData[i - 1].timestamp != null) {
            dt = Math.max(0.001, Math.min(0.1, (d.timestamp - processedData[i - 1].timestamp) / 1000));
        }
        /* Match StrokeProcessor: no integration when tracking_active false (no idle drift). */
        if (hasTracking && d.tracking_active === false) {
            rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
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

        const aMag = Math.hypot(wA.x, wA.y, wA.z);
        const ts = (d.timestamp != null) ? Number(d.timestamp) : i * 20;
        const gyro = d.angular_velocity || {};
        const gy = Number(gyro.gy || gyro.y || 0) || 0;
        const gyAbs = Math.abs(gy);
        /* Raw AP / SD replay without tracking_active: full stroke cycle per boundary when possible. */
        if (!hasTracking && useStrokeSegments) {
            const seg = strokeSegAt(i);
            if (!seg || i < seg.start) {
                rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
                vx = vy = vz = 0;
                continue;
            }
            if (i === seg.start) {
                vx = vy = vz = 0;
                px = py = pz = 0;
                prevStroke = sc;
            }
        } else if (!hasTracking) {
            if (!gateOn) {
                if (gyAbs > START_GY || aMag > START_A) {
                    gateOn = true;
                    gateStartMs = ts;
                    vx = vy = vz = 0;
                    px = py = pz = 0;
                }
            } else {
                const dur = ts - gateStartMs;
                const quiet = (gyAbs < END_GY && aMag < END_A);
                if (quiet && dur >= MIN_ON_MS) gateOn = false;
                else if (dur > 25000) gateOn = false;
            }
            if (!gateOn) {
                rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
                vx = vy = vz = 0;
                continue;
            }
        }

        /* Phase-adaptive deadzone/decay on replay+stroke boundaries; gate fallback fixed; firmware tracking_active unchanged. */
        let pp = { deadzone: 0.2, decay: 0.985 };
        if (!hasTracking && useStrokeSegments) {
            const segNow = strokeSegAt(i);
            pp = gfPhaseParamsForReplay(d, segNow, i, ts, ax, ay, az, gyro);
        } else if (!hasTracking && !useStrokeSegments) {
            pp = { deadzone: 0.2, decay: 0.985 };
        }
        const cal = d.cal || d.calibration || {};
        const ca = Number(cal.accel) || 0;
        const cg = Number(cal.gyro) || 0;
        let DEADZONE = pp.deadzone;
        if (ca >= 2 && cg >= 2) DEADZONE *= 0.85;

        let axw = wA.x, ayw = wA.y, azw = wA.z;
        if (aMag < DEADZONE) {
            axw = ayw = azw = 0;
        }
        vx += axw * dt; vy += ayw * dt; vz += azw * dt;
        let decay;
        if (hasTracking) {
            decay = (aMag < DEADZONE) ? 0.82 : 0.985;
        } else {
            decay = (aMag < DEADZONE) ? (pp.decay * 0.94) : pp.decay;
        }
        vx *= decay; vy *= decay; vz *= decay;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        const pm = Math.sqrt(px * px + py * py + pz * pz);
        if (pm > 1.0) {
            vx *= 0.95; vy *= 0.95; vz *= 0.95;
        }
        rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
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
