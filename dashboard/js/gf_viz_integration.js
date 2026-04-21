/**
 * GoldenForm  --  LIA / processor position integration and stroke-segmented trail prep.
 *
 * All integration uses 0401fbde StrokeProcessor math  --  the most accurate stroke visualization:
 *   Kalman-smoothed LIA (Q=0.3, R=0.1) → quaternion to world frame → per-axis deadzone (0.2 m/s²)
 *   → integrate velocity/position → fixed decay (0.98 active / 0.80 stationary).
 *   Motion gate: abs(gy) + accel_mag thresholds; position/velocity reset on segment start.
 *
 * Authoritative path: use `position` from Python `StrokeProcessor` when present.
 * Fallback: same Kalman → world-frame → integrate pipeline in JS.
 * After integration, `applyPerStrokeOriginOffset` subtracts each stroke’s start so the graph
 * resets to the origin every stroke (pool trail is stroke-local, not session drift).
 * Python (0401fbde uart_match) resets position/velocity when each motion segment starts,
 * including batch replay — limits double-integration drift; JS still subtracts per-stroke
 * origin (catch/pull anchor) for comparable stroke-to-stroke views.
 *
 * Reference: https://github.com/chaudhary-nikhil/E22_Senior_Design/tree/0401fbde9e7a69887208db1df3e5feff27011ba6
 */
/**
 * Map processor world position (px,py,pz) to Three.js scene coordinates.
 * Scene: Y up, pool grid in XZ. StrokeProcessor integrates in a world frame where the pull
 * often grows py; negating Y matches 0401fbde-style “down into the water” vs screen-up intuition.
 */
function vizMapProcessorPosition(px, py, pz) {
    const s = typeof positionScale !== 'undefined' ? positionScale : 1;
    const ySign = (typeof window !== 'undefined' && Number.isFinite(window.VIZ_WORLD_Y_SIGN))
        ? window.VIZ_WORLD_Y_SIGN
        : -1;
    return new THREE.Vector3(
        (Number(px) || 0) * s,
        ySign * (Number(py) || 0) * s,
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

/**
 * 0401fbde StrokeProcessor integration constants.
 * Fixed deadzone + decay  --  the original math that produced the most accurate stroke visualization.
 */
const GF_0401_DEADZONE = 0.2;   // m/s² per-axis
const GF_0401_DECAY_ACTIVE = 0.98;
const GF_0401_DECAY_STATIONARY = 0.80;
const GF_0401_START_GY = 0.6;   // rad/s  --  abs(gy) to start integration
const GF_0401_START_A = 0.25;   // m/s²  --  accel_mag to start integration
const GF_0401_END_GY = 0.2;
const GF_0401_END_A = 0.25;
const GF_0401_MIN_ON_MS = 500;
/** If Python batch replay never opens the motion gate, position stays ~0 — still "valid" fields. */
const GF_POS_DEGENERATE_SPAN_M = 1e-5;
/** HPF time constant (s) on world-frame accel. Subtracts a slow EMA baseline to remove
 *  residual gravity / quaternion-yaw-drift before double-integration. fc ≈ 0.08 Hz — well
 *  below the 0.4-1 Hz stroke band, so real pull accelerations pass through. Prevents the
 *  "smooth elliptical loop" drift artifact that replaces the true S-shaped pull path. */
const GF_WORLD_ACCEL_HPF_TAU_S = 2.0;

/**
 * Stroke-checkpoint template (RRT-style path planning) -- freestyle only.
 * A freestyle arm cycle ends where it began in body frame, but IMU integration drift
 * leaves the end of the integrated trail offset from the start. These templates define
 * the target positions (meters, stroke-local, Y = up, Z = forward along swim direction)
 * at 4 chronological checkpoints: catch (origin), deepest pull, recovery apex, re-entry.
 * Values tuned to a typical wrist-relative-to-shoulder magnitude for ~1 m of reach.
 * Negative Y = below shoulder (depth); positive Y = above shoulder (recovery high).
 * Note: scene coordinates use ySign = -1 via vizMapProcessorPosition, so template Y
 * retains its physical meaning when multiplied by -ySign inside the warp.
 */
const GF_STROKE_TEMPLATE_NADIR_M = { x: 0.00, y: -0.35, z: +0.20 };
const GF_STROKE_TEMPLATE_APEX_M  = { x: 0.00, y: +0.25, z: -0.10 };
/** Minimum stroke length (samples) below which the template warp is skipped -- nadir/apex
 *  detection becomes unreliable and the piecewise-linear correction risks spiking. */
const GF_STROKE_WARP_MIN_SAMPLES = 8;

/** Jump-bridge thresholds. A sample-to-sample position delta larger than
 *  `GF_JUMP_BRIDGE_K * median(|delta|)` inside a single stroke is treated as a spurious
 *  teleport (Python reset misaligned with firmware strokes, cal dropout re-zero, or a large
 *  dt gap). We stitch it out by subtracting the jump offset from every sample past the
 *  spike, preserving relative motion on both sides. Idempotent. */
const GF_JUMP_BRIDGE_K = 8;
const GF_JUMP_BRIDGE_MAX_CORRECTIONS = 3;
const GF_JUMP_BRIDGE_MIN_SAMPLES = 8;

/**
 * Stroke-type gate for the template warp. Today the entire dashboard assumes freestyle
 * (entry-angle targets, coaching copy, resampleTrailFreestyle), so there is no
 * stroke_type field on samples yet; this stub returns true for every segment. When
 * stroke-type classification lands (e.g. `seg.stroke_type === 'freestyle'`), tighten
 * this helper without touching the warp itself.
 */
function isFreestyleStroke(seg) {
    if (!seg) return false;
    return true;
}

function _processorPositionSpanMeters() {
    let minx = Infinity, miny = Infinity, minz = Infinity;
    let maxx = -Infinity, maxy = -Infinity, maxz = -Infinity;
    for (let i = 0; i < processedData.length; i++) {
        const p = processedData[i].position;
        if (!p || !Number.isFinite(p.px) || !Number.isFinite(p.py) || !Number.isFinite(p.pz)) {
            return 0;
        }
        minx = Math.min(minx, p.px); maxx = Math.max(maxx, p.px);
        miny = Math.min(miny, p.py); maxy = Math.max(maxy, p.py);
        minz = Math.min(minz, p.pz); maxz = Math.max(maxz, p.pz);
    }
    return Math.max(maxx - minx, maxy - miny, maxz - minz);
}

function buildRawIntegratedPositions() {
    rawIntegratedPositions = [];
    if (!processedData.length) return;
    refreshStrokeFieldMode();

    const hasProcessorPos = processedData.length > 0 && processedData.every(d =>
        d.position
        && typeof d.position.px === 'number'
        && typeof d.position.py === 'number'
        && typeof d.position.pz === 'number'
        && Number.isFinite(d.position.px)
        && Number.isFinite(d.position.py)
        && Number.isFinite(d.position.pz)
    );
    const processorSpreadOk = hasProcessorPos && _processorPositionSpanMeters() > GF_POS_DEGENERATE_SPAN_M;
    if (processorSpreadOk) {
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

    /* Python may set tracking_active on every sample to false (UART gate never opened). If we honored
     * that literally, the JS path would skip all LIA integration — flat trail, only attitude moves. */
    const hasTrackingField = processedData.some(d => d.tracking_active !== undefined);
    const trackingEverTrue = processedData.some(d => d.tracking_active === true);
    const hasTracking = hasTrackingField && trackingEverTrue;
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
    /* 0401fbde motion gate: abs(gy) + accel_mag thresholds, position/velocity reset on segment start. */
    let gateOn = false;
    let gateStartMs = 0;

    const kx = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const ky = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    const kz = gfKalman1D(0.3, 0.1, 1.0, 0.0);
    let vx = 0, vy = 0, vz = 0, px = 0, py = 0, pz = 0;
    /* World-frame accel HPF state (see GF_WORLD_ACCEL_HPF_TAU_S). */
    let emaAx = 0, emaAy = 0, emaAz = 0;
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

        /* HPF baseline update — always run so the baseline is accurate when the gate opens. */
        if (GF_WORLD_ACCEL_HPF_TAU_S > 0 && dt > 0) {
            const alpha = Math.min(1, dt / GF_WORLD_ACCEL_HPF_TAU_S);
            emaAx += alpha * (wA.x - emaAx);
            emaAy += alpha * (wA.y - emaAy);
            emaAz += alpha * (wA.z - emaAz);
            wA.x -= emaAx;
            wA.y -= emaAy;
            wA.z -= emaAz;
        }

        const aMag = Math.hypot(wA.x, wA.y, wA.z);
        const ts = (d.timestamp != null) ? Number(d.timestamp) : i * 20;
        const gyro = d.angular_velocity || {};
        const gy = Number(gyro.gy || gyro.y || 0) || 0;
        const gyAbs = Math.abs(gy);
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
                if (gyAbs > GF_0401_START_GY || aMag > GF_0401_START_A) {
                    gateOn = true;
                    gateStartMs = ts;
                    vx = vy = vz = 0;
                    px = py = pz = 0;
                }
            } else {
                const dur = ts - gateStartMs;
                const quiet = (gyAbs < GF_0401_END_GY && aMag < GF_0401_END_A);
                if (quiet && dur >= GF_0401_MIN_ON_MS) gateOn = false;
                else if (dur > 20000) gateOn = false;
            }
            if (!gateOn) {
                rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
                vx = vy = vz = 0;
                continue;
            }
        }

        /* 0401fbde integration: per-axis deadzone, fixed decay. */
        let axw = wA.x, ayw = wA.y, azw = wA.z;
        let isAccel = false;
        if (Math.abs(axw) < GF_0401_DEADZONE) axw = 0; else isAccel = true;
        if (Math.abs(ayw) < GF_0401_DEADZONE) ayw = 0; else isAccel = true;
        if (Math.abs(azw) < GF_0401_DEADZONE) azw = 0; else isAccel = true;

        vx += axw * dt; vy += ayw * dt; vz += azw * dt;
        const decay = isAccel ? GF_0401_DECAY_ACTIVE : GF_0401_DECAY_STATIONARY;
        vx *= decay; vy *= decay; vz *= decay;
        px += vx * dt; py += vy * dt; pz += vz * dt;
        rawIntegratedPositions.push(vizMapProcessorPosition(px, py, pz));
    }
}

function integratePositions() {
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    /* Invalidate the per-stroke sagittal-transform cache (PCA angle + mirror sign) so a
     * new session -- or a re-run of integration on the same session -- starts with fresh
     * decisions instead of reusing bounds from a previous session's processedData. */
    if (typeof invalidateSideViewSagittalCache === 'function') invalidateSideViewSagittalCache();
    if (!processedData.length) return;
    buildRawIntegratedPositions();
    applyPerStrokeOriginOffset(rawIntegratedPositions);
    buildPlaybackStrokeSegments();
    /* Stitch out single-sample teleports (Python reset misaligned with firmware strokes,
     * cal dropout hard-zero, long dt gap) before the checkpoint warp sees the stroke. */
    if (typeof window === 'undefined' || window.GF_JUMP_BRIDGE !== false) {
        rawIntegratedPositions = bridgeStrokeJumps(rawIntegratedPositions);
    }
    /* RRT-style 4-checkpoint warp: closes each freestyle stroke's integrated loop and
     * anchors the nadir/apex to the template. Toggle off for debugging via
     * `window.GF_STROKE_CHECKPOINT_WARP = false` in the console before reload. */
    if (typeof window === 'undefined' || window.GF_STROKE_CHECKPOINT_WARP !== false) {
        applyStrokeCheckpointWarp(rawIntegratedPositions);
    }
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

/**
 * Gaussian-kernel smoothing along the time axis.
 * Box-car averaging (the old approach) flattens peaks and introduces phase lag, which
 * converts the real back-and-forth S-shape of an underwater pull into a featureless
 * ellipse. A Gaussian with σ ≈ windowSize/4 preserves peaks and phase while still
 * knocking down high-frequency integration jitter.
 */
function smoothTrailPoints(points, windowSize) {
    if (!points.length || windowSize < 2) return points;
    const half = Math.floor(windowSize / 2);
    const sigma = Math.max(0.5, windowSize / 4);
    const two_s2 = 2 * sigma * sigma;
    const weights = new Array(half * 2 + 1);
    for (let k = -half; k <= half; k++) weights[k + half] = Math.exp(-(k * k) / two_s2);
    const out = [];
    for (let i = 0; i < points.length; i++) {
        let x = 0, y = 0, z = 0, wsum = 0;
        const jmin = Math.max(0, i - half);
        const jmax = Math.min(points.length - 1, i + half);
        for (let j = jmin; j <= jmax; j++) {
            const w = weights[j - i + half];
            x += points[j].x * w;
            y += points[j].y * w;
            z += points[j].z * w;
            wsum += w;
        }
        out.push(new THREE.Vector3(x / wsum, y / wsum, z / wsum));
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
    /* Smaller, per-stroke-sized window (σ ≈ w/4 in the Gaussian kernel). A 5-sample
     * window at ~50 Hz is σ ≈ 25 ms -- plenty to denoise integration jitter without
     * washing out the S-curve of a 1-2 s pull. Box-car 11 was turning real strokes
     * into ellipses. */
    const w = Math.min(5, Math.max(3, Math.floor(points.length / 20) * 2 + 1));
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
    if (!processedData || !processedData.length) return;
    /* One segment per firmware STROKE_DET edge. Each segment starts at a boundary sample
     * and ends at the sample before the next boundary on the same stream, clamped so it
     * never crosses into a different stream (interleaved multi-device timelines). */
    const canon = (typeof computeCanonicalStrokeBoundaries === 'function')
        ? computeCanonicalStrokeBoundaries()
        : [];
    if (!canon.length) return;
    const n = processedData.length;
    for (let k = 0; k < canon.length; k++) {
        const b = canon[k];
        let end = n - 1;
        for (let j = k + 1; j < canon.length; j++) {
            if (canon[j].streamKey === b.streamKey) { end = canon[j].index - 1; break; }
        }
        for (let i = b.index + 1; i <= end; i++) {
            if (getStreamKey(processedData[i]) !== b.streamKey) { end = i - 1; break; }
        }
        if (end > b.index) {
            playbackStrokeSegments.push({
                startIdx: b.index,
                endIdx: end,
                strokeNum: b.strokeNum,
                streamKey: b.streamKey
            });
        }
    }
}

/**
 * Each stroke’s trail starts at graph origin (0,0,0) so sessions don’t look like random drift.
 * For each sample, subtract integrated position at the first sample of (stream, stroke #) — works
 * for interleaved multi-device timelines, not only contiguous stroke blocks in file order.
 */
function applyPerStrokeOriginOffset(points) {
    if (!points.length || !processedData.length || points.length !== processedData.length) return;
    refreshStrokeFieldMode();
    const n = points.length;
    const rawSessionStart = (points[0] && points[0].clone) ? points[0].clone() : null;
    let anyStroke = false;
    for (let i = 0; i < n; i++) {
        if (strokeNumAt(processedData[i]) > 0) {
            anyStroke = true;
            break;
        }
    }
    if (!anyStroke && n > 0 && rawSessionStart) {
        for (let i = 0; i < n; i++) {
            if (points[i] && points[i].sub) points[i].sub(rawSessionStart);
        }
        return;
    }
    // Pick a more meaningful stroke origin: first catch/pull (or in_water_pull) inside the stroke.
    // This makes the stroke start with the downward/entry motion instead of recovery/glide.
    const originIdx = new Int32Array(n);
    const originByKey = new Map(); // key = `${sk}::${sc}` -> origin index
    function strokeKey(sk, sc) { return String(sk) + '::' + String(sc); }
    function phaseAt(i) {
        if (typeof phaseAtSample === 'function') return phaseAtSample(i);
        const d = processedData[i] || {};
        return String(d.stroke_phase || d.phase || '').toLowerCase();
    }
    function isStrokeEntryLike(i) {
        const d = processedData[i] || {};
        const ph = phaseAt(i);
        if (ph === 'catch' || ph === 'pull') return true;
        if (d.in_water_pull === true) return true;
        return false;
    }
    for (let i = 0; i < n; i++) {
        const d = processedData[i];
        const sk = getStreamKey(d);
        const sc = strokeNumAt(d);
        if (sc <= 0) { originIdx[i] = -1; continue; }
        const k = strokeKey(sk, sc);
        let oi = originByKey.get(k);
        if (oi == null) {
            // Find the contiguous block start for this stream/stroke.
            let start = i;
            for (let j = i; j >= 0; j--) {
                const dj = processedData[j];
                if (getStreamKey(dj) !== sk) break;
                if (strokeNumAt(dj) === sc) start = j;
                else break;
            }
            // Search forward for entry-like phase within a short window.
            oi = start;
            const searchEnd = Math.min(n - 1, start + 180);
            for (let t = start; t <= searchEnd; t++) {
                const dt = processedData[t];
                if (getStreamKey(dt) !== sk) break;
                if (strokeNumAt(dt) !== sc) break;
                if (isStrokeEntryLike(t)) { oi = t; break; }
            }
            originByKey.set(k, oi);
        }
        originIdx[i] = oi;
    }
    const clones = new Map();
    for (let i = 0; i < n; i++) {
        const oi = originIdx[i];
        if (oi < 0 || clones.has(oi)) continue;
        if (points[oi] && points[oi].clone) clones.set(oi, points[oi].clone());
    }
    for (let i = 0; i < n; i++) {
        const oi = originIdx[i];
        if (oi < 0) {
            /* Before first stroke (stroke_count still 0): keep motion vs session start — do not zero. */
            if (rawSessionStart && points[i] && points[i].sub) points[i].sub(rawSessionStart);
            continue;
        }
        const o = clones.get(oi);
        if (!o || !points[i] || !points[i].sub) continue;
        points[i].sub(o);
    }
}

/**
 * RRT-style path closure: warp each freestyle stroke's integrated path to pass through
 * 4 prescribed checkpoints -- catch (origin), deepest pull (template nadir), recovery
 * apex (template apex), re-entry (origin). Eliminates the end-of-stroke gap caused by
 * residual IMU integration drift while preserving the inter-checkpoint shape.
 *
 * The correction is piecewise-linear in sample index between consecutive checkpoints,
 * so the path passes through every checkpoint exactly and the existing Gaussian smoother
 * handles the kink at each checkpoint afterwards.
 *
 * Operates in-place on `points` (which are THREE.Vector3 in scene coords with `positionScale`
 * already baked in via `vizMapProcessorPosition`). Falls back to a simple end-only linear
 * loop closure when the stroke doesn't look freestyle-shaped (nadir after apex, apex at
 * end, too few samples).
 *
 * Requires `playbackStrokeSegments` to be built first.
 */
/**
 * bridgeStrokeJumps: per-stroke single-sample teleport fix.
 *
 * Scans each canonical stroke segment for a position delta whose magnitude is much larger
 * than the stroke's typical step (median * GF_JUMP_BRIDGE_K). Each such jump is treated
 * as an off-device reset (Python stroke_detected misaligned with firmware `strokes`, cal
 * dropout hard-zero, or a long dt gap) rather than real motion, and stitched out by
 * subtracting the excess offset from every sample at or after the spike. Shape on both
 * sides of the jump is preserved; only the tail is translated into alignment.
 *
 * Runs in the integration pipeline after `applyPerStrokeOriginOffset` and before
 * `applyStrokeCheckpointWarp` so the nadir/apex detector sees continuous geometry.
 * Returns a new array (points are cloned); caller should reassign.
 */
function bridgeStrokeJumps(points) {
    if (!points || !points.length) return points;
    if (!playbackStrokeSegments || !playbackStrokeSegments.length) return points;
    const out = points.map(p => (p && p.clone)
        ? p.clone()
        : new THREE.Vector3(
            (p && Number.isFinite(p.x)) ? p.x : 0,
            (p && Number.isFinite(p.y)) ? p.y : 0,
            (p && Number.isFinite(p.z)) ? p.z : 0
        )
    );
    for (const seg of playbackStrokeSegments) {
        const startIdx = seg.startIdx;
        const endIdx = seg.endIdx;
        if (!(endIdx > startIdx)) continue;
        if ((endIdx - startIdx + 1) < GF_JUMP_BRIDGE_MIN_SAMPLES) continue;

        const mags = [];
        for (let i = startIdx + 1; i <= endIdx; i++) {
            const a = out[i - 1], b = out[i];
            if (!a || !b) { mags.push(0); continue; }
            mags.push(Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z));
        }
        if (!mags.length) continue;
        const sorted = mags.slice().sort((a, b) => a - b);
        const med = sorted[Math.floor(sorted.length / 2)] || 0;
        const thr = Math.max(med * GF_JUMP_BRIDGE_K, 1e-5);

        const cands = [];
        for (let k = 0; k < mags.length; k++) {
            if (mags[k] > thr) cands.push({ i: startIdx + 1 + k, m: mags[k] });
        }
        if (!cands.length) continue;
        cands.sort((a, b) => b.m - a.m);
        const picks = cands.slice(0, GF_JUMP_BRIDGE_MAX_CORRECTIONS).sort((a, b) => a.i - b.i);

        for (const c of picks) {
            let ex = 0, ey = 0, ez = 0, n = 0;
            const kLo = Math.max(startIdx + 1, c.i - 2);
            const kHi = Math.min(endIdx, c.i + 2);
            for (let k = kLo; k <= kHi; k++) {
                if (k === c.i) continue;
                const a = out[k - 1], b = out[k];
                if (!a || !b) continue;
                const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
                if (Math.hypot(dx, dy, dz) > thr) continue;
                ex += dx; ey += dy; ez += dz; n++;
            }
            if (n > 0) { ex /= n; ey /= n; ez /= n; }
            const a = out[c.i - 1], b = out[c.i];
            if (!a || !b) continue;
            const ox = (b.x - a.x) - ex;
            const oy = (b.y - a.y) - ey;
            const oz = (b.z - a.z) - ez;
            if (!Number.isFinite(ox) || !Number.isFinite(oy) || !Number.isFinite(oz)) continue;
            for (let j = c.i; j <= endIdx; j++) {
                if (!out[j]) continue;
                out[j].x -= ox;
                out[j].y -= oy;
                out[j].z -= oz;
            }
        }
    }
    return out;
}

function applyStrokeCheckpointWarp(points) {
    if (!points || !points.length) return;
    if (!playbackStrokeSegments || !playbackStrokeSegments.length) return;
    if (typeof THREE === 'undefined') return;

    const s = (typeof positionScale !== 'undefined' && positionScale > 0) ? positionScale : 1;
    const ySign = (typeof window !== 'undefined' && Number.isFinite(window.VIZ_WORLD_Y_SIGN))
        ? window.VIZ_WORLD_Y_SIGN
        : -1;
    /* Template is specified in physical meters (Y = up). Scene Y is flipped by ySign in
     * vizMapProcessorPosition, so mirror that flip here so +Y meaning matches in both. */
    const nadirTarget = new THREE.Vector3(
        GF_STROKE_TEMPLATE_NADIR_M.x * s,
        ySign * -1 * GF_STROKE_TEMPLATE_NADIR_M.y * s,
        GF_STROKE_TEMPLATE_NADIR_M.z * s
    );
    const apexTarget = new THREE.Vector3(
        GF_STROKE_TEMPLATE_APEX_M.x * s,
        ySign * -1 * GF_STROKE_TEMPLATE_APEX_M.y * s,
        GF_STROKE_TEMPLATE_APEX_M.z * s
    );
    const originTarget = new THREE.Vector3(0, 0, 0);

    /* Apply a piecewise-linear offset correction given an ordered list of checkpoints.
     * For each sample i in [cps[0].idx..cps[last].idx], subtract
     *   lerp(cps[k].offset, cps[k+1].offset, alpha)
     * where [k, k+1] is the bracketing segment and alpha is the normalised position in
     * that segment. Each sample is corrected exactly once (no double-counting at shared
     * endpoints). Guarantees points[cp.idx] == cp.target for every checkpoint. */
    function applyPiecewiseOffsets(cps) {
        if (!cps || cps.length < 2) return;
        let k = 0;
        const first = cps[0].idx;
        const last  = cps[cps.length - 1].idx;
        for (let i = first; i <= last; i++) {
            while (k + 1 < cps.length - 1 && i >= cps[k + 1].idx) k++;
            const a = cps[k];
            const b = cps[k + 1];
            const span = b.idx - a.idx;
            const alpha = span > 0 ? (i - a.idx) / span : 0;
            const p = points[i];
            if (!p || !Number.isFinite(p.x)) continue;
            p.x -= a.offset.x + (b.offset.x - a.offset.x) * alpha;
            p.y -= a.offset.y + (b.offset.y - a.offset.y) * alpha;
            p.z -= a.offset.z + (b.offset.z - a.offset.z) * alpha;
        }
    }

    function applyLinearLoopClosure(startIdx, endIdx) {
        const pS = points[startIdx];
        const pE = points[endIdx];
        if (!pS || !pE) return;
        applyPiecewiseOffsets([
            { idx: startIdx, offset: pS.clone().sub(originTarget) },
            { idx: endIdx,   offset: pE.clone().sub(originTarget) }
        ]);
    }

    for (const seg of playbackStrokeSegments) {
        if (!seg || !isFreestyleStroke(seg)) continue;
        const s0 = seg.startIdx;
        const e0 = seg.endIdx;
        if (s0 == null || e0 == null || e0 <= s0) continue;
        if ((e0 - s0 + 1) < GF_STROKE_WARP_MIN_SAMPLES) {
            applyLinearLoopClosure(s0, e0);
            continue;
        }

        /* Detect nadir (deepest) and apex (highest) by scene-Y extrema inside the stroke.
         * Exclude the endpoints so the piecewise-linear segmentation is non-degenerate. */
        let iNadir = -1, iApex = -1;
        let yMin = Infinity, yMax = -Infinity;
        for (let i = s0 + 1; i < e0; i++) {
            const p = points[i];
            if (!p || !Number.isFinite(p.y)) continue;
            if (p.y < yMin) { yMin = p.y; iNadir = i; }
            if (p.y > yMax) { yMax = p.y; iApex  = i; }
        }

        const freestyleLike = (
            iNadir > s0 && iNadir < e0 &&
            iApex  > s0 && iApex  < e0 &&
            iNadir < iApex &&
            (iApex - iNadir) >= 2
        );
        if (!freestyleLike) {
            /* Non-freestyle-shaped cycle (or detection failed) -- close the loop but
             * leave the interior shape alone so we don't introduce a bogus template. */
            applyLinearLoopClosure(s0, e0);
            continue;
        }

        applyPiecewiseOffsets([
            { idx: s0,     offset: points[s0].clone().sub(originTarget) },
            { idx: iNadir, offset: points[iNadir].clone().sub(nadirTarget) },
            { idx: iApex,  offset: points[iApex].clone().sub(apexTarget) },
            { idx: e0,     offset: points[e0].clone().sub(originTarget) }
        ]);
    }
}
