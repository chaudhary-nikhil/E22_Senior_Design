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

/** PCA major-axis angle (rad) in 2D — rotate path so primary motion reads left→right (sagittal-ish). */
function principalAxisAngleRad2D(pts) {
    if (!pts || pts.length < 5) return 0;
    let mx = 0;
    let my = 0;
    for (const p of pts) {
        mx += p.x;
        my += p.y;
    }
    const n = pts.length;
    mx /= n;
    my /= n;
    let cxx = 0;
    let cyy = 0;
    let cxy = 0;
    for (const p of pts) {
        const dx = p.x - mx;
        const dy = p.y - my;
        cxx += dx * dx;
        cyy += dy * dy;
        cxy += dx * dy;
    }
    cxx /= n;
    cyy /= n;
    cxy /= n;
    return 0.5 * Math.atan2(2 * cxy, cxx - cyy);
}

function rotatePts2D(pts, ang) {
    const c = Math.cos(ang);
    const s = Math.sin(ang);
    return pts.map((p) => ({
        x: p.x * c - p.y * s,
        y: p.x * s + p.y * c,
        i: p.i
    }));
}

/**
 * PCA gives an axis but its sign is ambiguous (stroke can mirror left/right).
 * Normalize so "forward" motion during catch/pull always goes +X in side view.
 */
function _phaseForSideView(i) {
    if (typeof phaseAtSample === 'function') return phaseAtSample(i);
    const d = processedData[i];
    return (d && (d.stroke_phase || d.phase)) || '';
}

/**
 * Per-stroke sagittal-transform cache. Decisions (PCA angle + mirror sign) must be
 * stable across playback frames -- otherwise the growing partial polyline re-computes
 * its own sign each frame and the whole canvas mirrors mid-stroke (the "sudden jump"
 * between stroke-start and stroke-end screenshots). Cache invalidation is wired into
 * integratePositions() so a fresh session starts with fresh decisions.
 */
let _sagittalCache = new Map();          // "streamKey#strokeNum" -> { pcaAng, sign, ySign }
let _sagittalFirstPerStream = new Map(); // streamKey -> first confident xSign seen (session-wide lock)
/* Session-locked winding (+1 = CCW reads positive shoelace area, -1 = CW). Seeded once per
 * session by _ensureSessionWindingLock() by tallying the majority signed-area sign across all
 * canonical strokes on that stream. Without this, the X-mirror applied by _applySagittalSigns
 * flips 2D loop winding, so strokes that happen to need an X-flip end up rotating opposite to
 * the rest of the session (stroke #1 CW while strokes #2..#N are CCW in the reported session). */
let _sagittalWindingPerStream = new Map(); // streamKey -> +1 | -1 (majority winding)
let _sagittalWindingLockSeeded = false;

function invalidateSideViewSagittalCache() {
    _sagittalCache.clear();
    _sagittalFirstPerStream.clear();
    _sagittalWindingPerStream.clear();
    _sagittalWindingLockSeeded = false;
}

/** Shoelace signed area of a 2D polyline/loop. Positive = CCW in math-standard axes. */
function _signedArea2D(pts) {
    if (!pts || pts.length < 3) return 0;
    let a = 0;
    const n = pts.length;
    for (let i = 0; i < n; i++) {
        const p = pts[i];
        const q = pts[(i + 1) % n];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

/**
 * Return {sign, ambiguous} for a rotated full-stroke polyline. Sign is +1 if the
 * catch/pull window advances along +X already, -1 if the polyline needs to be
 * mirrored on X. `ambiguous` is true when |Δx| in the catch/pull window is tiny
 * relative to the stroke's x-range (glide-heavy stroke, missing phases); callers
 * should inherit the session-wide polarity in that case.
 */
function _decideSagittalPolarity(rotPts) {
    if (!rotPts || rotPts.length < 2) return { sign: 1, ambiguous: true };
    const n = rotPts.length;
    const iEnd = Math.min(n - 1, Math.max(1, Math.floor(n * 0.35)));
    let i0 = 0;
    let i1 = iEnd;
    for (let k = 0; k <= iEnd; k++) {
        const ph = _phaseForSideView(rotPts[k].i);
        if (ph === 'catch' || ph === 'pull') { i0 = k; break; }
    }
    for (let k = iEnd; k >= i0 + 1; k--) {
        const ph = _phaseForSideView(rotPts[k].i);
        if (ph === 'catch' || ph === 'pull') { i1 = k; break; }
    }
    const dx = rotPts[i1].x - rotPts[i0].x;
    let minX = Infinity, maxX = -Infinity;
    for (const p of rotPts) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
    }
    const range = Math.max(maxX - minX, 1e-9);
    const ambiguous = Math.abs(dx) < range * 0.01;
    const sign = dx >= 0 ? 1 : -1;
    return { sign, ambiguous };
}

/**
 * Decide Y-sign from phase physics: in-water phases (catch, pull) should plot BELOW the
 * out-of-water phases (recovery, glide) in the math-CCW frame (+y up). Picking ySign this
 * way keeps every stroke visually upright (pull dips, recovery arcs over the top), which
 * also produces consistent loop winding across strokes without the 180-degree mirror that
 * a shoelace-area based lock would apply. Returns {ySign, ambiguous}.
 */
function _decideVerticalPolarity(rotPtsAfterXSign) {
    if (!rotPtsAfterXSign || rotPtsAfterXSign.length < 4) {
        return { ySign: 1, ambiguous: true };
    }
    let inSum = 0, inCount = 0, outSum = 0, outCount = 0;
    let minY = Infinity, maxY = -Infinity;
    for (const p of rotPtsAfterXSign) {
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
        const ph = _phaseForSideView(p.i);
        if (ph === 'catch' || ph === 'pull') { inSum += p.y; inCount += 1; }
        else if (ph === 'recovery' || ph === 'glide') { outSum += p.y; outCount += 1; }
    }
    if (inCount < 2 || outCount < 2) return { ySign: 1, ambiguous: true };
    const inMean = inSum / inCount;
    const outMean = outSum / outCount;
    const range = Math.max(maxY - minY, 1e-9);
    const delta = outMean - inMean; // want positive (recovery/glide above catch/pull)
    const ambiguous = Math.abs(delta) < range * 0.05;
    return { ySign: delta >= 0 ? 1 : -1, ambiguous };
}

/**
 * Apply X and Y sign flips to a rotated 2D polyline. xSign normalizes "catch/pull reads +X",
 * ySign normalizes "pull is below recovery" across strokes. Either flip alone is a
 * reflection (flips winding); applying both is a 180-degree rotation (preserves winding).
 */
function _applySagittalSigns(rotPts, xSign, ySign) {
    if (!rotPts || !rotPts.length) return rotPts;
    const sx = xSign < 0 ? -1 : 1;
    const sy = ySign < 0 ? -1 : 1;
    if (sx === 1 && sy === 1) return rotPts;
    return rotPts.map(p => ({ x: sx * p.x, y: sy * p.y, i: p.i }));
}

/** Legacy 1-sign wrapper; internal callers use _applySagittalSigns with both signs. */
function _applySagittalSign(rotPts, sign) {
    return _applySagittalSigns(rotPts, sign, 1);
}

/**
 * Seed `_sagittalWindingPerStream` with the majority winding sign per stream, scanning every
 * canonical stroke in the session exactly once. Runs lazily on the first transform request
 * after a cache invalidation. Ties or all-zero areas leave the stream at +1 (CCW default).
 */
function _ensureSessionWindingLock() {
    if (_sagittalWindingLockSeeded) return;
    _sagittalWindingLockSeeded = true;
    if (!processedData || !processedData.length) return;
    if (typeof computeCanonicalStrokeBoundaries !== 'function') return;
    const canon = computeCanonicalStrokeBoundaries();
    if (!canon || !canon.length) return;
    const n = processedData.length;
    const tally = new Map(); // streamKey -> { pos: count, neg: count }
    for (let k = 0; k < canon.length; k++) {
        const b = canon[k];
        let end = n - 1;
        for (let j = k + 1; j < canon.length; j++) {
            if (canon[j].streamKey === b.streamKey) { end = canon[j].index - 1; break; }
        }
        for (let i = b.index + 1; i <= end; i++) {
            if (getStreamKey(processedData[i]) !== b.streamKey) { end = i - 1; break; }
        }
        if (end <= b.index) continue;
        const p0 = getLiaPositionSample(b.index);
        const raw = [];
        for (let i = b.index; i <= end; i++) {
            const p = getLiaPositionSample(i);
            raw.push({ x: p.pz - p0.pz, y: p.py - p0.py, i });
        }
        if (raw.length < 3) continue;
        const pcaAng = principalAxisAngleRad2D(raw);
        const rot = rotatePts2D(raw, -pcaAng);
        const decision = _decideSagittalPolarity(rot);
        const xSigned = _applySagittalSigns(rot, decision.sign, 1);
        const area = _signedArea2D(xSigned);
        let bbArea = 0;
        {
            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
            for (const p of xSigned) {
                if (p.x < minX) minX = p.x;
                if (p.x > maxX) maxX = p.x;
                if (p.y < minY) minY = p.y;
                if (p.y > maxY) maxY = p.y;
            }
            bbArea = Math.max((maxX - minX) * (maxY - minY), 1e-9);
        }
        /* Skip near-flat strokes from the vote (low confidence). They'll inherit the lock later. */
        if (Math.abs(area) < bbArea * 0.01) continue;
        const sk = b.streamKey || '0';
        if (!tally.has(sk)) tally.set(sk, { pos: 0, neg: 0 });
        const t = tally.get(sk);
        if (area >= 0) t.pos += 1;
        else t.neg += 1;
    }
    tally.forEach((t, sk) => {
        if (t.pos === 0 && t.neg === 0) return;
        _sagittalWindingPerStream.set(sk, t.pos >= t.neg ? 1 : -1);
    });
}

/**
 * Build (or fetch cached) {pcaAng, sign} for the stroke described by `b`. The decision
 * is computed ONCE from the full-stroke window and reused for every subsequent render
 * of that stroke -- partial playback polylines reuse the full-stroke polarity and can
 * therefore never mirror on X mid-stroke.
 */
function _getStrokeSagittalTransform(b) {
    if (!b || b.strokeNum <= 0) return { pcaAng: 0, sign: 1, ySign: 1 };
    const key = (b.streamKey || '0') + '#' + b.strokeNum;
    const hit = _sagittalCache.get(key);
    if (hit) return hit;
    /* Seed the session-wide winding lock (per-stream majority shoelace sign) once per
     * invalidation cycle. Idempotent after the first call. */
    _ensureSessionWindingLock();
    const p0 = getLiaPositionSample(b.start);
    const raw = [];
    for (let i = b.start; i <= b.end; i++) {
        const p = getLiaPositionSample(i);
        raw.push({ x: p.pz - p0.pz, y: p.py - p0.py, i });
    }
    const pcaAng = principalAxisAngleRad2D(raw);
    const rotFull = rotatePts2D(raw, -pcaAng);
    const decision = _decideSagittalPolarity(rotFull);
    let finalSign = decision.sign;
    const streamKey = b.streamKey || '0';
    if (decision.ambiguous && _sagittalFirstPerStream.has(streamKey)) {
        // Stroke has no confident catch/pull sweep -- inherit the first confident
        // polarity seen on this band so the session as a whole reads in one direction.
        finalSign = _sagittalFirstPerStream.get(streamKey);
    } else if (!decision.ambiguous && !_sagittalFirstPerStream.has(streamKey)) {
        _sagittalFirstPerStream.set(streamKey, finalSign);
    }
    /* Decide the Y-sign from phase physics: catch/pull below recovery/glide. This keeps
     * every stroke visually upright (pull dips underwater, recovery arcs overhead) and also
     * makes loop winding consistent across strokes as a side-effect -- without the 180-degree
     * rotation a shoelace-area based lock would apply, which made Y-flipped strokes look
     * like they rotated the opposite direction. */
    const xSignedFull = _applySagittalSigns(rotFull, finalSign, 1);
    const vert = _decideVerticalPolarity(xSignedFull);
    let ySign = vert.ySign;
    if (vert.ambiguous) {
        /* Phase info was insufficient (no confident catch/pull vs recovery/glide split).
         * Fall back to the session-majority shoelace winding lock so ambiguous strokes at
         * least agree with the rest of the session on rotation sense. */
        const areaX = _signedArea2D(xSignedFull);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const p of xSignedFull) {
            if (p.x < minX) minX = p.x;
            if (p.x > maxX) maxX = p.x;
            if (p.y < minY) minY = p.y;
            if (p.y > maxY) maxY = p.y;
        }
        const bbArea = Math.max((maxX - minX) * (maxY - minY), 1e-9);
        const lockedSign = _sagittalWindingPerStream.get(streamKey); // +1/-1 or undefined
        if (lockedSign && Math.abs(areaX) >= bbArea * 0.01) {
            const areaSign = areaX >= 0 ? 1 : -1;
            ySign = (areaSign === lockedSign) ? 1 : -1;
        }
    }
    const out = { pcaAng, sign: finalSign, ySign };
    _sagittalCache.set(key, out);
    return out;
}

/** Legacy wrappers -- kept so any outside caller still works. Internally they delegate
 *  to _decideSagittalPolarity so the behaviour is identical to the old inline logic,
 *  but the side view itself now uses the cached per-stroke transform. */
function normalizeSagittalSign(rotPts, startIdx, endIdx) {
    if (!rotPts || rotPts.length < 2) return rotPts;
    const { sign } = _decideSagittalPolarity(rotPts);
    return _applySagittalSign(rotPts, sign);
}

function sagittalFlipNeeded(rotPts) {
    if (!rotPts || rotPts.length < 2) return false;
    const { sign } = _decideSagittalPolarity(rotPts);
    return sign < 0;
}

/**
 * Transform used by side view for the stroke at `idx`.
 * - **rotX**: rotation around X applied to (z,y) so primary motion reads left→right in the plot
 * - **flipZ**: sign flip applied after rotation to remove PCA 180° ambiguity
 * - **flipY**: winding-lock sign flip that forces the stroke's 2D loop to rotate in the
 *   session's majority direction; 3D "lock side" camera must apply the same flip to stay
 *   in lockstep with the 2D canvas.
 */
function getSideViewTransformForIndex(idx) {
    if (!processedData || !processedData.length) return { rotX: 0, flipZ: false, flipY: false };
    const b = strokeBoundsForIndex(idx);
    if (b.strokeNum <= 0) return { rotX: 0, flipZ: false, flipY: false };
    const { pcaAng, sign, ySign } = _getStrokeSagittalTransform(b);
    return { rotX: -pcaAng, flipZ: sign < 0, flipY: ySign < 0 };
}

function clearSideViewCanvas() {
    const canvas = document.getElementById('canvas-side-view');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#0b1422';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function _buildSideViewStrokeSegmentsForExport() {
    if (!processedData || !processedData.length) return [];
    const n = processedData.length;
    const canon = (typeof computeCanonicalStrokeBoundaries === 'function')
        ? computeCanonicalStrokeBoundaries()
        : [];
    const out = [];
    for (let k = 0; k < canon.length; k++) {
        const b = canon[k];
        let end = n - 1;
        for (let j = k + 1; j < canon.length; j++) {
            if (canon[j].streamKey === b.streamKey) {
                end = canon[j].index - 1;
                break;
            }
        }
        for (let i = b.index + 1; i <= end; i++) {
            if (getStreamKey(processedData[i]) !== b.streamKey) {
                end = i - 1;
                break;
            }
        }
        if (end > b.index) {
            out.push({
                start: b.index,
                end: end,
                strokeNum: b.strokeNum,
                streamKey: b.streamKey
            });
        }
    }
    return out;
}

function exportSideViewCoordinates() {
    if (!processedData || !processedData.length) {
        alert('Load a session first, then export 2D stroke coordinates.');
        return;
    }
    if (typeof integratePositions === 'function' &&
        (!positionStreamPositions || positionStreamPositions.length !== processedData.length)) {
        integratePositions();
    }
    const segments = _buildSideViewStrokeSegmentsForExport();
    if (!segments.length) {
        alert('No stroke segments found for this session.');
        return;
    }
    const strokes = segments.map((seg) => {
        const p0 = getLiaPositionSample(seg.start);
        const rawPts = [];
        for (let i = seg.start; i <= seg.end; i++) {
            const p = getLiaPositionSample(i);
            rawPts.push({
                x: p.pz - p0.pz,
                y: p.py - p0.py,
                i: i
            });
        }
        const tf = _getStrokeSagittalTransform(seg);
        const pts = _applySagittalSigns(rotatePts2D(rawPts, -tf.pcaAng), tf.sign, tf.ySign);
        const area = _signedArea2D(pts);
        return {
            stroke_num: seg.strokeNum,
            stream_key: seg.streamKey,
            start_index: seg.start,
            end_index: seg.end,
            pca_angle_rad: tf.pcaAng,
            sagittal_sign: tf.sign,
            y_sign: tf.ySign,
            signed_area: area,
            winding: area >= 0 ? 'ccw' : 'cw',
            points: pts.map((p, localIdx) => {
                const d = processedData[p.i] || {};
                return {
                    point_index: localIdx,
                    sample_index: p.i,
                    timestamp: d.timestamp != null ? d.timestamp : null,
                    phase: (typeof phaseAtSample === 'function')
                        ? phaseAtSample(p.i)
                        : ((d.stroke_phase || d.phase) || 'idle'),
                    x: p.x,
                    y: p.y
                };
            })
        };
    });
    let sessionLabel = 'session';
    if (typeof activeSessionIdx === 'number' && activeSessionIdx >= 0 &&
        typeof savedSessions !== 'undefined' && savedSessions && savedSessions[activeSessionIdx]) {
        const s = savedSessions[activeSessionIdx];
        sessionLabel = s.filename || s.name || s.id || ('session-' + activeSessionIdx);
    }
    const safeLabel = String(sessionLabel).replace(/[^a-zA-Z0-9_.-]+/g, '_');
    const ts = new Date().toISOString().replace(/[:.]/g, '-');
    /* Surface the per-stream winding that was enforced (set by _ensureSessionWindingLock during
     * the strokes map above) so consumers can tell which direction the export was locked to. */
    const windingLock = {};
    _sagittalWindingPerStream.forEach((s, sk) => { windingLock[sk] = s >= 0 ? 'ccw' : 'cw'; });
    const payload = {
        format: 'goldenform_side_view_2d_v2',
        exported_at: new Date().toISOString(),
        session: {
            label: sessionLabel,
            sample_count: processedData.length,
            stroke_count: strokes.length,
            winding_lock: windingLock
        },
        strokes: strokes
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'side-view-2d-coordinates-' + safeLabel + '-' + ts + '.json';
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
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

    const strokePtsRaw = [];
    for (let i = b.start; i <= b.end; i++) {
        const p = getLiaPositionSample(i);
        strokePtsRaw.push({
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
    }

    /* Single cached decision per stroke -- partial playback polylines reuse the same
     * pcaAng + xSign + ySign, so the canvas cannot mirror mid-stroke even though the polyline
     * is growing sample-by-sample. ySign forces every stroke's loop winding (CCW vs CW) to
     * match the session majority so strokes all rotate in the same direction. */
    const { pcaAng, sign, ySign } = _getStrokeSagittalTransform(b);
    const strokePtsRot = _applySagittalSigns(rotatePts2D(strokePtsRaw, -pcaAng), sign, ySign);
    const otherPtsRot = _applySagittalSigns(rotatePts2D(otherPathPts, -pcaAng), sign, ySign);
    const allPts = [...strokePtsRot, ...otherPtsRot];

    const upto = Math.min(idx, b.end);
    const pathPtsRaw = [];
    for (let i = b.start; i <= upto; i++) {
        const p = getLiaPositionSample(i);
        pathPtsRaw.push({ x: p.pz - p0.pz, y: p.py - p0.py, i });
    }
    const pathPts = _applySagittalSigns(rotatePts2D(pathPtsRaw, -pcaAng), sign, ySign);

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
        const globalI = pathPts[k - 1].i;
        const d0 = processedData[globalI];
        const ph = (typeof phaseAtSample === 'function')
            ? phaseAtSample(globalI)
            : ((d0 && (d0.stroke_phase || d0.phase)) || 'idle');
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
    const phase = (typeof phaseAtSample === 'function')
        ? phaseAtSample(idx)
        : ((d && (d.stroke_phase || d.phase)) || 'idle');
    ctx.fillStyle = '#e8e8f0';
    ctx.font = '12px system-ui, sans-serif';
    const cap = multi
        ? ('Stroke #' + b.strokeNum + ' · ' + streamLabelShort(d) + ' (solid) · other band (dashed) · colors = band + phase')
        : ('Stroke #' + b.strokeNum + ' · phase: ' + phase + ' · path colors = phase (catch/pull/recovery/glide)');
    ctx.fillText(cap, 12, 18);

    ctx.fillStyle = '#9ca3af';
    ctx.font = '10px system-ui';
    ctx.fillText(
        'PCA-rotated sagittal slice (stroke-local) · vertical ≈ pull depth · smoothed trail matches 3D',
        12,
        32
    );

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
