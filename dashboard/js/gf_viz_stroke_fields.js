/**
 * GoldenForm: stroke indexing (firmware strokes vs stroke_count) and per-device stream keys.
 * Used for merged sessions: two bands can both use stroke 1,2,3… so boundaries use (device, stroke).
 */

function refreshStrokeFieldMode() {
    /* Prefer firmware `strokes` when present -- each increment corresponds to an on-device
     * `STROKE_DET: Stroke #N detected` event (the STROKE_DET monitor log is printed on the
     * same sample where `bno055_sample_t.stroke_count` increments). Python `stroke_count`
     * is a replay-side re-detection from raw IMU and only serves as a fallback for sessions
     * recorded before firmware counting existed. Mirrors the server rule in
     * wifi_session_processor._calculate_stroke_metrics (`use_fw = any(strokes>0)`). */
    const hasFw = processedData.some(d => (d.strokes || 0) > 0);
    const hasProc = processedData.some(d => (d.stroke_count || 0) > 0);
    useFwStrokesForViz = hasFw;
    if (!hasFw && hasProc) useFwStrokesForViz = false;
}

function strokeNumAt(d) {
    return useFwStrokesForViz ? (d.strokes || 0) : (d.stroke_count || 0);
}

/**
 * Single source of truth for stroke-detection edges across the dashboard.
 *
 * Walks `processedData` once and emits one entry per firmware STROKE_DET event: a sample
 * is a boundary iff `strokeNumAt(sample) > lastSeenFor(streamKey(sample))`. Crucially,
 * this does NOT emit a phantom boundary on a bare stream switch (which the old
 * `computeStrokeBoundaries` loop did via `sk !== prevSk`), so merged multi-band sessions
 * and replay-spliced streams stop producing ghost notches that nothing else agrees with.
 *
 * Returns `[{index, strokeNum, streamKey}, ...]`. O(N), pure -- safe to call from every
 * consumer (timeline, playback segments, summary, playback dropdown).
 */
function computeCanonicalStrokeBoundaries() {
    refreshStrokeFieldMode();
    const out = [];
    if (!processedData || !processedData.length) return out;
    const lastPerStream = new Map();
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        if (!d) continue;
        const sk = getStreamKey(d);
        const sn = strokeNumAt(d);
        if (sn <= 0) continue;
        const last = lastPerStream.get(sk);
        if (last === undefined || sn > last) {
            out.push({ index: i, strokeNum: sn, streamKey: sk });
        }
        lastPerStream.set(sk, sn);
    }
    return out;
}

/** Total strokes across all streams -- firmware-STROKE_DET-aligned ground truth. */
function canonicalStrokeCount() {
    return computeCanonicalStrokeBoundaries().length;
}

/**
 * Returns a stroke-phase string ('catch'|'pull'|'recovery'|'glide'|'idle') for sample `i`.
 *
 * Why this is needed: when the BNO055 drops below MIN_CAL_LEVEL the Python processor
 * short-circuits stroke-phase tracking and leaves `current_phase` at 'idle' for every
 * sample in that window. If the firmware detected strokes on-device the stroke number
 * still shows up (via `d.strokes`) but `stroke_phase` is 'idle' across the whole
 * stroke, so the side view / 3D trail render everything gray. This helper falls back
 * to a time-in-stroke heuristic that mirrors the Python phase-duration defaults
 * (catch 0-15%, pull 15-50%, recovery 50-80%, glide 80-100%). The ratios match the
 * `fallbacks` block near line 840 in simple_imu_visualizer.py.
 *
 * Returns the processor phase unchanged when it reports a meaningful phase.
 */
function phaseAtSample(i) {
    if (!processedData || i < 0 || i >= processedData.length) return 'idle';
    const d = processedData[i];
    if (!d) return 'idle';
    const reported = (d.stroke_phase || d.phase || '').toLowerCase();
    if (reported && reported !== 'idle') return reported;

    const sc = strokeNumAt(d);
    if (sc <= 0) return reported || 'idle';
    const sk = getStreamKey(d);

    /* Locate the sample range for this (stream, stroke) without relying on phase. */
    let start = i;
    for (let k = i - 1; k >= 0; k--) {
        const dk = processedData[k];
        if (!dk || getStreamKey(dk) !== sk) break;
        if (strokeNumAt(dk) === sc) start = k;
        else break;
    }
    let end = i;
    for (let k = i + 1; k < processedData.length; k++) {
        const dk = processedData[k];
        if (!dk || getStreamKey(dk) !== sk) break;
        if (strokeNumAt(dk) === sc) end = k;
        else break;
    }
    const n = end - start;
    if (n <= 0) return reported || 'idle';

    /* Prefer a timestamp-based ratio so variable sample rates (merged sessions, dropped
     * frames) don't skew the split. Fall back to index ratio if timestamps are missing. */
    let t0 = (processedData[start] && processedData[start].timestamp);
    let t1 = (processedData[end] && processedData[end].timestamp);
    let ts = (d.timestamp);
    let r;
    if (Number.isFinite(t0) && Number.isFinite(t1) && Number.isFinite(ts) && t1 > t0) {
        r = (ts - t0) / (t1 - t0);
    } else {
        r = (i - start) / n;
    }
    if (!Number.isFinite(r)) r = 0;
    if (r < 0) r = 0;
    else if (r > 1) r = 1;

    if (r < 0.15) return 'catch';
    if (r < 0.50) return 'pull';
    if (r < 0.80) return 'recovery';
    return 'glide';
}

/** Stable id for the physical band or merged session slice (for dual-wrist timelines). */
function getStreamKey(d) {
    if (!d) return '0';
    const id = d.device_id != null ? Number(d.device_id) : (d.dev_id != null ? Number(d.dev_id) : NaN);
    if (Number.isFinite(id) && id > 0) return 'hw' + id;
    if (d._origin_id != null && d._origin_id !== '') return 's' + d._origin_id;
    return '0';
}

/** Short label for UI: HW2, Sess12, or "Band". */
function streamLabelShort(d) {
    const k = getStreamKey(d);
    if (k.startsWith('hw')) return 'HW ' + k.slice(2);
    if (k.startsWith('s')) return 'S' + k.slice(1);
    return 'Band';
}

function formatStreamStrokeLabel(d, strokeNum) {
    return streamLabelShort(d) + ' #' + strokeNum;
}

function pitchDegFromThreeQuat(q) {
    if (!q) return 0;
    // pitch = asin(2*(w*y - z*x)) in standard quaternion->Euler (YXZ-ish).
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    return Math.asin(Math.max(-1, Math.min(1, sinp))) * 180 / Math.PI;
}

/** Two+ distinct bands in this session (merged or multi-sync). */
function hasMultipleDeviceStreams() {
    if (!processedData || processedData.length < 2) return false;
    const keys = new Set();
    for (const d of processedData) {
        keys.add(getStreamKey(d));
        if (keys.size > 1) return true;
    }
    return false;
}

const STREAM_VIZ_PALETTE_RGB = [
    [0.92, 0.28, 0.22],
    [0.22, 0.48, 0.95],
    [0.18, 0.72, 0.38],
    [0.95, 0.62, 0.12]
];

/** RGB 0-1 for this stream (stable for a given getStreamKey). */
function streamColorRgbForKey(sk) {
    let h = 0;
    const s = String(sk || '0');
    for (let i = 0; i < s.length; i++) h = (h * 31 + s.charCodeAt(i)) >>> 0;
    return STREAM_VIZ_PALETTE_RGB[h % STREAM_VIZ_PALETTE_RGB.length];
}

function streamBaseHexForKey(sk) {
    const [r, g, b] = streamColorRgbForKey(sk);
    return (Math.round(r * 255) << 16) | (Math.round(g * 255) << 8) | Math.round(b * 255);
}

/** First sample index for this stream’s current stroke # at global index `idx`. */
function strokeStartIndexForSample(idx) {
    if (!processedData || idx < 0 || idx >= processedData.length) return 0;
    const d = processedData[idx];
    const sk = getStreamKey(d);
    const sc = strokeNumAt(d);
    if (sc <= 0) return 0;
    let s = idx;
    for (let i = idx; i >= 0; i--) {
        const di = processedData[i];
        if (getStreamKey(di) !== sk) break;
        if (strokeNumAt(di) === sc) s = i;
        else break;
    }
    return s;
}

/**
 * Best entry angle (°) for the stroke containing `idx` — from processor at water impact,
 * relative to fusion frame; resets conceptually each stroke.
 */
function getEntryAngleForStrokeAtIndex(idx) {
    if (!processedData || idx < 0 || idx >= processedData.length) return 0;
    if (strokeNumAt(processedData[idx]) <= 0) return 0;
    const start = strokeStartIndexForSample(idx);
    const end = Math.min(processedData.length - 1, start + 120);
    // Prefer computing AoA relative to the stroke's start pose (matches "start at waterline" mental model).
    try {
        const d0 = processedData[start];
        const q0 = d0 && d0.quaternion ? nq(d0.quaternion) : null;
        if (q0) {
            const inv0 = q0.clone().invert();
            let best = 0;
            for (let i = start; i <= end && i <= idx; i++) {
                const di = processedData[i];
                if (!di || !di.quaternion) continue;
                const qi = nq(di.quaternion);
                const qRel = qi.clone().premultiply(inv0);
                const pitch = Math.abs(pitchDegFromThreeQuat(qRel));
                if (pitch > best) best = pitch;
            }
            if (best > 0.01) return best;
        }
    } catch (e) { /* fallback below */ }

    // Fallback: processor-provided entry_angle (absolute fusion frame).
    let best = 0;
    for (let i = start; i <= end && i <= idx; i++) {
        const a = Number(processedData[i].entry_angle || 0);
        if (a > 0.05 && a > best) best = a;
    }
    return best;
}
