/**
 * GoldenForm: stroke indexing (firmware strokes vs stroke_count) and per-device stream keys.
 * Used for merged sessions: two bands can both use stroke 1,2,3… so boundaries use (device, stroke).
 */

function refreshStrokeFieldMode() {
    /* Prefer Python stroke_count when present — it aligns with impact detection + integration.
       Firmware `strokes` alone can disagree and breaks per-stroke origin / playback bounds. */
    const hasProc = processedData.some(d => (d.stroke_count || 0) > 0);
    const hasFw = processedData.some(d => (d.strokes || 0) > 0);
    useFwStrokesForViz = hasFw && !hasProc;
}

function strokeNumAt(d) {
    return useFwStrokesForViz ? (d.strokes || 0) : (d.stroke_count || 0);
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
