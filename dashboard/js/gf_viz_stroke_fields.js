/**
 * GoldenForm: stroke indexing (firmware strokes vs stroke_count) and per-device stream keys.
 * Used for merged sessions: two bands can both use stroke 1,2,3… so boundaries use (device, stroke).
 */

function refreshStrokeFieldMode() {
    useFwStrokesForViz = processedData.some(d => (d.strokes || 0) > 0);
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
