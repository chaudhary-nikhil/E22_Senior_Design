/**
 * GoldenForm — Build ideal-stroke samples from session data (LIA + quaternion + entry angle).
 * Consumed by gf_ideal_device.js; uses stroke field helpers from gf_viz_stroke_fields.js.
 */
function buildIdealLiaSamplesFromStroke(strokeNum, streamKey) {
    const out = [];
    if (!processedData || !processedData.length) return out;
    refreshStrokeFieldMode();
    const pushRow = (d) => {
        const L = d.lia || null;
        const acc = d.acceleration || {};
        const q = d.quaternion || {};
        out.push({
            lia_x: (L && L.x != null) ? L.x : (acc.ax || 0),
            lia_y: (L && L.y != null) ? L.y : (acc.ay || 0),
            lia_z: (L && L.z != null) ? L.z : (acc.az || 0),
            qw: q.qw != null ? q.qw : 1,
            qx: q.qx || 0,
            qy: q.qy || 0,
            qz: q.qz || 0,
            entry_angle: d.entry_angle || 0
        });
    };
    if (typeof gfGetStrokeSampleRangeLoose === 'function') {
        const range = gfGetStrokeSampleRangeLoose(strokeNum, streamKey);
        if (range && range.end >= range.start) {
            for (let i = range.start; i <= range.end; i++) pushRow(processedData[i]);
            if (out.length) return out;
        }
    }
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        if (strokeNumAt(d) !== strokeNum) continue;
        if (streamKey != null && streamKey !== '' && getStreamKey(d) !== streamKey) continue;
        pushRow(d);
    }
    return out;
}

/** Full session in sample order (LIA + quaternion + entry angle per row). */
function buildIdealLiaSamplesFromFullSession() {
    const out = [];
    if (!processedData || !processedData.length) return out;
    refreshStrokeFieldMode();
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        const L = d.lia || null;
        const acc = d.acceleration || {};
        const q = d.quaternion || {};
        out.push({
            lia_x: (L && L.x != null) ? L.x : (acc.ax || 0),
            lia_y: (L && L.y != null) ? L.y : (acc.ay || 0),
            lia_z: (L && L.z != null) ? L.z : (acc.az || 0),
            qw: q.qw != null ? q.qw : 1,
            qx: q.qx || 0,
            qy: q.qy || 0,
            qz: q.qz || 0,
            entry_angle: d.entry_angle || 0
        });
    }
    return out;
}

function averageEntryAngleForStroke(strokeNum, streamKey) {
    refreshStrokeFieldMode();
    if (typeof gfGetStrokeSampleRangeLoose === 'function') {
        const range = gfGetStrokeSampleRangeLoose(strokeNum, streamKey);
        if (range && range.end >= range.start) {
            let sum = 0, n = 0;
            for (let i = range.start; i <= range.end; i++) {
                const ea = Number(processedData[i].entry_angle) || 0;
                if (ea > 0.5) {
                    sum += ea;
                    n++;
                }
            }
            if (n) return sum / n;
        }
    }
    let sum = 0, n = 0;
    for (const d of processedData) {
        if (strokeNumAt(d) !== strokeNum) continue;
        if (streamKey != null && streamKey !== '' && getStreamKey(d) !== streamKey) continue;
        const ea = Number(d.entry_angle) || 0;
        if (ea > 0.5) {
            sum += ea;
            n++;
        }
    }
    if (n) return sum / n;
    return sessionMetrics ? (sessionMetrics.avg_entry_angle || 30) : 30;
}

function cacheIdealLocal(samples, idealEntryAngle, name, meta) {
    try {
        const m = meta && typeof meta === 'object' ? meta : {};
        localStorage.setItem(LS_IDEAL_KEY, JSON.stringify({
            samples,
            ideal_entry_angle: idealEntryAngle,
            name: name || 'Ideal',
            savedAt: Date.now(),
            sourceSessionId: m.sourceSessionId != null ? m.sourceSessionId : undefined,
            sourceSessionName: m.sourceSessionName || undefined,
            serverCreatedAt: m.serverCreatedAt != null ? m.serverCreatedAt : undefined
        }));
    } catch (e) { /* quota */ }
}
