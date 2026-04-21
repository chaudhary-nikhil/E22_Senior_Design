/**
 * GoldenForm  --  Haptic: stroke/haptic timeline DOM (firmware haptic_fired flags).
 */
// ── STROKE BOUNDARIES & HAPTIC TIMELINE ──
/** Hard cap  --  each tick is a DOM node; dense haptic flags used to create 50k+ nodes and crash the tab. */

function limitTimelineEvents(events, max) {
    if (!events || events.length <= max) return events;
    const n = events.length;
    const out = [];
    for (let k = 0; k < max; k++) {
        const i = Math.min(n - 1, Math.floor((k + 0.5) * n / max));
        out.push(events[i]);
    }
    return out;
}

function computeStrokeBoundaries() {
    hapticEvents = [];
    /* Notches come from the single canonical source (computeCanonicalStrokeBoundaries)
     * so the timeline, summary card, and checkpoint warp can never disagree. */
    const canon = (typeof computeCanonicalStrokeBoundaries === 'function')
        ? computeCanonicalStrokeBoundaries()
        : [];
    strokeBoundaries = canon.map(b => ({
        index: b.index,
        strokeNum: b.strokeNum,
        streamKey: b.streamKey,
        label: formatStreamStrokeLabel(processedData[b.index], b.strokeNum)
    }));
    for (let i = 0; i < processedData.length; i++) {
        const d = processedData[i];
        if (!d) continue;
        // Leading edge only: firmware may leave haptic_fired true across many consecutive samples.
        // One marker per buzz event keeps the timeline usable and avoids DOM / layout meltdown.
        if (d.haptic_fired && (i === 0 || !processedData[i - 1].haptic_fired)) {
            let dev = d.deviation_score || 0;
            if (dev === 0) {
                for (let j = i; j >= Math.max(0, i - 100); j--) {
                    if (processedData[j].deviation_score > 0) { dev = processedData[j].deviation_score; break; }
                }
            }
            hapticEvents.push({ index: i, deviation: dev });
        }
    }
}

function buildHapticTimeline() {
    const container = document.getElementById('haptic-timeline');
    if (!container || !processedData.length) return;
    container.innerHTML = '';
    const total = processedData.length;
    const strokeTicks = limitTimelineEvents(strokeBoundaries, MAX_STROKE_TIMELINE_MARKERS);
    const hapticTicks = limitTimelineEvents(hapticEvents, MAX_HAPTIC_TIMELINE_MARKERS);
    // Stroke boundaries as blue ticks
    strokeTicks.forEach(sb => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick stroke-tick';
        tick.style.left = (sb.index / total * 100) + '%';
        tick.title = (sb.label || ('Stroke #' + sb.strokeNum));
        container.appendChild(tick);
    });
    // Haptic events as red diamonds
    hapticTicks.forEach(he => {
        const tick = document.createElement('div');
        tick.className = 'timeline-tick haptic-tick';
        tick.style.left = (he.index / total * 100) + '%';
        tick.title = 'Haptic fired (deviation: ' + he.deviation.toFixed(3) + ')';
        container.appendChild(tick);
    });
}
