/**
 * GoldenForm — Session summary + home stats strip.
 */
// ── SESSION SUMMARY ──
function resetSessionSummaryPlaceholders() {
    const dash = '-';
    setText('sum-strokes', dash);
    setText('sum-duration', dash);
    setText('sum-rate', dash);
    setText('sum-consistency', dash);
    setText('sum-haptic', dash);
    setText('sum-samples', dash);
}

function resetHomeStatsPlaceholders() {
    const dash = '-';
    setText('home-last-strokes', dash);
    setText('home-last-rate', dash);
    setText('home-last-consistency', dash);
    setText('home-last-haptic', dash);
}

function updateSessionSummary() {
    if (!sessionMetrics || activeSessionIdx < 0 || !processedData.length) {
        resetSessionSummaryPlaceholders();
        return;
    }
    const m = sessionMetrics;
    /* Re-derive stroke count from the same canonical boundary list that drives the
     * timeline notches and playbackStrokeSegments, so the card can never disagree with
     * what's rendered on the timeline. Fall back to the frozen metric for legacy sessions
     * where processedData isn't available (shouldn't happen in the dashboard flow). */
    const canonicalStrokes = (typeof canonicalStrokeCount === 'function')
        ? canonicalStrokeCount()
        : (m.stroke_count || 0);

    setText('sum-strokes', canonicalStrokes);
    setText('sum-duration', formatTime(m.duration));
    setText('sum-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) + '/min' : '--');
    setText('sum-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '--');
    setText('sum-haptic', m.haptic_count != null ? m.haptic_count : 0);
    setText('sum-samples', processedData.length);

    if (activeSessionIdx >= 0 && savedSessions[activeSessionIdx]) {
        setText('home-last-strokes', canonicalStrokes);
        setText('home-last-rate', m.stroke_rate ? m.stroke_rate.toFixed(1) : '-');
        setText('home-last-consistency', m.consistency ? m.consistency.toFixed(0) + '%' : '-');
        setText('home-last-haptic', m.haptic_count != null ? m.haptic_count : '-');
    }
}
function setText(id, val) { const el = document.getElementById(id); if (el) el.textContent = val; }

