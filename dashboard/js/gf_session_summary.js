/**
 * GoldenForm — Session summary + home stats strip.
 */
// ── SESSION SUMMARY ──
function resetSessionSummaryPlaceholders() {
    const dash = '-';
    setText('sum-strokes', dash);
    setText('sum-turns', dash);
    setText('sum-distance', dash);
    setText('sum-pace', dash);
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
    setText('home-last-distance', dash);
}

function updateSessionSummary() {
    if (!sessionMetrics || activeSessionIdx < 0 || !processedData.length) {
        resetSessionSummaryPlaceholders();
        return;
    }
    const m = sessionMetrics;
    const poolLen = userProfile ? (userProfile.pool_length || 25) : 25;
    /* Re-derive stroke count from the same canonical boundary list that drives the
     * timeline notches and playbackStrokeSegments, so the card can never disagree with
     * what's rendered on the timeline. Fall back to the frozen metric for legacy sessions
     * where processedData isn't available (shouldn't happen in the dashboard flow). */
    const canonicalStrokes = (typeof canonicalStrokeCount === 'function')
        ? canonicalStrokeCount()
        : (m.stroke_count || 0);
    const totalDist = (m.turn_count || 0) > 0 ? (m.turn_count + 1) * poolLen : (canonicalStrokes > 0 ? poolLen : 0);

    const pace = totalDist > 0 ? (m.duration / (totalDist / 100)) : 0; // seconds per 100m

    setText('sum-strokes', canonicalStrokes);
    setText('sum-turns', m.turn_count || 0);
    setText('sum-distance', totalDist + 'm');
    setText('sum-pace', pace > 0 ? formatTime(pace) + '/100m' : '--:--');
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
        setText('home-last-distance', totalDist + 'm');
    }
}
function setText(id, val) { const el = document.getElementById(id); if (el) el.textContent = val; }

