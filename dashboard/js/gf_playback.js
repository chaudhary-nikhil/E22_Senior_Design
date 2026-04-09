/**
 * GoldenForm — Playback: scrubber, play/pause, frame step, time formatters.
 */
// ── PLAYBACK ──
function getPlaybackBounds() {
    if (!processedData.length) return { start: 0, end: 0 };
    if (playbackStrokeFilter <= 0) return { start: 0, end: processedData.length - 1 };
    const seg = playbackStrokeSegments.find(s => s.strokeNum === playbackStrokeFilter);
    if (!seg) return { start: 0, end: processedData.length - 1 };
    return { start: seg.startIdx, end: seg.endIdx };
}

function togglePlayback() {
    if (!processedData.length) return;
    isPlaying = !isPlaying;
    const btn = document.getElementById('play-btn');
    if (isPlaying) {
        if (btn) btn.textContent = '⏸';
        if (playbackInterval) clearInterval(playbackInterval);
        const tick = () => {
            if (!processedData.length) return;
            const b = getPlaybackBounds();
            if (currentIndex < b.end) {
                currentIndex++;
            } else {
                if (playbackStrokeFilter > 0 && loopStrokePlayback) {
                    currentIndex = b.start;
                } else if (playbackStrokeFilter <= 0 && loopFullSession && processedData.length > 1) {
                    currentIndex = b.start;
                } else {
                    isPlaying = false;
                    if (btn) btn.textContent = '▶';
                    if (playbackInterval) clearInterval(playbackInterval);
                    playbackInterval = null;
                    return;
                }
            }
            renderFrame(currentIndex);
        };
        const spd = Math.max(0.05, Math.min(4, playbackSpeedMultiplier || 0.35));
        playbackInterval = setInterval(tick, PLAYBACK_BASE_MS / spd);
    } else {
        if (btn) btn.textContent = '▶';
        if (playbackInterval) clearInterval(playbackInterval);
        playbackInterval = null;
    }
}
function resetPlayback() {
    const b = getPlaybackBounds();
    currentIndex = b.start;
    isPlaying = false;
    if (playbackInterval) clearInterval(playbackInterval);
    playbackInterval = null;
    const btn = document.getElementById('play-btn');
    if (btn) btn.textContent = '▶';
    renderFrame(currentIndex);
}

/** Jump to range start and start playing (unlike reset, which stays paused). */
function replayFromStart() {
    if (!processedData.length) return;
    resetPlayback();
    togglePlayback();
}
function skipForward() {
    const b = getPlaybackBounds();
    currentIndex = Math.min(currentIndex + 30, b.end);
    renderFrame(currentIndex);
}
function skipBackward() {
    const b = getPlaybackBounds();
    currentIndex = Math.max(currentIndex - 30, b.start);
    renderFrame(currentIndex);
}

/** Seek scrubber from screen X (click or drag). */
function seekPlaybackFromPointer(clientX, trackEl) {
    const b = getPlaybackBounds();
    if (!trackEl || b.end <= b.start) return;
    const rect = trackEl.getBoundingClientRect();
    const w = rect.width || 1;
    const pct = Math.max(0, Math.min(1, (clientX - rect.left) / w));
    currentIndex = Math.round(b.start + pct * (b.end - b.start));
    currentIndex = Math.max(b.start, Math.min(b.end, currentIndex));
    renderFrame(currentIndex);
}

function seekPlayback(e) {
    seekPlaybackFromPointer(e.clientX, e.currentTarget);
}

/** Step by IMU samples (±1 = one timestep). Hold Shift in keyboard for ±10. */
function stepFrame(delta) {
    if (!processedData.length) return;
    const b = getPlaybackBounds();
    currentIndex = Math.max(b.start, Math.min(b.end, currentIndex + delta));
    renderFrame(currentIndex);
}

function initScrubberPointerHandlers() {
    const track = document.getElementById('progress-scrubber');
    if (!track || track.dataset.scrubBound === '1') return;
    track.dataset.scrubBound = '1';
    let activePointer = null;
    track.addEventListener('pointerdown', (e) => {
        if (e.button !== 0) return;
        e.preventDefault();
        track.focus({ preventScroll: true });
        track.setPointerCapture(e.pointerId);
        activePointer = e.pointerId;
        track.classList.add('scrubbing');
        seekPlaybackFromPointer(e.clientX, track);
    });
    track.addEventListener('pointermove', (e) => {
        if (activePointer !== e.pointerId) return;
        seekPlaybackFromPointer(e.clientX, track);
    });
    track.addEventListener('pointerup', (e) => {
        if (activePointer === e.pointerId) {
            try { track.releasePointerCapture(e.pointerId); } catch (err) { /* */ }
            activePointer = null;
        }
        track.classList.remove('scrubbing');
    });
    track.addEventListener('pointercancel', () => {
        activePointer = null;
        track.classList.remove('scrubbing');
    });
    track.addEventListener('keydown', (e) => {
        if (!processedData.length) return;
        const n = e.shiftKey ? 10 : 1;
        if (e.key === 'ArrowLeft' || e.key === 'ArrowDown') {
            stepFrame(-n);
            e.preventDefault();
        } else if (e.key === 'ArrowRight' || e.key === 'ArrowUp') {
            stepFrame(n);
            e.preventDefault();
        } else if (e.key === 'Home') {
            resetPlayback();
            e.preventDefault();
        }
    });
}
function jumpToStroke(n, streamKey) {
    let sb = null;
    if (streamKey != null && streamKey !== '') {
        sb = strokeBoundaries.find(s => s.strokeNum === n && s.streamKey === streamKey);
    }
    if (!sb) sb = strokeBoundaries.find(s => s.strokeNum === n);
    if (sb) { currentIndex = sb.index; renderFrame(currentIndex); }
}
function formatTime(s) {
    if (!s || !isFinite(s)) return '0:00';
    const m = Math.floor(s / 60), sec = Math.floor(s % 60);
    return m + ':' + String(sec).padStart(2, '0');
}

/** Wall-clock style with centiseconds (good for slow scrubbing). */
function formatTimeFine(s) {
    if (!isFinite(s) || s < 0) return '0:00.00';
    const m = Math.floor(s / 60);
    const sec = s - m * 60;
    const secStr = (sec < 10 ? '0' : '') + sec.toFixed(2);
    return m + ':' + secStr;
}

