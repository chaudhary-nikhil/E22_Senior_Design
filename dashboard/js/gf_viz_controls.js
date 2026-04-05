/**
 * GoldenForm — Viz playback controls, clear session, camera reset, position scale.
 */
function bindVizPlaybackControls() {
    const sp = document.getElementById('viz-playback-speed');
    if (sp) {
        playbackSpeedMultiplier = Math.max(0.05, Math.min(4, parseFloat(sp.value) || 0.35));
        const lab0 = document.getElementById('viz-speed-label');
        if (lab0) lab0.textContent = playbackSpeedMultiplier.toFixed(2) + '×';
        sp.addEventListener('input', () => {
            playbackSpeedMultiplier = Math.max(0.05, Math.min(4, parseFloat(sp.value) || 0.35));
            const lab = document.getElementById('viz-speed-label');
            if (lab) lab.textContent = playbackSpeedMultiplier.toFixed(2) + '×';
            if (isPlaying) {
                togglePlayback();
                togglePlayback();
            }
        });
    }
    const strokeSel = document.getElementById('viz-stroke-playback');
    if (strokeSel) {
        strokeSel.addEventListener('change', () => {
            playbackStrokeFilter = parseInt(strokeSel.value, 10) || 0;
            resetPlayback();
        });
    }
    const loopEl = document.getElementById('viz-loop-stroke');
    if (loopEl) {
        loopEl.addEventListener('change', () => { loopStrokePlayback = loopEl.checked; });
    }
    const loopFull = document.getElementById('viz-loop-full');
    if (loopFull) {
        loopFull.addEventListener('change', () => { loopFullSession = loopFull.checked; });
    }
    const followCb = document.getElementById('viz-follow-device');
    if (followCb && !followCb.dataset.bound) {
        followCb.dataset.bound = '1';
        followCb.addEventListener('change', () => {
            followDeviceInView = !!followCb.checked;
        });
    }
}

function resetLivePlaybackHud() {
    if (typeof setText !== 'function') return;
    setText('live-strokes', '-');
    const phaseEl = document.getElementById('live-phase');
    if (phaseEl) {
        phaseEl.textContent = '-';
        phaseEl.className = 'phase-label';
    }
    setText('live-angle', '-');
    setText('live-gyro', '-');
    setText('live-deviation', '-');
}

function clearViz() {
    processedData = [];
    sessionMetrics = null;
    currentIndex = 0;
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    hapticFlashUntil = 0;
    refreshStrokeFieldMode();
    playbackStrokeSegments = [];
    if (trailLine && trailLine.geometry) {
        trailLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        trailLine.geometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    }
    if (trailLineB && trailLineB.geometry) {
        trailLineB.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        trailLineB.geometry.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
        trailLineB.visible = false;
    }
    if (imuCube) {
        imuCube.position.set(0, 0, 0);
        imuCube.rotation.set(0, 0, 0);
        imuCube.quaternion.identity();
        cubeMaterials.forEach(m => {
            if (m.userData.baseHex != null) m.color.setHex(m.userData.baseHex);
            m.emissive.setHex(0x000000);
        });
    }
    if (imuCubeB) {
        imuCubeB.position.set(0, 0, 0);
        imuCubeB.rotation.set(0, 0, 0);
        imuCubeB.quaternion.identity();
        imuCubeB.visible = false;
        if (cubeMaterialsB) {
            cubeMaterialsB.forEach(m => {
                if (m.userData.baseHex != null) m.color.setHex(m.userData.baseHex);
                m.emissive.setHex(0x000000);
            });
        }
    }
    resetSessionSummaryPlaceholders();
    setText('play-time', '0:00.00 / 0:00.00');
    setText('play-time-fine', '');
    setText('play-frame', '-');
    setText('live-entry-gyro', '-');
    const fillEl = document.getElementById('progress-fill');
    if (fillEl) fillEl.style.width = '0%';
    const ph = document.getElementById('progress-playhead');
    if (ph) ph.style.display = 'none';
    clearSideViewCanvas();
    if (typeof updateSessionCalBanner === 'function') updateSessionCalBanner();
    if (typeof refreshIdealViewerContext === 'function') refreshIdealViewerContext();
    if (typeof resetAnalysisPlaceholders === 'function') resetAnalysisPlaceholders();
    if (typeof updateCoachingInsights === 'function') updateCoachingInsights();
    resetLivePlaybackHud();
}

function resetView() {
    followDeviceInView = false;
    if (camera) {
        camera.position.set(2.1, 2.15, 3.4);
        camera.lookAt(0, 0.1, 0.5);
        if (controls) { controls.target.set(0, 0.1, 0.5); controls.update(); }
    }
}

function updateScale(v) {
    positionScale = parseFloat(v) || 3;
    setText('scale-val', positionScale.toFixed(1) + 'x');
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
}
