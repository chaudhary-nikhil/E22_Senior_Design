/**
 * GoldenForm  --  Viz playback controls, clear session, camera reset, position scale.
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
    vizBaseQuatInv = null;
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
        camera.position.set(2.4, 2.5, 3.8);
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

function autoScaleStroke() {
    if (!processedData || !processedData.length) return;
    refreshStrokeFieldMode();
    const idx = Math.max(0, Math.min(currentIndex || 0, processedData.length - 1));
    const d0 = processedData[idx] || {};
    const sk = typeof getStreamKey === 'function' ? getStreamKey(d0) : '0';
    const sc = typeof strokeNumAt === 'function' ? strokeNumAt(d0) : 0;
    if (sc <= 0) return;
    // Ensure we have current integrated positions for bbox estimation.
    if (typeof integratePositions === 'function' &&
        (!integratedPositions || integratedPositions.length !== processedData.length)) {
        integratePositions();
    }
    // Find stroke bounds in the timeline (same stream, same stroke #).
    let start = idx;
    for (let i = idx - 1; i >= 0; i--) {
        const di = processedData[i];
        if (!di) break;
        if ((typeof getStreamKey === 'function' ? getStreamKey(di) : '0') !== sk) break;
        if ((typeof strokeNumAt === 'function' ? strokeNumAt(di) : 0) === sc) start = i;
        else break;
    }
    let end = idx;
    for (let j = idx + 1; j < processedData.length; j++) {
        const dj = processedData[j];
        if (!dj) break;
        if ((typeof getStreamKey === 'function' ? getStreamKey(dj) : '0') !== sk) break;
        if ((typeof strokeNumAt === 'function' ? strokeNumAt(dj) : 0) === sc) end = j;
        else break;
    }

    // Compute extents from the *actual 3D points* (already scaled & stroke-origin reset).
    const box = (typeof THREE !== 'undefined' && THREE.Box3) ? new THREE.Box3() : null;
    if (!box || !integratedPositions || integratedPositions.length !== processedData.length) return;
    for (let i = start; i <= end; i++) {
        const v = integratedPositions[i];
        if (v) box.expandByPoint(v);
    }
    if (box.isEmpty()) return;
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);
    const maxDim = Math.max(size.x, size.y, size.z, 0.05);

    // Scale so the stroke spans a comfortable fraction of the grid/camera view.
    // Grid is ~14 wide; targeting ~7 makes it visible without clipping.
    const targetSpan = 7.0;
    const factor = targetSpan / maxDim;
    const newScale = Math.max(0.5, Math.min(30, (Number(positionScale) || 1) * factor));
    positionScale = newScale;
    setText('scale-val', positionScale.toFixed(1) + 'x');
    const slider = document.getElementById('viz-scale');
    if (slider) slider.value = String(positionScale);
    integratedPositions = [];
    rawIntegratedPositions = [];
    positionStreamPositions = [];
    if (typeof integratePositions === 'function') integratePositions();

    // Reframe camera to the (newly scaled) stroke so it doesn't end up off-map.
    if (typeof frameVizOnStroke === 'function') {
        frameVizOnStroke(start, end);
    } else if (typeof frameVizOnTrail === 'function') {
        frameVizOnTrail();
    } else if (camera && controls) {
        controls.target.copy(center);
        camera.position.set(center.x + 3, center.y + 2.2, center.z + 3.2);
        camera.lookAt(center);
        controls.update();
    }
}

function frameVizOnStroke(startIdx, endIdx) {
    if (!camera || !controls || !processedData || !processedData.length) return;
    if (typeof integratePositions === 'function' &&
        (!integratedPositions || integratedPositions.length !== processedData.length)) {
        integratePositions();
    }
    if (!integratedPositions || integratedPositions.length !== processedData.length) return;
    const box = new THREE.Box3();
    const s = Math.max(0, Math.min(startIdx || 0, processedData.length - 1));
    const e = Math.max(s, Math.min(endIdx || s, processedData.length - 1));
    for (let i = s; i <= e; i++) {
        const v = integratedPositions[i];
        if (v) box.expandByPoint(v);
    }
    if (box.isEmpty()) return;
    const center = new THREE.Vector3();
    const size = new THREE.Vector3();
    box.getCenter(center);
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z, 0.4);
    const dist = maxDim * 1.25 + 1.1;
    controls.target.copy(center);
    camera.position.set(center.x + dist * 0.55, center.y + dist * 0.52, center.z + dist * 0.72);
    camera.lookAt(center);
    followDeviceInView = false;
    controls.update();
}
