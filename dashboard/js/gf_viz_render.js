/**
 * GoldenForm — Per-frame 3D + charts + side view + HUD updates during playback.
 */
function findNearestOtherStreamIndex(idx) {
    if (!processedData.length || idx < 0 || idx >= processedData.length) return -1;
    const sk = getStreamKey(processedData[idx]);
    const t = processedData[idx].timestamp || 0;
    let best = -1;
    let bestDt = Infinity;
    for (let i = 0; i < processedData.length; i++) {
        if (getStreamKey(processedData[i]) === sk) continue;
        const dt = Math.abs((processedData[i].timestamp || 0) - t);
        if (dt < bestDt) {
            bestDt = dt;
            best = i;
        }
    }
    return best;
}

function buildOneTrailForStream(streamKey, maxIdx, anim) {
    const rawPts = [];
    const colors = [];
    for (let i = 0; i <= maxIdx; i++) {
        if (getStreamKey(processedData[i]) !== streamKey) continue;
        const p = anim[i];
        rawPts.push(p ? p.clone() : new THREE.Vector3());
        const ph = processedData[i]?.stroke_phase || processedData[i]?.phase || 'idle';
        const c = PHASE_COLOR_RGB[ph] || PHASE_COLOR_RGB.idle;
        const rgb = streamColorRgbForKey(streamKey);
        const fade = 0.42 + 0.58 * (1 - (maxIdx - i) / Math.max(1, maxIdx));
        colors.push(
            (c[0] * 0.5 + rgb[0] * 0.5) * fade,
            (c[1] * 0.5 + rgb[1] * 0.5) * fade,
            (c[2] * 0.5 + rgb[2] * 0.5) * fade
        );
    }
    return { rawPts, colors };
}

function renderFrame(idx) {
    if (!processedData.length || idx >= processedData.length) return;
    const d = processedData[idx];

    if (integratedPositions.length !== processedData.length) integratePositions();
    const anim = currentAnimationPositions();
    const pos = anim[idx] || new THREE.Vector3();

    const ct = vizCoordinateTransform;
    const multiDevice = typeof hasMultipleDeviceStreams === 'function' && hasMultipleDeviceStreams();
    const skCur = getStreamKey(d);

    if (imuCube) {
        const showCube = document.getElementById('show-cube');
        const cubeOn = showCube ? showCube.checked : true;
        imuCube.visible = cubeOn;
        if (cubeMaterials.length) {
            const hx = multiDevice ? streamBaseHexForKey(skCur) : 0xdc2020;
            cubeMaterials.forEach(m => { m.userData.baseHex = hx; });
        }
        const qBase = nq(d.quaternion);
        imuCube.setRotationFromQuaternion(qBase);
        imuCube.rotation.x += ct.rotationX;
        imuCube.rotation.y += ct.rotationY;
        imuCube.rotation.z += ct.rotationZ;
        imuCube.position.copy(pos);
        const phase = d.stroke_phase || d.phase || 'idle';
        const phaseHex = PHASE_COLOR_HEX[phase] || PHASE_COLOR_HEX.idle;
        if (hapticFlashUntil <= Date.now()) {
            cubeMaterials.forEach(m => {
                const base = new THREE.Color(m.userData.baseHex != null ? m.userData.baseHex : 0x888888);
                const tint = new THREE.Color(phaseHex);
                m.color.copy(base.lerp(tint, 0.22));
                m.emissive.setHex(0x000000);
            });
        }
    }

    if (imuCubeB && cubeMaterialsB && cubeMaterialsB.length) {
        const showCube = document.getElementById('show-cube');
        const cubeOn = showCube ? showCube.checked : true;
        const oi = multiDevice ? findNearestOtherStreamIndex(idx) : -1;
        if (multiDevice && oi >= 0) {
            imuCubeB.visible = cubeOn;
            const d2 = processedData[oi];
            const pos2 = anim[oi];
            const hx2 = streamBaseHexForKey(getStreamKey(d2));
            cubeMaterialsB.forEach(m => { m.userData.baseHex = hx2; });
            const q2 = nq(d2.quaternion);
            imuCubeB.setRotationFromQuaternion(q2);
            imuCubeB.rotation.x += ct.rotationX;
            imuCubeB.rotation.y += ct.rotationY;
            imuCubeB.rotation.z += ct.rotationZ;
            imuCubeB.position.copy(pos2);
            const phase2 = d2.stroke_phase || d2.phase || 'idle';
            const phaseHex2 = PHASE_COLOR_HEX[phase2] || PHASE_COLOR_HEX.idle;
            if (hapticFlashUntil <= Date.now()) {
                cubeMaterialsB.forEach(m => {
                    const base = new THREE.Color(m.userData.baseHex != null ? m.userData.baseHex : 0x888888);
                    const tint = new THREE.Color(phaseHex2);
                    m.color.copy(base.lerp(tint, 0.22));
                    m.emissive.setHex(0x000000);
                });
            }
        } else {
            imuCubeB.visible = false;
        }
    }

    if (d.haptic_fired && hapticFlashUntil <= Date.now()) {
        hapticFlashUntil = Date.now() + 300;
        cubeMaterials.forEach(m => {
            m.color.setHex(0xff4444);
            m.emissive.setHex(0x440000);
        });
    }

    if (velocityArrow && idx > 0) {
        let prevSame = idx - 1;
        while (prevSame >= 0 && getStreamKey(processedData[prevSame]) !== skCur) prevSame--;
        const p0 = prevSame >= 0 ? anim[prevSame] : null;
        const p1 = pos;
        if (p0 && p1) {
            const delta = new THREE.Vector3().subVectors(p1, p0);
            const len = delta.length();
            if (len > 0.00012) {
                velocityArrow.visible = true;
                const dir = delta.multiplyScalar(1 / len);
                velocityArrow.setDirection(dir);
                const L = Math.min(0.7, len * 85);
                velocityArrow.setLength(L, Math.min(0.12, L * 0.2), Math.min(0.08, L * 0.12));
                velocityArrow.position.copy(p0);
            } else {
                velocityArrow.visible = false;
            }
        }
    } else if (velocityArrow) {
        velocityArrow.visible = false;
    }

    const followEl = document.getElementById('viz-follow-device');
    const wantFollow = followEl ? followEl.checked : false;
    if (controls && wantFollow && followDeviceInView && isPlaying) {
        controls.target.lerp(pos, 0.12);
    }

    function applyTrailGeometry(line, smoothed, colOut) {
        if (!line) return;
        if (smoothed.length >= 2) {
            const flat = [];
            smoothed.forEach(p => flat.push(p.x, p.y, p.z));
            line.geometry.setAttribute('position', new THREE.Float32BufferAttribute(flat, 3));
            line.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colOut, 3));
            line.geometry.attributes.position.needsUpdate = true;
            line.geometry.attributes.color.needsUpdate = true;
        } else {
            line.geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
        }
    }

    if (trailLine) {
        if (!multiDevice || !trailLineB) {
            if (trailLineB) trailLineB.visible = false;
            const seg0 = getSegStart(idx);
            const trailStart = Math.max(seg0, idx - TRAIL_MAX_POINTS);
            const rawPts = [];
            const colors = [];
            for (let i = trailStart; i <= idx; i++) {
                const p = anim[i];
                rawPts.push(p ? p.clone() : new THREE.Vector3());
                const ph = processedData[i]?.stroke_phase || processedData[i]?.phase || 'idle';
                const c = PHASE_COLOR_RGB[ph] || PHASE_COLOR_RGB.idle;
                const age = (idx - i) / Math.max(1, idx - trailStart);
                const fade = 0.42 + 0.58 * (1 - age);
                colors.push(c[0] * fade, c[1] * fade, c[2] * fade);
            }
            const { points: smoothed, colors: colOut } = resampleTrailFreestyle(rawPts, colors);
            applyTrailGeometry(trailLine, smoothed, colOut);
        } else {
            const streamKeys = [...new Set(processedData.map(getStreamKey))].sort();
            trailLineB.visible = true;
            if (streamKeys.length >= 2) {
                const a0 = buildOneTrailForStream(streamKeys[0], idx, anim);
                const a1 = buildOneTrailForStream(streamKeys[1], idx, anim);
                const r0 = resampleTrailFreestyle(a0.rawPts, a0.colors);
                const r1 = resampleTrailFreestyle(a1.rawPts, a1.colors);
                applyTrailGeometry(trailLine, r0.points, r0.colors);
                applyTrailGeometry(trailLineB, r1.points, r1.colors);
            } else {
                applyTrailGeometry(trailLine, [], []);
                applyTrailGeometry(trailLineB, [], []);
            }
        }
    }

    const bPlay = getPlaybackBounds();
    const pct = bPlay.end > bPlay.start ? (idx - bPlay.start) / (bPlay.end - bPlay.start) * 100 : (processedData.length > 1 ? idx / (processedData.length - 1) * 100 : 0);
    const fillEl = document.getElementById('progress-fill');
    if (fillEl) fillEl.style.width = pct + '%';
    const playheadEl = document.getElementById('progress-playhead');
    if (playheadEl) {
        playheadEl.style.left = pct + '%';
        playheadEl.style.display = processedData.length > 1 ? 'block' : 'none';
    }
    const scrubEl = document.getElementById('progress-scrubber');
    if (scrubEl && processedData.length > 1) {
        scrubEl.setAttribute('aria-valuemax', '100');
        scrubEl.setAttribute('aria-valuemin', '0');
        scrubEl.setAttribute('aria-valuenow', String(Math.round(pct)));
    }
    const t = processedData.length > 1 && d.timestamp != null && processedData[0].timestamp != null
        ? (d.timestamp - processedData[0].timestamp) / 1000 : 0;
    const total = processedData.length > 1 && processedData[processedData.length - 1].timestamp != null && processedData[0].timestamp != null
        ? (processedData[processedData.length - 1].timestamp - processedData[0].timestamp) / 1000 : 0;
    setText('play-time', formatTimeFine(t) + ' / ' + formatTimeFine(total));
    let dtMs = 0;
    if (idx > 0 && d.timestamp != null && processedData[idx - 1].timestamp != null) {
        dtMs = d.timestamp - processedData[idx - 1].timestamp;
    }
    const fineParts = [];
    if (dtMs > 0) fineParts.push('Δ ' + dtMs.toFixed(1) + ' ms (IMU)');
    if (d.haptic_fired) fineParts.push('⚡ haptic');
    setText('play-time-fine', fineParts.length ? fineParts.join(' · ') : '');
    const inSel = bPlay.end > bPlay.start ? (idx - bPlay.start + 1) + ' / ' + (bPlay.end - bPlay.start + 1) : '-';
    setText('play-frame', 'Sample ' + (idx + 1) + ' / ' + processedData.length + ' · in range ' + inSel);

    setText('live-strokes', strokeNumAt(d));
    const phase = d.stroke_phase || d.phase || 'idle';
    const phaseEl = document.getElementById('live-phase');
    if (phaseEl) {
        phaseEl.textContent = phase;
        phaseEl.className = 'phase-label phase-label-' + phase;
    }
    let liveAngle = 0, liveDeviation = 0;
    for (let i = idx; i >= Math.max(0, idx - 200); i--) {
        const p = processedData[i];
        if (liveAngle === 0 && (p.entry_angle || 0) > 0) liveAngle = p.entry_angle;
        if (liveDeviation === 0 && (p.deviation_score || 0) > 0) liveDeviation = p.deviation_score;
        if (liveAngle > 0 && liveDeviation > 0) break;
    }
    setText('live-angle', liveAngle.toFixed(1) + '°');
    const gx = d.angular_velocity?.gx || 0, gy = d.angular_velocity?.gy || 0, gz = d.angular_velocity?.gz || 0;
    const gyroNorm = Math.sqrt(gx * gx + gy * gy + gz * gz);
    setText('live-gyro', gyroNorm.toFixed(2));
    const eg = d.entry_gyro_mag;
    const entryGyroEl = document.getElementById('live-entry-gyro');
    if (entryGyroEl) {
        entryGyroEl.textContent = (eg != null && eg > 0) ? eg.toFixed(2) : '-';
    }
    setText('live-deviation', liveDeviation.toFixed(3));
    if (idx === 0 || idx === processedData.length - 1 || lastCalStripFrameIdx < 0 || Math.abs(idx - lastCalStripFrameIdx) >= 12) {
        lastCalStripFrameIdx = idx;
        updateCalibrationDisplay(d);
    }
    if (idx % 3 === 0) updateCharts(idx);
    drawSideViewViz(idx);
}

function refreshVizPlaybackUI() {
    const sel = document.getElementById('viz-stroke-playback');
    if (!sel) return;
    const n = sessionMetrics ? (sessionMetrics.stroke_count || 0) : 0;
    const prev = sel.value;
    let html = '<option value="0">All strokes</option>';
    for (let i = 1; i <= n; i++) html += '<option value="' + i + '">Stroke ' + i + ' only</option>';
    sel.innerHTML = html;
    if (prev && [...sel.options].some(o => o.value === prev)) sel.value = prev;
}

function focusVizOnDevice() {
    if (!camera || !controls || !processedData.length) return;
    if (integratedPositions.length !== processedData.length) integratePositions();
    const anim = currentAnimationPositions();
    const pos = anim[currentIndex];
    if (!pos) return;
    controls.target.copy(pos);
    camera.position.copy(pos.clone().add(new THREE.Vector3(0.7, 0.6, 0.85)));
    followDeviceInView = false;
    controls.update();
}

function frameVizOnTrail() {
    if (!camera || !controls || !processedData.length) return;
    if (integratedPositions.length !== processedData.length) integratePositions();
    const anim = currentAnimationPositions();
    if (!anim || !anim.length) return;
    const box = new THREE.Box3();
    for (let i = 0; i < anim.length; i++) {
        if (anim[i]) box.expandByPoint(anim[i]);
    }
    if (box.isEmpty()) return;
    const center = new THREE.Vector3();
    box.getCenter(center);
    const size = new THREE.Vector3();
    box.getSize(size);
    const maxDim = Math.max(size.x, size.y, size.z, 0.5);
    const dist = maxDim * 1.8 + 1.2;
    controls.target.copy(center);
    camera.position.set(center.x + dist * 0.55, center.y + dist * 0.5, center.z + dist * 0.7);
    camera.lookAt(center);
    followDeviceInView = false;
    controls.update();
}
