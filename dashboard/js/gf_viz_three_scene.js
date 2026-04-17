/**
 * GoldenForm  --  Three.js scene: wristband device, pool water surface, trail,
 * lights, orbit controls, and animation loop.
 */
function addVizAxisLabelSprites() {
    if (!scene || !window.THREE || scene.userData.gfAxisLabels) return;
    const mk = (text, color, pos) => {
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        canvas.width = 64;
        canvas.height = 64;
        ctx.fillStyle = color;
        ctx.font = 'Bold 22px system-ui,sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, 32, 32);
        const tex = new THREE.CanvasTexture(canvas);
        const mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
        const sprite = new THREE.Sprite(mat);
        sprite.position.copy(pos);
        sprite.scale.set(0.30, 0.30, 0.30);
        return sprite;
    };
    const g = new THREE.Group();
    g.add(mk('X', '#ff8888', new THREE.Vector3(1.05, 0, 0)));
    g.add(mk('Y', '#88ff88', new THREE.Vector3(0, 1.05, 0)));
    g.add(mk('Z', '#8888ff', new THREE.Vector3(0, 0, 1.05)));
    scene.add(g);
    scene.userData.gfAxisLabels = g;
}

function addPoolWaterSurface() {
    if (!scene || !window.THREE) return;
    /* y=0 = nominal water surface; IMU mesh offset so red top face meets this plane at stroke origin. */
    const waterGeo = new THREE.PlaneGeometry(16, 16, 1, 1);
    const waterMat = new THREE.MeshStandardMaterial({
        color: 0x1a6fb5, transparent: true, opacity: 0.18,
        roughness: 0.3, metalness: 0.05, side: THREE.DoubleSide,
    });
    const water = new THREE.Mesh(waterGeo, waterMat);
    water.rotation.x = -Math.PI / 2;
    water.position.y = 0;
    scene.add(water);

    const laneW = 2.5;
    const laneMat = new THREE.LineBasicMaterial({ color: 0x3a9ad9, transparent: true, opacity: 0.35 });
    for (let x = -5; x <= 5; x += laneW) {
        const pts = [new THREE.Vector3(x, 0.002, -8), new THREE.Vector3(x, 0.002, 8)];
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        scene.add(new THREE.Line(geo, laneMat));
    }
}

function createImuCube(baseHex, materialsOut) {
    if (typeof THREE === 'undefined') return null;
    const mats = materialsOut || cubeMaterials;
    mats.length = 0;
    const bodyGeo = new THREE.BoxGeometry(0.36, 0.14, 0.44);
    bodyGeo.translate(0, 0, 0);
    const mat = new THREE.MeshStandardMaterial({
        color: baseHex, roughness: 0.32, metalness: 0.12, emissive: 0x000000,
    });
    mat.userData.baseHex = baseHex;
    mats.push(mat);
    const mesh = new THREE.Mesh(bodyGeo, mat);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    const edgeGeo = new THREE.EdgesGeometry(bodyGeo);
    mesh.add(new THREE.LineSegments(edgeGeo, new THREE.LineBasicMaterial({
        color: 0xffffff, transparent: true, opacity: 0.30,
    })));

    /* Wearable band: watch-style loop you can slide your hand through. */
    try {
        const strapColor = 0x151515; // black strap (watch-like)
        const strapMat = new THREE.MeshStandardMaterial({
            color: strapColor, roughness: 0.85, metalness: 0.02,
        });

        // A single oval loop around the wrist.
        // Torus defaults to XY plane with hole axis Z; rotate so the "hand slides through" along +X.
        const majorR = 0.34;   // wrist radius (overall loop size)
        const tubeR = 0.030;  // strap thickness
        const strapGeo = new THREE.TorusGeometry(majorR, tubeR, 12, 72);
        const strapLoop = new THREE.Mesh(strapGeo, strapMat);
        // Hole axis along +X (hand slides through left↔right); keep the loop fully below the device.
        strapLoop.rotation.set(0, Math.PI / 2, 0);
        // Slight oval: thinner vertically, longer along the wrist.
        strapLoop.scale.set(1.0, 0.78, 1.28);
        // Drop lower so nothing intersects the top face.
        strapLoop.position.set(0, -0.33, 0);
        strapLoop.castShadow = true;
        strapLoop.receiveShadow = true;

        // Lugs / connectors so the body is "mounted" to the band.
        const lugGeo = new THREE.BoxGeometry(0.12, 0.06, 0.10);
        const lugL = new THREE.Mesh(lugGeo, strapMat);
        const lugR = new THREE.Mesh(lugGeo, strapMat);
        // Attach to the device sides near the underside; visually "connect" to the loop.
        lugL.position.set(-0.205, -0.11, 0);
        lugR.position.set(0.205, -0.11, 0);
        lugL.castShadow = lugR.castShadow = true;

        // Down-stems from lugs to the loop (so it looks connected, not floating).
        const stemGeo = new THREE.CylinderGeometry(0.018, 0.018, 0.22, 10);
        const stemL = new THREE.Mesh(stemGeo, strapMat);
        const stemR = new THREE.Mesh(stemGeo, strapMat);
        stemL.position.set(-0.205, -0.23, 0);
        stemR.position.set(0.205, -0.23, 0);
        stemL.castShadow = stemR.castShadow = true;

        const strap = new THREE.Group();
        strap.add(strapLoop);
        strap.add(lugL);
        strap.add(lugR);
        strap.add(stemL);
        strap.add(stemR);
        mesh.add(strap);
    } catch (e) {
        /* ignore strap build failures */
    }

    return mesh;
}

function init3D() {
    const canvas = document.getElementById('canvas3d');
    if (!canvas || !window.THREE) return;

    const clampSize = (w, h) => {
        // Prevent oversized WebGL backbuffers (can cause GPU reset / Chrome crash on some systems).
        const maxW = 1400;
        const maxH = 900;
        return [Math.min(Math.max(w, 400), maxW), Math.min(Math.max(h, 400), maxH)];
    };

    if (scene && renderer && camera) {
        let w = Math.max(canvas.clientWidth || 800, 400);
        let h = Math.max(canvas.clientHeight || 450, 400);
        [w, h] = clampSize(w, h);
        renderer.setSize(w, h, false);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        return;
    }

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a1628);
    scene.fog = new THREE.FogExp2(0x0a1628, 0.009);

    let w = Math.max(canvas.clientWidth || 800, 400);
    let h = Math.max(canvas.clientHeight || 450, 400);
    [w, h] = clampSize(w, h);
    camera = new THREE.PerspectiveCamera(50, w / h, 0.05, 200);
    camera.position.set(2.4, 2.5, 3.8);
    camera.lookAt(0, 0.1, 0.5);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(w, h, false);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.25));
    renderer.domElement.style.touchAction = 'none';
    renderer.domElement.style.cursor = 'grab';
    renderer.domElement.addEventListener('pointerdown', () => {
        renderer.domElement.style.cursor = 'grabbing';
        followDeviceInView = false;
        const fe = document.getElementById('viz-follow-device');
        if (fe) fe.checked = false;
    }, { passive: true });
    renderer.domElement.addEventListener('pointerup', () => {
        renderer.domElement.style.cursor = 'grab';
    }, { passive: true });

    if (!canvas.dataset.gfWebglListeners) {
        canvas.dataset.gfWebglListeners = '1';
        canvas.addEventListener('webglcontextlost', (e) => {
            e.preventDefault();
            if (vizRafId != null) {
                cancelAnimationFrame(vizRafId);
                vizRafId = null;
            }
            try {
                if (renderer) renderer.dispose();
            } catch (err) { /* ignore */ }
            scene = null;
            camera = null;
            renderer = null;
            controls = null;
        }, false);
        canvas.addEventListener('webglcontextrestored', () => {
            init3D();
        }, false);
    }
    if (THREE.sRGBEncoding !== undefined) renderer.outputEncoding = THREE.sRGBEncoding;

    const gh = new THREE.GridHelper(14, 28, 0x1a3354, 0x0f1f36);
    gh.position.y = -0.005;
    scene.add(gh);
    const axes = new THREE.AxesHelper(0.9);
    scene.add(axes);
    addVizAxisLabelSprites();
    addPoolWaterSurface();

    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    trailGeo.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.95, linewidth: 2,
    }));
    scene.add(trailLine);

    imuCube = createImuCube(0xdc2020, cubeMaterials);
    if (imuCube) scene.add(imuCube);

    if (!trailLineB) {
        const trailGeoB = new THREE.BufferGeometry();
        trailLineB = new THREE.Line(trailGeoB, new THREE.LineBasicMaterial({
            vertexColors: true, transparent: true, opacity: 0.85, linewidth: 2,
        }));
        trailLineB.visible = false;
        scene.add(trailLineB);
    }
    if (!imuCubeB) {
        cubeMaterialsB = [];
        imuCubeB = createImuCube(0x3b82f6, cubeMaterialsB);
        if (imuCubeB) {
            imuCubeB.visible = false;
            scene.add(imuCubeB);
        }
    }

    velocityArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.22,
        0xff8844, 0.07, 0.045
    );
    velocityArrow.visible = false;
    scene.add(velocityArrow);

    scene.add(new THREE.HemisphereLight(0x8ec8f0, 0x1a3050, 0.55));
    scene.add(new THREE.AmbientLight(0x607090, 0.25));
    const dl = new THREE.DirectionalLight(0xfff5e6, 1.1);
    dl.position.set(4, 8, 4);
    scene.add(dl);
    const dl2 = new THREE.DirectionalLight(0x99bbff, 0.4);
    dl2.position.set(-4, 3, -5);
    scene.add(dl2);
    const rim = new THREE.DirectionalLight(0x4488cc, 0.2);
    rim.position.set(0, -3, 6);
    scene.add(rim);

    if (window.THREE && THREE.OrbitControls) {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.08;
        controls.enableRotate = true;
        controls.enableZoom = true;
        controls.enablePan = true;
        controls.screenSpacePanning = false;
        controls.target.set(0, 0.1, 0.5);
        controls.minDistance = 0.25;
        controls.maxDistance = 28;
        controls.minPolarAngle = 0.001;
        controls.maxPolarAngle = Math.PI - 0.001;
        controls.rotateSpeed = 0.85;
        controls.zoomSpeed = 0.9;
    }

    /* Do not start RAF here when the Session page is hidden (e.g. load on Home). Running WebGL
     * every frame on a display:none canvas has triggered GPU resets / Chrome tab crashes on some Macs.
     * switchTab('session') / startVizLoop() drives the loop once the canvas is visible. */
    if (typeof currentTab !== 'undefined' && currentTab === 'session') {
        startVizLoop();
    }
}

function stopVizLoop() {
    if (vizRafId != null) {
        cancelAnimationFrame(vizRafId);
        vizRafId = null;
    }
}

function startVizLoop() {
    if (vizRafId != null) return;
    if (!renderer || !scene || !camera) return;
    vizRafId = requestAnimationFrame(animate);
}

function animate() {
    if (!renderer || !scene || !camera) {
        vizRafId = null;
        return;
    }
    if (imuCube && hapticFlashUntil > 0 && Date.now() > hapticFlashUntil) {
        const resetMats = (arr) => {
            arr.forEach(m => {
                if (m.userData.baseHex != null) m.color.setHex(m.userData.baseHex);
                m.emissive.setHex(0x000000);
            });
        };
        resetMats(cubeMaterials);
        if (cubeMaterialsB && cubeMaterialsB.length) resetMats(cubeMaterialsB);
        hapticFlashUntil = 0;
    }
    if (controls) controls.update();
    if (renderer && scene && camera) renderer.render(scene, camera);
    vizRafId = requestAnimationFrame(animate);
}
