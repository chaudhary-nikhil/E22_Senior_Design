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

    /* Wearable band: connect from both sides and wrap downward (more like a real wrist strap). */
    const strapMat = new THREE.MeshStandardMaterial({
        color: 0x444444, roughness: 0.92, metalness: 0.02,
    });
    const strap = new THREE.Group();

    // Under-band plate (sits under the device like a real strap section)
    const plateGeo = new THREE.BoxGeometry(0.62, 0.06, 0.52);
    const plate = new THREE.Mesh(plateGeo, strapMat);
    plate.position.set(0, -0.16, 0);
    plate.castShadow = true;
    plate.receiveShadow = true;

    // Side drops (left/right)
    const sideGeo = new THREE.BoxGeometry(0.06, 0.24, 0.36);
    const leftSide = new THREE.Mesh(sideGeo, strapMat);
    leftSide.position.set(-0.21, -0.18, 0);
    leftSide.castShadow = true;
    const rightSide = new THREE.Mesh(sideGeo, strapMat);
    rightSide.position.set(0.21, -0.18, 0);
    rightSide.castShadow = true;

    // Bottom wrap (half-torus): vertical arc like a wrist strap, not flat on the box face
    const wrapGeo = new THREE.TorusGeometry(0.24, 0.032, 10, 36, Math.PI);
    const wrap = new THREE.Mesh(wrapGeo, strapMat);
    wrap.rotation.set(0, Math.PI / 2, Math.PI);
    wrap.position.set(0, -0.30, 0);
    wrap.castShadow = true;

    strap.add(plate);
    strap.add(leftSide);
    strap.add(rightSide);
    strap.add(wrap);
    strap.position.set(0, 0.02, 0);
    mesh.add(strap);

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
