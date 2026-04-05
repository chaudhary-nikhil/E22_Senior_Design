/**
 * GoldenForm — Three.js scene: IMU cube, trail, lights, orbit controls, animation loop.
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
        sprite.scale.set(0.35, 0.35, 0.35);
        return sprite;
    };
    const g = new THREE.Group();
    g.add(mk('+X', '#ff6666', new THREE.Vector3(1.15, 0, 0)));
    g.add(mk('+Y', '#66ff66', new THREE.Vector3(0, 1.15, 0)));
    g.add(mk('+Z', '#6666ff', new THREE.Vector3(0, 0, 1.15)));
    scene.add(g);
    scene.userData.gfAxisLabels = g;
}

function createImuCube(baseHex, materialsOut) {
    if (typeof THREE === 'undefined') return null;
    const mats = materialsOut || cubeMaterials;
    mats.length = 0;
    const geometry = new THREE.BoxGeometry(0.42, 0.42, 0.11);
    /* Device body; phase tint applied in renderFrame */
    const mat = new THREE.MeshStandardMaterial({
        color: baseHex, roughness: 0.36, metalness: 0.07, emissive: 0x000000
    });
    mat.userData.baseHex = baseHex;
    mats.push(mat);
    const mesh = new THREE.Mesh(geometry, mat);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    const wire = new THREE.WireframeGeometry(geometry);
    mesh.add(new THREE.LineSegments(wire, new THREE.LineBasicMaterial({
        color: 0xffffff, transparent: true, opacity: 0.42
    })));
    return mesh;
}

function init3D() {
    const canvas = document.getElementById('canvas3d');
    if (!canvas || !window.THREE) return;

    if (scene && renderer && camera) {
        const w = Math.max(canvas.clientWidth || 800, 400);
        const h = Math.max(canvas.clientHeight || 450, 400);
        renderer.setSize(w, h);
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        return;
    }

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x12121a);
    scene.fog = new THREE.FogExp2(0x12121a, 0.012);

    const w = Math.max(canvas.clientWidth || 800, 400);
    const h = Math.max(canvas.clientHeight || 450, 400);
    camera = new THREE.PerspectiveCamera(55, w / h, 0.05, 200);
    camera.position.set(2.1, 2.15, 3.4);
    camera.lookAt(0, 0.1, 0.5);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(w, h);
    /* Cap DPR — very high ratios + WebGL + reload races can stress GPU memory in Chrome */
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.75));
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

    const gh = new THREE.GridHelper(14, 28, 0x3a3a48, 0x1e1e28);
    gh.position.y = -0.65;
    scene.add(gh);
    const axes = new THREE.AxesHelper(1.1);
    scene.add(axes);
    addVizAxisLabelSprites();

    const trailGeo = new THREE.BufferGeometry();
    trailGeo.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    trailGeo.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.93, linewidth: 2
    }));
    scene.add(trailLine);

    imuCube = createImuCube(0xdc2020, cubeMaterials);
    if (imuCube) scene.add(imuCube);

    if (!trailLineB) {
        const trailGeoB = new THREE.BufferGeometry();
        trailLineB = new THREE.Line(trailGeoB, new THREE.LineBasicMaterial({
            vertexColors: true, transparent: true, opacity: 0.85, linewidth: 2
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

    scene.add(new THREE.AmbientLight(0x6688aa, 0.45));
    const dl = new THREE.DirectionalLight(0xfff5e6, 1.05);
    dl.position.set(4, 7, 3.5);
    scene.add(dl);
    const dl2 = new THREE.DirectionalLight(0xaabbff, 0.35);
    dl2.position.set(-3.5, 3, -4);
    scene.add(dl2);
    const rim = new THREE.DirectionalLight(0x4488cc, 0.25);
    rim.position.set(0, -2, 6);
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
