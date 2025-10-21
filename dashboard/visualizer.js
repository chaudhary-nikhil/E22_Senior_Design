// BNO055 Session Visualizer - Fast JSON Data Processing
// Optimized for real-time playback of logged IMU data

class SessionVisualizer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.imuCube = null;
        this.sessionData = [];
        this.currentPlaybackIndex = 0;
        this.playbackInterval = null;
        this.isPlaying = false;
        this.selectedFile = null;
        
        // Chart instances
        this.accelChart = null;
        this.gyroChart = null;
        this.magChart = null;
        
        // Note: No position tracking - will use GPS later for physical position
        
        // Swimming motion analysis
        this.strokeDetection = {
            lastStrokeTime: 0,
            strokeCount: 0,
            strokeFrequency: 0
        };
        
        this.init();
    }

    // Note: Motion trail removed - focusing on rotation only, GPS will handle position later
    
    // Detect swimming strokes from gyroscope data
    detectStroke(data) {
        const gyroMagnitude = Math.sqrt(data.gx * data.gx + data.gy * data.gy + data.gz * data.gz);
        const currentTime = data.session_time || 0;
        
        // Simple stroke detection based on gyroscope magnitude peaks
        if (gyroMagnitude > 0.5 && (currentTime - this.strokeDetection.lastStrokeTime) > 1.0) {
            this.strokeDetection.strokeCount++;
            this.strokeDetection.lastStrokeTime = currentTime;
            
            // Calculate stroke frequency
            if (currentTime > 0) {
                this.strokeDetection.strokeFrequency = this.strokeDetection.strokeCount / currentTime;
            }
            
            console.log(`🏊 Stroke detected! Count: ${this.strokeDetection.strokeCount}, Frequency: ${this.strokeDetection.strokeFrequency.toFixed(2)} strokes/sec`);
        }
    }
    
    // Create axis labels for better orientation understanding
    createAxisLabels() {
        const group = new THREE.Group();
        
        const createTextSprite = (text, color, position) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 64;
            canvas.height = 64;
            
            context.fillStyle = color;
            context.font = 'Bold 24px Arial';
            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.fillText(text, 32, 32);
            
            const texture = new THREE.CanvasTexture(canvas);
            const material = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(material);
            sprite.position.copy(position);
            sprite.scale.set(1, 1, 1);
            
            return sprite;
        };
        
        // Add axis labels
        group.add(createTextSprite('+X', '#ff0000', new THREE.Vector3(3, 0, 0)));
        group.add(createTextSprite('-X', '#00ff00', new THREE.Vector3(-3, 0, 0)));
        group.add(createTextSprite('+Y', '#0000ff', new THREE.Vector3(0, 3, 0)));
        group.add(createTextSprite('-Y', '#ffff00', new THREE.Vector3(0, -3, 0)));
        group.add(createTextSprite('+Z', '#ff00ff', new THREE.Vector3(0, 0, 3)));
        group.add(createTextSprite('-Z', '#00ffff', new THREE.Vector3(0, 0, -3)));
        
        return group;
    }

    // Initialize charts for fast data visualization
    initCharts() {
        const commonOptions = {
            responsive: true,
            maintainAspectRatio: false,
            animation: false, // Disable animations for speed
            interaction: {
                mode: 'index',
                intersect: false
            },
            scales: {
                x: {
                    ticks: { 
                        color: '#fff',
                        maxRotation: 45,
                        minRotation: 45,
                        autoSkip: true,
                        maxTicksLimit: 10
                    },
                    grid: { color: '#444' }
                },
                y: {
                    ticks: { color: '#fff' },
                    grid: { color: '#444' }
                }
            },
            plugins: {
                legend: {
                    labels: { color: '#fff' }
                }
            },
            layout: {
                padding: { bottom: 10 }
            }
        };

        // Acceleration chart
        const accelCtx = document.getElementById('accelChart').getContext('2d');
        this.accelChart = new Chart(accelCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { 
                        label: 'X', 
                        data: [], 
                        borderColor: 'rgb(255, 99, 132)', 
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Y', 
                        data: [], 
                        borderColor: 'rgb(54, 162, 235)', 
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Z', 
                        data: [], 
                        borderColor: 'rgb(75, 192, 192)', 
                        backgroundColor: 'rgba(75, 192, 192, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        ...commonOptions.scales.y, 
                        min: -2, 
                        max: 12,
                        ticks: {
                            color: '#fff',
                            stepSize: 2
                        }
                    }
                }
            }
        });

        // Gyroscope chart
        const gyroCtx = document.getElementById('gyroChart').getContext('2d');
        this.gyroChart = new Chart(gyroCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { 
                        label: 'X', 
                        data: [], 
                        borderColor: 'rgb(255, 159, 64)', 
                        backgroundColor: 'rgba(255, 159, 64, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Y', 
                        data: [], 
                        borderColor: 'rgb(153, 102, 255)', 
                        backgroundColor: 'rgba(153, 102, 255, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Z', 
                        data: [], 
                        borderColor: 'rgb(201, 203, 207)', 
                        backgroundColor: 'rgba(201, 203, 207, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        ...commonOptions.scales.y, 
                        min: -1, 
                        max: 1,
                        ticks: {
                            color: '#fff',
                            stepSize: 0.25
                        }
                    }
                }
            }
        });

        // Magnetometer chart
        const magCtx = document.getElementById('magChart').getContext('2d');
        this.magChart = new Chart(magCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    { 
                        label: 'X', 
                        data: [], 
                        borderColor: 'rgb(255, 99, 132)', 
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Y', 
                        data: [], 
                        borderColor: 'rgb(54, 162, 235)', 
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    },
                    { 
                        label: 'Z', 
                        data: [], 
                        borderColor: 'rgb(75, 192, 192)', 
                        backgroundColor: 'rgba(75, 192, 192, 0.1)',
                        tension: 0.4,
                        pointRadius: 0,
                        borderWidth: 2
                    }
                ]
            },
            options: {
                ...commonOptions,
                scales: {
                    ...commonOptions.scales,
                    y: { 
                        ...commonOptions.scales.y, 
                        min: -50, 
                        max: 50,
                        ticks: {
                            color: '#fff',
                            stepSize: 10
                        }
                    }
                }
            }
        });
    }

    // Initialize Three.js scene for fast rendering
    init() {
        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a1a);

        // Camera
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.camera.position.set(0, 0, 8);

        // Renderer with optimized settings
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        document.getElementById('container').appendChild(this.renderer.domElement);

        // Create optimized IMU cube
        const geometry = new THREE.BoxGeometry(2, 2, 0.5);
        
        const materials = [
            new THREE.MeshPhongMaterial({ color: 0xff0000, shininess: 100, specular: 0x222222 }), // +X (Red)
            new THREE.MeshPhongMaterial({ color: 0x00ff00, shininess: 100, specular: 0x222222 }), // -X (Green)  
            new THREE.MeshPhongMaterial({ color: 0x0000ff, shininess: 100, specular: 0x222222 }), // +Y (Blue)
            new THREE.MeshPhongMaterial({ color: 0xffff00, shininess: 100, specular: 0x222222 }), // -Y (Yellow)
            new THREE.MeshPhongMaterial({ color: 0xff00ff, shininess: 100, specular: 0x222222 }), // +Z (Magenta)
            new THREE.MeshPhongMaterial({ color: 0x00ffff, shininess: 100, specular: 0x222222 })  // -Z (Cyan)
        ];
        
        this.imuCube = new THREE.Mesh(geometry, materials);
        this.imuCube.castShadow = true;
        this.imuCube.receiveShadow = true;
        this.imuCube.position.set(0, 0, 0);
        this.scene.add(this.imuCube);

        // Add wireframe for better visibility
        const wireframe = new THREE.WireframeGeometry(geometry);
        const line = new THREE.LineSegments(wireframe, new THREE.LineBasicMaterial({ 
            color: 0xffffff, 
            transparent: true, 
            opacity: 0.4,
            linewidth: 2
        }));
        this.imuCube.add(line);

        // Add axis labels
        const axisLabels = this.createAxisLabels();
        this.scene.add(axisLabels);

        // Add coordinate axes
        const axesHelper = new THREE.AxesHelper(4);
        axesHelper.material.linewidth = 3;
        this.scene.add(axesHelper);

        // Add reference grid
        const gridHelper = new THREE.GridHelper(15, 15, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // Optimized lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        this.scene.add(directionalLight);

        const rimLight = new THREE.DirectionalLight(0xffffff, 0.3);
        rimLight.position.set(-10, -10, -5);
        this.scene.add(rimLight);

        // Mouse controls for camera
        this.setupMouseControls();

        // Start animation loop
        this.animate();

        // Initialize charts
        this.initCharts();
        
        // Load available sessions
        this.loadSessions();
    }

    // Setup mouse controls for camera
    setupMouseControls() {
        let mouseDown = false;
        let mouseX = 0, mouseY = 0;
        let rotationX = 0, rotationY = 0;

        this.renderer.domElement.addEventListener('mousedown', (event) => {
            mouseDown = true;
            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        this.renderer.domElement.addEventListener('mousemove', (event) => {
            if (!mouseDown) return;
            const deltaX = event.clientX - mouseX;
            const deltaY = event.clientY - mouseY;
            rotationY += deltaX * 0.005;
            rotationX += deltaY * 0.005;
            this.camera.position.x = 8 * Math.sin(rotationY) * Math.cos(rotationX);
            this.camera.position.z = 8 * Math.cos(rotationY) * Math.cos(rotationX);
            this.camera.position.y = 8 * Math.sin(rotationX);
            this.camera.lookAt(0, 0, 0);
            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        this.renderer.domElement.addEventListener('mouseup', () => {
            mouseDown = false;
        });

        // Zoom controls
        this.renderer.domElement.addEventListener('wheel', (event) => {
            this.camera.position.z += event.deltaY * 0.01;
            this.camera.position.z = Math.max(3, Math.min(20, this.camera.position.z));
            this.camera.lookAt(0, 0, 0);
        });
    }

    // Optimized animation loop
    animate() {
        requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }

    // Session Management
    startLogging() {
        fetch('/start_logging', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'logging_started' || data.status === 'already_logging') {
                    document.getElementById('logging-status').textContent = 'Active';
                    document.getElementById('session-file').textContent = data.session_id;
                    document.getElementById('start-btn').disabled = true;
                    document.getElementById('stop-btn').disabled = false;
                    console.log('Logging started:', data.session_id);
                }
            })
            .catch(error => console.error('Error starting logging:', error));
    }

    stopLogging() {
        fetch('/stop_logging', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                if (data.status === 'logging_stopped') {
                    document.getElementById('logging-status').textContent = 'Inactive';
                    document.getElementById('start-btn').disabled = false;
                    document.getElementById('stop-btn').disabled = true;
                    console.log('Logging stopped. Data points:', data.data_points);
                    this.loadSessions(); // Refresh session list
                }
            })
            .catch(error => console.error('Error stopping logging:', error));
    }

    // Playback Management
    loadSessions() {
        fetch('/sessions')
            .then(response => response.json())
            .then(sessions => {
                const select = document.getElementById('session-select');
                select.innerHTML = '<option value="">-- Select a session --</option>';
                sessions.forEach(session => {
                    const option = document.createElement('option');
                    option.value = session;
                    option.textContent = session;
                    select.appendChild(option);
                });
            })
            .catch(error => console.error('Error fetching sessions:', error));
    }

    loadSelectedSession() {
        this.selectedFile = document.getElementById('session-select').value;
        if (this.selectedFile) {
            console.log(`📂 Loading session: ${this.selectedFile}`);
            
            fetch(`/data/${this.selectedFile}`)
                .then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
                    }
                    return response.json();
                })
                .then(data => {
                    // Validate data structure
                    if (!Array.isArray(data) || data.length === 0) {
                        throw new Error('Invalid or empty session data');
                    }
                    
                    // Check first sample has required fields
                    const sample = data[0];
                    const requiredFields = ['session_time', 'qx', 'qy', 'qz', 'qw'];
                    const missingFields = requiredFields.filter(field => !(field in sample));
                    if (missingFields.length > 0) {
                        throw new Error(`Missing required fields: ${missingFields.join(', ')}`);
                    }
                    
                    this.sessionData = data;
                    document.getElementById('loaded-file').textContent = this.selectedFile;
                    document.getElementById('sample-count').textContent = this.sessionData.length;
                    document.getElementById('charts-section').style.display = 'block';
                    document.getElementById('play-btn').disabled = false;
                    document.getElementById('reset-btn').disabled = false;
                    
                    // Populate charts with session data
                    this.populateCharts(data);
                    
                    console.log(`✅ Session loaded: ${this.sessionData.length} data points`);
                })
                .catch(error => {
                    console.error('❌ Error loading session data:', error);
                    this.sessionData = [];
                    document.getElementById('loaded-file').textContent = 'Error loading';
                    document.getElementById('sample-count').textContent = '0';
                    document.getElementById('play-btn').disabled = true;
                    document.getElementById('reset-btn').disabled = true;
                    
                    // Show user-friendly error message
                    alert(`Failed to load session: ${error.message}`);
                });
        } else {
            this.sessionData = [];
            document.getElementById('loaded-file').textContent = 'None';
            document.getElementById('sample-count').textContent = '0';
            document.getElementById('play-btn').disabled = true;
            document.getElementById('reset-btn').disabled = true;
            document.getElementById('charts-section').style.display = 'none';
            this.resetPlayback();
        }
    }

    // Fast chart population with optimized data processing
    populateCharts(data) {
        // Clear existing data
        this.accelChart.data.labels = [];
        this.accelChart.data.datasets.forEach(dataset => dataset.data = []);
        this.gyroChart.data.labels = [];
        this.gyroChart.data.datasets.forEach(dataset => dataset.data = []);
        this.magChart.data.labels = [];
        this.magChart.data.datasets.forEach(dataset => dataset.data = []);

        // Optimized sampling for fast rendering
        const sampleRate = Math.max(1, Math.floor(data.length / 200)); // Max 200 points
        
        for (let i = 0; i < data.length; i += sampleRate) {
            const sample = data[i];
            const timeLabel = sample.session_time ? `${sample.session_time.toFixed(1)}s` : `${i}`;
            
            // Add labels
            this.accelChart.data.labels.push(timeLabel);
            this.gyroChart.data.labels.push(timeLabel);
            this.magChart.data.labels.push(timeLabel);
            
            // Add raw data directly from logged JSON
            this.accelChart.data.datasets[0].data.push(sample.ax || 0);
            this.accelChart.data.datasets[1].data.push(sample.ay || 0);
            this.accelChart.data.datasets[2].data.push(sample.az || 0);
            
            this.gyroChart.data.datasets[0].data.push(sample.gx || 0);
            this.gyroChart.data.datasets[1].data.push(sample.gy || 0);
            this.gyroChart.data.datasets[2].data.push(sample.gz || 0);
            
            this.magChart.data.datasets[0].data.push(sample.mx || 0);
            this.magChart.data.datasets[1].data.push(sample.my || 0);
            this.magChart.data.datasets[2].data.push(sample.mz || 0);
        }
        
        // Update charts without animation for speed
        this.accelChart.update('none');
        this.gyroChart.update('none');
        this.magChart.update('none');
    }

    // Optimized playback with realistic motion timing
    playSession() {
        if (this.sessionData.length === 0) return;
        
        this.isPlaying = true;
        document.getElementById('play-btn').disabled = true;
        document.getElementById('pause-btn').disabled = false;
        
        // Calculate optimal playback speed based on data density
        const totalTime = this.sessionData[this.sessionData.length - 1]?.session_time || 0;
        const playbackSpeed = Math.max(5, Math.min(50, totalTime / this.sessionData.length * 1000)); // 5-50ms intervals
        
        this.playbackInterval = setInterval(() => {
            if (this.currentPlaybackIndex < this.sessionData.length) {
                this.updateVisualization(this.sessionData[this.currentPlaybackIndex]);
                this.updateProgress();
                this.currentPlaybackIndex++;
            } else {
                this.pauseSession();
            }
        }, playbackSpeed);
        
        console.log(`🎬 Playback started: ${playbackSpeed}ms intervals for realistic motion`);
    }

    pauseSession() {
        this.isPlaying = false;
        clearInterval(this.playbackInterval);
        document.getElementById('play-btn').disabled = false;
        document.getElementById('pause-btn').disabled = true;
    }

    resetPlayback() {
        this.pauseSession();
        this.currentPlaybackIndex = 0;
        document.getElementById('playback-time').textContent = '0.0s / 0.0s';
        
        // Reset IMU visualization to initial state
        this.imuCube.setRotationFromQuaternion(new THREE.Quaternion(0, 0, 0, 1));
        
        // Reset stroke detection
        this.strokeDetection = {
            lastStrokeTime: 0,
            strokeCount: 0,
            strokeFrequency: 0
        };
        
        // Clear all display metrics
        document.getElementById('roll').textContent = '0.0°';
        document.getElementById('pitch').textContent = '0.0°';
        document.getElementById('yaw').textContent = '0.0°';
        document.getElementById('cal').textContent = '0/0/0/0';
        document.getElementById('temp').textContent = '0.0°C';
        document.getElementById('accel').textContent = '0.0/0.0/0.0';
        document.getElementById('gyro').textContent = '0.00/0.00/0.00';
        document.getElementById('mag').textContent = '0.0/0.0/0.0';
        document.getElementById('strokes').textContent = '0';
        document.getElementById('stroke-rate').textContent = '0.0/min';
    }

    updateProgress() {
        const currentTime = this.sessionData[this.currentPlaybackIndex]?.session_time || 0;
        const totalTime = this.sessionData[this.sessionData.length - 1]?.session_time || 0;
        document.getElementById('playback-time').textContent = `${currentTime.toFixed(1)}s / ${totalTime.toFixed(1)}s`;
    }

    // Rotation-only visualization update (no position tracking - GPS will handle position later)
    updateVisualization(data) {
        if (!data) return;

        // Process quaternion data for realistic rotation
        let qx = data.qx || 0;
        let qy = data.qy || 0;
        let qz = data.qz || 0;
        let qw = data.qw || 1;
        
        // Normalize quaternion to ensure valid rotation
        const magnitude = Math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
        if (magnitude > 0) {
            qx /= magnitude;
            qy /= magnitude;
            qz /= magnitude;
            qw /= magnitude;
        }
        
        // Create quaternion and apply to cube for realistic motion
        const quaternion = new THREE.Quaternion(qx, qy, qz, qw);
        this.imuCube.setRotationFromQuaternion(quaternion);

        // Detect swimming strokes
        this.detectStroke(data);

        // Update all display values directly from logged data
        document.getElementById('roll').textContent = (data.roll || 0).toFixed(1) + '°';
        document.getElementById('pitch').textContent = (data.pitch || 0).toFixed(1) + '°';
        document.getElementById('yaw').textContent = (data.yaw || 0).toFixed(1) + '°';

        // Update calibration status from logged data
        if (data.cal) {
            const calStr = `${data.cal.sys || 0}/${data.cal.gyro || 0}/${data.cal.accel || 0}/${data.cal.mag || 0}`;
            document.getElementById('cal').textContent = calStr;
        }

        // Update temperature from logged data
        document.getElementById('temp').textContent = (data.temp || 0).toFixed(1) + '°C';

        // Update raw sensor data from logged data with proper formatting
        document.getElementById('accel').textContent = `${(data.ax || 0).toFixed(1)}/${(data.ay || 0).toFixed(1)}/${(data.az || 0).toFixed(1)}`;
        document.getElementById('gyro').textContent = `${(data.gx || 0).toFixed(2)}/${(data.gy || 0).toFixed(2)}/${(data.gz || 0).toFixed(2)}`;
        document.getElementById('mag').textContent = `${(data.mx || 0).toFixed(1)}/${(data.my || 0).toFixed(1)}/${(data.mz || 0).toFixed(1)}`;
        
        // Update stroke detection metrics
        document.getElementById('strokes').textContent = this.strokeDetection.strokeCount;
        document.getElementById('stroke-rate').textContent = (this.strokeDetection.strokeFrequency * 60).toFixed(1) + '/min';
    }

    // Handle window resize
    handleResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }
}

// Global instance
let visualizer;

// Initialize on load
document.addEventListener('DOMContentLoaded', () => {
    visualizer = new SessionVisualizer();
    
    // Handle window resize
    window.addEventListener('resize', () => visualizer.handleResize());
    
    // Global functions for HTML onclick handlers
    window.startLogging = () => visualizer.startLogging();
    window.stopLogging = () => visualizer.stopLogging();
    window.loadSelectedSession = () => visualizer.loadSelectedSession();
    window.playSession = () => visualizer.playSession();
    window.pauseSession = () => visualizer.pauseSession();
    window.resetPlayback = () => visualizer.resetPlayback();
});
