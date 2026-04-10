/**
 * GoldenForm  --  Quaternion helper + gravity-based orientation for 3D viz alignment.
 */
function nq(q) {
    if (!q) return new THREE.Quaternion(0, 0, 0, 1);
    const w = q.qw ?? 1, x = q.qx ?? 0, y = q.qy ?? 0, z = q.qz ?? 0;
    const m = Math.hypot(w, x, y, z);
    return m > 1e-8 ? new THREE.Quaternion(x / m, y / m, z / m, w / m) : new THREE.Quaternion(0, 0, 0, 1);
}

function accelSampleForOrientation(d) {
    if (!d) return null;
    const acc = d.acceleration || {};
    let ax = acc.ax, ay = acc.ay, az = acc.az;
    if (ax == null && d.lia) {
        ax = d.lia.x; ay = d.lia.y; az = d.lia.z;
    }
    ax = ax || 0; ay = ay || 0; az = az || 0;
    const mag = Math.hypot(ax, ay, az);
    if (mag < 2.5) return null;
    return { ax, ay, az };
}

function computeVizCoordinateTransformFromAccel(first) {
    const ax = first.ax, ay = first.ay, az = first.az;
    const absAx = Math.abs(ax), absAy = Math.abs(ay), absAz = Math.abs(az);
    let rotationX = 0, rotationY = 0, rotationZ = 0;
    if (absAx > absAy && absAx > absAz) {
        rotationZ = ax > 0 ? Math.PI / 2 : -Math.PI / 2;
    } else if (absAy > absAx && absAy > absAz) {
        rotationZ = ay > 0 ? 0 : Math.PI;
    } else {
        rotationX = az > 0 ? Math.PI / 2 : -Math.PI / 2;
    }
    return { rotationX, rotationY, rotationZ };
}

function refreshVizCoordinateTransform() {
    vizCoordinateTransform = { rotationX: 0, rotationY: 0, rotationZ: 0 };
    if (!processedData || !processedData.length) return;
    const d0 = processedData[0];
    if (d0 && d0.quaternion) {
        const qw = d0.quaternion.qw ?? d0.quaternion.w;
        const qx = d0.quaternion.qx ?? d0.quaternion.x;
        if (qw != null && qx != null && (Math.abs(qw) + Math.abs(qx)) > 0.01) {
            return;
        }
    }
    for (let i = 0; i < Math.min(processedData.length, 400); i++) {
        const a = accelSampleForOrientation(processedData[i]);
        if (a) {
            vizCoordinateTransform = computeVizCoordinateTransformFromAccel(a);
            return;
        }
    }
}
