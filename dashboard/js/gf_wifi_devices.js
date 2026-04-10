/**
 * GoldenForm  --  Device registry list (GET/POST /api/devices), IMU cube wrist flip.
 */
async function loadDevices() {
    const res = await apiGet('/api/devices');
    const devices = (res && res.devices) || [];
    lastDevicesList = devices;
    cachedDeviceListLength = devices.length;
    const wristDevice = devices.find(d => String(d.role || '').toLowerCase().startsWith('wrist'));
    if (wristDevice) {
        activeWristDeviceRole = wristDevice.role.toLowerCase();
        if (typeof imuCube !== 'undefined' && imuCube) {
            const flip = activeWristDeviceRole === 'wrist_left' ? -1 : 1;
            imuCube.scale.set(flip, 1, 1);
            if (typeof imuCubeB !== 'undefined' && imuCubeB) imuCubeB.scale.set(flip, 1, 1);
        }
    }
    const list = document.getElementById('device-list');
    if (list) {
        if (!devices.length) {
            list.innerHTML = '<p style="color:var(--text3);font-size:0.85em;">No wearables yet. Connect to each band\u2019s Wi-Fi and tap <strong>Add this wearable</strong> above.</p>';
        } else {
            const roleIcons = { wrist_right: '🤚', wrist_left: '✋' };
            list.innerHTML = devices.map((d, i) => {
                const idx = i + 1;
                const colorClass = 'device-card--' + ((i % 6) + 1);
                const roleL = formatRoleLabel(d.role);
                const wifiLine = (d.wifi_ssid && String(d.wifi_ssid).trim())
                    ? '<div class="device-card-wifi">Network: <strong>' + escapeHtml(String(d.wifi_ssid).trim()) + '</strong></div>'
                    : '';
                return '<div class="device-card ' + colorClass + '">' +
                    '<div class="device-card-index" title="Order added">' + idx + '</div>' +
                    '<div class="device-card-body">' +
                    '<div class="device-card-title">' + (roleIcons[d.role] || '📱') + ' ' + escapeHtml(roleL) + '</div>' +
                    '<div class="device-card-meta"><strong>HW #' + escapeHtml(String(d.device_hw_id)) + '</strong></div>' +
                    wifiLine +
                    '</div>' +
                    '<button type="button" class="btn btn-sm btn-outline" onclick="deleteDevice(' + d.id + ')" title="Remove from this app">✕</button></div>';
            }).join('');
        }
    }
    renderSetupJourney();
    updateSyncPlaybookMulti();
}

async function deleteDevice(id) {
    const res = await apiPost('/api/devices/delete', { device_id: id });
    if (res && res.status === 'ok') { showToast('Device removed', 'success'); loadDevices(); }
    else showToast('Failed to remove device', 'error');
}

/** Legacy: registration is handled by addWearableFromConnection(); kept for compatibility. */
async function registerDevice(e) {
    if (e) e.preventDefault();
    await addWearableFromConnection();
}
