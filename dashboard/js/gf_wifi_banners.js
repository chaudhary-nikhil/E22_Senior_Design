/**
 * GoldenForm: wearable Wi‑Fi banner and add-wearable helpers.
 */

/** Avoid overwriting the wrist dropdown on every poll; refresh from device when HW ID changes. */
let lastBannerDeviceIdForRole = null;

function escapeHtml(s) {
    if (s == null || s === '') return '';
    return String(s)
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;');
}

function updateWearableConnectionBanner(res) {
    const el = document.getElementById('connected-wearable-banner');
    const hid = document.getElementById('dev-wifi-ssid');
    const hwHidden = document.getElementById('dev-hw-id');
    const roleEl = document.getElementById('dev-role');
    if (!el) return;
    if (!res || res.error || res.status === 'disconnected' || res.device_id === undefined) {
        lastBannerDeviceIdForRole = null;
        el.innerHTML =
            '<span class="wearable-banner-inner wearable-banner-inner--offline">' +
            'Join the band’s <strong>GoldenForm</strong> Wi‑Fi on this computer, then <strong>Add this wearable</strong>.</span>';
        if (hid) hid.value = '';
        if (hwHidden) hwHidden.value = '';
        return;
    }
    const ssid = res.ssid || ('GoldenForm_' + res.device_id);
    if (hid) hid.value = ssid;
    if (hwHidden) hwHidden.value = String(res.device_id);
    if (roleEl) {
        if (lastBannerDeviceIdForRole !== res.device_id) {
            lastBannerDeviceIdForRole = res.device_id;
            roleEl.value = normalizeDeviceRole(res.device_role);
        }
    }
    const displayRole = roleEl
        ? normalizeDeviceRole(roleEl.value)
        : normalizeDeviceRole(res.device_role);
    const roleL = formatRoleLabel(displayRole);
    let sdNote = '';
    if (res.storage_ok === false) {
        sdNote = ' <span class="wearable-banner-sdwarn">SD card not mounted. Insert a FAT32 card if you plan to record.</span>';
    }
    el.innerHTML =
        '<span class="wearable-banner-inner wearable-banner-inner--online">' +
        'Connected: <strong>' + escapeHtml(ssid) + '</strong> · HW <strong>#' + escapeHtml(String(res.device_id)) + '</strong> · ' +
        escapeHtml(roleL) + '. Tap <strong>Add this wearable</strong> to save it (adjust wrist first if needed).' +
        sdNote + '</span>';
}

async function refreshWearableConnectionBanner() {
    try {
        const res = await apiGet('/api/device_info');
        if (res && !res.error && res.status !== 'disconnected' && res.device_id !== undefined) {
            updateWearableConnectionBanner(res);
        } else {
            updateWearableConnectionBanner(null);
        }
    } catch (e) {
        updateWearableConnectionBanner(null);
    }
}

/**
 * One step: read the device, register with the wrist chosen in Settings (defaults from device when online).
 */
async function addWearableFromConnection() {
    try {
        const res = await apiGet('/api/device_info');
        if (!res || res.error || res.status === 'disconnected' || res.device_id === undefined) {
            showToast('Join this computer to the band’s GoldenForm Wi‑Fi, then try again.', 'error');
            return;
        }
        const roleEl = document.getElementById('dev-role');
        const finalRole = normalizeDeviceRole(roleEl ? roleEl.value : res.device_role);
        const ssid = res.ssid || ('GoldenForm_' + res.device_id);
        const hid = document.getElementById('dev-wifi-ssid');
        const hw = document.getElementById('dev-hw-id');
        if (hid) hid.value = ssid;
        if (hw) hw.value = String(res.device_id);

        const body = {
            user_id: userProfile ? userProfile.id : 1,
            device_hw_id: res.device_id,
            role: finalRole,
            name: formatRoleLabel(finalRole),
            wifi_ssid: ssid
        };
        const reg = await apiPost('/api/devices/register', body);
        updateWearableConnectionBanner(res);
        if (reg && reg.status === 'ok') {
            showToast('Added ' + formatRoleLabel(finalRole) + ' · HW #' + res.device_id, 'success');
            await loadDevices();
            if (typeof pushUserConfigToDevice === 'function') {
                await pushUserConfigToDevice(true);
            }
            /* Tell the wearable registration finished so it stops the status LED blink and closes the AP. */
            if (typeof apiPost === 'function') {
                await apiPost('/api/registration_done', {});
            }
        } else {
            showToast((reg && reg.error) || 'Could not add wearable', 'error');
        }
    } catch (e) {
        showToast('Could not reach the device. Check Wi‑Fi and try again.', 'error');
    }
}

function hasWifiSyncCompleted() {
    try {
        return !!localStorage.getItem(LS_WIFI_SYNC_OK);
    } catch (e) {
        return false;
    }
}

function recordWifiSyncSuccess() {
    try {
        localStorage.setItem(LS_WIFI_SYNC_OK, String(Date.now()));
    } catch (e) { /* ignore */ }
}
