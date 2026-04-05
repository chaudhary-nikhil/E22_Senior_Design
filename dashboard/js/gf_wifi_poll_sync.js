/**
 * GoldenForm — Connection polling; manual Session → Sync only (no auto /process on connect).
 */
function setConnStatus(state) {
    if (state === lastConnStatusApplied) return;
    lastConnStatusApplied = state;
    navDisplayOnline = state === 'connected' || state === 'syncing';
    const dot = document.querySelector('.status-dot');
    const txt = document.getElementById('conn-text');
    if (dot) dot.className = 'status-dot ' + state;
    if (txt) txt.textContent = state === 'connected' ? 'Connected' : state === 'syncing' ? 'Syncing...' : 'Offline';
}

function startDevicePolling() {
    if (devicePollInterval) clearInterval(devicePollInterval);
    pollDevice();
    devicePollInterval = setInterval(pollDevice, 2000);
}

async function pollDevice() {
    try {
        const res = await apiGet('/api/device_info');
        const rawOnline = !!(res && !res.error && res.status !== 'disconnected');
        const wasOnline = isDeviceOnline;
        isDeviceOnline = rawOnline;

        const txt = document.getElementById('conn-text');
        const midSync = txt && txt.textContent === 'Syncing...';

        if (!midSync) {
            if (rawOnline) {
                pollOfflineStreak = 0;
                setConnStatus('connected');
            } else {
                pollOfflineStreak++;
                if (pollOfflineStreak >= POLL_OFFLINE_AFTER_FAILS) {
                    setConnStatus('offline');
                }
            }
        }

        if (res && res.cal) {
            updateCalibrationDisplay(res);
        } else {
            updateNavCalPill(res || null);
            if (typeof clearCalibrationLiveDisplay === 'function') {
                clearCalibrationLiveDisplay();
            }
        }

        /* No auto /process here — avoids data.json calls before the user joins the band Wi‑Fi on purpose. */
        if (!wasOnline && isDeviceOnline) {
            if (pendingConfigSync) {
                await pushUserConfigToDevice(true);
                pendingConfigSync = false;
            }
            if (pendingIdealSync) {
                await pushIdealToDevice(true);
                pendingIdealSync = false;
            }
        }
        updateSyncPlaybookConnectionState();
        if (rawOnline && res && res.device_id !== undefined) {
            updateWearableConnectionBanner(res);
            maybeAutoSaveCalibrationFromPoll(res);
        } else {
            updateWearableConnectionBanner(null);
        }
    } catch (e) {
        isDeviceOnline = false;
        updateWearableConnectionBanner(null);
        const txt = document.getElementById('conn-text');
        if (txt && txt.textContent !== 'Syncing...') {
            pollOfflineStreak++;
            if (pollOfflineStreak >= POLL_OFFLINE_AFTER_FAILS) {
                setConnStatus('offline');
            }
        }
        updateSyncPlaybookConnectionState();
        updateNavCalPill(null);
        if (typeof clearCalibrationLiveDisplay === 'function') {
            clearCalibrationLiveDisplay();
        }
    }
}

async function syncFromDevice() {
    const statusEl = document.getElementById('sync-status');
    const btn = document.getElementById('sync-btn');
    if (statusEl) { statusEl.style.display = 'block'; statusEl.textContent = 'Connecting to device...'; statusEl.className = 'badge badge-amber'; }
    if (btn) btn.disabled = true;
    setConnStatus('syncing');

    const devInfo = await apiGet('/api/device_info');
    let pickedRole = null;
    if (devInfo && devInfo.device_id !== undefined) {
        lastSyncedDeviceInfo = devInfo;
        const roleEl = document.getElementById('dev-role');
        pickedRole = roleEl ? normalizeDeviceRole(roleEl.value) : normalizeDeviceRole(devInfo.device_role);
        if (statusEl) statusEl.textContent = `Connected to ${devInfo.ssid || 'GoldenForm'} (${pickedRole || 'wrist'})...`;
        if (userProfile && userProfile.id) {
            const role = pickedRole;
            await apiPost('/api/devices/register', {
                user_id: userProfile.id,
                device_hw_id: devInfo.device_id,
                role,
                name: formatRoleLabel(role),
                wifi_ssid: devInfo.ssid || ''
            });
        }
    }

    try {
        const res = await apiGet('/process');
        if (!res || res.error) {
            if (statusEl) { statusEl.textContent = res ? res.error : 'No response'; statusEl.className = 'badge badge-red'; }
            setConnStatus('offline');
            showToast('Failed to connect to device', 'error');
            return;
        }
        const sessions = res.sessions || [];
        recordWifiSyncSuccess();
        syncDeviceCount++;
        const regTail = devInfo && devInfo.device_id !== undefined && pickedRole
            ? ` · registry: ${formatRoleLabel(pickedRole)} · HW #${devInfo.device_id}`
            : '';
        if (statusEl) {
            statusEl.textContent = sessions.length === 0
                ? `Linked: 0 sessions (normal before first swim) · sync #${syncDeviceCount}`
                : `Synced ${sessions.length} session(s) from device #${syncDeviceCount}`;
            statusEl.className = 'badge badge-green';
        }
        setConnStatus('connected');
        const zeroHint = sessions.length === 0 ? ' After your first swim, sync again to download sessions.' : '';
        showToast(`Synced ${sessions.length} session(s) (sync #${syncDeviceCount})${regTail}${zeroHint ? '. ' + zeroHint : ''}`, 'success');

        for (const s of sessions) {
            addSession({
                name: s.name,
                processed_data: s.processed_data,
                raw_data: s.raw_data,
                metrics: s.metrics,
                duration: s.duration,
                syncedAt: s.syncedAt
            });
        }
        if (sessions.length > 0) {
            const selIdx = savedSessions.length - 1;
            setTimeout(() => {
                try {
                    if (selIdx >= 0 && selIdx < savedSessions.length) selectSession(selIdx);
                } catch (e) { /* ignore */ }
            }, 150);
        }

        await Promise.all([pushUserConfigToDevice(true), pushIdealToDevice(true)]).catch(() => {});

        if (syncDeviceCount > 1) {
            showToast(`${syncDeviceCount} devices synced. Use "Merge Devices" to combine.`, 'info');
        }
    } catch (e) {
        if (statusEl) {
            statusEl.textContent = 'Device unreachable. Join GoldenForm Wi‑Fi and retry.';
            statusEl.className = 'badge badge-red';
        }
        setConnStatus('offline');
        showToast('Sync failed: connect to the device hotspot, then try Sync again.', 'error');
    } finally {
        if (btn) btn.disabled = false;
        try {
            await loadDevices();
        } catch (e) { /* ignore refresh errors */ }
        updateSyncPlaybookConnectionState();
    }
}
