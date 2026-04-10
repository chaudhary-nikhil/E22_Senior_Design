/**
 * GoldenForm — Connection polling; manual Session → Sync only (no auto /process on connect).
 */

let gfPollDeviceInFlight = false;

/** Normalize cal object from /api/device_info (handles alternate keys and string payloads). */
function normalizeDeviceInfoCal(res) {
    if (!res || typeof res !== 'object') return null;
    let c = res.cal || res.calibration;
    if (typeof c === 'string') {
        try { c = JSON.parse(c); } catch (e) { c = null; }
    }
    if (!c || typeof c !== 'object' || Array.isArray(c)) return null;
    return {
        sys: Math.min(3, Math.max(0, Number(c.sys) || 0)),
        gyro: Math.min(3, Math.max(0, Number(c.gyro) || 0)),
        accel: Math.min(3, Math.max(0, Number(c.accel) || 0)),
        mag: Math.min(3, Math.max(0, Number(c.mag) || 0))
    };
}

/** When the laptop is on the band AP, the browser can read the ESP32 directly (CORS * on device). */
async function fetchDeviceInfoDirectFromBand() {
    try {
        const ctrl = new AbortController();
        const to = setTimeout(() => ctrl.abort(), 1000);
        const r = await fetch('http://192.168.4.1/api/device_info', {
            method: 'GET',
            cache: 'no-store',
            credentials: 'omit',
            signal: ctrl.signal
        });
        clearTimeout(to);
        if (!r.ok) return null;
        const text = await r.text();
        try { return text ? JSON.parse(text) : null; } catch (e) { return null; }
    } catch (e) {
        return null;
    }
}

async function fetchDataJsonDirectFromBand() {
    try {
        const ctrl = new AbortController();
        const to = setTimeout(() => ctrl.abort(), 4500);
        const r = await fetch('http://192.168.4.1/data.json', {
            method: 'GET',
            cache: 'no-store',
            credentials: 'omit',
            signal: ctrl.signal
        });
        clearTimeout(to);
        const text = await r.text();
        let data = null;
        try { data = text ? JSON.parse(text) : null; } catch { data = null; }
        return {
            ok: r.ok,
            status: r.status,
            data,
            text: text || ''
        };
    } catch (e) {
        return { ok: false, status: 0, data: null, text: '' };
    }
}

async function fetchStatusDirectFromBand() {
    try {
        const ctrl = new AbortController();
        const to = setTimeout(() => ctrl.abort(), 2500);
        const r = await fetch('http://192.168.4.1/status', {
            method: 'GET',
            cache: 'no-store',
            credentials: 'omit',
            signal: ctrl.signal
        });
        clearTimeout(to);
        const text = await r.text();
        let data = null;
        try { data = text ? JSON.parse(text) : null; } catch { data = null; }
        return { ok: r.ok, status: r.status, data, text: text || '' };
    } catch (e) {
        return { ok: false, status: 0, data: null, text: '' };
    }
}

function _gfStreamKeyFromProcessedSample(d) {
    if (!d) return '0';
    const id = d.device_id != null ? Number(d.device_id) : (d.dev_id != null ? Number(d.dev_id) : NaN);
    if (Number.isFinite(id) && id > 0) return 'hw' + id;
    if (d._origin_id != null && d._origin_id !== '') return 's' + d._origin_id;
    return '0';
}

/** Count stroke segments without mutating global strokeBoundaries (used for AP replay metrics). */
function _countStrokeSegmentsFromProcessed(pd) {
    if (!pd || !pd.length) return 0;
    const useFw = pd.some((x) => (Number(x.strokes) || 0) > 0);
    let prevSk = null;
    let prevSn = -1;
    let n = 0;
    for (let i = 0; i < pd.length; i++) {
        const d = pd[i];
        const sk = _gfStreamKeyFromProcessedSample(d);
        const sn = useFw ? (Number(d.strokes) || 0) : (Number(d.stroke_count) || 0);
        if (sn <= 0) continue;
        const isNew = (prevSn < 0) || sk !== prevSk || (sk === prevSk && sn > prevSn);
        if (isNew) {
            n++;
            prevSk = sk;
            prevSn = sn;
        }
    }
    return n;
}

function _apSampleToProcessed(sample) {
    if (!sample || typeof sample !== 'object') return null;
    const t = Number(sample.t);
    if (!Number.isFinite(t)) return null;
    const q = {
        qw: Number(sample.qw ?? 1) || 1,
        qx: Number(sample.qx ?? 0) || 0,
        qy: Number(sample.qy ?? 0) || 0,
        qz: Number(sample.qz ?? 0) || 0,
    };
    const lia = {
        x: Number(sample.lia_x ?? 0) || 0,
        y: Number(sample.lia_y ?? 0) || 0,
        z: Number(sample.lia_z ?? 0) || 0,
    };
    const av = {
        gx: Number(sample.gx ?? 0) || 0,
        gy: Number(sample.gy ?? 0) || 0,
        gz: Number(sample.gz ?? 0) || 0,
    };
    const cal = sample.cal && typeof sample.cal === 'object' ? sample.cal : null;
    return {
        timestamp: t,
        quaternion: q,
        lia,
        // Keep compatibility with any code expecting `acceleration` for charts.
        acceleration: { ax: lia.x, ay: lia.y, az: lia.z },
        angular_velocity: av,
        calibration: cal || undefined,
        cal: cal || undefined,
        haptic_fired: Number(sample.haptic ?? sample.haptic_fired ?? 0) ? 1 : 0,
        deviation_score: Number(sample.deviation ?? sample.deviation_score ?? 0) || 0,
        haptic_reason: Number(sample.haptic_reason ?? 0) || 0,
        pull_duration_ms: Number(sample.pull_duration_ms ?? 0) || 0,
        strokes: Number(sample.strokes ?? sample.stroke_count ?? 0) || 0,
        stroke_count: Number(sample.stroke_count ?? sample.strokes ?? 0) || 0,
        turns: Number(sample.turns ?? 0) || 0,
        entry_angle: Number(sample.entry_angle ?? 0) || 0,
        device_id: Number(sample.dev_id ?? sample.device_id ?? 0) || 0,
        device_role: Number(sample.dev_role ?? sample.device_role ?? 0) || 0,
    };
}

function _metricsFromProcessed(pd) {
    const m = {
        stroke_count: 0,
        turn_count: 0,
        haptic_count: 0,
        avg_deviation: 0,
        avg_entry_angle: 0,
        samples: pd.length,
    };
    if (!pd.length) return m;
    let devSum = 0, devN = 0, angleSum = 0, angleN = 0, hN = 0;
    let lastStroke = 0;
    let maxTurns = 0;
    for (const p of pd) {
        const s = Number(p.strokes || 0) || 0;
        if (s > lastStroke) lastStroke = s;
        const tn = Number(p.turns || 0) || 0;
        if (tn > maxTurns) maxTurns = tn;
        const d = Number(p.deviation_score || 0);
        if (Number.isFinite(d) && d > 0) { devSum += d; devN++; }
        const a = Number(p.entry_angle || 0);
        if (Number.isFinite(a) && a > 0) { angleSum += a; angleN++; }
        if (Number(p.haptic_fired || 0) && (i === 0 || !Number(pd[i - 1].haptic_fired || 0))) hN++;
    }
    const segCount = _countStrokeSegmentsFromProcessed(pd);
    m.stroke_count = Math.max(lastStroke, segCount);
    m.turn_count = maxTurns;
    m.haptic_count = hN;
    m.avg_deviation = devN ? devSum / devN : 0;
    m.avg_entry_angle = angleN ? angleSum / angleN : 0;
    return m;
}

/**
 * Build session object from ESP `data.json` root (same shape for live fetch or stored copy).
 * Returns null if payload has no session samples.
 */
function buildSessionFromApJsonRoot(raw) {
    if (!raw || typeof raw !== 'object') return null;
    let sessions = raw.sessions;
    if (Array.isArray(sessions) && !sessions.length && Array.isArray(raw.data) && raw.data.length) {
        sessions = [{ id: 0, name: 'AP session', data: raw.data }];
    }
    if (!Array.isArray(sessions) || !sessions.length) return null;
    const s0 = sessions[0];
    const data = Array.isArray(s0.data) ? s0.data : [];
    const processed = [];
    for (const row of data) {
        const p = _apSampleToProcessed(row);
        if (p) processed.push(p);
    }
    processed.sort((a, b) => (a.timestamp || 0) - (b.timestamp || 0));
    const duration = processed.length > 1
        ? ((processed[processed.length - 1].timestamp - processed[0].timestamp) / 1000)
        : 0;
    return {
        name: s0.name ? (s0.name + ' · AP') : 'AP data.json',
        processed_data: processed,
        raw_data: raw,
        metrics: _metricsFromProcessed(processed),
        duration,
        syncedAt: new Date().toISOString()
    };
}

/** Download raw `data.json` equivalent saved on the selected session (safe; no band required). */
function exportSessionRawJson() {
    if (typeof savedSessions === 'undefined' || typeof activeSessionIdx === 'undefined' || activeSessionIdx < 0) {
        if (typeof showToast === 'function') showToast('Select a session in the list first.', 'error');
        return;
    }
    const s = savedSessions[activeSessionIdx];
    if (!s || !s.raw_data) {
        if (typeof showToast === 'function') {
            showToast('No raw JSON on this session. Sync from the band on GoldenForm Wi‑Fi (Sync now) or use Replay AP JSON while transfer is active.', 'info');
        }
        return;
    }
    const blob = new Blob([JSON.stringify(s.raw_data, null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    const base = String(s.name || s.id || 'session').replace(/[^\w\-]+/g, '_').slice(0, 80);
    a.download = 'goldenform_' + base + '_raw.json';
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(a.href);
    if (typeof showToast === 'function') showToast('Raw JSON download started', 'success');
}

async function replayApJsonToSession() {
    const btn = document.getElementById('replay-ap-json-btn');
    const statusEl = document.getElementById('sync-status');
    if (btn) btn.disabled = true;
    if (statusEl) {
        statusEl.style.display = 'block';
        statusEl.textContent = 'Fetching data.json from the band…';
        statusEl.className = 'badge badge-amber';
    }
    try {
        const res = await fetchDataJsonDirectFromBand();
        const raw = res && res.data;
        let sessions = raw && raw.sessions;
        if (Array.isArray(sessions) && !sessions.length && raw && Array.isArray(raw.data) && raw.data.length) {
            sessions = [{ id: 0, name: 'AP session', data: raw.data }];
        }
        if (!res || !res.ok || !raw || !Array.isArray(sessions) || !sessions.length) {
            // Distinguish: wrong network vs transfer not started vs no files on SD.
            let hint = 'Join the wearable’s GoldenForm_ Wi‑Fi on this computer, then try again.';
            if (res && res.status === 503) {
                hint = 'The band HTTP server is busy or transfer mode is not active. On the wearable: start Session sync / transfer, keep this device on the band’s Wi‑Fi, then retry.';
            } else if (res && res.status === 404) {
                // Band firmware uses 404 "No data available" when sync not started or no files.
                const st = await fetchStatusDirectFromBand();
                if (st && st.ok && st.data && typeof st.data === 'object') {
                    const syncing = !!st.data.syncing;
                    const files = Number(st.data.files ?? st.data.file_count ?? 0) || 0;
                    const samples = Number(st.data.samples ?? 0) || 0;
                    if (!syncing) {
                        hint = 'The band is reachable, but sync is not active yet. Hold Sync on the band (hotspot + sync), then retry Replay.';
                    } else if (files === 0 || samples === 0) {
                        hint = 'The band is reachable, but there are no session files ready to export yet. Record a swim first, then sync/replay.';
                    } else {
                        hint = 'Band reachable and syncing, but JSON is not ready yet. Wait a few seconds and retry.';
                    }
                } else {
                    hint = 'Band responded 404 (no data available). Start sync on the band, then retry.';
                }
            } else if (res && res.ok && raw && Array.isArray(sessions) && sessions.length === 0) {
                hint = 'data.json loaded but contains no sessions. Finish recording, sync files to the band, then retry.';
            }
            if (statusEl) {
                statusEl.textContent = 'Could not read data.json. ' + hint;
                statusEl.className = 'badge badge-red';
            }
            showToast('Replay failed: ' + hint, 'error');
            return;
        }
        const built = buildSessionFromApJsonRoot(raw);
        if (!built || !built.processed_data || !built.processed_data.length) {
            if (statusEl) {
                statusEl.textContent = 'data.json had no sample rows.';
                statusEl.className = 'badge badge-red';
            }
            showToast('No samples in JSON', 'error');
            return;
        }
        const obj = Object.assign({ id: 'apjson:' + String(Date.now()) }, built);
        addSession(obj);
        if (statusEl) {
            statusEl.textContent = `Loaded ${processed.length} samples from AP JSON`;
            statusEl.className = 'badge badge-green';
        }
        showToast('Loaded AP data.json into Session', 'success');
        selectSession(savedSessions.length - 1);
    } catch (e) {
        if (statusEl) {
            statusEl.textContent = 'Replay failed. Join GoldenForm Wi‑Fi and retry.';
            statusEl.className = 'badge badge-red';
        }
        showToast('Replay failed: join GoldenForm Wi‑Fi and retry.', 'error');
    } finally {
        if (btn) btn.disabled = false;
    }
}

/**
 * Prefer JSON from the ESP32 when the browser can reach 192.168.4.1. The Python host may not be
 * on the GoldenForm AP (different machine, VPN, firewall), so /api/device_info would stay disconnected
 * even though the laptop sees the wearable.
 *
 * When the dashboard proxy already returns a connected device with calibration, skip the direct
 * 192.168.4.1 fetch so polling stays fast (avoids ~1s dead wait every 2s while offline from the band).
 */
async function fetchDeviceInfoMerged() {
    const res = await apiGet('/api/device_info');
    let cal = normalizeDeviceInfoCal(res);
    const proxyOk = !!(res && !res.error && res.status !== 'disconnected' && !res._httpError);
    const hasDevice = res && res.device_id !== undefined;
    if (proxyOk && hasDevice && cal) {
        return { res: { ...res, cal }, cal };
    }
    const direct = await fetchDeviceInfoDirectFromBand();
    const dcal = direct ? normalizeDeviceInfoCal(direct) : null;
    if (dcal && direct && typeof direct === 'object') {
        return { res: { ...direct, cal: dcal }, cal: dcal };
    }
    return { res, cal };
}

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
    if (gfPollDeviceInFlight) return;
    try {
        if (typeof document !== 'undefined' && document.visibilityState === 'hidden') {
            return;
        }
    } catch (e) { /* ignore */ }
    gfPollDeviceInFlight = true;
    try {
        const merged = await fetchDeviceInfoMerged();
        let res = merged.res;
        let cal = merged.cal;

        const rawOnline = !!(res && !res.error && res.status !== 'disconnected' && !res._httpError);
        const wasOnline = isDeviceOnline;
        isDeviceOnline = rawOnline;

        const txt = document.getElementById('conn-text');
        const midSync = txt && txt.textContent === 'Syncing...';

        /* Always show Connected when the proxy can reach the band. Do not gate on midSync:
         * previously we skipped updates while conn-text was "Syncing...", which left the nav
         * stuck on Offline even though /api/device_info succeeded (wearables banner looked OK). */
        if (rawOnline) {
            pollOfflineStreak = 0;
            setConnStatus('connected');
        } else {
            pollOfflineStreak++;
            if (pollOfflineStreak >= POLL_OFFLINE_AFTER_FAILS) {
                if (!midSync) {
                    setConnStatus('offline');
                }
            }
        }

        if (res && cal) {
            res.cal = cal;
            updateCalibrationDisplay(res);
        } else {
            /* If we're connected but no cal is present, keep the UI user-friendly.
             * (Debugging details should not appear in the product UI.) */
            try {
                const hintEl = document.getElementById('cal-hint');
                if (hintEl && rawOnline && res && res.device_id !== undefined && !cal) {
                    hintEl.textContent = 'Connected. Waiting for live calibration… keep this tab open and move through the calibration guide.';
                }
            } catch (e) { /* ignore */ }
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
    } finally {
        gfPollDeviceInFlight = false;
    }
}

async function syncFromDevice() {
    const statusEl = document.getElementById('sync-status');
    const btn = document.getElementById('sync-btn');
    if (statusEl) { statusEl.style.display = 'block'; statusEl.textContent = 'Connecting to device...'; statusEl.className = 'badge badge-amber'; }
    if (btn) btn.disabled = true;
    setConnStatus('syncing');

    let devInfo = null;
    if (typeof fetchDeviceInfoMerged === 'function') {
        const merged = await fetchDeviceInfoMerged();
        devInfo = merged.res;
    } else {
        devInfo = await apiGet('/api/device_info');
    }
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

        /* Tell firmware registration is complete so it stops the status LED blink and closes the AP. */
        try { await apiPost('/api/registration_done', {}); } catch (_) { /* device may already be offline */ }

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
