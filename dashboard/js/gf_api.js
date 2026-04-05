/**
 * GoldenForm — shared HTTP helpers (used by app.js).
 * Load order: gf_api → gf_roles → gf_constants → app.js
 */
const API = '';

async function apiGet(path) {
    try {
        const r = await fetch(API + path);
        const text = await r.text();
        let data = null;
        try { data = text ? JSON.parse(text) : null; } catch { data = null; }
        if (!r.ok) return data && typeof data === 'object' ? { _httpError: r.status, ...data } : null;
        return data;
    } catch (e) {
        return null;
    }
}

async function apiPost(path, body) {
    try {
        const r = await fetch(API + path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
        const text = await r.text();
        let data = {};
        try { data = text ? JSON.parse(text) : {}; } catch { data = { error: text || 'Invalid JSON from server' }; }
        if (!r.ok) {
            return {
                status: 'error',
                error: data.error || data.message || ('HTTP ' + r.status),
                httpStatus: r.status
            };
        }
        return data;
    } catch (e) {
        const offline = typeof navigator !== 'undefined' && navigator.onLine === false;
        return { status: 'error', error: offline ? 'Offline. Connect to the GoldenForm WiFi or open the dashboard.' : (e.message || 'Failed to fetch') };
    }
}

function showToast(msg, type = 'info') {
    const t = document.createElement('div');
    t.className = 'toast toast-' + type;
    t.textContent = msg;
    document.body.appendChild(t);
    requestAnimationFrame(() => t.classList.add('show'));
    setTimeout(() => { t.classList.remove('show'); setTimeout(() => t.remove(), 300); }, 3000);
}
