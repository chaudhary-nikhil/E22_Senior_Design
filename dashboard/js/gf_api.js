/**
 * GoldenForm  --  shared HTTP helpers (used by app.js).
 * Load order: gf_api → gf_roles → gf_constants → app.js
 */
const API = '';
const AUTH_TOKEN_KEY = 'gf_session_token';

function getAuthHeaders() {
    try {
        const t = localStorage.getItem(AUTH_TOKEN_KEY);
        return t ? { 'Authorization': 'Bearer ' + t } : {};
    } catch (e) {
        return {};
    }
}

function setAuthToken(token) {
    try {
        if (token) localStorage.setItem(AUTH_TOKEN_KEY, token);
        else localStorage.removeItem(AUTH_TOKEN_KEY);
    } catch (e) { /* ignore */ }
}

async function apiGet(path, fetchOpts = {}) {
    const timeoutMs = fetchOpts.timeoutMs;
    const rest = { ...fetchOpts };
    delete rest.timeoutMs;
    let timer = null;
    let abortForTimeout = null;
    try {
        const headers = { ...getAuthHeaders(), ...(rest.headers || {}) };
        const init = { ...rest, headers };
        if (timeoutMs && timeoutMs > 0 && !rest.signal) {
            abortForTimeout = new AbortController();
            timer = setTimeout(() => {
                try { abortForTimeout.abort(); } catch (e) { /* ignore */ }
            }, timeoutMs);
            init.signal = abortForTimeout.signal;
        }
        const r = await fetch(API + path, init);
        const text = await r.text();
        let data = null;
        try { data = text ? JSON.parse(text) : null; } catch { data = null; }
        if (r.status === 401 && !path.startsWith('/api/auth/')) {
            setAuthToken(null);
        }
        if (!r.ok) return data && typeof data === 'object' ? { _httpError: r.status, ...data } : null;
        return data;
    } catch (e) {
        return null;
    } finally {
        if (timer) clearTimeout(timer);
    }
}

async function apiPost(path, body, fetchOpts = {}) {
    try {
        const headers = {
            'Content-Type': 'application/json',
            ...getAuthHeaders(),
            ...(fetchOpts.headers || {}),
        };
        const r = await fetch(API + path, {
            ...fetchOpts,
            method: 'POST',
            headers,
            body: JSON.stringify(body),
        });
        const text = await r.text();
        let data = {};
        try { data = text ? JSON.parse(text) : {}; } catch { data = { error: text || 'Invalid JSON from server' }; }
        if (r.status === 401 && !path.startsWith('/api/auth/')) {
            setAuthToken(null);
        }
        if (!r.ok) {
            return {
                ...data,
                status: data.status || 'error',
                error: data.error || data.message || ('HTTP ' + r.status),
                httpStatus: r.status,
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
