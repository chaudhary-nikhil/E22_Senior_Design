/**
 * GoldenForm  --  Sync browser storage with server instance id (demo: fresh start each processor run).
 * Returns true if the page will reload  --  caller must not run init3D / polling on the dying document
 * (otherwise WebGL + intervals start right before unload and can crash Chrome).
 */
async function syncBootstrapInstance() {
    try {
        const r = await apiGet('/api/bootstrap', { timeoutMs: 12000 });
        if (r && typeof r.demo === 'boolean') {
            window.GF_DEMO_MODE = r.demo;
            /* Do not clear the session token on every load  --  that made demo logins impossible to
             * keep across refresh. Stale auth is cleared when instance_id changes (server restart). */
        }
        if (!r || !r.instance_id) return false;
        /* Persist on localStorage (not sessionStorage) so a new browser tab or restart
         * still sees the last server instance id  --  otherwise demo -- wipes DB but the
         * client never clears goldenform_* after the user closed the tab. */
        const key = 'gf_bootstrap_instance_id';
        const prev = localStorage.getItem(key);
        if (prev && prev !== r.instance_id) {
            try {
                const toRemove = [];
                for (let i = 0; i < localStorage.length; i++) {
                    const k = localStorage.key(i);
                    if (k && k.startsWith('goldenform_')) toRemove.push(k);
                }
                toRemove.forEach((k) => localStorage.removeItem(k));
                localStorage.removeItem('gf_session_token');
                localStorage.removeItem('gf_user_profile_cache_v1');
            } catch (e) { /* ignore */ }
            localStorage.setItem(key, r.instance_id);
            location.reload();
            return true;
        }
        if (!prev) {
            localStorage.setItem(key, r.instance_id);
        }
    } catch (e) {
        /* dashboard offline */
    }
    return false;
}
