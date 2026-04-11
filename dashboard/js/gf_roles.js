/**
 * GoldenForm  --  wearable roles (two wrists only for stretch multi-device).
 * PDP stretch 3-1-1: register and configure wearables; we support left + right hand.
 */
const LS_EXPECTED_WEARABLES = 'goldenform_expected_wearables';

const ROLE_LABELS = {
    wrist_right: 'Right wrist',
    wrist_left: 'Left wrist'
};

function normalizeDeviceRole(role) {
    const r = String(role || 'wrist_right').toLowerCase();
    if (r === 'wrist_left' || r === 'wrist_right') return r;
    return 'wrist_right';
}

function formatRoleLabel(role) {
    if (!role) return 'Wrist';
    const k = String(role).toLowerCase();
    return ROLE_LABELS[k] || String(role).replace(/_/g, ' ');
}

function getExpectedWearableCount() {
    try {
        const n = parseInt(localStorage.getItem(LS_EXPECTED_WEARABLES) || '1', 10);
        return Math.max(1, Math.min(2, n > 0 ? n : 1));
    } catch (e) {
        return 1;
    }
}
