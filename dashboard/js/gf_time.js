/**
 * GoldenForm  --  display times in US Eastern (handles DST; shown as "ET").
 */
function formatEasternDateTime(msOrIso) {
    let d;
    if (msOrIso == null || msOrIso === '') return '-';
    if (typeof msOrIso === 'number') d = new Date(msOrIso);
    else d = new Date(String(msOrIso));
    if (isNaN(d.getTime())) return '-';
    return d.toLocaleString('en-US', {
        timeZone: 'America/New_York',
        month: 'short',
        day: 'numeric',
        year: 'numeric',
        hour: 'numeric',
        minute: '2-digit',
        hour12: true
    }) + ' ET';
}
