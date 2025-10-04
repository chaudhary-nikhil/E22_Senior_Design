#include "fusion/fusion.h"
#include <math.h>

void fusion_init(fusion_quat_t *q) {
    q->q0 = 1.f; q->q1 = q->q2 = q->q3 = 0.f;
}

// Placeholder: integrates gyro only (for template). Replace with real Madgwick later.
void fusion_update_madgwick(fusion_quat_t *q, float dt,
                            float ax, float ay, float az,
                            float gx, float gy, float gz) {
    (void)ax; (void)ay; (void)az; // unused in placeholder
    // Integrate small-angle gyro (rad/s) into quaternion very crudely
    float half_dt = 0.5f * dt;
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    q->q0 += (-q1*gx - q2*gy - q3*gz) * half_dt;
    q->q1 += ( q0*gx + q2*gz - q3*gy) * half_dt;
    q->q2 += ( q0*gy - q1*gz + q3*gx) * half_dt;
    q->q3 += ( q0*gz + q1*gy - q2*gx) * half_dt;
    // Normalize
    float n = 1.0f / sqrtf(q->q0*q->q0 + q->q1*q->q1 + q->q2*q->q2 + q->q3*q->q3);
    q->q0 *= n; q->q1 *= n; q->q2 *= n; q->q3 *= n;
}
