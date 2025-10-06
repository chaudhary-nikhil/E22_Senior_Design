#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float q0, q1, q2, q3; // unit quaternion
} fusion_quat_t;

void fusion_init(fusion_quat_t *q);
void fusion_update_madgwick(fusion_quat_t *q, float dt,
                            float ax, float ay, float az,
                            float gx, float gy, float gz);

#ifdef __cplusplus
}
#endif
