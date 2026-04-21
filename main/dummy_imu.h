#pragma once

#include "bno055.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Replay dummy IMU samples derived from demo_jsons.
 */
void dummy_imu_reset(void);

/**
 * Fill the next sample from the replay stream.
 * Returns false only if arguments are invalid.
 */
bool dummy_imu_next_sample(bno055_sample_t *out);

#ifdef __cplusplus
}
#endif
