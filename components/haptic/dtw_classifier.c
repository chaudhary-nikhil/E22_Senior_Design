#include "dtw_classifier.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_DTW_LEN 200

// Helper to compute Euclidean distance between two 3D float arrays
static inline float dist_sq(const float *a, const float *b) {
  float dx = a[0] - b[0];
  float dy = a[1] - b[1];
  float dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

static inline float min3(float a, float b, float c) {
  float m = a < b ? a : b;
  return m < c ? m : c;
}

float process_dtw_buffer(const float *current, int cur_count,
                         const float *ideal, int ideal_count) {
  if (cur_count == 0 || ideal_count == 0)
    return 1.0f;
  if (cur_count > MAX_DTW_LEN)
    cur_count = MAX_DTW_LEN;
  if (ideal_count > MAX_DTW_LEN)
    ideal_count = MAX_DTW_LEN;

  // Allocate 1D array to represent the previous row and current row to save
  // memory. Instead of a full O(N*M) grid, we only need O(M) memory.
  float *prev_row = (float *)malloc((ideal_count + 1) * sizeof(float));
  float *curr_row = (float *)malloc((ideal_count + 1) * sizeof(float));

  if (!prev_row || !curr_row) {
    if (prev_row)
      free(prev_row);
    if (curr_row)
      free(curr_row);
    return 1.0f;
  }

  // Initialize the boundaries
  for (int j = 0; j <= ideal_count; j++) {
    prev_row[j] = INFINITY;
  }
  prev_row[0] = 0.0f;

  // DTW DP computation
  for (int i = 1; i <= cur_count; i++) {
    curr_row[0] = INFINITY;
    for (int j = 1; j <= ideal_count; j++) {
      float cost = sqrtf(dist_sq(&current[(i - 1) * 3], &ideal[(j - 1) * 3]));

      float match = prev_row[j - 1];  // Diagonal
      float insert = curr_row[j - 1]; // Left
      float delete = prev_row[j];     // Top

      curr_row[j] = cost + min3(match, insert, delete);
    }
    // Swap rows
    float *temp = prev_row;
    prev_row = curr_row;
    curr_row = temp;
  }

  float final_dist = prev_row[ideal_count];

  free(prev_row);
  free(curr_row);

  // Normalize path length. The actual path length in DTW is roughly between
  // max(N, M) and N+M. We normalize by the length of the ideal sequence to keep
  // scores consistent. Divide by ~5.0f to scale the deviation score around 0-1
  // range for a typical "bad" stroke.
  return (final_dist / (float)ideal_count) / 5.0f;
}
