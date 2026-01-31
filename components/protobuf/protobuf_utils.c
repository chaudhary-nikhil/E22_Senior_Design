#include "protobuf_utils.h"
#include "imu_sample.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "PROTOBUF";

// Maximum samples per batch (must match .options file max_count)
#define MAX_BATCH_SAMPLES 256

// Estimated size per sample in protobuf format (with margin for 2-byte tags on fields 16+)
// Actual size ~125 bytes: fields 1-15 (~74B) + fields 16-25 (~48B) + submessage wrapper (~3B)
#define ESTIMATED_SAMPLE_SIZE 140

// Helper to copy bno055_sample_t to formsync_ImuSample
static void sample_to_proto(const bno055_sample_t *sample, formsync_ImuSample *msg)
{
    msg->t_ms = sample->t_ms;
    msg->ax = sample->ax;
    msg->ay = sample->ay;
    msg->az = sample->az;
    msg->gx = sample->gx;
    msg->gy = sample->gy;
    msg->gz = sample->gz;
    msg->mx = sample->mx;
    msg->my = sample->my;
    msg->mz = sample->mz;
    msg->roll = sample->roll;
    msg->pitch = sample->pitch;
    msg->yaw = sample->yaw;
    msg->qw = sample->qw;
    msg->qx = sample->qx;
    msg->qy = sample->qy;
    msg->qz = sample->qz;
    msg->lia_x = sample->lia_x;
    msg->lia_y = sample->lia_y;
    msg->lia_z = sample->lia_z;
    msg->temp = sample->temp;
    msg->sys_cal = sample->sys_cal;
    msg->gyro_cal = sample->gyro_cal;
    msg->accel_cal = sample->accel_cal;
    msg->mag_cal = sample->mag_cal;
}

// Helper to copy formsync_ImuSample to bno055_sample_t
static void proto_to_sample(const formsync_ImuSample *msg, bno055_sample_t *sample)
{
    sample->t_ms = msg->t_ms;
    sample->ax = msg->ax;
    sample->ay = msg->ay;
    sample->az = msg->az;
    sample->gx = msg->gx;
    sample->gy = msg->gy;
    sample->gz = msg->gz;
    sample->mx = msg->mx;
    sample->my = msg->my;
    sample->mz = msg->mz;
    sample->roll = msg->roll;
    sample->pitch = msg->pitch;
    sample->yaw = msg->yaw;
    sample->qw = msg->qw;
    sample->qx = msg->qx;
    sample->qy = msg->qy;
    sample->qz = msg->qz;
    sample->lia_x = msg->lia_x;
    sample->lia_y = msg->lia_y;
    sample->lia_z = msg->lia_z;
    sample->temp = msg->temp;
    sample->sys_cal = msg->sys_cal;
    sample->gyro_cal = msg->gyro_cal;
    sample->accel_cal = msg->accel_cal;
    sample->mag_cal = msg->mag_cal;
}

esp_err_t protobuf_encode_sample(const bno055_sample_t *sample,
                                  uint8_t *buffer, size_t buffer_size,
                                  size_t *encoded_size)
{
    if (!sample || !buffer || !encoded_size) {
        return ESP_ERR_INVALID_ARG;
    }

    // Create protobuf message
    formsync_ImuSample msg = formsync_ImuSample_init_zero;
    sample_to_proto(sample, &msg);

    // Create output stream
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    
    // Encode
    if (!pb_encode(&stream, formsync_ImuSample_fields, &msg)) {
        ESP_LOGE(TAG, "Encode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    *encoded_size = stream.bytes_written;
    return ESP_OK;
}

esp_err_t protobuf_encode_batch(const bno055_sample_t *samples, size_t sample_count,
                                 uint8_t *buffer, size_t buffer_size,
                                 size_t *encoded_size)
{
    if (!samples || !buffer || !encoded_size || sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Limit to max batch size
    if (sample_count > MAX_BATCH_SAMPLES) {
        ESP_LOGW(TAG, "Batch size %zu exceeds max %d, truncating", sample_count, MAX_BATCH_SAMPLES);
        sample_count = MAX_BATCH_SAMPLES;
    }

    // Create batch message with fixed array
    // Use heap_caps_calloc from internal RAM only (required for proper free() behavior)
    formsync_ImuSampleBatch *batch = heap_caps_calloc(1, sizeof(formsync_ImuSampleBatch), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!batch) {
        ESP_LOGE(TAG, "Failed to allocate batch structure from internal RAM");
        return ESP_ERR_NO_MEM;
    }
    
    batch->samples_count = (pb_size_t)sample_count;
    
    // Copy samples to batch
    for (size_t i = 0; i < sample_count; i++) {
        sample_to_proto(&samples[i], &batch->samples[i]);
    }

    // Create output stream
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    
    // Encode
    bool encode_ok = pb_encode(&stream, formsync_ImuSampleBatch_fields, batch);
    heap_caps_free(batch);  // Must use heap_caps_free for heap_caps_calloc
    
    if (!encode_ok) {
        ESP_LOGE(TAG, "Batch encode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    *encoded_size = stream.bytes_written;
    return ESP_OK;
}

esp_err_t protobuf_decode_sample(const uint8_t *buffer, size_t buffer_size,
                                  bno055_sample_t *sample)
{
    if (!buffer || !sample || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    formsync_ImuSample msg = formsync_ImuSample_init_zero;
    
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    
    if (!pb_decode(&stream, formsync_ImuSample_fields, &msg)) {
        ESP_LOGE(TAG, "Decode failed: %s", PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    proto_to_sample(&msg, sample);
    return ESP_OK;
}

esp_err_t protobuf_decode_batch(const uint8_t *buffer, size_t buffer_size,
                                 bno055_sample_t *samples, size_t max_samples,
                                 size_t *decoded_count)
{
    if (!buffer || !samples || !decoded_count || buffer_size == 0 || max_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate batch on heap - structure is too large for stack!
    // This matches protobuf_encode_batch which also uses heap allocation
    formsync_ImuSampleBatch *batch = calloc(1, sizeof(formsync_ImuSampleBatch));
    if (!batch) {
        ESP_LOGE(TAG, "Failed to allocate batch structure for decode");
        return ESP_ERR_NO_MEM;
    }
    
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    
    if (!pb_decode(&stream, formsync_ImuSampleBatch_fields, batch)) {
        ESP_LOGE(TAG, "Batch decode failed: %s", PB_GET_ERROR(&stream));
        free(batch);
        return ESP_FAIL;
    }

    // Copy samples from batch
    size_t to_copy = (batch->samples_count < max_samples) ? batch->samples_count : max_samples;
    for (size_t i = 0; i < to_copy; i++) {
        proto_to_sample(&batch->samples[i], &samples[i]);
    }

    *decoded_count = to_copy;
    free(batch);
    return ESP_OK;
}

size_t protobuf_estimate_batch_size(size_t sample_count)
{
    // Each sample is approximately 100-110 bytes in protobuf format
    // Add overhead for batch message wrapper and length prefixes
    return (sample_count * ESTIMATED_SAMPLE_SIZE) + 16;
}

esp_err_t protobuf_write_delimited(FILE *file, const bno055_sample_t *samples,
                                    size_t sample_count)
{
    if (!file || !samples || sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Limit batch size
    if (sample_count > MAX_BATCH_SAMPLES) {
        sample_count = MAX_BATCH_SAMPLES;
    }

    // Allocate encoding buffer from internal RAM only (required for proper free() behavior)
    size_t buffer_size = protobuf_estimate_batch_size(sample_count);
    uint8_t *buffer = heap_caps_malloc(buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes from internal RAM for protobuf encode buffer", buffer_size);
        return ESP_ERR_NO_MEM;
    }

    // Encode the batch
    size_t encoded_size = 0;
    esp_err_t err = protobuf_encode_batch(samples, sample_count, buffer, buffer_size, &encoded_size);
    if (err != ESP_OK) {
        heap_caps_free(buffer);
        return err;
    }

    // Write 4-byte little-endian length prefix
    uint32_t len = (uint32_t)encoded_size;
    if (fwrite(&len, sizeof(len), 1, file) != 1) {
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    // Write encoded data
    if (fwrite(buffer, 1, encoded_size, file) != encoded_size) {
        heap_caps_free(buffer);
        return ESP_FAIL;
    }

    heap_caps_free(buffer);  // Must use heap_caps_free for heap_caps_malloc
    return ESP_OK;
}

esp_err_t protobuf_read_delimited(FILE *file, bno055_sample_t *samples,
                                   size_t max_samples, size_t *decoded_count)
{
    if (!file || !samples || !decoded_count || max_samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *decoded_count = 0;

    // Read 4-byte length prefix
    uint32_t len = 0;
    if (fread(&len, sizeof(len), 1, file) != 1) {
        return ESP_ERR_NOT_FOUND;  // EOF
    }

    // Sanity check on length
    if (len == 0) {
        ESP_LOGE(TAG, "Invalid message length: 0");
        return ESP_FAIL;
    }
    if (len > 1024 * 1024) {  // Max 1MB message
        ESP_LOGE(TAG, "Invalid message length: %lu", (unsigned long)len);
        return ESP_FAIL;
    }

    // Allocate buffer - use malloc() which works for all heap types on ESP-IDF
    // On ESP-IDF, malloc() can allocate from PSRAM when internal RAM is low,
    // and free() correctly handles all heap types (internal RAM and PSRAM)
    uint8_t *buffer = malloc(len);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate %lu bytes for protobuf decode", (unsigned long)len);
        return ESP_ERR_NO_MEM;
    }

    // Read encoded data
    if (fread(buffer, 1, len, file) != len) {
        free(buffer);
        return ESP_FAIL;
    }

    // Decode the batch
    esp_err_t err = protobuf_decode_batch(buffer, len, samples, max_samples, decoded_count);
    
    // Free buffer - free() handles all heap types on ESP-IDF
    free(buffer);
    
    return err;
}

/**
 * @brief Peek at the sample count in the next length-delimited protobuf batch
 * 
 * Reads the length prefix and decodes just enough to get the sample count,
 * then seeks back to the original position. Used to check if a batch will fit
 * in available buffer space before reading it.
 * 
 * @param file File pointer (must be positioned at start of 4-byte length prefix)
 * @param sample_count Output: number of samples in the batch
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND at EOF, ESP_FAIL on error
 */
esp_err_t protobuf_peek_batch_size(FILE *file, size_t *sample_count) {
    if (!file || !sample_count) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Save current position
    long start_pos = ftell(file);
    if (start_pos < 0) {
        return ESP_FAIL;
    }
    
    // Read 4-byte length prefix
    uint32_t len = 0;
    if (fread(&len, sizeof(len), 1, file) != 1) {
        return ESP_ERR_NOT_FOUND;  // EOF
    }
    
    // Sanity check on length
    if (len == 0 || len > 1024 * 1024) {
        fseek(file, start_pos, SEEK_SET);  // Restore position
        return ESP_FAIL;
    }
    
    // Allocate buffer to read the batch
    uint8_t *buffer = malloc(len);
    if (!buffer) {
        fseek(file, start_pos, SEEK_SET);  // Restore position
        return ESP_ERR_NO_MEM;
    }
    
    // Read encoded data
    if (fread(buffer, 1, len, file) != len) {
        free(buffer);
        fseek(file, start_pos, SEEK_SET);  // Restore position
        return ESP_FAIL;
    }
    
    // Decode just to get sample count (allocate minimal batch structure)
    formsync_ImuSampleBatch *batch = calloc(1, sizeof(formsync_ImuSampleBatch));
    if (!batch) {
        free(buffer);
        fseek(file, start_pos, SEEK_SET);  // Restore position
        return ESP_ERR_NO_MEM;
    }
    
    pb_istream_t stream = pb_istream_from_buffer(buffer, len);
    bool decode_ok = pb_decode(&stream, formsync_ImuSampleBatch_fields, batch);
    
    if (decode_ok) {
        *sample_count = batch->samples_count;
    } else {
        *sample_count = 0;
    }
    
    free(batch);
    free(buffer);
    
    // Restore file position
    fseek(file, start_pos, SEEK_SET);
    
    return decode_ok ? ESP_OK : ESP_FAIL;
}

/**
 * @brief Skip a length-delimited protobuf message in a file
 * 
 * Used when we can't allocate memory to read a message - we skip it to continue
 * @param file File pointer (must be positioned at start of 4-byte length prefix)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND at EOF, ESP_FAIL on error
 */
esp_err_t protobuf_skip_delimited(FILE *file) {
    if (!file) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read 4-byte length prefix
    uint32_t len = 0;
    if (fread(&len, sizeof(len), 1, file) != 1) {
        return ESP_ERR_NOT_FOUND;  // EOF
    }
    
    // Sanity check on length
    if (len == 0) {
        ESP_LOGE(TAG, "Invalid message length: 0");
        return ESP_FAIL;
    }
    if (len > 1024 * 1024) {  // Max 1MB message
        ESP_LOGE(TAG, "Invalid message length: %lu", (unsigned long)len);
        return ESP_FAIL;
    }
    
    // Skip the protobuf data by seeking forward
    if (fseek(file, (long)len, SEEK_CUR) != 0) {
        ESP_LOGE(TAG, "Failed to seek past protobuf message of %lu bytes", (unsigned long)len);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}
