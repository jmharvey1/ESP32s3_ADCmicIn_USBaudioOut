/* created 2025-11-07 JMH
 * voice_nr.h
 *
 * Header file for VoiceNR class that provides noise reduction functionality
 * using ESP32s3 AFE and esp-sr components.
 *
 * This class encapsulates the initialization, configuration, and processing
 * of audio data for noise reduction purposes. It allows users to easily
 * integrate noise reduction into their applications by providing a simple
 * interface for feeding audio data and retrieving processed output.
 *
 * Note: This implementation relies on the ESP32s3 AFE and esp-sr libraries.
 * Ensure that these dependencies are properly included in your project.
 */
#ifndef VOICE_NR_H
#define VOICE_NR_H

#include <cstdint>
#include <cstddef>
#include "src/include/model_path.h"
#include "include/esp32s3/esp_afe_config.h"
#include "include/esp32s3/esp_afe_sr_models.h"
#include "esp_nsn_models.h"
#include "esp_afe_sr_models.h"

extern int RingBufPtr;
extern int AUC_cb_len;
extern int abPtr_USb;

class VoiceNR {
public:
    // Create/destroy
    VoiceNR(uint8_t *audio_RbufferPtr, int aucrb_len = 12288);
    ~VoiceNR();

    // Non-copyable
    VoiceNR(const VoiceNR&) = delete;
    VoiceNR& operator=(const VoiceNR&) = delete;

    // Initialize the noise reduction engine.
    // sampleRate: audio sample rate in Hz (e.g. 16000, 48000)
    // channels: number of interleaved channels (1 = mono, 2 = stereo)
    
    void esp_sr_init(void);
    void feed(int16_t *feed_buff);
    esp_err_t TaskStart(uint32_t stack_size_bytes, UBaseType_t priority); //uint32_t stack_size_bytes = 4096, UBaseType_t priority = 5
    esp_err_t TaskStop(void);

private:
    // struct Impl;
    // Impl* impl_;
    int feed_size = 512;
    afe_config_t *afe_config;
    esp_afe_sr_iface_t *afe_handle;
    esp_afe_sr_data_t *afe_data;
    srmodel_list_t *models;
    char *model_name = nullptr;
    afe_task_into_t task_info{nullptr, nullptr, nullptr, nullptr};
};

#endif // VOICE_NR_H