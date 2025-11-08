/**
 * @file voice_nr.cpp
 * @brief Voice processing and USB audio ring-buffer management using FreeRTOS and esp-afe.
 *
 * This collection of functions and the VoiceNR helper class implement a small
 * voice-processing pipeline for an ESP32-based system. Audio frames are
 * fetched from an Audio Front End (AFE) implementation, optionally processed
 * by noise-reduction/speech-recognition models, and copied into a shared
 * ring buffer for USB audio transmission.
 *
 * The implementation expects several external symbols and globals to be
 * provided by the application or other modules (examples: afe interfaces,
 * USB audio buffer pointers, synchronization flags). The module creates a
 * FreeRTOS task pinned to a CPU core to continuously fetch and copy AFE
 * output into the ring buffer.
 *
 * -------------------------------------------------------------------------
 * Global/External Dependencies
 * -------------------------------------------------------------------------
 * - audio_Rbuffer (uint8_t*): pointer to the raw byte ring buffer used for
 *   USB audio data. VoiceNR constructor initializes this pointer.
 * - _AucRB_len (int): length (bytes) of audio_Rbuffer.
 * - RingBufPtr (int): current write index into audio_Rbuffer (module uses and
 *   updates this to append new chunks).
 * - abPtr_USb (int): current read/transfer index used by the USB audio writer.
 * - AUC_cb_len (int): a length/offset used to compute overlap windows between
 *   RingBufPtr and abPtr_USb.
 * - ESP_SR (bool-like): flag indicating whether the speech/AFE subsystem is
 *   armed/run-enabled; afe_fetch_Task polls this before actively fetching.
 * - Txt2Dsply_mutx (SemaphoreHandle_t) and Print2Dsply(): optional display
 *   primitives used by esp_sr_init() to print model information.
 *
 * Note: These symbols are expected to be defined elsewhere in the system.
 *
 * -------------------------------------------------------------------------
 * Threading and Real-Time Behavior
 * -------------------------------------------------------------------------
 * - afe_fetch_Task runs as a FreeRTOS task. It continuously:
 *     1. waits for DSP_ON/ESP_SR flag to be true,
 *     2. calls afe_handle->fetch() to obtain processed audio frames,
 *     3. copies audio bytes safely into the ring buffer while avoiding
 *        overwriting unsent USB data (detects overlap with abPtr_USb),
 *     4. wraps RingBufPtr at _AucRB_len.
 *
 * - afe_fetch_Task uses vTaskDelay() for sleep/backoff when:
 *     - the AFE is not armed,
 *     - overlap with USB read pointer is detected and it must wait,
 *     - other commented synchronization/timing heuristics are present and
 *       can be re-enabled as needed.
 *
 * - esp_sr_init() may create the afe_fetch_Task with xTaskCreatePinnedToCore.
 * - voice_nr_init() creates a standalone afe_fetch_Task via xTaskCreate and
 *   returns ESP_OK or ESP_FAIL depending on task creation success.
 *
 * - The code expects real-time behavior to be guided by the AFE feed chunk
 *   size (number of samples) and the USB transfer rate. Some timing code is
 *   present but commented out; the system currently relies on the periodic
 *   arrival of AFE frames and the USB endpoint consumption rate for pacing.
 *
 * -------------------------------------------------------------------------
 * afe_fetch_Task
 * -------------------------------------------------------------------------
 * @brief Fetch AFE-processed audio and append it to the shared USB ring buffer.
 *
 * @param arg Pointer to afe_task_into_t containing:
 *    - afe_handle: pointer to esp_afe_sr_iface_t (AFE interface)
 *    - afe_data: pointer to esp_afe_sr_data_t (AFE instance data)
 *
 * Behavior:
 * - Queries the AFE for feed chunk size and uses it to compute the number of
 *   output bytes per fetch (FdChnkBytes = 2 * feed_size) assuming 16-bit
 *   samples.
 * - Waits for a global arm/run flag (ESP_SR) before fetching data.
 * - Calls afe_handle->fetch(afe_data) to obtain processed audio frames
 *   (returns afe_fetch_result_t* with int16_t* data).
 * - Ensures that a write of FdChnkBytes does not overlap with the yet-to-be-
 *   sent USB region (abPtr_USb .. abPtr_USb + AUC_cb_len). If overlap is
 *   possible, the task delays briefly and re-checks until safe.
 * - Writes contiguous bytes into audio_Rbuffer if enough tail room exists.
 *   Otherwise writes sample-by-sample with wrap-around handling.
 *
 * Concurrency / Safety:
 * - The task coordinates with the USB writer via the shared indices
 *   (RingBufPtr, abPtr_USb). Both indices must be updated atomically by
 *   their respective owners to avoid corruption (not enforced here).
 * - The code assumes audio_Rbuffer and indices are visible and maintained by
 *   other components; no mutex is used for the ring buffer itself.
 *
 * Notes:
 * - Several timing/synchronization heuristics are present but commented out,
 *   including explicit delays to match sample-rate pacing and periodic
 *   diagnostic counters. They can be re-enabled for tighter control.
 * - The task loops indefinitely until the process terminates or the task is
 *   explicitly deleted.
 *
 * -------------------------------------------------------------------------
 * VoiceNR class
 * -------------------------------------------------------------------------
 * @brief Lightweight wrapper that binds a ring-buffer pointer/length and
 *        initializes the AFE/speech model pipeline.
 *
 * Constructor:
 * - VoiceNR(uint8_t *audio_RbufferPtr, int aucrb_len)
 *     - Stores audio_RbufferPtr into the module-global audio_Rbuffer and
 *       stores aucrb_len into _AucRB_len so the fetching task can write into
 *       the expected memory region.
 *     - Ownership: The class does NOT allocate or free audio_Rbuffer; the
 *       caller retains ownership and must ensure the buffer remains valid for
 *       the lifetime of the task(s) that use it.
 *
 * Destructor:
 * - ~VoiceNR()
 *     - Currently a no-op. If task shutdown or resource cleanup is required,
 *       the destructor should be extended to stop the fetch task and release
 *       AFE resources.
 *
 * esp_sr_init():
 * - @brief Initialize speech-recognition models and the AFE based on a model
 *   name filter, configure AFE options, and create/start a fetch task.
 * - Key actions:
 *     - Initializes model metadata via esp_srmodel_init("model"),
 *       enumerates and optionally prints available models.
 *     - Filters models by a prefix (ESP_NSNET_PREFIX) to select NS model.
 *     - Prepares afe_config via afe_config_init and sets multiple options:
 *         - disables wakenet and VAD initialization here by default,
 *         - picks preferred core/priority, memory alloc mode (e.g. PSRAM),
 *         - enables NS/AGC per the configuration,
 *         - points ns_model_name to the filtered model.
 *     - Creates afe_handle and afe_data from the configuration and spawns
 *       afe_fetch_Task pinned to afe_config->afe_perferred_core.
 * - Synchronization:
 *     - Uses Txt2Dsply_mutx to serialize display updates when printing
 *       model information. If that semaphore is missing, printing should be
 *       guarded by other means.
 *
 * feed(int16_t *feed_buff):
 * - @brief Send a PCM frame (or frames) to the AFE input pipeline.
 * - @param feed_buff Pointer to interleaved/int16_t samples matching the
 *   expected feed chunk size and channel count used when the AFE was created.
 * - Behavior: simply forwards the buffer to afe_handle->feed(afe_data, feed_buff).
 *
 * -------------------------------------------------------------------------
 * voice_nr_init
 * -------------------------------------------------------------------------
 * @brief Public initializer that creates the afe_fetch_Task as a FreeRTOS task.
 *
 * Prototype:
 *   extern "C" esp_err_t voice_nr_init(uint32_t stack_size_bytes = 4096,
 *                                      UBaseType_t priority = 5)
 *
 * Parameters:
 * - stack_size_bytes: desired stack size in bytes for the created task.
 *   Note: the call translates this to words/StackType_t for xTaskCreate on
 *   platforms where the task creation expects stack words instead of bytes.
 * - priority: FreeRTOS task priority for the spawned task.
 *
 * Return:
 * - ESP_OK if the task was created successfully.
 * - ESP_FAIL if xTaskCreate fails (task not created).
 *
 * Usage / Limitations:
 * - This helper provides a minimal framework to create a fetching task if a
 *   caller wants a standalone task separate from esp_sr_init().
 * - The created task receives nullptr as its parameter in the default usage
 *   here; if afe_fetch_Task expects a valid afe_task_into_t pointer, the
 *   caller should instead construct and pass a properly initialized structure.
 *
 * -------------------------------------------------------------------------
 * Implementation Notes & Recommendations
 * -------------------------------------------------------------------------
 * - Ensure external indices (RingBufPtr, abPtr_USb, AUC_cb_len) are updated
 *   consistently and with appropriate atomicity or protected by a mutex if
 *   updated concurrently by multiple tasks.
 * - The module currently assumes 16-bit PCM (int16_t) samples when converting
 *   to bytes. If other bit-depths or formats are used, adjust the copying
 *   logic accordingly.
 * - Consider exposing a controlled start/stop API that can safely stop the
 *   fetch task, destroy the AFE instance, and release resources. The current
 *   destructor is a no-op and resource management is manual.
 * - Re-enable or refine timing logic if precise pacing relative to USB
 *   transfers is required (e.g., to maintain jitter bounds or to implement
 *   flow-control heuristics).
 *
 * -------------------------------------------------------------------------
 * Warnings
 * -------------------------------------------------------------------------
 * - The module writes directly into a shared ring buffer with no built-in
 *   mutex around the buffer updates. Correct operation depends on careful
 *   coordination with the USB writer and proper memory visibility across
 *   tasks/cores.
 * - Many of the configuration and AFE APIs are platform-specific (esp-afe /
 *   esp-sr). This documentation assumes those APIs behave as described by the
 *   surrounding code; consult their documentation for exact semantics.
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <inttypes.h>
#include "lv_port_disp.h"
#include "Text2Dsply.h"
#include "freertos/semphr.h"
#include "voice_nr.h"

static const char *TAG = "voice_nr";
uint8_t *audio_Rbuffer = nullptr;
int _AucRB_len;


// Weak hooks: if the esp-sr / adc-mic / usb-audio components are present they
// should provide strong implementations of these symbols. Otherwise the
// weak stubs below will run (no-op).
// extern "C" void esp_sr_init(void) __attribute__((weak));
// extern "C" void esp_sr_start(void) __attribute__((weak));
// extern "C" void esp_adc_mic_init(void) __attribute__((weak));
// extern "C" void usb_audio_init(void) __attribute__((weak));

// Default (weak) implementations: they are intentionally lightweight so this
// file can compile and link even if esp-sr and other components are not
// included. If the real components are linked, their functions will override
// these weak stubs.
//extern "C" void esp_sr_init(void)


/*** @brief Task function to fetch processed audio data from the AFE (Audio Front End) 
 * and manage its transfer to a ring buffer for USB audio output.
 *
 * This FreeRTOS task continuously fetches audio data from the AFE, handles buffer alignment to prevent overwriting unsent data,
 * and synchronizes with the outgoing USB audio stream. It manages overlap detection between ring buffer pointers, copies audio
 * data into the ring buffer, and ensures timing consistency with the USB audio transfer rate.
 *
 * @note
 * - The task waits for 'DSP_ON' flag to be true, before starting audio fetch operations.
 * - Uses vTaskDelay for timing and synchronization.
 * - Periodically updates counters and prints status information for monitoring.
 */
static void afe_fetch_Task(void *arg)
{
    ESP_LOGI(TAG, "afe_fetch_Task starting");
    auto *afe_task_info = static_cast<afe_task_into_t *>(arg);
    esp_afe_sr_iface_t *afe_handle = afe_task_info->afe_handle;
    esp_afe_sr_data_t *afe_data = afe_task_info->afe_data;
    int feed_size = afe_handle->get_feed_chunksize(afe_data);
    int detect_cnt = 0;
    int64_t start = 0;
    int FdChnkBytes = 2 * feed_size;
    int32_t localCntr = 0;

    while (ESP_SR)
    {
        while (!ESP_SR)
        {
            // arm = true;
            // cntr = 0;
            detect_cnt = 0;
            vTaskDelay(pdMS_TO_TICKS(50));
            start = esp_timer_get_time();
            // EvntStart = start;
        }

        afe_fetch_result_t *result = afe_handle->fetch(afe_data);
        int16_t *processed_audio = result->data;
        localCntr += FdChnkBytes;

        detect_cnt++;
        if (detect_cnt == 2)
        {
            detect_cnt = 0;
            bool ovrlp = true;
            int RingBufPtrMax = RingBufPtr + FdChnkBytes;
            while (ovrlp)
            {
                int USBptrMax = AUC_cb_len + abPtr_USb;
                if (((RingBufPtr > abPtr_USb) && (RingBufPtr < USBptrMax)) ||
                    ((RingBufPtrMax > abPtr_USb) && (RingBufPtrMax < USBptrMax)))
                {
                    ovrlp = true;
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                else
                    ovrlp = false;
            }
        }

        if (RingBufPtr + FdChnkBytes <= _AucRB_len)
        {
            memcpy(&audio_Rbuffer[RingBufPtr], processed_audio, FdChnkBytes);
            RingBufPtr += FdChnkBytes;
            if (RingBufPtr >= _AucRB_len)
                RingBufPtr = 0;
        }
        else
        {
            for (int Ptr = 0; Ptr < FdChnkBytes / 2; Ptr++)
            {
                uint16_t cursmpl = processed_audio[Ptr];
                audio_Rbuffer[RingBufPtr++] = static_cast<uint8_t>(cursmpl & 0xFF);
                audio_Rbuffer[RingBufPtr++] = static_cast<uint8_t>(cursmpl >> 8);
                if (RingBufPtr >= _AucRB_len)
                    RingBufPtr = 0;
            }
        }
    }
    vTaskDelete(nullptr);
}

VoiceNR::VoiceNR(uint8_t *audio_RbufferPtr, int aucrb_len)
{
    audio_Rbuffer = audio_RbufferPtr;
    _AucRB_len = aucrb_len;
}
VoiceNR::~VoiceNR()
{
}

void VoiceNR::esp_sr_init(void) 
{
    ESP_LOGW(TAG, "esp_sr_init()");
    char TxtBuf[70];
    /*Initialize speech recognition models*/
    vTaskDelay(pdMS_TO_TICKS(1000));
    models = esp_srmodel_init("model");
    if (models->num > 0)
    {
        for (int i = 0; i < models->num; i++)
        {
            model_name = models->model_name[i];
            printf("%d. model_name: %s\n", i, model_name);
            snprintf(TxtBuf, sizeof(TxtBuf), "%d. model_name: %s\n", i, model_name);
            printf("%s", TxtBuf);
            if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
            {
                Print2Dsply(TxtBuf);
                xSemaphoreGive(Txt2Dsply_mutx);
            }
        }
    }
    else
    {
        snprintf(TxtBuf, sizeof(TxtBuf), "models->num == 0\n");
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
    }

    model_name = esp_srmodel_filter(models, ESP_NSNET_PREFIX, nullptr);
    if (model_name == nullptr)
    {
        snprintf(TxtBuf, sizeof(TxtBuf), "Filtered model_name = NULL\n");
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
    }
    else
    {
        snprintf(TxtBuf, sizeof(TxtBuf), "Filtered model_name = %s\n", model_name);
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
    }
    afe_config = (afe_config_init("M", models, AFE_TYPE_VC, AFE_MODE_HIGH_PERF)); // AFE_MODE_LOW_COST
    afe_config->wakenet_init = false;
    afe_config->vad_init = false;
    afe_config->afe_perferred_core = 1;
    afe_config->afe_perferred_priority = 21;
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config->ns_init = true;
    afe_config->agc_init = true;
    afe_config->agc_mode = AFE_AGC_MODE_WAKENET;
    afe_config->afe_ns_mode = AFE_NS_MODE_NET;
    afe_config->ns_model_name = model_name;
    afe_config_print(afe_config);
    afe_handle = esp_afe_handle_from_config(afe_config);
    afe_data = afe_handle->create_from_config(afe_config);
    int feed_nch = afe_handle->get_feed_channel_num(afe_data);
    feed_size = afe_handle->get_feed_chunksize(afe_data);
    //auto *feed_buff = static_cast<int16_t *>(malloc(feed_size * feed_nch * sizeof(int16_t)));
    task_info.afe_data = afe_data;
    task_info.afe_handle = afe_handle;
    //xTaskCreatePinnedToCore(afe_fetch_Task, "fetch_task", 8 * 1024, &task_info, 8, &task_info.fetch_task, afe_config->afe_perferred_core);
};
void VoiceNR::feed(int16_t *feed_buff) 
{
    /*send this chunk to the sound processor*/
    afe_handle->feed(afe_data, feed_buff);
};
esp_err_t VoiceNR::TaskStart(uint32_t stack_size_bytes, UBaseType_t priority)
{
    BaseType_t res = xTaskCreatePinnedToCore(
        afe_fetch_Task,
        "afe_fetch_Task",
        (stack_size_bytes ), // xTaskCreate expects words on some ports; keep conservative
        &task_info,
        priority,
        &task_info.fetch_task,
        afe_config->afe_perferred_core);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create voice_nr task (res=%d)", (int)res);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "voice_nr task created");
    return ESP_OK;
};
esp_err_t VoiceNR::TaskStop(void){
    // To be implemented if needed
    vTaskDelete(task_info.fetch_task);
    return ESP_OK;
};
// extern "C" void esp_sr_start(void)
// {
//     ESP_LOGW(TAG, "esp_sr_start() weak stub called - real esp-sr not linked");
// }
// extern "C" void esp_adc_mic_init(void)
// {
//     ESP_LOGW(TAG, "esp_adc_mic_init() weak stub called - adc mic init skipped");
// }
// extern "C" void usb_audio_init(void)
// {
//     ESP_LOGW(TAG, "usb_audio_init() weak stub called - usb audio init skipped");
// }


// Public initializer: create the FreeRTOS task. Returns ESP_OK on success.
/*Currently not used, but left in place, if later needed as (a framework) for a start/stop task strategy*/
extern "C" esp_err_t voice_nr_init(uint32_t stack_size_bytes = 4096, UBaseType_t priority = 5)
{
    BaseType_t res = xTaskCreate(
        afe_fetch_Task,
        "afe_fetch_Task",
        (stack_size_bytes / sizeof(StackType_t)), // xTaskCreate expects words on some ports; keep conservative
        nullptr,
        priority,
        nullptr);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "Failed to create voice_nr task (res=%d)", (int)res);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "voice_nr task created");
    return ESP_OK;
}