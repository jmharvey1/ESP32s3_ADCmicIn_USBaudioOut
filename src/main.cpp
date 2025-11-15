/*Build notes:
commands needed to build/move the srmodels.bin file to the build directory & flash to esp32s3:
1st, review 'README.rst' file in the 'components/esp-sr' directory
2nd, from 'Platformio' run 'New Terminal' command
3rd, from the new terminal window, run the following command:
    . /home/jim/.platformio/packages/framework-espidf/export.sh
    (creates needed env. vars. for python)
4th, run the following command:
    python3.13 ./components/esp-sr/model/movemodel.py -d1 ./sdkconfig.esp32-s3-devkitc-1 -d2 ./components/esp-sr -d3 .pio/build/esp32-s3-devkitc-1
    (this creates the 'srmodels.bin' file in the build directory)
5ft, run the following command:
    close the terminal, & then simply reopen the terminal, before running the following command:
    esptool.py -p /dev/ttyACM0 -b 460800 --before default-reset --after hard-reset --chip esp32s3 write_flash --flash-mode dio --flash-size detect --flash-freq 40m 0x250000 .pio/build/esp32-s3-devkitc-1/srmodels/srmodels.bin
    (this flashes the 'srmodels.bin' file to the esp32s3)
    */
// Best practices applied:
// - Consistent naming conventions
// - Proper use of const and static where appropriate
// - Reduced global variables, grouped related variables in structs where possible
// - Added missing includes guards in headers (if any)
// - Improved comments and removed commented-out code blocks
// - Used nullptr instead of NULL for C++ code
// - Used C++ style casts
// - Used constexpr for constants
// - Used RAII for memory management where possible
// - Checked return values for error handling
// - Minimized use of magic numbers
// - Used enum for states/flags where appropriate
// - Improved function signatures and parameter passing
// - Used std::array or std::vector for buffers if possible (if not, kept C arrays for compatibility)
// - Ensured thread safety and proper synchronization
// - Used static functions for internal linkage
// - Used modern C++ features where possible, but kept compatibility with ESP-IDF and FreeRTOS

// Note: Some ESP-IDF APIs are C-based and require C-style code, so not all C++ best practices can be applied.
/*20251109 added display button to support swapping Audio Output modes I2s-PDM/USB-UAC 
*        Note: to complete the swap, a 'reset', or 'power off/power on' operation has to follow the button selection
*/
/* 20251115 Text2Dsply.cpp - Changed Button1 event detectioin to 'LV_EVENT_SHORT_CLICKED' to reduce false detections*/
#include <stdio.h>
#include <inttypes.h>
#include <cmath>
#include <cstring>

#include "../../managed_components/espressif__esp-dsp/modules/fft/include/dsps_fft2r.h"
#include "dsps_math.h"
#include "esp_dsp.h"
#include "LP_Filter_Coef.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "adc_mic.h"
#include "esp_codec_dev.h"
#include "lv_port_disp.h"
#include "Text2Dsply.h"
#include "esp_err.h"
#include "usb_device_uac.h"
#include "espressif__tinyusb/src/class/audio/audio_device.h"
#include "dcd.h"
//I2S & PDM drivers
#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#include "soc/clk_tree_defs.h"
#include "hal/i2s_types.h"
/*Added the following to populate the chart display 'Y' values used in the FFT Spectrum view*/
#include "lvgl/src/widgets/chart/lv_chart.h"
#include "lvgl/src/widgets/chart/lv_chart_private.h"

#include "voice_nr.h"
// save settings to NVS
#include "NVS_Suprt.h"
// Global configuration flags
#define I2S_PDM_TX_PIN_CLK GPIO_NUM_43 //USE Waveshare UART2 TX as PDM clock out(Audio out)
#define I2S_PDM_TX_PIN_DATA GPIO_NUM_44 //USE Waveshare UART2 RX as PDM data out(Audio out)
#define I2S_PDM_TX_CHANNEL  I2S_NUM_0 // PDM TX is only supported on I2S0 on ESP32-S3
constexpr int SAMPLING_RATE = 16000;
constexpr int ADC_SAMPLE_CNT = 1024;
constexpr int CUTOFF_FREQ = 2000;
constexpr int AucRB_len = 12288;

//Wiener Filter parameters
#define FFT_SIZE ADC_SAMPLE_CNT//256 // or 512, depending on latency/memory constraints

float input_buffer[FFT_SIZE];
float output_buffer[FFT_SIZE];
float fft_real[FFT_SIZE];
float fft_imag[FFT_SIZE];
float fft_Cmplx[2*FFT_SIZE];
float wind[FFT_SIZE];
float noise_psd[FFT_SIZE/2];
float signal_psd[FFT_SIZE/2];
float alphaW = 0.98f; // Smoothing factor for PSD estimation
int PkFreqBin = 48;
float Bstfreq = 750.0f;
float ph = 0.0f;
float AvgMag = 0.0f;
float AvgTone = 750.0f;
float MaxMag = 0.0f;
float Sqlcthresh = 100.0f;
static const char *TAG = "JMH Test";

// Buffers
static uint8_t Adc_buffer[2048];
static uint8_t audio_Rbuffer[AucRB_len];

// Synchronization
static SemaphoreHandle_t AuBufr_semaphore = nullptr;

// State variables
static unsigned long NSEvntStart = 0;
static int64_t UACcb_Start = 0;
static uint64_t LastStart = 0;
static int32_t dacSmplCnt = 0;
static int32_t AUCSmplCnt = 0;
static int32_t afeSmplCnt = 0;
static int afe_intrvl = 0;
static int cntr = 0;

static int abPtr_Adc = 0;
int abPtr_USb = 0;
int RingBufPtr = 0;
int AUC_cb_len = 0;
static bool MicDatRdy = false;
static bool wait4RB = false;
static bool afe_fetch_Task_run = false;
static adc_continuous_handle_t ADC_Hndl = nullptr;
static esp_codec_dev_handle_t dev = nullptr;
static TaskHandle_t Read_ADC_TaskHndl = nullptr;

// DSP filter coefficients
static float in_z2 = 0, in_z1 = 0, out_z2 = 0, out_z1 = 0;
static float in_z3 = 0, in_z4 = 0, out_z3 = 0, out_z4 = 0, out_z5 = 0;
static float a0 = 0.08599609327133607f;
static float a1 = 0.0f;
static float a2 = -0.08599609327133607f;
static float b0 = 1.0f;
static float b1 = -1.309777513510262f;
static float b2 = 0.8280078134573279f;

// Lo-pass filter state
static float prev_sample = 0.0f;

//I2S PDM TX handle
i2s_chan_handle_t tx_handle = nullptr;
//I2S/PDM Flag
bool i2s_pdm_running = false;
// Forward declarations
extern "C" void app_main();
static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg);
static void Mic_ADC_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static void Calc_IIR_BPFltrCoef(float Fc, float Fs, float Q);

// I2S write task variables
TaskHandle_t i2s_task_handle = NULL;
size_t bytes_written;
constexpr int NUM_SAMPLES = 256;// translates to 16ms @ 16kHz and or 512 bytes
int16_t i2s_samples[NUM_SAMPLES];  // Buffer for I2S samples translates 512 bytes
float SINE_FREQ_HZ = 750.0f;
short int *p = &i2s_samples[0];
void *i2s_samplesPtr = (void *)p;
const uint8_t *p2 = &audio_Rbuffer[0];
void *audio_RbufferPtr = (void *)p2;
int byteStep = sizeof(int16_t);
int i2s_sample_size = sizeof(i2s_samples);
int AudioOutMode = 0; //0=I2S PDM, 1=USB Audio  

VoiceNR voiceNR(&audio_Rbuffer[0], AucRB_len); // Create an instance of the VoiceNR class
NVS_Suprt nvs_suprt; // Create an instance of the NVS_Suprt class

///////////////////////////////////////////////////////////////////
void i2s_write_task(void *arg) {
    esp_err_t ret;
    while (i2s_pdm_running) {
        if (MicDatRdy) {

            int Bfptr = 0;
            int offset = 0;
            while (Bfptr < i2s_sample_size)
            {
                memcpy(i2s_samplesPtr + Bfptr, audio_RbufferPtr+Bfptr + abPtr_USb - offset, byteStep);
                Bfptr += byteStep;
                //printf("%d\n", i2s_samples[Bfptr/2 -1]);
                if (Bfptr + abPtr_USb == AucRB_len)
                {
                    abPtr_USb = 0;
                    offset = Bfptr;
                }
            }
            AUCSmplCnt += i2s_sample_size;
            abPtr_USb += (Bfptr - offset);


            size_t bytes_written;
            ret = i2s_channel_write(tx_handle, i2s_samples, i2s_sample_size, &bytes_written, portMAX_DELAY);
            if (ret != ESP_OK) {
            ESP_LOGE("PDM_TX_EXAMPLE", "Failed to write to I2S PDM TX channel: %s", esp_err_to_name(ret));
            }
            abPtr_Adc += bytes_written;
            //printf("I2S written bytes: %d, abPtr_Adc: %d\n", bytes_written, abPtr_Adc);
            if (abPtr_Adc >= sizeof(Adc_buffer)) { //bytes_written) should always be 512 which is an interger multiple of Adc_buffer size
                abPtr_Adc = 0;
                //MicDatRdy = false;
            }
        } else {
           // if(!MicDatRdy) printf("!MicDatRdy\n");
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    vTaskDelete(nullptr);
    ret = i2s_channel_disable(tx_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE("PDM_TX_EXAMPLE", "Failed to Disable I2S PDM TX channel: %s", esp_err_to_name(ret));
        return;
    }
}

//Wiener filter functions
void estimate_noise(float *mag, int len) {
    // Assume first N frames are noise only, or use VAD
    float avgNoiseMag = 0.0f;
    for (int i = 0; i < len; i++) {
        if(i == PkFreqBin) { // || i == 464
           //noise_psd[i] = (noise_psd[i+10] + noise_psd[i-10])/2; //avoid notch frequencies
            noise_psd[i] = (noise_psd[i+3] + noise_psd[i-3])/2; //avoid notch frequencies
        }
        else{
            //noise_psd[i] = alphaW * noise_psd[i] + (1.0f - alphaW) * mag[i] * mag[i];
            noise_psd[i] =  mag[i]; // Temporary: use current magnitude as noise PSD
        }
        if (i >= 44 && i <= 56) //only look/test for frequencies between 450 & 950 Hz
        {
             avgNoiseMag += noise_psd[i];
        }
    }
    avgNoiseMag /= (56 - 44 + 1);
    Sqlcthresh = avgNoiseMag * 4.0f; //set threshold for signal detection
}

void estimate_signal(float *mag, int len) {
    float CurMaxMag = 00.0f;
    int curPkFreqBin = 0;
    for (int i = 0; i < len; i++)
    {
        // signal_psd[i] = alphaW * signal_psd[i] + (1.0f - alphaW) * mag[i] * mag[i];
        signal_psd[i] = mag[i]; // Temporary: use current magnitude as signal PSD
        if (i >= 28 && i <= 61) //only look/test for frequencies between 450 & 950 Hz
        {
            if (signal_psd[i] > CurMaxMag)
            {
                CurMaxMag = signal_psd[i];
                curPkFreqBin = i;
                //printf("New Peak Mag: %0.0f at Bin: %d\n", CurMaxMag, curPkFreqBin);
            }
        }
    }
}
/*Note: 'output' is loaded with Goertzel magnitudes taken @ 4ms intervals*/
void wiener_filter_process(float* input, float* output)
{
    //int N = FFT_SIZE;
    // Forward FFT
    //dsps_fft2r_fc32_aes3(input, FFT_SIZE); // Using optimized AES3 version works OK by itself; but Not when combined with nsnet2 noise reduction model
    dsps_fft2r_fc32_ansi(input, FFT_SIZE); // Using ANSI C version works best when combined with nsnet2 noise reduction model
    // Bit reverse
    dsps_bit_rev_fc32(input, FFT_SIZE);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(input, FFT_SIZE);
    // Compute magnitude and update PSDs
    float mag[FFT_SIZE / 2];
    float CurMaxMag = 00.0f;
    int curPkFreqBin = 0;
    for (int i = 0; i < FFT_SIZE / 2; i++)
    {
        float real = input[2 * i];
        float imag = input[2 * i + 1];
        mag[i] = sqrtf(real * real + imag * imag) / FFT_SIZE;
        if (i >= 28 && i <= 61) // only look/test for frequencies between 450 & 950 Hz
        {
            if (mag[i] > CurMaxMag)
            {
                CurMaxMag = mag[i];
                curPkFreqBin = i;
                // printf("New Peak Mag: %0.0f at Bin: %d\n", CurMaxMag, curPkFreqBin);
            }
        }

        if (i == 61)
        {
            if (CurMaxMag > 35)
            {
                MaxMag = CurMaxMag;
                PkFreqBin = curPkFreqBin;
                float avgBin = (float)PkFreqBin;
                int bincntr = 1;
                if (mag[curPkFreqBin - 1] > (0.7f * CurMaxMag))
                {
                    avgBin += (float)(curPkFreqBin - 1);
                    bincntr++;
                }
                if (mag[curPkFreqBin + 1] > (0.7f * CurMaxMag))
                {
                    avgBin += (float)(curPkFreqBin + 1);
                    bincntr++;
                }
                Bstfreq = (avgBin / (float)bincntr) * 15.625;
                // printf("New MaxMag: %0.0f at Bin: %d\n", MaxMag, PkFreqBin);
            }
        }
        if (ScopeTrigr)
        {
            if (ui_Chart1_series_1 != NULL && i < 100) // only plot first 100 bins
            {
                ui_Chart1_series_1->y_points[i] = (int32_t)(mag[i]);
            }
            if (i == 99)
            {
                SmplSetRdy = true;
                ScopeTrigr = false;
            }
        }
    }
    float S = mag[PkFreqBin];
    float Nf = (mag[PkFreqBin + 10] + mag[PkFreqBin - 10]) / 2; // avoid notch frequencies;
    float gain = 1.0f;
    if (S < Sqlcthresh || Nf > 17)
    { //didn't find an obvious signal bin based on last estimate
        int OldPkFreqBin = PkFreqBin;
        estimate_signal(mag, FFT_SIZE / 2); // or estimate_noise if in noise-only segment
        estimate_noise(mag, FFT_SIZE / 2);  // or use VAD to decide
        float S = signal_psd[PkFreqBin];
        float Nf = noise_psd[PkFreqBin];
        if (S < Sqlcthresh || Nf > 17){//still didn't find a good signal bin
            gain = 0.0f; // suppress very low signal bins
            PkFreqBin = OldPkFreqBin; //No useful signal found, retain old freq bin
        }
    }
    
    // Pseudo Inverse FFT
    AvgTone = ((Bstfreq) * 0.4f) + (AvgTone * 0.6f); //smooth tone freq
    float fr = 2 * M_PI * ((AvgTone) / SAMPLING_RATE);
    //int cntr = 0; //for debugging
    float curMag = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++)
    {
        /*Uncomment the following for debugging */
        if (i % 64 == 0)
        {
            cntr++;
            //Blue (gain), Red (output), Green (S), Orange (Nf)
            //printf("%0.0f %0.0f %0.0f %0.0f %d\n", 110 * gain, output[i], S, Nf, PkFreqBin);
            //printf("%d\n", PkFreqBin);
        }
        if(i2s_pdm_running) {
            curMag = 8*fabsf(output[i]);
        }
        else{
            curMag = fabsf(output[i]);
        }
        if(curMag > 2000.0f) curMag = 2000.0f;    
        if(gain == 0.0f || curMag<50 ) curMag = 0.0f;
        AvgMag = curMag * 0.1f + AvgMag * 0.9f; //smooth output
        // if(i % 32 == 0){
        //     printf("%0.0f\n", AvgMag);
        // }
        if(AvgMag < 5.0f)
        {ph = 0.0f;}
        output[i] = AvgMag * sin(ph); // compute tone data sample using geortzel magnitude FFT bin/freq with largest magnitude
        ph += fr;
        if (ph > 2 * M_PI)
        {
            ph -= 2 * M_PI;
        }
        if (ph < -2 * M_PI)
        {
            ph += 2 * M_PI;
        }
    }
}

// Utility: Low-pass filter
static float applyLowPassFilter(float sample)
{
    constexpr float RC = 1.0f / (CUTOFF_FREQ * 2 * 3.14159f);
    constexpr float dt = 1.0f / SAMPLING_RATE;
    const float alpha = dt / (RC + dt);

    float filtered_sample = alpha * sample + (1.0f - alpha) * prev_sample;
    prev_sample = filtered_sample;
    return filtered_sample;
}

int rptcntr = 0;
int sentcnt = 0;
//get current time in microseconds
uint64_t lstusecVal = esp_timer_get_time();
bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void) rhport;
  (void) n_bytes_copied;
  (void) itf;
  (void) ep_in;
  (void) cur_alt_setting;
  //passes every 1000us at 16Khz, 32 bytes
  //so only print every 4000*1000us = 4.0 seconds
  sentcnt += n_bytes_copied;
  uint64_t CurusecVal = esp_timer_get_time();
  uint16_t delta = (uint16_t)(CurusecVal - lstusecVal);
  // ESP_LOGI(TAG, "TX done %d, dt=%d", n_bytes_copied, delta);
  lstusecVal = CurusecVal;

  return true;
}


/**
 * @brief Task function to read ADC data, process it, and feed it to the sound processor, 
 * or directly to the USB audio buffer.
 *
 * This function initializes speech recognition models and AFE (Audio Front End) configuration,
 * sets up buffers, and enters a loop to continuously read ADC samples. Depending on the DSP_ON
 * and TWO_STAGE flags, it applies digital signal processing (DSP) such as IIR or FIR filtering
 * to the input data. The processed data is then either fed to the sound processor or stored in
 * a ring buffer for USB audio output. The function also manages synchronization using semaphores,
 * prints diagnostic information, and tracks sample counts for performance monitoring.
 *
 * @param param Pointer to task parameters (unused).
 *
 * @note This function is designed to run as a FreeRTOS task and should not be called directly.
 *       It is driven by ADC DMA interrupt, with a 16Khz sample rate, and runs once every 64ms
 *       It interacts with hardware ADC, sound processor, and USB audio subsystems.
 */
void Read_ADC(void *param)
{
    char TxtBuf[70];
    int shwcntr = 150;
    int64_t EvntStart = 0;

    int feed_size = 512;
    int16_t feed_buff[512];
    int oldptrval = 0;
    bool arm = true;
    int DACdataCnt = 0;
    uint8_t result[ADC_SAMPLE_CNT * SOC_ADC_DIGI_RESULT_BYTES] = {0};
    /*Wiener filter variables & constants*/
    esp_err_t ret;
    ret = dsps_fft2r_init_fc32(NULL, ADC_SAMPLE_CNT);
    if (ret != ESP_OK)
    {
        ESP_LOGE("FFT", "Failed to initialize FFT: %d", ret);
        return;
    }
    ESP_LOGI("FFT", "FFT initialized successfully");
    dsps_wind_hann_f32(wind, CONFIG_DSP_MAX_FFT_SIZE);
    /*Goertzel varialables (base on sample rate 16Khz & Tone Freq 750Hz)*/
    int N = 64;
    float TwoPi = 2.0f * 3.14159f;
    float k = ((N * Bstfreq) / SAMPLING_RATE);
    float Alpha = TwoPi * k / N;
    float Beta = TwoPi * k * (N - 1) / N;
    // Precompute network coefficients
    float Two_cos_Alpha = 2 * cos(Alpha);
    float a = cos(Beta);
    float b = -sin(Beta);
    float c = sin(Alpha) * sin(Beta) - cos(Alpha) * cos(Beta);
    float d = sin(TwoPi * k);

    while (true)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            /*At a sample rate of 16000Hz, This buffer will need to be read every 64ms or ~15Hz*/
            uint32_t ret_num;
            esp_err_t ret = adc_continuous_read(ADC_Hndl, result, ADC_SAMPLE_CNT * SOC_ADC_DIGI_RESULT_BYTES, &ret_num, 0);
            if (ret == ESP_OK)
            {
                if (ESP_SR)
                { // ESP Voice noise suppression AFE enabled
                    if(!afe_fetch_Task_run){
                        afe_fetch_Task_run = true;
                        ESP_LOGI(TAG, "Starting AFE fetch task");
                        ret = voiceNR.TaskStart((uint32_t)4096, (UBaseType_t)5);
                        if(ret != ESP_OK){
                            ESP_LOGE(TAG, "Failed to start AFE fetch task: %d", ret);
                        }
                    }

                    int FdBufPtr = 0;
                    DACdataCnt = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
                    for (int i = 0; i < DACdataCnt; i++)
                    {
                        auto *p0 = reinterpret_cast<adc_digi_output_data_t *>(&result[i * SOC_ADC_DIGI_RESULT_BYTES]);
                        short val16_t = static_cast<short>(p0->type2.data);
                        val16_t -= 1977;
                        
                        if (i2s_pdm_running)
                        {
                           val16_t *= 8; // doing this to get to a 'louder' signal for I2S PDM testing
                        }
                        else
                        {
                            val16_t *= 1; // simple gain adjustment
                        } // simple gain adjustment
                        feed_buff[FdBufPtr++] = val16_t << 4; // multiply by 16 to scale to full 16bit range
                        /*this happens twice per for loop*/
                        if (FdBufPtr == feed_size)
                        {
                            /*send this chunk to the sound processor*/
                            voiceNR.feed(feed_buff);
                            FdBufPtr = 0;
                        }
                    }
                }
                else
                {
                    if(afe_fetch_Task_run){
                        afe_fetch_Task_run = false;
                        // ESP_LOGI(TAG, "Stopping AFE fetch task");
                        // ret = voiceNR.TaskStop();
                        // if(ret != ESP_OK){
                        //     ESP_LOGE(TAG, "Failed to stop AFE fetch task: %d", ret);
                        // }
                    }
                    if (!DSP_ON)
                    {
                        /*Part of Wiener filter/NR process*/
                        /*Init Goertzel delay line contents Based on last found Bstfreq*/
                        float k = ((N * Bstfreq) / SAMPLING_RATE);
                        float Alpha = TwoPi * k / N;
                        float Beta = TwoPi * k * (N - 1) / N;
                        // Precompute network coefficients
                        float Two_cos_Alpha = 2 * cos(Alpha);
                        float a = cos(Beta);
                        float b = -sin(Beta);
                        float c = sin(Alpha) * sin(Beta) - cos(Alpha) * cos(Beta);
                        float d = sin(TwoPi * k);
                        float w1 = 0;
                        float w2 = 0;
                        int GoetzelCnt = 0;
                        int step = 0;
                        // int FdBufPtr = 0;
                        DACdataCnt = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
                        for (int i = 0; i < DACdataCnt; i++)
                        {
                            auto *p0 = reinterpret_cast<adc_digi_output_data_t *>(&result[i * SOC_ADC_DIGI_RESULT_BYTES]);
                            float sample = static_cast<float>(p0->type2.data);
                            sample -= 1977.0f;
                            /*Load input FFT array/dataset*/
                            fft_Cmplx[2 * i] = sample;
                            fft_Cmplx[2 * i + 1] = 0.0f;
                            // add sample to Goertzel calculation
                            float w0 = sample + Two_cos_Alpha * w1 - w2;
                            // Delay line data shifting
                            w2 = w1;
                            w1 = w0;
                            GoetzelCnt++;
                            if (GoetzelCnt == N) // process every 64 samples (4ms at 16Khz )
                            {
                                float real = w1 * a + w2 * c;
                                float imag = (w1 * b + w2 * d);
                                float magnitude = sqrtf(real * real + imag * imag) / 25.0f; // normalize
                                //printf("%0.0f\n", magnitude);
                                /*Reset Goertzel for next block*/
                                GoetzelCnt = 0;
                                w1 = w2 = 0.0f;
                                /*load next N (64) samples (or 4ms) with the current Goertzel magnitude */
                                for (int k = 0; k < N; k++)
                                {
                                    output_buffer[(step * N) + k] = magnitude; // archive Goertzel magnitude for later use in wiener output filter
                                }
                                step++;
                            }
                            // printf("%d; %f\n", i, fft_Cmplx[2*i]);
                        }
                        // This dataset is ready, Apply Wiener filter
                        wiener_filter_process(fft_Cmplx, output_buffer);
                        /*Ensure we're not going to overwrite what hasn't been sent (to USB) yet*/
                        // bool ovrlp = true;
                        // int RingBufPtrMax = RingBufPtr + (2* FFT_SIZE);
                        // while (ovrlp)
                        // {
                        //     int USBptrMax = AUC_cb_len + abPtr_USb;
                        //     if (((RingBufPtr > abPtr_USb) && (RingBufPtr < USBptrMax)) ||
                        //         ((RingBufPtrMax > abPtr_USb) && (RingBufPtrMax < USBptrMax)))
                        //     {
                        //         ovrlp = true;
                        //         vTaskDelay(pdMS_TO_TICKS(10));
                        //     }
                        //     else
                        //         ovrlp = false;
                        // }

                        for (int i = 0; i < FFT_SIZE; i++)
                        {
                            float Fval = output_buffer[i];
                            short wrdbuf = ((short)(Fval)) << 4;
                            audio_Rbuffer[RingBufPtr] = (uint8_t)(wrdbuf & 0xFF);
                            RingBufPtr++;
                            audio_Rbuffer[RingBufPtr] = (uint8_t)(wrdbuf >> 8);
                            RingBufPtr++;
                            if (RingBufPtr == AucRB_len)
                                RingBufPtr = 0;
                        }
                        arm = true;
                    }
                    else
                    {
                        if (arm)
                        {
                            arm = false;
                            abPtr_USb = 6144;
                            RingBufPtr = 0;
                        }
                        DACdataCnt = ret_num / SOC_ADC_DIGI_RESULT_BYTES;
                        short wrdbuf;
                        for (int i = 0; i < DACdataCnt; i++)
                        {
                            auto *p0 = reinterpret_cast<adc_digi_output_data_t *>(&result[i * SOC_ADC_DIGI_RESULT_BYTES]);
                            float Fval = static_cast<float>(p0->type2.data);
                            Fval -= 1977;
                            if (i2s_pdm_running)
                            {
                                Fval *= 5; // doing this to get to a 'louder' signal for I2S PDM testing
                            }
                            else
                            {
                                Fval *= 1; // simple gain adjustment
                            }
                            
                            if (TWO_STAGE)
                            {
                                float IIRBPOut = a0 * Fval + a1 * in_z1 + a2 * in_z2 - b1 * out_z1 - b2 * out_z2;
                                in_z2 = in_z1;
                                in_z1 = Fval;
                                out_z2 = out_z1;
                                out_z1 = IIRBPOut;
                                Fval = IIRBPOut;
                                /*stage 2*/
                                if (0) //(TWO_STAGE)
                                {
                                    float IIRBPOut = a0 * Fval + a1 * in_z3 + a2 * in_z4 - b1 * out_z3 - b2 * out_z4;
                                    in_z4 = in_z3;
                                    in_z3 = Fval;
                                    out_z4 = out_z3;
                                    out_z3 = IIRBPOut;
                                    Fval = IIRBPOut;
                                }
                            }
                            else
                            {
                                Fval = applyLowPassFilter(Fval);
                            }
                            /*End DSP code*/
                            wrdbuf = ((short)(Fval)) << 4;
                            audio_Rbuffer[RingBufPtr] = (uint8_t)(wrdbuf & 0xFF);
                            RingBufPtr++;
                            audio_Rbuffer[RingBufPtr] = (uint8_t)(wrdbuf >> 8);
                            RingBufPtr++;
                            if (RingBufPtr == AucRB_len)
                                RingBufPtr = 0;
                        }
                        dacSmplCnt += 2 * DACdataCnt;
                        NSEvntStart = pdTICKS_TO_MS(xTaskGetTickCount());
                        cntr = 0;
                    }
                }
                MicDatRdy = true;
            }
            shwcntr++;
            if (shwcntr == 200)
            {
                shwcntr = 0;
                if (!DSP_ON)
                {
                    RingBufPtr = (abPtr_USb >= 6144) ? abPtr_USb - 6144 : abPtr_USb + 6144;
                }
                int64_t now = esp_timer_get_time();
                int gap = static_cast<int>((now - EvntStart) / 1000);
                EvntStart = now;
                int PtrDelta = abPtr_USb - oldptrval;
                oldptrval = abPtr_USb;
                snprintf(TxtBuf, sizeof(TxtBuf), "dacSmplCnt %d; AUCSmplCnt %d; afeSmplCnt %d; afe_intrvl %d\n",
                         static_cast<int>(dacSmplCnt), static_cast<int>(AUCSmplCnt), static_cast<int>(afeSmplCnt), afe_intrvl);
                dacSmplCnt = 0;
                AUCSmplCnt = 0;
                afeSmplCnt = 0;
                afe_intrvl = 0;
            }
        }
    }
}

// ADC conversion done callback
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(Read_ADC_TaskHndl, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

// usb_auc functions
/*This callback is here to support 'incoming' audio stream; i.e., for a speaker/headphones
But, for this project, is currently NOT supported*/
static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    size_t bytes_written = 0;
    char TxtBuf[50];
    sprintf(TxtBuf, "uac_device_output_cb    Fired\n");
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
    {
        Print2Dsply(TxtBuf);
        xSemaphoreGive(Txt2Dsply_mutx);
    }
    /*For this project, this feature is NOT supported*/
    // bsp_extra_i2s_write(buf, len, &bytes_written, 0);
    return ESP_OK;
}

/*this is the 'call back' used to pass audio. from the the ESP32's ADC,
 to the host computer(i.e., the 'microphone' input).
 In its current form, it generates a triangle (linear ramp from maxima to maxima),
 with a period of 4.096 seconds*/
// unsigned long LastStart = 0;
// float oldRadnVal = 0;
// uint16_t interval = 0;
// //float smplPrd = 1000/16000;
// bool rampup = true;
// int cyclcnt = 0;
// static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
// {
//     char TxtBuf[50];// here just for debugging
//     esp_err_t ret = ESP_OK;
//     int byteCntr = 0;

//     if (pdTRUE == xSemaphoreTake(AuBufr_semaphore, 400 / portTICK_PERIOD_MS))
//     {
//         int i = 0;
//         int ptr = 0;
//         //float PiFrctn = 0;
//         float radin = oldRadnVal;
//         unsigned long EvntStart = pdTICKS_TO_MS(xTaskGetTickCount());
//         interval = (uint16_t)(EvntStart - LastStart);
//         while (interval < 44) // 40->3.855; 44->4.092; 45->4.15; 47->4.295 ;49->4.313 ;50 -> 4.325 (4.096 Ideal)
//         {
//             vTaskDelay(pdMS_TO_TICKS(1));
//             unsigned long EvntStart = pdTICKS_TO_MS(xTaskGetTickCount());
//             interval = (uint16_t)(EvntStart - LastStart);
//         }
//         LastStart = EvntStart;
//         while (i < len / 2) // with a sample rate of 16000, len = 1600 i.e. 1/10 of the sample rate with a 50ms refresh interval
//         {
//             if (rampup)
//                 radin++; //= (float)i; //((float)i) / len;
//             else
//                 radin--; // -= (float)i;
//             if (radin == 32767)
//             {
//                 cyclcnt++;
//                 /* sprintf(TxtBuf, "%d cycles\n", cyclcnt); //(int)len)
//                 if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//                 {
//                     Print2Dsply(TxtBuf);
//                     xSemaphoreGive(Txt2Dsply_mutx);
//                 } */
//                 rampup = false;
//             }
//             if (radin == -32767)
//                 rampup = true;
//             float Fval = radin;
//             int intval = (int)Fval;
//             uint16_t wrdbuf = (uint16_t)(intval & 0xffff);
//             buf[(i * 2) + 0] = (uint8_t)(wrdbuf & 0xFF);        // Lo byte
//             buf[(i * 2) + 1] = (uint8_t)((wrdbuf >> 8) & 0xFF); // Hi Byte
//             i++;
//             byteCntr += 2;
//         }
//         oldRadnVal = radin;
//         abPtr_USb += ptr;
//         xSemaphoreGive(AuBufr_semaphore);
//         MicDatRdy = true;
//     }
//     else
//     {
//         ret = ESP_ERR_INVALID_STATE;
//     }
//     *bytes_read = (size_t)(byteCntr);
//     return ret;
// }

/////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Callback function for USB Audio Class (UAC) device input.
 *
 * This function is called to provide audio data from the ADC buffer to the USB audio buffer.
 * It waits until the microphone data is ready, then transfers data from the ADC ring buffer to the USB buffer.
 * The function also logs timing and buffer information for debugging and ensures enough time has passed
 * since the last data pull to maintain a usable ADC data stream.
 *
 * @param buf Pointer to the buffer where audio data should be copied.
 * @param len Length of the buffer in bytes.
 * @param bytes_read Pointer to a variable to store the number of bytes actually read.
 * @param arg User-defined argument (unused).
 * @return esp_err_t ESP_OK on success, or ESP_ERR_INVALID_STATE if microphone data is not ready.
 */
static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    char TxtBuf[70];
    esp_err_t ret = ESP_OK;
    uint32_t interval = 0;
    size_t byteCntr = 0;
    if (!MicDatRdy)
        return ESP_ERR_INVALID_STATE;
    AUC_cb_len = len;
    wait4RB = true;
    int Bfptr = 0;
    int offset = 0;
    while (Bfptr < static_cast<int>(len))
    {
        byteCntr++;
        buf[Bfptr] = audio_Rbuffer[Bfptr + abPtr_USb - offset];
        Bfptr++;
        if (Bfptr + abPtr_USb == AucRB_len)
        {
            abPtr_USb = 0;
            offset = Bfptr;
        }
    }
    AUCSmplCnt += byteCntr;
    abPtr_USb += (Bfptr - offset);
    *bytes_read = byteCntr;
    UACcb_Start = esp_timer_get_time();
    interval = static_cast<uint32_t>(UACcb_Start - LastStart);
    if (interval >= 50000)
    {
        snprintf(TxtBuf, sizeof(TxtBuf), "int %d; USB %d, ADC %d\n", static_cast<int>(interval), abPtr_USb, RingBufPtr);
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
    }
    constexpr int32_t wait_cnt = 49950; // 49900;
    while ((interval < wait_cnt))       //  && (sentcnt < 1600) ensure enough time has passed since last pull to maintain a usable ADC data stream
    {
        UACcb_Start = esp_timer_get_time();
        interval = static_cast<uint32_t>(UACcb_Start - LastStart);
        if (interval < 44000)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            interval += 5000;
        }
        else if (interval > 44000 && interval < 49000)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            interval += 1000;
        }
    }
    sentcnt = 0;
    LastStart = UACcb_Start;
    return ret;
}
///////////////////////////////////////////////////////////////////////////

// USB mute callback
static void uac_device_set_mute_cb(uint32_t mute, void *arg)
{
    char TxtBuf[50];
    ESP_LOGI(TAG, "uac_device_set_mute_cb: %" PRIu32 "", mute);
    snprintf(TxtBuf, sizeof(TxtBuf), "uac_device_set_mute_cb: %" PRIu32 "\n", mute);
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
    {
        Print2Dsply(TxtBuf);
        xSemaphoreGive(Txt2Dsply_mutx);
    }
}

// USB volume callback
static void uac_device_set_volume_cb(uint32_t volume, void *arg)
{
    // Not supported for this project
    return;
}

/**
 * @brief Main application entry point for ESP32S3 ADC microphone input and USB audio output.
 *
 * This function initializes the GUI, configures ADC channels, sets up synchronization primitives,
 * and starts the ADC reading task. It also configures the bandpass filter coefficients and sets up
 * the ADC using a custom initialization method. Event callbacks for ADC conversion completion are
 * registered and the ADC is started in continuous mode.
 *
 * The function then configures the USB Audio Class (UAC) device, providing callbacks for audio input,
 * mute, and volume control. It initializes the UAC device and displays the result of the initialization.
 *
 * Key steps:
 * - Initialize GUI and synchronization primitives.
 * - Configure ADC channels and bandpass filter.
 * - Start ADC reading task on a specific core.
 * - Register ADC event callbacks and start ADC.
 * - Set up USB Audio Class device with appropriate callbacks.
 * - Display status messages for initialization steps.
 *
 * @note Some sections of code are commented out for reference or testing purposes.
 */

void app_main()
{
    //vTaskDelay(pdMS_TO_TICKS(4000));
    printf("app_main: running\n");
    char TxtBuf[60];

    /* test/check NVS to see if user setting/param have been stored */
    nvs_result_t Rstat = nvs_suprt.nvs_init();
    if (Rstat != NVS_OK)
    {
        /*No settings found; load factory settings*/
        printf("\nNVS FAILED TO INITIALIZE\n");
       
    }
    else
    {
        printf("\nNVS initialized OK\n");
        //vTaskDelay(pdMS_TO_TICKS(1250));
        Rstat = nvs_suprt.get_AudioOutMode(AudioOutMode);//nvs_suprt.nvs_read_val("AudioOutMode", AudioOutMode);
        if (Rstat != NVS_OK)
        {
            /*No settings found; load factory settings*/
            printf("\nNo stored USER params Found\nUsing default PDM output\n");
            nvs_suprt.SaveUsrVals(); // save default to NVS
        }
        else
        {
            /*found 'AudioOutMode' stored setting, go get the other user settings */
            printf("\nFound 'AudioOutMode' stored setting: %d\n", AudioOutMode);
            //int strdAT;
            //nvs_suprt.get_StdAFCfg(strdAT);//nvs_suprt.nvs_read_val("StdAFCfg", strdAT);
            }
    }
    Txt_GUI_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    // Initialize ESP Voice Noise Reduction AFE (nsnet2 noise suppression model)
    voiceNR.esp_sr_init();

    static adc_channel_t channel[1] = {ADC_CHANNEL_5};
    uint8_t channel_num = sizeof(channel) / sizeof(adc_channel_t);

    if (AuBufr_semaphore == nullptr)
    {
        AuBufr_semaphore = xSemaphoreCreateMutex();
    }
    esp_log_level_set(TAG, ESP_LOG_INFO);
    if (Txt2Dsply_mutx == nullptr)
    {
        printf("Txt2Dsply_mutx == NULL\n");
        Txt2Dsply_mutx = xSemaphoreCreateMutex();
    }

    Calc_IIR_BPFltrCoef(750.0f, SAMPLING_RATE, 8.0f);

    xTaskCreatePinnedToCore(Read_ADC, "Read_ADC Task", 11264, nullptr, 10, &Read_ADC_TaskHndl, 1);

    Mic_ADC_init(channel, channel_num, &ADC_Hndl);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(ADC_Hndl, &cbs, nullptr));
    ESP_ERROR_CHECK(adc_continuous_start(ADC_Hndl));

    /*Uncomment the following 'while' loop to see afe settings via USB serial monitor*/
    // while (1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(5000));
    // }
    vTaskDelay(pdMS_TO_TICKS(1000)); // this here to allow time for the I2C gt911 touch screen to report before starting USB

    if (AudioOutMode == 0) // set to 0 to enable I2S PDM TX mode; set to 1 to use USB UAC mode
    {
        /* I2S to PDM TX channel setup
         * Configures and starts the I2S PDM TX channel:
         * - Builds an i2s_chan_config_t and creates a channel with i2s_new_channel.
         * - Prepares PDM TX clock and slot configuration (uses DAC default clock
         *   config macro and 16-bit mono PCM slot).
         * - Copies clock/slot defaults into the tx_cfg and initializes PDM TX mode.
         */

        i2s_chan_config_t chan_cfg = {
            .id = I2S_PDM_TX_CHANNEL,
            .role = I2S_ROLE_MASTER,
            .dma_desc_num = 8,
            .dma_frame_num = 256,
            .auto_clear = true,
        };

        esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
        if (ret != ESP_OK)
        {
            ESP_LOGE("PDM_TX", "Failed to create I2S PDM TX channel: %s", esp_err_to_name(ret));
            return;
        }
        // I2S PDM TX configuration
        //i2s_pdm_tx_clk_config_t clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(SAMPLING_RATE);
        i2s_pdm_tx_clk_config_t clk_cfg = I2S_PDM_TX_CLK_DAC_DEFAULT_CONFIG(SAMPLING_RATE);
        clk_cfg.up_sample_fs = 480; //SAMPLING_RATE / 100;
        //i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_RAW_FMT_DAC_DEFAULT_CONFIG((i2s_data_bit_width_t)16, I2S_SLOT_MODE_MONO);
        i2s_pdm_tx_slot_config_t slot_cfg = I2S_PDM_TX_SLOT_PCM_FMT_DEFAULT_CONFIG((i2s_data_bit_width_t)16, I2S_SLOT_MODE_MONO);
        i2s_pdm_tx_config_t tx_cfg = {
            .gpio_cfg = {
                .clk = I2S_PDM_TX_PIN_CLK,
                .dout = I2S_PDM_TX_PIN_DATA,
                .invert_flags = {
                    .clk_inv = false,
                    //.dout_inv = false,
                },
            },
        };
        // copy/add the default DAC/PDM based configurations to tx_cfg
        memcpy(&tx_cfg.clk_cfg, &clk_cfg, sizeof(i2s_pdm_tx_clk_config_t));
        memcpy(&tx_cfg.slot_cfg, &slot_cfg, sizeof(i2s_pdm_tx_slot_config_t));
        ret = i2s_channel_init_pdm_tx_mode(tx_handle, &tx_cfg);
        if (ret != ESP_OK)
        {
            ESP_LOGE("PDM_TX", "Failed to initialize I2S PDM TX mode: %s", esp_err_to_name(ret));
            return;
        }

        /*
         *    - Enables the I2S channel and marks i2s_pdm_running = true.
         *    - Creates a pinned write task to stream PDM data to the I2S peripheral:
         *      - Task function: i2s_write_task
         *      - Stack size: 2048 bytes
         *      - Priority: 5
         *      - Pinned to core 0
         *   - If not using I2S PDM TX, sets up USB UAC device with appropriate callbacks.
         */
        ret = i2s_channel_enable(tx_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE("PDM_TX", "Failed to enable I2S PDM TX channel: %s", esp_err_to_name(ret));
            return;
        }
        i2s_pdm_running = true;
        // Create I2S write task
        xTaskCreatePinnedToCore(i2s_write_task, "i2s_write_task", 2048, NULL, 5, &i2s_task_handle, 0);
    }
    else
    {
        i2s_pdm_running = false;
        /*Setup/configure USB for UAC operation */
        uac_device_config_t config = {
            .skip_tinyusb_init = false,
            .output_cb = nullptr,
            .input_cb = uac_device_input_cb,
            .set_mute_cb = uac_device_set_mute_cb,
            .set_volume_cb = uac_device_set_volume_cb,
            .cb_ctx = nullptr,
#if CONFIG_USB_DEVICE_UAC_AS_PART
            .spk_itf_num = nullptr,
            .mic_itf_num = nullptr,
#endif
        };
        esp_err_t er = uac_device_init(&config);
        char TxtBuf[70];
        snprintf(TxtBuf, sizeof(TxtBuf), "uac_device_init(&config)      %s\n", (er == ESP_OK) ? "DONE" : "FAILED!!");
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
        {
            Print2Dsply(TxtBuf);
            xSemaphoreGive(Txt2Dsply_mutx);
        }
    }
}
// end app_main

// ADC initialization
static void Mic_ADC_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = nullptr;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = (SOC_ADC_DIGI_RESULT_BYTES * ADC_SAMPLE_CNT) * 8,
        .conv_frame_size = SOC_ADC_DIGI_RESULT_BYTES * ADC_SAMPLE_CNT,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    uint32_t freq = static_cast<uint32_t>(SAMPLING_RATE);

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = freq,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        uint8_t unit = ADC_UNIT_1;
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_12;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;
}
// end ADC init

// Bandpass filter coefficient calculation
static void Calc_IIR_BPFltrCoef(float Fc, float Fs, float Q)
{
    float K = tanf(M_PI * (Fc / Fs));
    float norm = 1.0f / (1.0f + K / Q + K * K);

    a0 = (K / Q) * norm;
    a1 = 0.0f;
    a2 = -a0;
    b1 = 2.0f * (K * K - 1.0f) * norm;
    b2 = (1.0f - K / Q + K * K) * norm;
}
///////////////////////////////////////////
// uint8_t Write_NVS_Val(const char *key, int value)
// /* write numeric data to NVS; If the data is "stored",return with an exit status of "1" */
// {
//   uint8_t stat = 0;
//   char TxtBuf[50];
//   // const uint16_t* p = (const uint16_t*)(const void*)&value;
//   //  Handle will automatically close when going out of scope or when it's reset.
//   std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
//   if (ret != ESP_OK)
//   {
//     sprintf(TxtBuf, "Error (%s) opening WRITE NVS value handle for: %s!\n", esp_err_to_name(ret), key);
//     if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//   }
//   else
//   {
//     ret = handle->set_item(key, value);
//     switch (ret)
//     {
//     case ESP_OK:
//       ret = handle->commit();
//       if (ret != ESP_OK)
//       {
//         sprintf(TxtBuf, "Commit Failed (%s) on %s!\n", esp_err_to_name(ret), key);
//         if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       }
//       else
//         /* exit point when everything works as it should */
//         stat = 1;
//       break;
//     default:
//       sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
//       if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       //delay(3000);
//       vTaskDelay(pdMS_TO_TICKS(3000));
//     }
//   }
//   return stat;
// }
// ////////////////////////////////////////////
// /*Save data to NVS memory routines*/

// uint8_t Write_NVS_Str(const char *key, char *value)
// /* write string data to NVS */
// {
//   char TxtBuf[50];  
//   uint8_t stat = 0;
//   //  Handle will automatically close when going out of scope or when it's reset.
//   std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
//   if (ret != ESP_OK)
//   {
//     // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(ret));
//     sprintf(TxtBuf, "Error (%s) opening WRITE NVS string handle for: %s!\n", esp_err_to_name(ret), key);
//     if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//   }
//   else
//   {

//     ret = handle->set_string(key, value);
//     switch (ret)
//     {
//     case ESP_OK:
//       /* write operation worked; go ahead and /lock the data in */
//       ret = handle->commit();
//       if (ret != ESP_OK)
//       {
//         sprintf(TxtBuf, "Commit Failed (%s) on %s!\n", esp_err_to_name(ret), key);
//         if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       }
//       else
//       {
//         /* exit point when everything works as it should */
//         stat = 1;
//         break;
//       }
//     default:
//       sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
//       if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       //delay(3000);
//       vTaskDelay(pdMS_TO_TICKS(3000));
//     }
//   }
//   return stat;
// }
// /////////////////////////////////////////////////////////////
// uint8_t Read_NVS_Val(const char *key, int &value)
// /* read numeric data from NVS; If the data is "found",return with an exit status of "1" */
// {
//   uint8_t stat = 0;
//   char TxtBuf[50];
//   //  Handle will automatically close when going out of scope or when it's reset.
//   std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READONLY, &ret);
//   if (ret != ESP_OK)
//   {
//     sprintf(TxtBuf, "Error (%s) opening READ NVS value handle for: %s!\n", esp_err_to_name(ret), key);
//     if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//   }
//   else
//   {
//     ret = handle->get_item(key, value);
//     switch (ret)
//     {
//     case ESP_OK:
//       /* exit point when everything works as it should */
//       stat = 1;
//       break;
//     case ESP_ERR_NVS_NOT_FOUND:
//       sprintf(TxtBuf, "The value for %s is not initialized yet!\n", key);
//       if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       break;
//     default:
//       sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
//       if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       //delay(3000);
//       vTaskDelay(pdMS_TO_TICKS(3000));
//     }
//   }
//   return stat;
// }
// /////////////////////////////////////////////////////
// /* the following code handles the read from & write to NVS processes needed to handle the User CW prefences/settings  */

// uint8_t Read_NVS_Str(const char *key, char *value)
// /* read string data from NVS*/
// {
//   uint8_t stat = 0;
//   char TxtBuf[50];
//   // const uint16_t* p = (const uint16_t*)(const void*)&value;
//   //  Handle will automatically close when going out of scope or when it's reset.
//   std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &ret);
//   if (ret != ESP_OK)
//   {
//     sprintf(TxtBuf, "Error (%s) opening READ NVS string handle for: %s!\n", esp_err_to_name(ret), key);
//     if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//   }
//   else
//   {
//     size_t required_size;
//     ret = handle->get_item_size(nvs::ItemType::SZ, key, required_size);
//     switch (ret)
//     {
//     case ESP_OK:

//       if (required_size > 0 && required_size < 100)
//       {
//           char temp[100];
//           ret = handle->get_string(key, temp, required_size);
//           int i;
//           for (i = 0; i < 100; i++)
//           {
//               value[i] = temp[i];
//               if (temp[i] == 0)
//                   break;
//           }

//           sprintf(TxtBuf, "%d characters copied to %s\n", i, key);
//           if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//           {
//               Print2Dsply(TxtBuf);
//               xSemaphoreGive(Txt2Dsply_mutx);
//           }

//           stat = 1;
//       }
//       break;
//     case ESP_ERR_NVS_NOT_FOUND:
//       /*TODO could need support code here*/
//       break;
//     default:
//       sprintf(TxtBuf, "Error (%s) reading %s!\n", esp_err_to_name(ret), key);
//       if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(TxtBuf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
//       //delay(3000);
//       vTaskDelay(pdMS_TO_TICKS(3000));
//     }
//   }
//   return stat;
// }
// /////////////////////////////////////////////////////////////
// void SaveUsrVals(void)
// {
// 	/* Unlock the Flash Program Erase controller */
// 	char buf[30];
// 	bool GudFlg = true;
// 	char call[10];
// 	char mem[80];
// 	int i;
// 	uint8_t Rstat;
// 	bool zeroFlg = false;
// 	/*Save Debug Setting*/
// 	Rstat = Write_NVS_Val("AudioOutMode", AudioOutMode);
// 	if (Rstat != 1)
// 		GudFlg = false;
// 	/* Save current Decoder NiteMode value */
	
// 	if (GudFlg)
// 	{
// 		sprintf(buf, "User Params SAVED");
// 		if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(buf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
// 	}
// 	else
// 	{
// 		sprintf(buf, "SAVE FAILED");
// 		if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
//         {
//             Print2Dsply(buf);
//             xSemaphoreGive(Txt2Dsply_mutx);
//         }
// 	}
// }
// ////////////////////////////////////////////////////////////