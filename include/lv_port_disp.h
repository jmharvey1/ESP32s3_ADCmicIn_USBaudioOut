/**
 * @file lv_port_disp_templ.h
 *
 */
/*20250131 JMH(KW4KD)*/

#ifndef LV_PORT_DISP_TEMPL_H
#define LV_PORT_DISP_TEMPL_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
#include "stdint.h" // need this to use char type
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
/*Added the following for Waveshare & lvgl support*/
#include "esp_timer.h"
#include <lv_conf.h>
#include "lvgl.h"

//extern SemaphoreHandle_t lvgl_mux;
static SemaphoreHandle_t lvgl_semaphore = NULL;
extern TaskHandle_t lvgl_task_handle;
extern SemaphoreHandle_t lvgl_mux;
extern lv_display_t * disp;
static esp_timer_handle_t lvgl_tick_timer;
extern bool SmplSetRdy;
extern bool ScopeActive;
extern bool ScopeTrigr;
extern float Bstfreq;
extern float MaxMag;
extern lv_chart_series_t * ui_Chart1_series_1;
/*********************
 *      DEFINES
 *********************/
/*Waveshare 800x480 display & touch specific parameters*/
#define LV_TICK_PERIOD_MS (20) //JMH changed this from 100 to 2, v9.2 Waveshare display demos & examples;

#define CONFIG_MSGBX_AVOID_TEAR_EFFECT_WITH_SEM 1

#define I2C_MASTER_SCL_IO 9 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8 /*!< GPIO number used for I2C master data  */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// the following configuration for Waveshare's ESP32s3 7 inch Touch Display //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MSGBX_LCD_PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define MSGBX_LCD_BK_LIGHT_ON_LEVEL 1
#define MSGBX_LCD_BK_LIGHT_OFF_LEVEL !MSGBX_LCD_BK_LIGHT_ON_LEVEL
#define MSGBX_PIN_NUM_BK_LIGHT -1
#define MSGBX_PIN_NUM_HSYNC 46
#define MSGBX_PIN_NUM_VSYNC 3
#define MSGBX_PIN_NUM_DE 5
#define MSGBX_PIN_NUM_PCLK 7
#define MSGBX_PIN_NUM_DATA0 14  // B3
#define MSGBX_PIN_NUM_DATA1 38  // B4
#define MSGBX_PIN_NUM_DATA2 18  // B5
#define MSGBX_PIN_NUM_DATA3 17  // B6
#define MSGBX_PIN_NUM_DATA4 10  // B7
#define MSGBX_PIN_NUM_DATA5 39  // G2
#define MSGBX_PIN_NUM_DATA6 0	  // G3
#define MSGBX_PIN_NUM_DATA7 45  // G4
#define MSGBX_PIN_NUM_DATA8 48  // G5
#define MSGBX_PIN_NUM_DATA9 47  // G6
#define MSGBX_PIN_NUM_DATA10 21 // G7
#define MSGBX_PIN_NUM_DATA11 1  // R3
#define MSGBX_PIN_NUM_DATA12 2  // R4
#define MSGBX_PIN_NUM_DATA13 42 // R5
#define MSGBX_PIN_NUM_DATA14 41 // R6
#define MSGBX_PIN_NUM_DATA15 40 // R7
#define MSGBX_PIN_NUM_DISP_EN -1

// The pixel number in horizontal and vertical
#define MSGBX_LCD_H_RES 800
#define MSGBX_LCD_V_RES 480

#if CONFIG_MSGBX_DOUBLE_FB
#define MSGBX_LCD_NUM_FB 2
#else
#define MSGBX_LCD_NUM_FB 1
#endif // CONFIG_MSGBX_DOUBLE_FB

// #define MSGBX_LVGL_TICK_PERIOD_MS    2
#define MSGBX_LVGL_TASK_MAX_DELAY_MS 500
#define MSGBX_LVGL_TASK_MIN_DELAY_MS 1 // JMH changed to 4 for My_Txt_test.c; default setting was 1
#define MSGBX_LVGL_TASK_STACK_SIZE (8 * 1024)
#define MSGBX_LVGL_TASK_PRIORITY 3 // JMH changed from 2
#define MSGBX_LVGL_TASK_CORE                     (-1)        // The core of the LVGL timer task, `-1` means the don't specify the core
                                                            // This can be set to `1` only if the SoCs support dual-core,
                                                            // otherwise it should be set to `-1` or `0`
/*end Waveshare Params*/
/*taken from lvgl_port_v8*/
/**
 * LVGL timer handle task related parameters, can be adjusted by users
 *
 */
#define LVGL_PORT_TASK_MAX_DELAY_MS             (500)       // The maximum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_MIN_DELAY_MS             (2)         // The minimum delay of the LVGL timer task, in milliseconds
#define LVGL_PORT_TASK_STACK_SIZE               (6 * 1024)  // The stack size of the LVGL timer task, in bytes
#define LVGL_PORT_TASK_PRIORITY                 (2)         // The priority of the LVGL timer task
#define LVGL_PORT_TASK_CORE                     (-1)        // The core of the LVGL timer task, `-1` means the don't specify the core
                                                            // This can be set to `1` only if the SoCs support dual-core,
                                                            // otherwise it should be set to `-1` or `0`

/**
 * Avoid tering related configurations, can be adjusted by users.
 *
 *  (Currently, This function only supports RGB LCD)
 *
 */
/**
 * Set the avoid tearing mode:
 *      - 0: Disable avoid tearing function
 *      - 1: LCD double-buffer & LVGL full-refresh
 *      - 2: LCD triple-buffer & LVGL full-refresh
 *      - 3: LCD double-buffer & LVGL direct-mode (recommended)
 *
 */
#define LVGL_PORT_AVOID_TEARING_MODE            (0)

#if LVGL_PORT_AVOID_TEARING_MODE != 0
/**
 * As the anti-tearing feature typically consumes more PSRAM bandwidth, for the ESP32-S3, we need to utilize the Bounce
 * buffer functionality to enhance the RGB data bandwidth.
 *
 * This feature will occupy `LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE * 2 * bytes_per_pixel` of SRAM memory.
 *
 */
#define LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE        (LVGL_PORT_DISP_WIDTH * 10)
/**
 * When avoid tearing is enabled, the LVGL software rotation `lv_disp_set_rotation()` is not supported.
 * But users can set the rotation degree(0/90/180/270) here, but this funciton will extremely reduce FPS.
 * So it is recommended to be used when using a low resolution display.
 *
 * Set the rotation degree:
 *      - 0: 0 degree
 *      - 90: 90 degree
 *      - 180: 180 degree
 *      - 270: 270 degree
 *
 */
#endif
#define LVGL_PORT_ROTATION_DEGREE               (0)

/*Added this to aviod having to reference LVGL_port_v8.h */
#define LVGL_PORT_DISP_WIDTH                    (ESP_PANEL_LCD_WIDTH)   // The width of the display
#define LVGL_PORT_DISP_HEIGHT                   (ESP_PANEL_LCD_HEIGHT)  // The height of the display

#define HiRes // uncomment for 480x340 screens
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
void lv_port_disp_init(void);
/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void);

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void);

/**
 * @brief Lock the LVGL mutex. This function should be called before calling any LVGL APIs when not in LVGL task,
 *        and the `lvgl_port_unlock_WShr()` function should be called later.
 *
 * @param timeout_ms The timeout of the mutex lock, in milliseconds. If the timeout is set to `-1`, it will wait indefinitely.
 *
 * @return ture if success, otherwise false
 */
bool lvgl_port_lock(int timeout_ms);

/**
 * @brief Unlock the LVGL mutex. This function should be called after using LVGL APIs when not in LVGL task, and the
 *        `lvgl_port_lock()` function should be called before.
 *
 * @return ture if success, otherwise false
 */
bool lvgl_port_unlock_WShr(void);
/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_PORT_DISP_H*/

