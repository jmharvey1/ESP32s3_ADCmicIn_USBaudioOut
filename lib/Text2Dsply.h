/**
 * @file Text2Dsply.h
 *
 */
/*20250704 JMH(KW4KD)*/

#ifndef TEXT_2_DSPLY_H
#define TEXT_2_DSPLY_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/
// #include "stdint.h" // need this to use char type
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
// /*Added the following for Waveshare & lvgl support*/
// #include <lv_conf.h>
// #include "lvgl.h"

/**********************
 * GLOBAL PROTOTYPES
 **********************/
extern SemaphoreHandle_t Txt2Dsply_mutx;
extern bool DSP_ON;
extern bool TWO_STAGE;
extern bool ESP_SR;
/* Create Text Area GUI
princapaly to show debug text inplace of passing them to a remote serial terminal via USB */
void Txt_GUI_init(void);
/* 
Display debug text on contianed in 'NuTxt' on WaveShare Display using LVGL graphics library
 */
void Print2Dsply(char NuTxt[]);


#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*TEXT_2_DSPLY_H*/