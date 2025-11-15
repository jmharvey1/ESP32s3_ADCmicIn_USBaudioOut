/*20251115 Changed Button1 event detectioin to 'LV_EVENT_SHORT_CLICKED' to reduce false detections*/
#include "Text2Dsply.h"
#include "stdint.h" // need this to use char type
#include "esp_log.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <memory> //needed for call/ref to std::string
#include <string.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
/*Added the following for Waveshare & lvgl support*/
#include "lvgl.h"
#include "lv_conf.h"
#include "misc/lv_event_private.h"
#include "lv_port_disp.h"
#include "lvgl/src/lv_api_map_v9_1.h"
#include "lvgl/src/core/lv_refr.h"
/*Added to support updating saved NVS user settings */
#include "NVS_Suprt.h"

#include <stdio.h>

TaskHandle_t DisplayTaskHandle;
SemaphoreHandle_t Txt2Dsply_mutx = NULL;

// i2c_master_transmit_multi_buffer_info_t dummyI2CBufr;

lv_obj_t *DBtextArea;
lv_obj_t *win1;
lv_obj_t *win2;
lv_obj_t *scr_1;
lv_obj_t * Btn1_label;
lv_obj_t * Btn2_label;
/*Scope/chart variables*/
static lv_obj_t *ui_Scope;
static lv_obj_t *ui_Chart1;
lv_chart_series_t *ui_Chart1_series_1 = NULL;
static lv_obj_t *ui_Label1;
static lv_obj_t *ui_Label2;
bool SmplSetRdy = false;
bool ScopeTrigr = true;
bool ScopeActive = false;


static lv_obj_t *cont1;
static lv_obj_t *bar1;
static lv_style_t style_win1;
static lv_style_t style_bar;
static lv_style_t TAstyle;
static lv_style_t Cursorstyle;
static lv_style_t style_label;
//SemaphoreHandle_t lvgl_semaphore = NULL;
int ta_charCnt = 0;
int CurKyBrdCharCnt = 0;
static QueueHandle_t Nu_Txt_que;
bool QuequeFulFlg = false;
bool NuLineFlg = false;
bool DSP_ON = false;
bool TWO_STAGE = false;
bool ESP_SR = false;
unsigned long LastStart1 = 0;
int NoiseSupprLvl = 0;

NVS_Suprt nvs_suprt2; // Create an instance of the NVS_Suprt class

static void textarea_event_handler(lv_event_t *e);
void Update_textarea(lv_obj_t *TxtArea, char bufChar);
void DisplayTask(void *param);


static const char *TAG = "Text2Dsply";
/*
This just creates a new task.
The task itself actually starts LVGL &
creates the Text Area Display GUI.
This approach was used to avoid a 
'Stack overfow', in the 'main' task, crash.
*/
void Txt_GUI_init(void)
{
    Nu_Txt_que = xQueueCreate(20 * 50, sizeof(char));
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 8192, NULL, 4, &DisplayTaskHandle, 0); // priority used to be 10
    if (DisplayTaskHandle == NULL)
    {
        ESP_LOGI(TAG, "Display Task Task handle FAILED");
    }
    //vTaskDelay(pdMS_TO_TICKS(500));
    return;
}
/**
 * @brief external method/function to pass a null-terminated character array to a queue 
 * for display, handling queue full conditions.
 *
 * This function iterates through the input character array `NuTxt`, sending each character to the queue `Nu_Txt_que`.
 * If the queue is full and a character cannot be sent, it sets the `QuequeFulFlg` flag and displays an error message
 * using `Update_textarea`. The function waits until the queue is no longer full before continuing.
 * After all characters are sent, it waits until the queue depth drops below a specified threshold.
 * So a typical call, to this function, looks like this:
 *   char TxtBuf[50];
 *   sprintf(TxtBuf, "uac_device_set_volume_cb: %" PRIu32 "\n", volume);
 *   if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
 *   {
 *       Print2Dsply(TxtBuf);
 *       xSemaphoreGive(Txt2Dsply_mutx);
 *   }
 *
 * @param NuTxt Null-terminated character array to be sent to the display queue.
 */
void Print2Dsply(char NuTxt[])
{
    int i = 0;
    while (NuTxt[i] != 0)
    {
        /*Push NuTxt values on the que */
        if (xQueueSend(Nu_Txt_que, &NuTxt[i], pdMS_TO_TICKS(100)) == pdFALSE)
        {
            QuequeFulFlg = true;
            char ermsg[30];
            sprintf(ermsg, "xQueueSend FAILED\n");
            int i = 0;
            while (ermsg[i] != 0)
            {
                Update_textarea(DBtextArea, ermsg[i]);
                
            }
            //lv_display_refr_timer(NULL);
        }
        while (QuequeFulFlg)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        i++;
    }
    bool chkagn = true;
    while (chkagn)
    {
        int uxQueueDepth = (int)uxQueueMessagesWaiting(Nu_Txt_que);
        if (uxQueueDepth < 15*50)
        {
            chkagn = false;
        }
    }
    //lv_textarea_set_cursor_pos(DBtextArea, LV_TEXTAREA_CURSOR_LAST);
    //lv_display_refr_timer(NULL);
}

static void textarea_event_handler(lv_event_t *e)
{
    lv_obj_t *DBtextArea = (lv_obj_t *)lv_event_get_target(e);
    lv_textarea_set_cursor_pos(DBtextArea, LV_TEXTAREA_CURSOR_LAST);
    char TxtBuf[50];
    sprintf(TxtBuf, "YOU TOUCHED ME!!\n");
    if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
    {  
        Print2Dsply(TxtBuf); 
        xSemaphoreGive(Txt2Dsply_mutx);
    }
}
static void btn1_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_SHORT_CLICKED) { //LV_EVENT_CLICKED) {
        /*Following 4 lines act as a button 'de-bounce' circuit*/
        unsigned long EvntStart = pdTICKS_TO_MS(xTaskGetTickCount());
        uint16_t interval = (uint16_t)(EvntStart - LastStart1);
        // if(interval < 500) return;
        LastStart1 = EvntStart;
        if(interval< 1000) return;
        NoiseSupprLvl++;
        if(NoiseSupprLvl > 3) NoiseSupprLvl = 0;
        switch (NoiseSupprLvl){
            case 0:
                DSP_ON = false;
                TWO_STAGE = false;
                ESP_SR = false;
                lv_label_set_text(Btn1_label, "WIENER");
                break;
            case 1:
                DSP_ON = true;
                lv_label_set_text(Btn1_label, "NR OFF");
                break;
            case 2:
                //DSP_ON = false;
                TWO_STAGE = true;
                lv_label_set_text(Btn1_label, "IIR");
                break;
            case 3:
                DSP_ON = false;
                TWO_STAGE = false;
                ESP_SR = true;
                lv_label_set_text(Btn1_label, "NSNET2");
                break;  
        }
        //
        //DSP_ON = !DSP_ON;
        //if(DSP_ON) lv_label_set_text(Btn1_label, "NR OFF");
        //else lv_label_set_text(Btn1_label, "NR ON");
    }
    else if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
    }
}
static void btn2_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        /*Following 4 lines act as a button 'de-bounce' circuit*/
        unsigned long EvntStart = pdTICKS_TO_MS(xTaskGetTickCount());
        uint16_t interval = (uint16_t)(EvntStart - LastStart1);
        LastStart1 = EvntStart;
        if(interval< 1000) return;
        AudioOutMode++;
        if(AudioOutMode > 1) AudioOutMode = 0;
        nvs_suprt2.nvs_write_val("AudioOutMode", AudioOutMode);
        if(AudioOutMode == 0){ //AudioOutMode; //0=I2S PDM, 1=USB Audio

            lv_label_set_text(Btn2_label, "I2S/PDM");
        }
        else{
            lv_label_set_text(Btn2_label, "USB/UAC");
        }
    }
    else if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
    }
}
/*internal method to pass individal text characters to 'LVGL's Text Area' Buffer */
void Update_textarea(lv_obj_t *TxtArea, char bufChar)
{
    //char buf2[25];
    //bool tryagn = true;
    //bool updateCharCnt = false;
    // if (DBtextArea == TxtArea)
    // 	updateCharCnt = true;

    //	else
    //	{
    //int trycnt = 0;
    //while (tryagn)
    //{
        //printf("xSemaphoreTake(lvgl_semaphore, 100\n");
        if(lvgl_semaphore == NULL){
            printf("lvgl_semaphore == NULL\n");
            lvgl_semaphore = xSemaphoreCreateMutex();
        }
        //if (pdTRUE == xSemaphoreTake(lvgl_semaphore, 100 / portTICK_PERIOD_MS))
        if(lvgl_port_lock(-1))
        {
            // MutexLckId = 1;
            // if (TxtArea == DBtextArea)
            // {
            //     lv_textarea_set_cursor_pos(TxtArea, LV_TEXTAREA_CURSOR_LAST);
            //     int CurPos = 0;
            //     if (TxtArea == DBtextArea)
            //     {
            //         lv_textarea_set_cursor_pos(TxtArea, LV_TEXTAREA_CURSOR_LAST);
                    
            //     }
            // }
            // if (bufChar == 0x08)
            // {
            // 	lv_textarea_del_char(TxtArea);
            // }
            // else if (bufChar == 0xFF)
            // {
            // 	lv_textarea_set_text(TxtArea, "");
            // 	if (updateCharCnt)
            // 		CurKyBrdCharCnt = 0;
            // 	// printf("CLEAR TEXT AREA 1\n");
            // }
            // else
            // {
            lv_textarea_set_cursor_pos(TxtArea, LV_TEXTAREA_CURSOR_LAST);
            if(bufChar == '\t'){
                for(int i =0; i < 5; i++){lv_textarea_add_char(TxtArea, ' ');}
            } 
            else lv_textarea_add_char(TxtArea, bufChar);
            //ta_charCnt++;
            // if (updateCharCnt && (TxtArea == DBtextArea))
            // {
            //     const char *p = lv_textarea_get_text(TxtArea);
            //     CurKyBrdCharCnt = strlen(p);
            // }
            // lv_textarea_set_cursor_pos(TxtArea, LV_TEXTAREA_CURSOR_LAST);
            // if (TxtArea == DecdTxtArea)
            // {
            // 	sprintf(buf2, "CharCnt: %d", ta_charCnt);
            // 	lv_label_set_text(label2, buf2);
            // }
            //xSemaphoreGive(lvgl_semaphore);
            lvgl_port_unlock_WShr();
            
        }
        // else
        // {
        //     trycnt++;
        //     if (trycnt > 5)
        //     {
        //         trycnt = 5;
        //         tryagn = false;
        //         // printf("LVGLMsgBox::update_text2 timed out; MutexLckId = %d\n", MutexLckId);
        //     }
        //     //vTaskDelay(pdMS_TO_TICKS(20));
        // }
    //}
    return;
}

static void Screen_event_handler(lv_event_t *e)
{
	const char *TAG1 = "Screen_event_handler";
	lv_event_code_t code = lv_event_get_code(e);
	switch (code)
	{
	case LV_EVENT_CLICKED:
	{
		// if (pdTRUE == xSemaphoreTake(lvgl_semaphore, portMAX_DELAY))
        // {
            lv_textarea_set_cursor_pos(DBtextArea, LV_TEXTAREA_CURSOR_LAST);
        //    xSemaphoreGive(lvgl_semaphore);
        // }    
        char TxtBuf[50];
        sprintf(TxtBuf, "YOU TOUCHED ME!!\n");
        if (pdTRUE == xSemaphoreTake(Txt2Dsply_mutx, pdMS_TO_TICKS(500)))
    {  
        Print2Dsply(TxtBuf); 
        xSemaphoreGive(Txt2Dsply_mutx);
    }
		
	}
	break;

	default:
		break;
	}
}
////////////////////////////////////////////////////////////////////////////////

void Bld_Scope_scrn(void)
{
	if (ui_Scope == NULL)
    {
        ui_Scope = lv_obj_create(NULL);
        win2 = lv_win_create(ui_Scope);
        lv_obj_add_style(win2, &style_win1, 0);
        //lv_obj_set_style_bg_color(win2, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
        //lv_obj_set_style_bg_opa(win2, 100, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_clear_flag(ui_Scope, LV_OBJ_FLAG_SCROLLABLE); /// Flags

        ui_Chart1 = lv_chart_create(ui_Scope);
        lv_obj_set_width(ui_Chart1, 712);
        lv_obj_set_height(ui_Chart1, 370);
        lv_obj_set_x(ui_Chart1, 0);
        lv_obj_set_y(ui_Chart1, 17);
        lv_obj_set_align(ui_Chart1, LV_ALIGN_CENTER);
        lv_chart_set_type(ui_Chart1, LV_CHART_TYPE_LINE);
        lv_chart_set_point_count(ui_Chart1, 100);
        lv_chart_set_range(ui_Chart1, LV_CHART_AXIS_PRIMARY_Y, -10, 200);
        lv_chart_set_range(ui_Chart1, LV_CHART_AXIS_PRIMARY_X, 0, 100);
        lv_chart_set_div_line_count(ui_Chart1, 22, 33);
        /*Create a Horizontal ('x') scale with 100% width*/
        lv_obj_t *scale_bottom = lv_scale_create(ui_Chart1);
        lv_scale_set_mode(scale_bottom, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
        // Set the numeric range for the scale, for example from 0 to 100
        lv_scale_set_range(scale_bottom, 0, 1562);
        // Align the scale to the bottom of the chart widget
        lv_obj_align_to(scale_bottom, ui_Chart1, LV_ALIGN_OUT_BOTTOM_LEFT, 2, -30);
        lv_obj_set_size(scale_bottom, lv_pct(104), 25);
        // Set the number of major ticks and how frequently they appear
        lv_scale_set_total_tick_count(scale_bottom, 17); // 17 ticks in total
        lv_scale_set_major_tick_every(scale_bottom, 1);  // Every tick is major
        lv_obj_set_style_pad_hor(scale_bottom, lv_chart_get_first_point_center_offset(ui_Chart1), 0);

        /*Create a Vertical ('Y') scale with 100% width*/
        lv_obj_t *scale_Y = lv_scale_create(ui_Chart1);
        lv_scale_set_mode(scale_Y, LV_SCALE_MODE_VERTICAL_LEFT);
        // Set the numeric range for the scale, for example from 0 to 100
        lv_scale_set_range(scale_Y, 0, 200);
        // Align the scale to the bottom of the chart widget
        lv_obj_align_to(scale_Y, ui_Chart1, LV_ALIGN_OUT_TOP_LEFT, 2, 145);
        lv_obj_set_size(scale_Y, 25, lv_pct(95));
        // Set the number of major ticks and how frequently they appear
        lv_scale_set_total_tick_count(scale_Y, 21); // 15 ticks in total
        lv_scale_set_major_tick_every(scale_Y, 2);  // Every 2nd tick is major
        // lv_obj_set_style_pad_ver(scale_Y, lv_chart_get_first_point_center_offset(ui_Chart1), -185);

        ui_Chart1_series_1 = lv_chart_add_series(ui_Chart1, lv_color_hex(0xFF0000),
                                                 LV_CHART_AXIS_PRIMARY_Y);
        static lv_coord_t ui_Chart1_series_1_array[100] = {0};
        lv_chart_set_ext_y_array(ui_Chart1, ui_Chart1_series_1, ui_Chart1_series_1_array);

        ui_Label1 = lv_label_create(ui_Scope);
        lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);  /// 1
        lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT); /// 1
        lv_obj_set_x(ui_Label1, -100);
        lv_obj_set_y(ui_Label1, -202);
        lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
        lv_label_set_text(ui_Label1, "Peak Mag:");
        lv_obj_set_style_text_color(ui_Label1, lv_color_hex(0xFD0A0A), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_opa(ui_Label1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_Label1, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

        ui_Label2 = lv_label_create(ui_Scope);
        lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);  /// 1
        lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT); /// 1
        lv_obj_set_x(ui_Label2, 195);
        lv_obj_set_y(ui_Label2, -203);
        lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
        lv_label_set_text(ui_Label2, "Frequency:");
        lv_obj_set_style_text_color(ui_Label2, lv_color_hex(0x2669F1), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_opa(ui_Label2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_24, LV_PART_MAIN | LV_STATE_DEFAULT);

        lv_obj_t *btn1 = lv_win_add_button(win2, LV_SYMBOL_DUMMY, 100);
        lv_obj_add_event_cb(btn1, btn1_event_handler, LV_EVENT_ALL, NULL);
        lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
        lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

        Btn1_label = lv_label_create(btn1);
        lv_label_set_text(Btn1_label, "WIENER");
        lv_obj_center(Btn1_label);

        lv_obj_t *btn2 = lv_win_add_button(win2, LV_SYMBOL_DUMMY, 100);
        lv_obj_add_event_cb(btn2, btn2_event_handler, LV_EVENT_ALL, NULL);
        lv_obj_align(btn2, LV_ALIGN_CENTER, 150, -40);
        lv_obj_remove_flag(btn2, LV_OBJ_FLAG_PRESS_LOCK);

        Btn2_label = lv_label_create(btn2);
        if(AudioOutMode == 0){ //AudioOutMode; //0=I2S PDM, 1=USB Audio

            lv_label_set_text(Btn2_label, "I2S/PDM");
        }
        else{
            lv_label_set_text(Btn2_label, "USB/UAC");
        }
        //lv_label_set_text(Btn2_label, "IIR OFF");
        lv_obj_center(Btn2_label);
    }

    lv_scr_load(ui_Scope);
}

/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Task function to initialize and manage the LVGL-based display.
 *
 * This function sets up the LVGL display, styles, and UI elements such as windows,
 * text areas, and buttons. It manages display refreshing explicitly and handles
 * incoming text messages from a queue, updating the display accordingly.
 * Mutex locking is used to ensure thread safety when interacting with LVGL APIs.
 * The task continuously waits for new text messages, processes them, and updates
 * the display area. It also manages button event callbacks and display refresh cycles.
 *
 * @param param Pointer to task parameters (unused).
 *
 * @note
 * - LVGL v9.2 is initialized and configured.
 * - RGB bus configuration is performed if tearing avoidance is enabled.
 * - Styles and fonts are set for various UI elements.
 * - Text messages are received from 'Nu_Txt_que' and displayed in the text area.
 * - Mutex locking/unlocking is used for LVGL API calls.
 * - Display refresh timer is managed explicitly.
 * - Button event handlers are registered for UI interaction.
 */
void DisplayTask(void *param)
{
    printf("DisplayTask started\n");
#if LVGL_PORT_AVOID_TEAR
    // When avoid tearing function is enabled, configure the RGB bus according to the LVGL configuration
    ESP_PanelBus_RGB *rgb_bus = static_cast<ESP_PanelBus_RGB *>(panel->getLcd()->getBus());
    rgb_bus->configRgbFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    rgb_bus->configRgbBounceBufferSize(LVGL_PORT_RGB_BOUNCE_BUFFER_SIZE);
#endif
    /*Start LVGL v9.2 */
    printf("Start/Initialize LVGL v9.2\n");
    lv_port_disp_init();
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    if (lvgl_port_lock(-1))
        printf("Text2Dsply: TLVGL TASK LOCKED\n");
    else
        printf("Text2Dsply: TLVGL TASK LOCK FAILED\n");
    // apply lvlg setup code here
    printf("Text2Dsply: TEXT Screen SetUp START\n");
    lv_style_init(&style_win1);
    lv_style_init(&style_bar);
    lv_style_init(&TAstyle);
    lv_style_init(&Cursorstyle);
    lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, &lv_font_montserrat_16); // Set desired font
    lv_style_set_bg_color(&style_win1, lv_palette_main(LV_PALETTE_DEEP_ORANGE));
    lv_style_set_text_font(&TAstyle, &lv_font_montserrat_16);
    //int title_height = 20;
    char Title[100];
    char RevDate[12] = "08/30/25";
    sprintf(Title, "DAC to UAC DSP test bed - %s", RevDate);
   // printf("Text2Dsply: TEXT Screen STEP 1 COMPLETE\n");

    scr_1 = lv_obj_create(NULL);
    win1 = lv_win_create(scr_1);
    lv_obj_add_style(win1, &title_style, LV_PART_ANY);
    //printf("Text2Dsply: TEXT Screen STEP 2 win1 COMPLETE\n");

    lv_win_add_title(win1, Title);
    lv_obj_set_size(win1, 800, 480);
    lv_obj_add_style(win1, &style_win1, 0);
    cont1 = lv_win_get_content(win1); /*Content can be added here*/
    lv_obj_clear_flag(cont1, LV_OBJ_FLAG_SCROLLABLE); /// Flags
    //printf("Text2Dsply: TEXT Screen STEP 2 COMPLETE\n");
    DBtextArea = lv_textarea_create(cont1);
    lv_obj_set_size(DBtextArea, 760, 360); // width & Height
    lv_obj_set_pos(DBtextArea, 0, 0);
    //lv_textarea_set_max_length(DBtextArea, 804); // 1000-(2*98) = 804
    lv_obj_add_style(DBtextArea, &TAstyle, 0);   // sets the fonte used
    //printf("Text2Dsply: TEXT Screen STEP 3 COMPLETE\n");
    // lv_textarea_set_one_line(DBtextArea, false);// causes problems
    lv_obj_align(DBtextArea, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_add_state(DBtextArea, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/
    lv_obj_set_scroll_dir(DBtextArea, LV_DIR_VER);/*Supposed to help with scrolling */
    lv_obj_clear_flag(DBtextArea, LV_OBJ_FLAG_SCROLL_ELASTIC); /*Supposed to help with scrolling */
    

    //lv_obj_t * btn1 = lv_button_create(cont1);
    lv_obj_t * btn1 = lv_win_add_button(win1, LV_SYMBOL_DUMMY, 100);
    lv_obj_add_event_cb(btn1, btn1_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(btn1, 430 , LV_ALIGN_BOTTOM_MID);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    Btn1_label = lv_label_create(btn1);
    lv_label_set_text(Btn1_label, "NR ON");
    lv_obj_center(Btn1_label);

    // lv_obj_t * btn2 = lv_button_create(cont1);
    lv_obj_t * btn2 = lv_win_add_button(win1, LV_SYMBOL_DUMMY, 100);
    lv_obj_add_event_cb(btn2, btn2_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_set_pos(btn2, 630 , LV_ALIGN_BOTTOM_MID);
    lv_obj_remove_flag(btn2, LV_OBJ_FLAG_PRESS_LOCK);

    Btn2_label = lv_label_create(btn2);
    lv_label_set_text(Btn2_label, "IIR OFF");
    lv_obj_center(Btn2_label);
    /* the following are possible options but currently arent needed */
    //lv_obj_add_event_cb(DBtextArea, Screen_event_handler, LV_EVENT_CLICKED, NULL);
    //lv_obj_add_event_cb(DBtextArea, textarea_event_handler, LV_EVENT_CLICKED, DBtextArea);
    
    //lv_screen_load_anim(scr_1, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
    //lv_scr_load(scr_1);
    Bld_Scope_scrn();
    ScopeActive = true;
    printf("Text2Dsply: TEXT Screen STEP 4 COMPLETE\n");
   
    /*now because we are going to manage screen refreshing explicitly within this project, delete lvgl's internal refresh process*/
    lv_display_delete_refr_timer(disp);
    Txt2Dsply_mutx = xSemaphoreCreateMutex();
    vTaskDelay(pdMS_TO_TICKS(20));
    bool Unlock = false;
     /* Release the mutex */
    while (!Unlock)
    {
        printf("Text2Dsply: START TLVGL TASK LOCK RELEASE\n");
        Unlock = lvgl_port_unlock_WShr();
        if (Unlock)
            printf("Text2Dsply: COMPLETED TLVGL TASK LOCK RELEASE\n");
    }
    //lv_display_refr_timer(NULL);
    printf("Text2Dsply: TEXT Screen SetUp Complete\n");

 /*Now sit & wait for new text messages, to show up, in the 'Nu-Txt_que'*/
 /*When they do, post them, to the WaveShare Display*/
    while (1)
    {
        bool lpagn = true;
        //bool UpDtScrn = false;
        while (lpagn)
        {
            char NuQchr;
            int i = 0;
            int rfrshcnt = 60;
            char NxtTxtSet[61];
            if (xQueueReceive(Nu_Txt_que, (void *)&NuQchr, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                //Update_textarea(DBtextArea, NuQchr);
                lvgl_port_lock(-1);
                lv_textarea_set_cursor_pos(DBtextArea, LV_TEXTAREA_CURSOR_LAST);
                //lv_textarea_add_char(DBtextArea, NuQchr);
                lvgl_port_unlock_WShr();
                if (NuQchr == '\t')
                {
                    for (int i = 0; i < 5; i++)
                    {
                        // lv_textarea_add_char(TxtArea, ' ');
                        NxtTxtSet[60 - rfrshcnt] = ' ';
                        rfrshcnt--;
                        NxtTxtSet[60 - rfrshcnt] = 0;
                    }
                }
                else
                {
                    NxtTxtSet[60 - rfrshcnt] = NuQchr;
                    rfrshcnt--;
                    NxtTxtSet[60 - rfrshcnt] = 0;
                }
                while (xQueueReceive(Nu_Txt_que, (void *)&NuQchr, pdMS_TO_TICKS(10)) == pdTRUE)
                {

                    QuequeFulFlg = false;
                    // Update_textarea(DBtextArea, NuQchr);
                    if (NuQchr == '\t')
                    {
                        for (int i = 0; i < 5; i++)
                        {
                            // lv_textarea_add_char(TxtArea, ' ');
                            NxtTxtSet[60 - rfrshcnt] = ' ';
                            rfrshcnt--;
                            NxtTxtSet[60 - rfrshcnt] = 0;
                            if(rfrshcnt == 0) break;
                        }
                    }
                    else
                    {
                        NxtTxtSet[60 - rfrshcnt] = NuQchr;
                        rfrshcnt--;
                        NxtTxtSet[60 - rfrshcnt] = 0;
                    }
                    if (rfrshcnt == 0)
                    {
                        lvgl_port_lock(-1);
                        lv_textarea_add_text(DBtextArea, NxtTxtSet);
                        // lv_display_refr_timer(NULL);
                        lvgl_port_unlock_WShr();
                        rfrshcnt = 60;
                    }
                    // i++;
                }
                if (rfrshcnt != 0)
                {
                    lvgl_port_lock(-1);
                    lv_textarea_add_text(DBtextArea, NxtTxtSet);
                    // lv_display_refr_timer(NULL);
                    lvgl_port_unlock_WShr();
                    rfrshcnt = 60;
                    NuLineFlg = true;
                }
            }
            else
                lpagn = false;
        }
        lvgl_port_lock(-1);
        if(NuLineFlg){
            NuLineFlg = false;
            //lv_textarea_set_cursor_pos(DBtextArea, LV_TEXTAREA_CURSOR_LAST);
            char NxtTxtSet[17];
            sprintf(NxtTxtSet, "#*#*#*#*#*#*#*#\n");
            lv_textarea_add_text(DBtextArea, NxtTxtSet);
        }
        if (ScopeActive)
        { // ready to refresh "scope" view
            if (SmplSetRdy)
            {
                char bias[30];
                char freq[30];
                sprintf(bias, "Peak Mag: %0.0f", MaxMag);
                lv_label_set_text(ui_Label1, bias);
                sprintf(freq, "Frequency: %0.0f Hz", Bstfreq);
                lv_label_set_text(ui_Label2, freq);
                lv_chart_refresh(ui_Chart1);
               //printf("DisplayTask: Scope Refresh Done\n");
               SmplSetRdy = false;
               ScopeTrigr = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(25));
        lv_display_refr_timer(NULL);
        lvgl_port_unlock_WShr();
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
