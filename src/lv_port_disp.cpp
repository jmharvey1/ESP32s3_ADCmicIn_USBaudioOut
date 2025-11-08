/**
 * @file lv_port_disp.c
 *
 */

/*20250131 JMH(KW4KD)*/
/* 
*            !!! important note !!!
* To get this to work with the latest ESPIDF, I had to modify 
* esp_lcd_panel_io_i2c_v2.c 'static esp_err_t panel_io_i2c_rx_buffer(esp_lcd_panel_io_t *io, int lcd_cmd, void *buffer, size_t buffer_size)'
* by adding a 2ms pause, at the point shown here:
*   ESP_GOTO_ON_ERROR(i2c_master_transmit_receive(i2c_panel_io->i2c_handle, write_buffer, write_size, buffer, buffer_size, -1), err, TAG, "i2c transaction failed");
*   vTaskDelay(pdMS_TO_TICKS(2));//JMH ADD 
*
*/

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdbool.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "touch/base/esp_lcd_touch_gt911.h" //added for waveshare touch support
#include "touch/base/esp_lcd_touch.h"
//20250711 added the following to support latest I2C setup strategy
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
/*Added the following for (input device) touch support*/
#include "driver/i2c_master.h" //added for waveshare touch support
/*********************
 *      DEFINES
 *********************/

#define BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565)) /*will be 2 for RGB565 */
///////////////////////////////////////////////////////////////////////////////////////////////////////
/*Added the following for (input device) touch support*/
// I2C Configuration
#define I2C_MASTER_SCL_IO    GPIO_NUM_9  //GPIO_NUM_15  // Replace with your SCL pin
#define I2C_MASTER_SDA_IO    GPIO_NUM_8   //GPIO_NUM_16  // Replace with your SDA pin
#define I2C_MASTER_NUM       I2C_NUM_0    // I2C port 0
#define I2C_MASTER_FREQ_HZ   100000       // 400kHz
#define GT911_I2C_ADDR       0x5D         // Default I2C address, might be 0x14
esp_lcd_touch_handle_t out_touch = NULL;

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);
static void lvgl_tick(void *arg);
static bool lvgl_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data);
static void disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);
static void touchpad_read_cb(lv_indev_t * indev, lv_indev_data_t * data);//lvgl9
//static bool touchpad_is_pressed(void);
//static void touchpad_get_xy(int32_t * x, int32_t * y);
static void lvgl_port_task(void *arg);

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "lvgl_port_disp";
lv_indev_t * indev_touchpad;
#if CONFIG_MSGBX_AVOID_TEAR_EFFECT_WITH_SEM
static SemaphoreHandle_t sem_vsync_end = NULL;
static SemaphoreHandle_t sem_gui_ready = NULL;
#endif
//static SemaphoreHandle_t lvgl_semaphore = NULL;
SemaphoreHandle_t lvgl_mux = NULL;
SemaphoreHandle_t RingBuf_semaphore = NULL;
TaskHandle_t lvgl_task_handle = NULL;
i2c_master_bus_handle_t i2c_master_bus_handle = NULL; // added here for waveshare touch support
i2c_master_dev_handle_t Touch_dev_handle = NULL;
esp_lcd_panel_io_handle_t tp_io_handle = NULL;
static esp_lcd_touch_handle_t tp = NULL;
lv_display_t * disp;
static esp_lcd_panel_handle_t display_handle = NULL;
//static esp_timer_handle_t lvgl_tick_timer;
esp_lcd_touch_config_t lcd_touch_config = {
	.x_max = MSGBX_LCD_H_RES,
	.y_max = MSGBX_LCD_V_RES,
	.rst_gpio_num = (gpio_num_t)-1,
	.int_gpio_num = (gpio_num_t)-1,
	.levels = {
		.reset = 0,		// 0: low level, 1: high level
		.interrupt = 0, // 0: low level, 1: high level
	},
	.flags = {
		.swap_xy = false,
		.mirror_x = false,
		.mirror_y = false,
	},
	.process_coordinates = NULL,
	.interrupt_callback = NULL,
	.user_data = NULL,
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
/** Main entry point to this code**/
void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize hardware side of
     * the display & touch screen
     * -----------------------*/
    disp_init();

    /*------------------------------------
     * Create a display and set a flush_cb
     * -----------------------------------*/
    disp = lv_display_create(MSGBX_LCD_H_RES, MSGBX_LCD_V_RES);
    assert(disp); // Ensure the display initialization was successful
    lv_display_set_flush_cb(disp, disp_flush);
   
    int buffer_size = (MSGBX_LCD_H_RES) * (MSGBX_LCD_V_RES);
    void *buf1 = NULL;                                                            // will be used by lvgl as a staging area (memory space) to create display images/maps before posting to display ST7262
    void *buf2 = NULL;                                                            // same as above, is the 2nd of two buffer spaces
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM); // MALLOC_CAP_DMA
    assert(buf1);
    buf2 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM); // MALLOC_CAP_DMA
    assert(buf2);
    lv_display_set_buffers(disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL); // render_mode – LV_DISPLAY_RENDER_MODE_PARTIAL/DIRECT/FULL
   
    /*Register a touchpad input device LVGL9*/
    indev_touchpad = lv_indev_create();
    lv_indev_set_type(indev_touchpad, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchpad, touchpad_read_cb);
    assert(indev_touchpad); // Ensure the input device initialization was successful
   
    ESP_LOGD(TAG, "Create mutex for LVGL");
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    // ESP_PANEL_CHECK_NULL_RET(lvgl_mux, false, "Create LVGL mutex failed");

    ESP_LOGD(TAG, "Create LVGL task");
    BaseType_t core_id = (MSGBX_LVGL_TASK_CORE < 0) ? tskNO_AFFINITY : MSGBX_LVGL_TASK_CORE;
    BaseType_t ret = xTaskCreatePinnedToCore(lvgl_port_task, "lvgl", MSGBX_LVGL_TASK_STACK_SIZE, NULL,
                                             MSGBX_LVGL_TASK_PRIORITY, &lvgl_task_handle, core_id);
    if (ret != pdPASS)
        printf("Create LVGL task failed\n");
    
#if LVGL_PORT_AVOID_TEAR
    lcd->attachRefreshFinishCallback(onRgbVsyncCallback, (void *)lvgl_task_handle);
#endif
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick Interface for LVGL using esp_timer to generate 2ms periodic event
    const esp_timer_create_args_t lvgl_tick_timer_args =
            {
                    .callback = &lvgl_tick,
                    .name = "lvgl_tick",
                    .skip_unhandled_events = true};
    
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000));
}

/**********************
 *   STATIC FUNCTIONS
 **********************/


/*Initialize display (hardware) including i2c 'touch'.*/
static void disp_init(void)
{
    /*JMH code*/
    static const char *TAG = "LVPortDisp::Init";
         
#if CONFIG_MSGBX_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if MSGBX_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << MSGBX_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Install RGB LCD Display driver");
    //esp_lcd_panel_handle_t display_handle = NULL;

    esp_lcd_rgb_panel_config_t display_config = {
            .clk_src = LCD_CLK_SRC_DEFAULT,
            .timings = {
                    .pclk_hz = (14 * 1000 * 1000),
                    .h_res = MSGBX_LCD_H_RES,
                    .v_res = MSGBX_LCD_V_RES,
                    .hsync_pulse_width = 4,
                    .hsync_back_porch = 8,
                    .hsync_front_porch = 8,
                    .vsync_pulse_width = 4,
                    .vsync_back_porch = 8,
                    .vsync_front_porch = 8,
                    .flags = {
                            .hsync_idle_low = 0,
                            .vsync_idle_low = 0,
                            .de_idle_high = 0,
                            .pclk_active_neg = 0,
                            .pclk_idle_high = 0,
                    },
            },
            .data_width = 16,								 // 8 | 16
            .bits_per_pixel = 16,							 // 24 | 16
            .num_fbs = 1,									 // 1/2/3
            .bounce_buffer_size_px = (MSGBX_LCD_H_RES * 10), // Bounce buffer size in bytes. This function is used to avoid screen drift.
            .sram_trans_align = 4,
            .psram_trans_align = 64,
            .hsync_gpio_num = MSGBX_PIN_NUM_HSYNC,
            .vsync_gpio_num = MSGBX_PIN_NUM_VSYNC,
            .de_gpio_num = MSGBX_PIN_NUM_DE,
            .pclk_gpio_num = MSGBX_PIN_NUM_PCLK,
            .disp_gpio_num = -1, // not used
            .data_gpio_nums = {
                    MSGBX_PIN_NUM_DATA0,
                    MSGBX_PIN_NUM_DATA1,
                    MSGBX_PIN_NUM_DATA2,
                    MSGBX_PIN_NUM_DATA3,
                    MSGBX_PIN_NUM_DATA4,
                    MSGBX_PIN_NUM_DATA5,
                    MSGBX_PIN_NUM_DATA6,
                    MSGBX_PIN_NUM_DATA7,
                    MSGBX_PIN_NUM_DATA8,
                    MSGBX_PIN_NUM_DATA9,
                    MSGBX_PIN_NUM_DATA10,
                    MSGBX_PIN_NUM_DATA11,
                    MSGBX_PIN_NUM_DATA12,
                    MSGBX_PIN_NUM_DATA13,
                    MSGBX_PIN_NUM_DATA14,
                    MSGBX_PIN_NUM_DATA15,
            },
            .flags = {
                    .disp_active_low = 0,
                    .refresh_on_demand = 0,
                    .fb_in_psram = 1,
                    .double_fb = 0,
                    .no_fb = 0,
                    .bb_invalidate_cache = 0,
            },
    };
    /*Register display & get a pointer/handle to it*/
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&display_config, &display_handle));
    ESP_LOGI(TAG, "Register Display event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
            .on_vsync = lvgl_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(display_handle, &cbs, disp));

    ESP_LOGI(TAG, "Initialize RGB LCD Display");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(display_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(display_handle));

    
    /* start gt911 / i2c config setup */
    /* setup I2C link to GT911 touch sensor */

    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,  // I2C clock source
        .glitch_ignore_cnt = 7,  // Glitch ignore count
        .intr_priority = NULL,
        .trans_queue_depth = 64,
        .flags{
            .enable_internal_pullup = true,  // Enable internal pull-ups
            //.allow_pd = false, //power down in light sleep
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_master_bus_handle));
    ESP_LOGI(TAG, "Create I2C Master BUS - DONE");

    /* 'i2c_master_bus_add_device' not needed here. Its done via 'esp_lcd_new_panel_io_i2c_v2()' */
    // i2c_device_config_t lcd_touch_panel_io_config = {
	// 	.dev_addr_length = I2C_ADDR_BIT_LEN_7,
	// 	.device_address = (0x5D), // GT911 chip address
	// 	.scl_speed_hz = 100000,
	// };
	// ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus_handle, &lcd_touch_panel_io_config, &Touch_dev_handle));
    // ESP_LOGI(TAG, "GT911 Device Added");


    /* instantuate touch object */
    ESP_LOGI(TAG, "Initialize GT911 I2C touch sensor");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG(); // defined in esp_lcd_touch_gt911.h
    /*note the following, actually calls 'esp_lcd_new_panel_io_i2c_v2'.
     As part of this call,'i2c_master_bus_add_device(tool_bus_handle, &i2c_dev_conf, &dev_handle)' gets done.
     see esp_lcd_panel_io_i2c_v2.c for more detail*/
    if (esp_lcd_new_panel_io_i2c_v2(i2c_master_bus_handle, &tp_io_config, &tp_io_handle) != ESP_OK)
    {
            //ESP_LOGE(TAG, "create GT911 touch interface FAILED");
            while(1)
            {
                printf("touch controller GT911 setup FAILED\n");
                vTaskDelay(pdMS_TO_TICKS(100));
            }
    }
    else
    {
            ESP_LOGI(TAG, "touch interface handle %p", tp_io_handle);
    }
    ESP_LOGI(TAG, "Initialize/configure GT911 touch controller");
    /*if the following works it will return/report 'TouchPad_ID' & 'TouchPad_Config_Version'*/
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &lcd_touch_config, &tp));//this is now handled in 'gt911_init()'
    lvgl_semaphore = xSemaphoreCreateMutex();
    RingBuf_semaphore = xSemaphoreCreateMutex();// JMH ADD 20250717
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    /*Moved 20250717 */
    // ESP_LOGI(TAG, "Install LVGL tick timer");
    // // Tick Interface for LVGL using esp_timer to generate 2ms periodic event
    // const esp_timer_create_args_t lvgl_tick_timer_args =
    //         {
    //                 .callback = &lvgl_tick,
    //                 .name = "lvgl_tick",
    //                 .skip_unhandled_events = true};
    
    // ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    // /*Delete the Default display refresh timer; 
    // Only when some other process/task in your program is going to do this for you*/
    // lv_timer_del(disp->refr_timer);
    // disp->refr_timer = NULL;
    // /*Note: Call '_lv_disp_refr_timer(NULL);'
    // whenever you want to refresh the dirty areas */
    
/*in this project this will fire 10 times a second, mainly to keep the display 'touch' working*/
/* Will start timer later, after everything is fully configured */
//    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000)); // here time is in micro seconds
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

/*Flush the content of the internal buffer the specific area on the display.
 *`px_map` contains the rendered image as raw pixel map and it should be copied to `area` on the display.
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_display_flush_ready()' has to be called when it's finished.*/
static void disp_flush(lv_display_t * disp_drv, const lv_area_t * area, uint8_t * px_map)
{
    /*JMH Note: this routine does NOT use 'disp_drv' */
	//esp_lcd_panel_handle_t display_handle = display_handle// (esp_lcd_panel_handle_t) disp_drv;//
    if(disp_flush_enabled) {
	const int offsetx1 = area->x1;
	const int offsetx2 = area->x2;
	const int offsety1 = area->y1;
	const int offsety2 = area->y2;
	//printf("offsetx1: %d; offsetx2: %d; offsety1: %d; offsety2: %d; \n", offsetx1, offsetx2, offsety1 , offsety2);
    /*espIDF framework function*/
    esp_lcd_panel_draw_bitmap(display_handle, offsetx1, offsety1, offsetx2+1, offsety2+1, px_map);
/* Waiting for the last frame buffer to complete transmission */
#if CONFIG_MSGBX_AVOID_TEAR_EFFECT_WITH_SEM
	xSemaphoreGive(sem_gui_ready);
	if (xSemaphoreTake(sem_vsync_end, pdMS_TO_TICKS(100)) == pdTRUE)
		lv_disp_flush_ready(disp_drv); // JMH ADD
	else
		printf("lvgl_flush_cb: failed to take 'sem_vsync_end' Semaphore\n");
#else
	lv_disp_flush_ready(drv);
#endif
    }
}


/*lvgl_tick_timer Call back routine*/
static void lvgl_tick(void *arg)
{
	(void)arg;

	lv_tick_inc(LV_TICK_PERIOD_MS);
}

static bool lvgl_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
	BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_MSGBX_AVOID_TEAR_EFFECT_WITH_SEM
	if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
	{
		xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
	}
#endif
	return high_task_awoken == pdTRUE;
}

/* Called periodically by the LVGL timer, to read the touchpad */
static void touchpad_read_cb(lv_indev_t * indev_drv, lv_indev_data_t * data)
{   /*LVGL9*/
    static int32_t last_x = 0;
    static int32_t last_y = 0;
    uint16_t touchpad_x[1] = {0};
	uint16_t touchpad_y[1] = {0};
    uint16_t strength[1] = {0};
	uint8_t touchpad_cnt = 0;
    uint8_t track_id[1] = {0};
    uint8_t point_num[1] = {0};
    //esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    /*have to 1st read/update the touch data, before it can be analyzed*/
    //printf("i2c read touch Start\n");
    tp->read_data(tp);
   
    /*Test & Save the pressed coordinates and the state*/
    //esp_lcd_touch_get_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *track_id, uint8_t *point_num, uint8_t max_point_num)
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
    //if(touchpad_pressed) printf("touchpad_read_cb: touchpad_cnt: %d\n", touchpad_cnt);
    if(touchpad_pressed && touchpad_cnt > 0)
    {
        // printf("touchpad_read_cb\n");    
        //touchpad_get_xy(&last_x, &last_y);
        last_x = touchpad_x[0];
 		last_y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;
    //printf("i2c read touch DONE\n");
}


bool lvgl_port_lock(int timeout_ms)
{
    //ESP_PANEL_CHECK_NULL_RET(lvgl_mux, false, "LVGL mutex is not initialized");
    assert(lvgl_mux);
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return (xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE);
}

bool lvgl_port_unlock_WShr(void)
{
    //ESP_PANEL_CHECK_NULL_RET(lvgl_mux, false, "LVGL mutex is not initialized");
    if(lvgl_mux == NULL){
        printf("lvgl_port_unlock_WShr: lvgl_mux is NULL\n");
        return false;
    }
    assert(lvgl_mux);
    //xSemaphoreGiveRecursive(lvgl_mux);
    xQueueGiveMutexRecursive(lvgl_mux);

    return true;
}

/* MAIN LVGL Loop/Task */
static void lvgl_port_task(void *arg)
{
    ESP_LOGD(TAG, "Starting LVGL task");

    uint32_t task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
    while (1) {
        if (lvgl_port_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            lvgl_port_unlock_WShr();
        }
        if (task_delay_ms > LVGL_PORT_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_PORT_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_PORT_TASK_MIN_DELAY_MS;
        }
        //printf("lvgl_port_task Wait(ms): %d\n", (int)task_delay_ms);
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

