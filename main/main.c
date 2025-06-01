#if 0
/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"


#include "lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define LV_TICK_PERIOD_MS 1

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);
static void create_demo_application(void);

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    xTaskCreatePinnedToCore(guiTask, "gui", 4096*2, NULL, 0, NULL, 1);

    for (int i = 1000; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

SemaphoreHandle_t xGuiSemaphore;
static void guiTask(void *pvParameter)
{
    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    /* Use double buffered when not working with monochrome displays */
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
    static lv_disp_draw_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;
    /* Initialize the working buffer depending on the selected display. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
    create_demo_application();

    while (1)
    {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }
}

static void create_demo_application(void)
{
    /* Get the current screen  */
    lv_obj_t * scr = lv_disp_get_scr_act(NULL);
    /*Create a Label on the currently active screen*/
    lv_obj_t * label1 =  lv_label_create(scr);
    /*Modify the Label's text*/
    lv_label_set_text(label1, "Hello\nworld");
    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}
#endif

/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "esp_err.h"
 #include "esp_log.h"
 #include "esp_check.h"
 #include "driver/i2c.h"
 #include "driver/gpio.h"
 #include "driver/spi_master.h"
 #include "esp_lcd_panel_io.h"
 #include "esp_lcd_panel_vendor.h"
 #include "esp_lcd_panel_ops.h"
 #include "esp_lvgl_port.h"

 #include "nofrendo.h"

 /* LCD size */
 #define EXAMPLE_LCD_H_RES   (240)
 #define EXAMPLE_LCD_V_RES   (320)

 /* LCD settings */
 #define EXAMPLE_LCD_SPI_NUM         (SPI3_HOST)
 #define EXAMPLE_LCD_PIXEL_CLK_HZ    (40 * 1000 * 1000)
 #define EXAMPLE_LCD_CMD_BITS        (8)
 #define EXAMPLE_LCD_PARAM_BITS      (8)
 #define EXAMPLE_LCD_COLOR_SPACE     (ESP_LCD_COLOR_SPACE_BGR)
 #define EXAMPLE_LCD_BITS_PER_PIXEL  (16)
 #define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
 #define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)
 #define EXAMPLE_LCD_BL_ON_LEVEL     (1)

 /* LCD pins */
 #define EXAMPLE_LCD_GPIO_SCLK       (GPIO_NUM_42)
 #define EXAMPLE_LCD_GPIO_MOSI       (GPIO_NUM_41)
 #define EXAMPLE_LCD_GPIO_RST        (GPIO_NUM_40)
 #define EXAMPLE_LCD_GPIO_DC         (GPIO_NUM_39)
 #define EXAMPLE_LCD_GPIO_CS         (GPIO_NUM_38)
 #define EXAMPLE_LCD_GPIO_BL         (GPIO_NUM_37)

 static const char *TAG = "EXAMPLE";

 // LVGL image declare
LV_IMG_DECLARE(esp_logo)

 /* LCD IO and panel */
 static esp_lcd_panel_io_handle_t lcd_io = NULL;
 static esp_lcd_panel_handle_t lcd_panel = NULL;

 /* LVGL display and touch */
 static lv_display_t *lvgl_disp = NULL;

 static esp_err_t app_lcd_init(void)
 {
     esp_err_t ret = ESP_OK;

     /* LCD backlight */
     gpio_config_t bk_gpio_config = {
         .mode = GPIO_MODE_OUTPUT,
         .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL
     };
     ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

     /* LCD initialization */
     ESP_LOGD(TAG, "Initialize SPI bus");
     const spi_bus_config_t buscfg = {
         .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
         .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
         .miso_io_num = GPIO_NUM_NC,
         .quadwp_io_num = GPIO_NUM_NC,
         .quadhd_io_num = GPIO_NUM_NC,
         .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
     };
     ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

     ESP_LOGD(TAG, "Install panel IO");
     const esp_lcd_panel_io_spi_config_t io_config = {
         .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
         .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
         .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
         .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
         .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
         .spi_mode = 0,
         .trans_queue_depth = 10,
     };
     ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

     ESP_LOGD(TAG, "Install LCD driver");
     const esp_lcd_panel_dev_config_t panel_config = {
         .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
         .color_space = EXAMPLE_LCD_COLOR_SPACE,
         .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
     };
     ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

     esp_lcd_panel_reset(lcd_panel);
     esp_lcd_panel_init(lcd_panel);
     esp_lcd_panel_mirror(lcd_panel, true, true);
     esp_lcd_panel_disp_on_off(lcd_panel, true);

     /* LCD backlight on */
     ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

     return ret;

 err:
     if (lcd_panel) {
         esp_lcd_panel_del(lcd_panel);
     }
     if (lcd_io) {
         esp_lcd_panel_io_del(lcd_io);
     }
     spi_bus_free(EXAMPLE_LCD_SPI_NUM);
     return ret;
 }

 static esp_err_t app_lvgl_init(void)
 {
     /* Initialize LVGL */
     const lvgl_port_cfg_t lvgl_cfg = {
         .task_priority = 4,         /* LVGL task priority */
         .task_stack = 4096,         /* LVGL task stack size */
         .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
         .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
         .timer_period_ms = 5        /* LVGL timer tick period in ms */
     };
     ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

     /* Add LCD screen */
     ESP_LOGD(TAG, "Add LCD screen");
     const lvgl_port_display_cfg_t disp_cfg = {
         .io_handle = lcd_io,
         .panel_handle = lcd_panel,
         .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
         .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
         .hres = EXAMPLE_LCD_H_RES,
         .vres = EXAMPLE_LCD_V_RES,
         .monochrome = false,
 #if LVGL_VERSION_MAJOR >= 9
         .color_format = LV_COLOR_FORMAT_RGB565,
 #endif
         .rotation = {
             .swap_xy = false,
             .mirror_x = true,
             .mirror_y = true,
         },
         .flags = {
             .buff_dma = true,
 #if LVGL_VERSION_MAJOR >= 9
             .swap_bytes = true,
 #endif
         }
     };
     lvgl_disp = lvgl_port_add_disp(&disp_cfg);

     return ESP_OK;
 }

 static void _app_button_cb(lv_event_t *e)
 {
     lv_disp_rotation_t rotation = lv_disp_get_rotation(lvgl_disp);
     rotation++;
     if (rotation > LV_DISPLAY_ROTATION_270) {
         rotation = LV_DISPLAY_ROTATION_0;
     }

     /* LCD HW rotation */
     lv_disp_set_rotation(lvgl_disp, rotation);
 }

 static void app_main_display(void)
 {
     lv_obj_t *scr = lv_scr_act();

     /* Task lock */
     lvgl_port_lock(0);

     /* Your LVGL objects code here .... */

    /* Create image */
    lv_obj_t *img_logo = lv_img_create(scr);
    lv_img_set_src(img_logo, &esp_logo);
    lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 20);

     /* Label */
     lv_obj_t *label = lv_label_create(scr);
     lv_obj_set_width(label, EXAMPLE_LCD_H_RES);
     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
 #if LVGL_VERSION_MAJOR == 8
     lv_label_set_recolor(label, true);
     lv_label_set_text(label, "#FF0000 "LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"#\n#FF9400 "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING" #");
 #else
     lv_label_set_text(label, LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"\n "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING);
 #endif
     lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

     /* Button */
     lv_obj_t *btn = lv_btn_create(scr);
     label = lv_label_create(btn);
     lv_label_set_text_static(label, "Rotate screen");
     lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -30);
     lv_obj_add_event_cb(btn, _app_button_cb, LV_EVENT_CLICKED, NULL);

     /* Task unlock */
     lvgl_port_unlock();
 }

#include "spi_flash_mmap.h"
#include "esp_partition.h"
#include "nvs_flash.h"
char *osd_getromdata() {
	char* romdata;
	const esp_partition_t* part;
	spi_flash_mmap_handle_t hrom;
	esp_err_t err;
	nvs_flash_init();
	part=esp_partition_find_first(0x40, 1, NULL);
	if (part==0) printf("Couldn't find rom part!\n");
	err=esp_partition_mmap(part, 0, 3*1024*1024, SPI_FLASH_MMAP_DATA, (const void**)&romdata, &hrom);
	if (err!=ESP_OK) printf("Couldn't map rom part!\n");
	printf("Initialized. ROM@%p\n", romdata);
    return (char*)romdata;
}

 void app_main(void)
 {
     /* LCD HW initialization */
     ESP_ERROR_CHECK(app_lcd_init());

     /* LVGL initialization */
     ESP_ERROR_CHECK(app_lvgl_init());

     printf("NES 52Pi Version Start!\n");
     nofrendo_main(0, NULL);
     printf("WtF?\n");

     /* Show LVGL objects */
     app_main_display();
 }