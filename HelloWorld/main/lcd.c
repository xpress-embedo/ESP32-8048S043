/*
 * lcd.c
 *
 *  Created on: Mar 2, 2024
 *      Author: xpress_embedo
 */

#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h"

#include "lvgl.h"

#include "lcd.h"

// Private Function Prototypes
static void lcd_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);

// Private Variables
static const char *TAG = "LCD";
static SemaphoreHandle_t lvgl_semaphore = NULL;
static TaskHandle_t lcd_task_handle;


// Public Function Definition
esp_err_t lcd_init( void )
{
  static esp_lcd_panel_handle_t panel_handle = nullptr;
  // contains internal graphics buffer (draw buffers)
  static lv_disp_draw_buf_t disp_buf;
  // display drivers, contains callback functions
  static lv_disp_drv_t disp_drv;

  gpio_config_t backlight_config = {
    .pin_bit_mask = 1u << LCD_PIN_BK_LIGHT,
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE,
  };

  esp_lcd_rgb_panel_config_t panel_config = {
    .clk_src                = LCD_CLK_SRC_DEFAULT,
    .timings = {
      .pclk_hz              = LCD_PIXEL_CLOCK_HZ,
      .h_res                = LCD_H_RES,
      .v_res                = LCD_V_RES,
      .hsync_pulse_width    = 4,
      .hsync_back_porch     = 8,
      .hsync_front_porch    = 8,
      .vsync_pulse_width    = 4,
      .vsync_back_porch     = 8,
      .vsync_front_porch    = 8,
      .flags {
        .hsync_idle_low   = false,
        .vsync_idle_low   = false,
        .de_idle_high     = false,
        .pclk_active_neg  = true,
        .pclk_idle_high   = false
      }
    },
    .data_width             = 16,
    .bits_per_pixel         = 0,
    .num_fbs                = 2,
    .bounce_buffer_size_px  = 0, //4 * LCD_H_RES,
    .sram_trans_align       = 0,
    .psram_trans_align      = 64,

    .hsync_gpio_num         = LCD_PIN_HSYNC,
    .vsync_gpio_num         = LCD_PIN_VSYNC,
    .de_gpio_num            = LCD_PIN_DE,
    .pclk_gpio_num          = LCD_PIN_PCLK,
    .disp_gpio_num          = LCD_PIN_DISP_EN,
    .data_gpio_nums = {
      LCD_PIN_DATA0,
      LCD_PIN_DATA1,
      LCD_PIN_DATA2,
      LCD_PIN_DATA3,
      LCD_PIN_DATA4,
      LCD_PIN_DATA5,
      LCD_PIN_DATA6,
      LCD_PIN_DATA7,
      LCD_PIN_DATA8,
      LCD_PIN_DATA9,
      LCD_PIN_DATA10,
      LCD_PIN_DATA11,
      LCD_PIN_DATA12,
      LCD_PIN_DATA13,
      LCD_PIN_DATA14,
      LCD_PIN_DATA15
    },
    .flags
    {
      .disp_active_low      = 0,
      .refresh_on_demand    = 0,
      .fb_in_psram          = true,
      .double_fb            = true,
      .no_fb                = 0,
      .bb_invalidate_cache  = 0
    }
  };

  ESP_LOGI(TAG, "Turn off LCD Back-Light");
  ESP_ERROR_CHECK( gpio_config(&backlight_config) );

  ESP_LOGI(TAG, "Install RGB LCD panel driver");
  ESP_ERROR_CHECK( esp_lcd_new_rgb_panel(&panel_config, &panel_handle) );

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  ESP_ERROR_CHECK( esp_lcd_panel_reset(panel_handle) );
  ESP_ERROR_CHECK( esp_lcd_panel_init(panel_handle) );

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();

  ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  void *buf1 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  void *buf2 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);

  // initialize the LVGL draw buffers
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 100);

  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res    = LCD_H_RES;
  disp_drv.ver_res    = LCD_V_RES;
  disp_drv.flush_cb   = lcd_flush_cb;
  disp_drv.draw_buf   = &disp_buf;
  disp_drv.user_data  = panel_handle;

  lv_disp_drv_register( &disp_drv );

}

// Private Function Definition
static void lcd_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  auto panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);

  // touch handling todo

  lvgl_semaphore = xSemaphoreCreateMutex();

  xTaskCreate( &lcd_task, "LCD Task", 4096, NULL, tskIDLE_PRIORITY, &lcd_task_handle);
}

