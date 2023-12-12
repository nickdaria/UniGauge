#include "TRGB.h"

static bool touch_pin_get_int=false;

/*
    Library & Board
*/
bool TRGBModule::i2c_init() {
    return Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t) 400000);
}

void TRGBModule::xl_init() {
    this->xl.begin();
	uint8_t pin = (1 << PWR_EN_PIN)  | (1 << LCD_CS_PIN)  | (1 << TP_RES_PIN)
			    | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) | (1 << LCD_RST_PIN)
			    | (1 << SD_CS_PIN);

	this->xl.pinMode8(0, pin, OUTPUT);
	this->xl.digitalWrite(PWR_EN_PIN, 1);

	// Enable CS for SD card
	this->xl.digitalWrite(SD_CS_PIN, 1); // To use SDIO one-line mode, you need to pull the CS pin high
}

void TRGBModule::touch_init() {
    //  Only if enabled
    if(this->_useDisplayTouch == false) {
        return;
    }

    pinMode(TP_INT_PIN, INPUT_PULLUP);
	attachInterrupt(TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);
}

void TRGBModule::io_init() {
    //  Voltage Input
    pinMode(BAT_VOLT_PIN, ANALOG);
}

TRGB_ERR TRGBModule::start() {
    if(this->i2c_init() == false) { return TRGB_ERR_NOT_INIT; };
    this->xl_init();
    this->tft_init();
    this->draw_boot_logo();
    this->touch_init();
    this->io_init();
    this->lvgl_start();

    return TRGB_OK;
}

/*
    Display Supporting
*/
void TRGBModule::tft_init(void) {
    this->xl.digitalWrite(LCD_CS_PIN, 1);
    this->xl.digitalWrite(LCD_SDA_PIN, 1);
    this->xl.digitalWrite(LCD_CLK_PIN, 1);

    // Reset the display and touch

    //  xl.digitalWrite(LCD_RST_PIN, 1);
    //  vTaskDelay(200 / portTICK_PERIOD_MS);
    this->xl.digitalWrite(LCD_RST_PIN, 0);
    this->xl.digitalWrite(TP_RES_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    this->xl.digitalWrite(LCD_RST_PIN, 1);
    this->xl.digitalWrite(TP_RES_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ft3267_init(Wire);

    // Switch on backlight
    pinMode(PIN_NUM_BK_LIGHT, OUTPUT);
    digitalWrite(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    int cmd = 0;
    while (st_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    cmd++;
    }
    // Switch on backlight
    pinMode(PIN_NUM_BK_LIGHT, OUTPUT);
    digitalWrite(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);

    this->panel_handle = register_tft();
}

void TRGBModule::lcd_cmd(const uint8_t cmd) {
    this->xl.digitalWrite(LCD_CS_PIN, 0);
    this->xl.digitalWrite(LCD_SDA_PIN, 0);
    this->xl.digitalWrite(LCD_CLK_PIN, 0);
    this->xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(cmd);
    this->xl.digitalWrite(LCD_CS_PIN, 1);
}

void TRGBModule::lcd_data(const uint8_t *data, int len) {
    uint32_t i = 0;
    if (len == 0)
    return; // no need to send anything
    do {
        this->xl.digitalWrite(LCD_CS_PIN, 0);
        this->xl.digitalWrite(LCD_SDA_PIN, 1);
        this->xl.digitalWrite(LCD_CLK_PIN, 0);
        this->xl.digitalWrite(LCD_CLK_PIN, 1);
        lcd_send_data(*(data + i));
        this->xl.digitalWrite(LCD_CS_PIN, 1);
        i++;
    } while (len--);
}

void TRGBModule::lcd_send_data(uint8_t data) {
    uint8_t n;
    for (n = 0; n < 8; n++) {
        if (data & 0x80)
        this->xl.digitalWrite(LCD_SDA_PIN, 1);
        else
        this->xl.digitalWrite(LCD_SDA_PIN, 0);

        data <<= 1;
        this->xl.digitalWrite(LCD_CLK_PIN, 0);
        this->xl.digitalWrite(LCD_CLK_PIN, 1);
    }
}

static void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  if (touch_pin_get_int) {
    uint8_t touch_points_num;
    uint16_t x, y;
    ft3267_read_pos(&touch_points_num, &x, &y);
    if (touch_points_num > 0) {
    	data->point.x = x;
    	data->point.y = y;
    	data->state = LV_INDEV_STATE_PRESSED;
    } else {
    	data->state =  LV_INDEV_STATE_RELEASED;
    }
    touch_pin_get_int = false;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

esp_lcd_panel_handle_t TRGBModule::register_tft() {
    esp_lcd_panel_handle_t panel_handle = NULL;
	esp_lcd_rgb_panel_config_t panel_config = { .clk_src = LCD_CLK_SRC_PLL160M,
			.timings = { .pclk_hz = LCD_PIXEL_CLOCK_HZ, .h_res =
					LCD_H_RES,
					.v_res = LCD_V_RES, // The following parameters should refer to LCD spec
					.hsync_pulse_width = 1, .hsync_back_porch = 30,
					.hsync_front_porch = 50, .vsync_pulse_width = 1,
					.vsync_back_porch = 30, .vsync_front_porch = 20, .flags = {
							.pclk_active_neg = 1 } },
			.data_width = 16, // RGB565 in parallel mode, thus 16bit in width
			.psram_trans_align = 64, .hsync_gpio_num = PIN_NUM_HSYNC,
			.vsync_gpio_num = PIN_NUM_VSYNC, .de_gpio_num =
					PIN_NUM_DE, .pclk_gpio_num = PIN_NUM_PCLK,
			.data_gpio_nums = { // PIN_NUM_DATA0,
					PIN_NUM_DATA13, PIN_NUM_DATA14,
							PIN_NUM_DATA15, PIN_NUM_DATA16,
							PIN_NUM_DATA17, PIN_NUM_DATA6,
							PIN_NUM_DATA7, PIN_NUM_DATA8,
							PIN_NUM_DATA9, PIN_NUM_DATA10,
							PIN_NUM_DATA11, // PIN_NUM_DATA12,
							PIN_NUM_DATA1, PIN_NUM_DATA2,
							PIN_NUM_DATA3, PIN_NUM_DATA4,
							PIN_NUM_DATA5 }, .disp_gpio_num =
					PIN_NUM_DISP_EN, .on_frame_trans_done = NULL,
			.user_ctx = NULL, .flags = { .fb_in_psram = 1 } };
	// allocate frame buffer in PSRAM
	ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
	return panel_handle;
}

/*
    Display
*/
void TRGBModule::display_backlight(bool state) {
  digitalWrite(PIN_NUM_BK_LIGHT, state ? LCD_BK_LIGHT_ON_LEVEL : LCD_BK_LIGHT_OFF_LEVEL);
}

void TRGBModule::draw_boot_logo() {
    //  Only if provided
    if(this->_bootImage == NULL) {
        return;
    }

    //  Only if display init
    if(this->_useDisplay == false) {
        return;
    }

    //  Print bitmap boot image
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 480, 480, this->_bootImage);
}

void TRGBModule::lvgl_start() {
    lv_init();
	// alloc draw buffers used by LVGL from PSRAM
	lv_color_t *buf1 = (lv_color_t*) heap_caps_malloc(
			LCD_H_RES * LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf1);
	lv_color_t *buf2 = (lv_color_t*) heap_caps_malloc(
			LCD_H_RES * LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
	assert(buf2);
	lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);
    //  Display driver registered to LVGL
    
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = LCD_H_RES;
	disp_drv.ver_res = LCD_V_RES;
	disp_drv.flush_cb = lvgl_flush_cb;
	disp_drv.draw_buf = &disp_buf;
	disp_drv.user_data = panel_handle;
	lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = lv_touchpad_read;
	lv_indev_drv_register(&indev_drv);
}

/*
    Power
*/
void TRGBModule::power_deepSleep(bool enableTouchWakeup) {
    if (enableTouchWakeup) {
        //  Configure touch wakeup
        detachInterrupt(TP_INT_PIN);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT_PIN, 0);
    }

    //  Configure XL9535
    this->xl.pinMode8(0, 0xff, INPUT);
    this->xl.pinMode8(1, 0xff, INPUT);
    this->xl.read_all_reg();

    //  Sleep
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    esp_deep_sleep_start();
}

void TRGBModule::power_restart() {
    esp_restart();
}

float TRGBModule::power_getVoltage() {
    return (analogRead(BAT_VOLT_PIN) * 2 * 3.3) / 4096;
}