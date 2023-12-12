#pragma once

/*
    TRGB Library by Nick Daria

    Much borrowed code from LILYGO and ian, but a better Arduino-style structure with more configuration.
*/

#include <Arduino.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"

#include "XL9535_driver.h"
#include "lvgl.h"
#include "pins.h"
#include "panel.h"
#include "ft3267.h"

enum TRGB_ERR {
    TRGB_OK,
    TRGB_ERR_NOT_INIT,
    TRGB_ERR_NOT_ENABLED,
};

class TRGBModule {
    public:
        //  Library Required Functions
        TRGB_ERR start();

        //  Display Configuration Setters
        void useDisplay(bool enable_display = true) { this->_useDisplay = enable_display; };
        void useDisplayTouch(bool enable_touch = true) { this->_useDisplayTouch = enable_touch; }
        void useLvgl(bool useLvgl = true) { this->_useLvgl = useLvgl; }
        void useBootImage(const uint8_t* bootImage) {this->_bootImage = bootImage; }   //  Could add a simple check, but I suppose you'll find any issues with it fairly quickly

        //  Display Functions
        void display_backlight(bool state);

        //  Power Functions
        void power_deepSleep(bool enableTouchWakeup = true);
        void power_restart();
        float power_getVoltage();

        //  IO Expander
        XL9535 xl;
    private:
        //  Display Configuration Variables
        bool _useDisplay;
        bool _useLvgl;
        bool _useDisplayTouch;
        const uint8_t* _bootImage;

        //  Display Supporting Functions
        lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
        lv_disp_drv_t disp_drv;      // contains callback functions
        lv_indev_drv_t indev_drv;
        esp_lcd_panel_handle_t panel_handle;

        void tft_init(void);
        void lcd_cmd(const uint8_t cmd);
        void lcd_data(const uint8_t *data, int len);
        void lcd_send_data(uint8_t data);
        static esp_lcd_panel_handle_t register_tft();

        bool i2c_init();
        void xl_init();
        void touch_init();
        void io_init();
        void lvgl_start();
        void draw_boot_logo();
};