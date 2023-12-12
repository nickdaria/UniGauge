#include <Arduino.h>
#include <TRGB.h>
#include "img.h"
//#include "ui/ui.h"

TRGBModule TRGB;

void setup() {
    TRGB.useDisplay(true);
    TRGB.useDisplayTouch(true);
    TRGB.useLvgl(false);
    TRGB.useBootImage(bootLogo_map);

    TRGB.start();

    //ui_init();
}

void loop() {
    //lv_timer_handler();
    
}