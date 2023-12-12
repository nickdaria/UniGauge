#pragma once

#define LCD_PIXEL_CLOCK_HZ     (8 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT       46
#define PIN_NUM_HSYNC          47
#define PIN_NUM_VSYNC          41
#define PIN_NUM_DE             45
#define PIN_NUM_PCLK           42
// #define PIN_NUM_DATA0          44
#define PIN_NUM_DATA1          21
#define PIN_NUM_DATA2          18
#define PIN_NUM_DATA3          17
#define PIN_NUM_DATA4          16
#define PIN_NUM_DATA5          15
#define PIN_NUM_DATA6          14
#define PIN_NUM_DATA7          13
#define PIN_NUM_DATA8          12
#define PIN_NUM_DATA9          11
#define PIN_NUM_DATA10         10
#define PIN_NUM_DATA11         9
// #define PIN_NUM_DATA12         43
#define PIN_NUM_DATA13         7
#define PIN_NUM_DATA14         6
#define PIN_NUM_DATA15         5
#define PIN_NUM_DATA16         3
#define PIN_NUM_DATA17         2
#define PIN_NUM_DISP_EN        -1

// The pixel number in horizontal and vertical
#define LCD_H_RES              480
#define LCD_V_RES              480

#define IIC_SCL_PIN                    48
#define IIC_SDA_PIN                    8

#define SD_CLK_PIN                     39
#define SD_CMD_PIN                     40
#define SD_D0_PIN                      38

#define BAT_VOLT_PIN                   4
#define TP_INT_PIN                     1

#define BOOT_BTN_PIN 0

/* XL9535 --- PIN - P0*/
#define TP_RES_PIN   1
#define PWR_EN_PIN   2
#define LCD_CS_PIN   3
#define LCD_SDA_PIN  4
#define LCD_CLK_PIN  5
#define LCD_RST_PIN  6
#define SD_CS_PIN    7
