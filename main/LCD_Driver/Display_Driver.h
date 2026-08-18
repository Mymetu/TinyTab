#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "lvgl.h"

/* The Waveshare BSP drives the 1024x600 EK79007 MIPI-DSI panel and GT911. */
#define EXAMPLE_LCD_H_RES 1024
#define EXAMPLE_LCD_V_RES 600
#define Offset_X 0
#define Offset_Y 0
#define Backlight_MAX 100

extern uint8_t LCD_Backlight;

bool Display_Init(void);
bool Display_Is_Ready(void);
void Backlight_Init(void);
void Set_Backlight(uint8_t light);

/* Kept for source compatibility with the original 2.8-inch application. */
void LCD_Init(void);
