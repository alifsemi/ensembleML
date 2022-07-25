/*
 * Copyright (c) 2021 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "stubs_ensemble.h"

#include "bsp_core_log.h"

#include <inttypes.h>

#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h"
#include "base_def.h"
#include "delay.h"
#include "image_processing.h"
#include "display.h"
#include "LCD_panel.h"
#include "lvgl.h"

extern void lv_port_disp_init(void);
extern uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][RGB_BYTES];
extern uint8_t raw_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES];
extern uint8_t rgb_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES];
extern ARM_DRIVER_GPIO Driver_GPIO1;

void SetupLEDs()
{
	// Green LED
	Driver_GPIO1.Initialize(PIN_NUMBER_15,NULL);
	Driver_GPIO1.PowerControl(PIN_NUMBER_15,  ARM_POWER_FULL);
	Driver_GPIO1.SetValue(PIN_NUMBER_15, GPIO_PIN_OUTPUT_STATE_LOW);
	Driver_GPIO1.SetDirection(PIN_NUMBER_15, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_0);

	// Red LED
	Driver_GPIO1.Initialize(PIN_NUMBER_14,NULL);
	Driver_GPIO1.PowerControl(PIN_NUMBER_14,  ARM_POWER_FULL);
	Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_LOW);
	Driver_GPIO1.SetDirection(PIN_NUMBER_14, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_0);
}

uint32_t GetCoreClock(void)
{
    return 1;
}

void GLCD_Initialize(void) 
{
	volatile int test = 3;
	SetupLEDs();

	SysTick_Config(SystemCoreClock/1000);

	while(test) {
		Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_HIGH);
		sleep_or_wait_msec(300);
		Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_LOW);
		sleep_or_wait_msec(300);
		test--;
	}

	static volatile int dinit = 0;

	dinit = Display_initialization(&lcd_image[0][0][0]);

	if (dinit != 0) {
		while(1) {
			Driver_GPIO1.SetValue(PIN_NUMBER_15, GPIO_PIN_OUTPUT_STATE_LOW);
			sleep_or_wait_msec(300);
			Driver_GPIO1.SetValue(PIN_NUMBER_15, GPIO_PIN_OUTPUT_STATE_HIGH);
			sleep_or_wait_msec(300);
		}
	}
	Driver_GPIO1.SetValue(PIN_NUMBER_15, GPIO_PIN_OUTPUT_STATE_HIGH);
   	lv_port_disp_init();
}

void GLCD_Bitmap(unsigned int x,  unsigned int y,
    unsigned int w, unsigned int h, unsigned short *bitmap)
{
    UNUSED(x);
    UNUSED(y);
    UNUSED(w);
    UNUSED(h);
    UNUSED(bitmap);
}

#define XOFFS 16
#define YOFFS 16

void write_to_lcd(uint8_t src[MIMAGE_Y][MIMAGE_X][RGB_BYTES], uint8_t dst[DIMAGE_Y][DIMAGE_X][RGB_BYTES]) {
	int32_t x, x1, y, y1;
	uint8_t r, g, b;

	for (y1 = 0; y1 < MIMAGE_Y; y1++) {
		for (x1 = 0; x1 < MIMAGE_X; x1++) {
			b = src[y1][x1][0];
			r = src[y1][x1][1];
			g = src[y1][x1][2];

			x = XOFFS + (x1 << 1);
			y = YOFFS + (y1 << 1);

			dst[y][x][0] = b;
			dst[y][x][1] = r;
			dst[y][x][2] = g;

			dst[y][x+1][0] = b;
			dst[y][x+1][1] = r;
			dst[y][x+1][2] = g;

			dst[y+1][x][0] = b;
			dst[y+1][x][1] = r;
			dst[y+1][x][2] = g;

			dst[y+1][x+1][0] = b;
			dst[y+1][x+1][1] = r;
			dst[y+1][x+1][2] = g;
		}
	}
}


void GLCD_Image(void *data, const uint32_t width, const uint32_t height,
    const uint32_t channels, const uint32_t pos_x,
    const uint32_t pos_y, const uint32_t downsample_factor)
{
    UNUSED(data);
    UNUSED(pos_x);
    UNUSED(pos_y);
    UNUSED(width);
    UNUSED(height);
    UNUSED(channels);
    UNUSED(downsample_factor);
    
    write_to_lcd((uint8_t (*)[MIMAGE_Y][RGB_BYTES])data, lcd_image);

	lv_task_handler();
}

void GLCD_Clear(unsigned short color)
{
    UNUSED(color);
}

void GLCD_SetTextColor(unsigned short color)
{
    UNUSED(color);
}

void GLCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char fi,
    unsigned char c)
{
    UNUSED(ln);
    UNUSED(col);
    UNUSED(fi);
    UNUSED(c);
}

void GLCD_DisplayString(unsigned int ln, unsigned int col, unsigned char fi,
    char *s)
{
    UNUSED(ln);
    UNUSED(col);
    UNUSED(fi);
    UNUSED(s);
    debug("text display: %s\n", s);
}

void GLCD_Box(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
    unsigned short color)
{
    UNUSED(x);
    UNUSED(y);
    UNUSED(w);
    UNUSED(h);
    UNUSED(color);
}

void LED_Initialize(uint32_t port)
{
    UNUSED(port);
}

void LED_On(uint32_t num, uint32_t port)
{
    UNUSED(num);
    UNUSED(port);
    debug("LED %" PRIu32 " ON\n", num);
}

void LED_Off(uint32_t num, uint32_t port)
{
    UNUSED(num);
    UNUSED(port);
    debug("LED %" PRIu32 " OFF\n", num);
}
