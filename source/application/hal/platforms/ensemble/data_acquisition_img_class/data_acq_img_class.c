#include "data_acq.h"

#include "bsp.h"
#include "base_def.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "delay.h"
#include "image_processing.h"
#include "Driver_CPI.h"
#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h"

extern uint8_t raw_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES + 0x460];
extern uint8_t rgb_image[CIMAGE_X*CIMAGE_Y*RGB_BYTES];
extern uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][RGB_BYTES];
extern ARM_DRIVER_GPIO Driver_GPIO1;

int init_data_acq_img_class(int idx)
{
	(void)(idx);
	int cinit = camera_init(raw_image);
	if (cinit != 0) {
		while(1) {
			Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_LOW);
			sleep_or_wait_msec(300);
			Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_HIGH);
			sleep_or_wait_msec(300);
		}
	}
	DEBUG_PRINTF("Camera initialized... \n");
	Driver_GPIO1.SetValue(PIN_NUMBER_14, GPIO_PIN_OUTPUT_STATE_HIGH);

    return 0;
}

int get_data_img_class(int idx)
{
uint8_t *ml_image = (uint8_t *)idx;

    camera_start(CAMERA_MODE_SNAPSHOT);
    camera_wait(100);
    // RGB conversion and frame resize
    bayer_to_RGB(raw_image+0x460, rgb_image);
    // Cropping and scaling
    crop_and_interpolate(rgb_image, CIMAGE_X, CIMAGE_Y, raw_image, MIMAGE_X, MIMAGE_Y, RGB_BYTES * 8);
    // Color correction for white balance
    white_balance(raw_image, ml_image);
    return 0;
}

