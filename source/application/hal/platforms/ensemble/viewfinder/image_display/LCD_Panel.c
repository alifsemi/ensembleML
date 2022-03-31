#include <stdint.h>
#include <stdio.h>
#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h"
#include "mipi_dsi_host.h"
#include "RTE_Components.h"
#include "Driver_CDC200.h"
#include "display.h"
#include "delay.h"
#include "image_processing.h"
#include CMSIS_device_header

extern ARM_DRIVER_GPIO Driver_GPIO4;

int Display_reset(void)
{
	Driver_GPIO4.Initialize(PIN_NUMBER_6, NULL);
	Driver_GPIO4.PowerControl(PIN_NUMBER_6, ARM_POWER_FULL);
	Driver_GPIO4.SetDirection(PIN_NUMBER_6, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config(PORT_NUMBER_4, PIN_NUMBER_6, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config(PORT_NUMBER_4, PIN_NUMBER_6, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));

	Driver_GPIO4.SetValue(PIN_NUMBER_6, GPIO_PIN_OUTPUT_STATE_HIGH);

	sleep_or_wait_msec(10);

	Driver_GPIO4.SetValue(PIN_NUMBER_6, GPIO_PIN_OUTPUT_STATE_LOW);

	sleep_or_wait_msec(100);

	Driver_GPIO4.SetValue(PIN_NUMBER_6, GPIO_PIN_OUTPUT_STATE_HIGH);

	sleep_or_wait_msec(10);
	return ARM_DRIVER_OK;
}

int Display_BL_LED_EN(void)
{
	Driver_GPIO4.Initialize(PIN_NUMBER_4, NULL);
	Driver_GPIO4.PowerControl(PIN_NUMBER_4, ARM_POWER_FULL);
	Driver_GPIO4.SetDirection(PIN_NUMBER_4, GPIO_PIN_DIRECTION_OUTPUT);
	PINMUX_Config(PORT_NUMBER_4, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config(PORT_NUMBER_4, PIN_NUMBER_4, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS));

	Driver_GPIO4.SetValue(PIN_NUMBER_4, GPIO_PIN_OUTPUT_STATE_HIGH);
	return ARM_DRIVER_OK;
}

void LCD_config ()
{
	//***************************************************************//LCD SETING
	// Change to Page 1 CMD
	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x01);
	DCSW1_S(0x08, 0x00);
	DCSW1_S(0x20, 0x00);
	DCSW1_S(0x21, 0x01);
	DCSW1_S(0x30, 0x02);
	DCSW1_S(0x31, 0x00);
	DCSW1_S(0x40, 0x00);
	DCSW1_S(0x41, 0x33);
	DCSW1_S(0x42, 0x02);
	DCSW1_S(0x43, 0x89);
	DCSW1_S(0x44, 0x89);
	DCSW1_S(0x46, 0x34);

	DCSW1_S(0x50, 0xA8);
	DCSW1_S(0x51, 0xA8);
	DCSW1_S(0x52, 0x00);
	DCSW1_S(0x53, 0x78);
	DCSW1_S(0x54, 0x00);
	DCSW1_S(0x55, 0x78);

	DCSW1_S(0x60, 0x07);
	DCSW1_S(0x61, 0x04);
	DCSW1_S(0x62, 0x08);
	DCSW1_S(0x63, 0x04);

	DCSW1_S(0xA0, 0x00);
	DCSW1_S(0xA1, 0x0B);
	DCSW1_S(0xA2, 0x13);
	DCSW1_S(0xA3, 0x0D);
	DCSW1_S(0xA4, 0x07);
	DCSW1_S(0xA5, 0x0B);
	DCSW1_S(0xA6, 0x07);
	DCSW1_S(0xA7, 0x06);
	DCSW1_S(0xA8, 0x07);
	DCSW1_S(0xA9, 0x0A);
	DCSW1_S(0xAA, 0x12);
	DCSW1_S(0xAB, 0x0D);
	DCSW1_S(0xAC, 0x11);
	DCSW1_S(0xAD, 0x0F);
	DCSW1_S(0xAE, 0x0E);
	DCSW1_S(0xAF, 0x0B);

	DCSW1_S(0xC0, 0x00);
	DCSW1_S(0xC1, 0x0B);
	DCSW1_S(0xC2, 0x13);
	DCSW1_S(0xC3, 0x0D);
	DCSW1_S(0xC4, 0x06);
	DCSW1_S(0xC5, 0x0B);
	DCSW1_S(0xC6, 0x07);
	DCSW1_S(0xC7, 0x06);
	DCSW1_S(0xC8, 0x07);
	DCSW1_S(0xC9, 0x0A);
	DCSW1_S(0xCA, 0x12);
	DCSW1_S(0xCB, 0x0D);
	DCSW1_S(0xCC, 0x11);
	DCSW1_S(0xCD, 0x0F);
	DCSW1_S(0xCE, 0x0E);
	DCSW1_S(0xCF, 0x0B);

	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x06);

	DCSW1_S(0x00, 0x21);
	DCSW1_S(0x01, 0x09);
	DCSW1_S(0x02, 0x00);
	DCSW1_S(0x03, 0x00);
	DCSW1_S(0x04, 0x01);
	DCSW1_S(0x05, 0x01);
	DCSW1_S(0x06, 0x98);
	DCSW1_S(0x07, 0x05);
	DCSW1_S(0x08, 0x02);
	DCSW1_S(0x09, 0x00);
	DCSW1_S(0x0A, 0x00);
	DCSW1_S(0x0B, 0x00);
	DCSW1_S(0x0C, 0x01);
	DCSW1_S(0x0D, 0x01);
	DCSW1_S(0x0E, 0x00);
	DCSW1_S(0x0F, 0x00);
	DCSW1_S(0x10, 0xE0);
	DCSW1_S(0x11, 0xE0);
	DCSW1_S(0x12, 0x00);
	DCSW1_S(0x13, 0x00);
	DCSW1_S(0x14, 0x00);
	DCSW1_S(0x15, 0x43);
	DCSW1_S(0x16, 0x08);
	DCSW1_S(0x17, 0x00);
	DCSW1_S(0x18, 0x00);
	DCSW1_S(0x19, 0x00);
	DCSW1_S(0x1A, 0x00);
	DCSW1_S(0x1B, 0x00);
	DCSW1_S(0x1C, 0x00);
	DCSW1_S(0x1D, 0x00);
	DCSW1_S(0x20, 0x01);
	DCSW1_S(0x21, 0x23);
	DCSW1_S(0x22, 0x45);
	DCSW1_S(0x23, 0x67);
	DCSW1_S(0x24, 0x01);
	DCSW1_S(0x25, 0x23);
	DCSW1_S(0x26, 0x45);
	DCSW1_S(0x27, 0x67);

	DCSW1_S(0x30, 0x01);
	DCSW1_S(0x31, 0x11);
	DCSW1_S(0x32, 0x00);
	DCSW1_S(0x33, 0x22);
	DCSW1_S(0x34, 0x22);
	DCSW1_S(0x35, 0xCB);
	DCSW1_S(0x36, 0xDA);
	DCSW1_S(0x37, 0xAD);
	DCSW1_S(0x38, 0xBC);
	DCSW1_S(0x39, 0x66);
	DCSW1_S(0x3A, 0x77);
	DCSW1_S(0x3B, 0x22);
	DCSW1_S(0x3C, 0x22);
	DCSW1_S(0x3D, 0x22);
	DCSW1_S(0x3E, 0x22);
	DCSW1_S(0x3F, 0x22);
	DCSW1_S(0x40, 0x22);

	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x07);

	DCSW1_S(0x18, 0x1D);
	DCSW1_S(0x02, 0x77);
	DCSW1_S(0xE1, 0x79);

	DCSW_L(0xFF, 0x98, 0x06, 0x04, 0x00);

	DCSW1_S(0x36, 0x01);
	DCSW1_S(0x3A, 0x70); //24BIT
	DCSWN_S(0x11);

	//delay(120);
	for(int i=0;i<120;i++);

	DCSWN_S(0x29);

	//delay(25);
	for(int i=0;i<125;i++);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//Normal Display mode on
	DCSW1_S(0x05, 0x13);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//All Pixel On
	DCSW1_S(0x05, 0x23);

	//05 DCS Write, No Parameter SPa (Short Packet) DCSWN-S
	//Display On
	DCSW1_S(0x05, 0x29);
}

// video format rgb888
#define	HSYNC 64
#define	VSYNC 10
#define	HBP   94
#define	VBP   20
#define	HACT  480
#define	VACT  800
#define	HFP   0x48B 		//adjust this to sync up DSI host controller
#define	VFP   20

const static display_panel cdc = {

	.height = VACT,
	.width = HACT,
	.hsync = HSYNC,
	.vsync = VSYNC,
	.h_back_porch = HBP,
	.v_back_porch = VBP,
	.h_front_porch = HFP,
	.v_front_porch = VFP,
};


int Display_initialization(uint8_t *buffer)
{
	uint32_t i = 0;
	uint32_t *p;

	int retry = 5;

	////////////////////////////////////////////////////////////////
	//Frame buffer cleanup
	////////////////////////////////////////////////////////////////
	for (i = 0, p = (uint32_t *)buffer; i < (DISPLAY_BUFFER_SIZE / 4); i++, p++)
		*p = 0xF5F5F5F5;

	do {
		////////////////////////////////////////////////////////////////
		//display setup
		////////////////////////////////////////////////////////////////
		Display_reset();


		////////////////////////////////////////////////////////////////////////////
		// CDC200 Generator
		////////////////////////////////////////////////////////////////////////////

		display_controller_setup((uint32_t)(buffer), RGB888, &cdc);

		sleep_or_wait_msec(10);
		////////////////////////////////////////////////////////////////
		// PHY SETUP
		////////////////////////////////////////////////////////////////

		//turn on Dig 38.4MHz clock
		HW_REG_WORD(0x7100a100, 0x00) = 0x00010A29;
		sleep_or_wait_msec(10);

		//configure source to crystal
		HW_REG_WORD(0x71007410,0x00) = 0x00000002;
		sleep_or_wait_msec(10);

		//enable phy power
		HW_REG_WORD(0x7004001C,0x00) |= (1U << 10);
		sleep_or_wait_msec(100);

		if (tx_phyconfig() == 0)
			break;

		retry--;
	} while (retry > 0);

	if (retry <= 0) {
		return -1;
	}

	////////////////////////////////////////////////////////////////////////////
	// Video Mode Pattern Generator
	////////////////////////////////////////////////////////////////////////////

	dsi_reset();
	dsi_command_mode_initialization();
	dsi_powerup();

	LCD_config();

	////////////////////////////////////////////////////////////////////////////
	// video mode
	////////////////////////////////////////////////////////////////////////////
	dsi_reset();
	dsi_video_mode_initialization();
	dsi_powerup();

	Display_BL_LED_EN();
	return 0;
}
