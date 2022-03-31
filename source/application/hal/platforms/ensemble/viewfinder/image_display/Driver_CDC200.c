/* Copyright (c) 2021 ALIF SEMICONDUCTOR

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ALIF SEMICONDUCTOR nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

/**************************************************************************//**
 * @file     Driver_CDC200.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     30-Sep-2021
 * @brief    Display controller driver source file.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "display.h"
#include "Driver_CDC200.h"
#include "RTE_Components.h"
#include CMSIS_device_header


void display_controler_clock_config (void)
{
	volatile int *ptr;

	//cfgslv1; clock divider need to match pixel clock
	ptr = (int *) (CFGSLV1_BASE + EXPSLV1_CDC200_PIXCLK_CTRL);
	*ptr = (EXPSLV1_PIXEL_CLOCK_ENABLE | EXPSLV1_PIXEL_CLOCK_DIVISOR);
}

/**
 \fn            void set_display_controller_image_dimension (uint32_t image_format, uint16_t height, uint16_t width);
 \brief         Configuring the display controller for Image dimension.
 \param[in]     image_format: Image standard or type
 \param[in]     height: Height of the display.
 \param[in]     width: Width of the display
 */
void set_display_controller_image_dimension (uint32_t image_format, uint16_t height, uint16_t width)
{
	CDC200_RegInfo *regbase = (CDC200_RegInfo *) CDC200_BASE;

	switch (image_format)
	{
		case 0: //ARGB8888
		{
			// In ARGB8888 standard One pixel handled by 4 byte, so width size multiplied with 4 //
			regbase->layer1_reg.fb_length = (((width * 4) << 16) | ((width * 4) + BUS_WIDTH));
			break;
		}
		case 1: //RGB888
		{
			// In RGB888 standard One pixel handled by 3 byte, so width size multiplied with 3 //
			regbase->layer1_reg.fb_length = (((width * 3) << 16) | ((width * 3) + BUS_WIDTH));

			//Alpha constant
			regbase->layer1_reg.alpha = ALPHA_CONSTANT;
			break;
		}
		case 2: //RGB565
		{
			// In ARGB8888 standard One pixel handled by 2 byte, so width size multiplied with 2 //
			regbase->layer1_reg.fb_length = (((width * 2) << 16) | ((width * 2) + BUS_WIDTH));

			//Alpha constant
			regbase->layer1_reg.alpha = ALPHA_CONSTANT;
			break;
		}
	}

	regbase->layer1_reg.pixel_format = image_format;

	regbase->layer1_reg.fb_lines = height;
}

/**
 \fn            int32_t display_controller_setup (uint32_t image_buff_address, uint32_t image_format, const display_panel *panel)
 \brief         Configuring the display controller.
 \param[in]     image_buff_address: Image stored memory starting address.
 \param[in]     image_format: Image standard or type
 \param[in]     panel: pointer to panel info.
 \param[out]    execution status.
 */
int32_t display_controller_setup (uint32_t image_buff_address, uint32_t image_format, const display_panel *panel)
{
	uint32_t rd_data = 0;
	uint32_t sync_size = (((panel->hsync - 1) <<16 ) + (panel->vsync - 1));
	uint32_t back_porch = (panel->h_back_porch << 16) + panel->v_back_porch + sync_size;
	uint32_t active_width = (panel->width <<16) + panel->height + back_porch;
	uint32_t total_width = (panel->h_front_porch <<16) + panel->v_front_porch + active_width;

	CDC200_RegInfo *regbase = (CDC200_RegInfo *) CDC200_BASE;

	if (image_buff_address == 0)                                         { return DRIVER_ERROR; }
	if (image_format > RGB565)                                           { return DRIVER_ERROR; }
	if ((panel->h_back_porch == 0) || (panel->v_back_porch == 0))        { return DRIVER_ERROR; }
	if ((panel->width == 0) || (panel->height == 0))                     { return DRIVER_ERROR; }
	if ((panel->h_front_porch == 0) || (panel->h_front_porch == 0))      { return DRIVER_ERROR; }

	display_controler_clock_config();

	//enable single frame mode
	rd_data = regbase->global_reg.global_control;
	rd_data = ((rd_data & 0xF0000000) | 0x00000001);
	regbase->global_reg.global_control = rd_data;

	regbase->global_reg.background_color = 0;

	regbase->global_reg.irq_enable = 0;

	regbase->global_reg.sync_size = sync_size;

	regbase->global_reg.back_porch = back_porch;

	regbase->global_reg.active_width = active_width;

	regbase->global_reg.total_width = total_width;

	regbase->layer1_reg.control = 0x00000001;

	regbase->layer1_reg.window_h = ((active_width & 0xffff0000u) | ((back_porch >> 16)+1));

	regbase->layer1_reg.window_v = (((active_width & 0xffffu) << 16) | ((back_porch & 0xffffu) + 1));

	regbase->layer1_reg.fb_start = image_buff_address;

	set_display_controller_image_dimension (image_format, panel->height, panel->width);

	regbase->global_reg.shadow_reload_control = 0x00000001;

	regbase->global_reg.irq_enable = 0x00000001;

	regbase->global_reg.irq_clear = 0x00000001;

	regbase->global_reg.global_configuration_1 = 0;

	rd_data = regbase->global_reg.global_configuration_2;
	regbase->global_reg.global_configuration_2 = (rd_data | 0x00001000);

	rd_data = regbase->global_reg.global_control;
	regbase->global_reg.global_control = (rd_data | 0x00000001);

	return DRIVER_OK;
}
