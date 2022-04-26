/* Copyright (c) 2021 - 2022 ALIF SEMICONDUCTOR

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
 * @file     arx3A0_camera_sensor.h
 * @author   xxx
 * @email    xxx@alifsemi.com
 * @version  V1.0.0
 * @date     25-April-2022
 * @brief    Initialize, start and stop camera.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#ifndef ARX3A0_CAMERA_SENSOR_H_
#define ARX3A0_CAMERA_SENSOR_H_

#include <stdint.h>

/* TODO: Need to Move this to RTE_device.h*/
#define RTE_ARX3A0_CAMERA_SENSOR_ENABLE                   1
#define RTE_ARX3A0_CAMERA_SENSOR_I2C_USING_I3Cx_INSTANCE  0

/* Proceed only if ARX3A0 Camera Sensor is enabled. */
#if RTE_ARX3A0_CAMERA_SENSOR_ENABLE

typedef enum {
    ARX3A0_CAMERA_RESOLUTION_560x560,
} ARX3A0_Camera_Resolution;

/**
*  @brief  Initialize ARX3A0 Camera Sensor
*          - initialize i2c using i3c instance
*          - software reset ARX3A0 Camera Sensor
*          - read ARX3A0 chip-id, proceed only it is correct.
*          - Camera Output Format
*             (currently supports only RAW Bayer10 format)
*          - Issue Change-Config Command to re-configure
*             all the ARX3A0 Camera Sensor sub-system and registers.
*             This command must be issued after any change in
*             sensor registers to take effect, for detail refer data-sheet.
*  @return      0 if successful, error code otherwise.
*/
int32_t arx3A0_Init();

/**
*  @brief  Initialize ARX3A0 Camera Sensor.
*          - configure Camera Sensor resolution registers as per input parameter.
*            (currently supports only 560x560(WxH) Camera resolution)
*          - configure Camera Sensor output format registers as per input parameter
*            (currently supports only RAW Bayer10 Format)
*          - configure Camera Sensor slew rate.
*  @param[in]  cam_resolution Camera Sensor Resolution @ref ARX3A0_Camera_Resolution
*
*  @return 0 if successful, error code otherwise.
*/
int32_t arx3A0_camera_init(ARX3A0_Camera_Resolution cam_resolution);

/**
*  @brief  Start ARX3A0 Camera Sensor Streaming.
*  @return 0 if successful, error code otherwise.
*/
int32_t arx3A0_Start(void);

/**
*  @brief  Stop ARX3A0 Camera Sensor Streaming.
*  @return 0 if successful, error code otherwise.
*/
int32_t arx3A0_Stop(void);

#endif

#endif /* ARX3A0_CAMERA_SENSOR_H_ */
