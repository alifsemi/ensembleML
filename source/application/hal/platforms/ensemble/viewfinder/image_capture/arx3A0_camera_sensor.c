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
 * @file     arx3A0_camera_sensor.c
 * @author   xxx
 * @email    xxx@alifsemi.com
 * @version  V1.0.0
 * @date     25-April-2022
 * @brief    Initialize, start and stop camera.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <assert.h>
#include "RTE_Device.h"

/* Project Includes */
#include "arx3A0_camera_sensor.h"
#include "arx3A0_camera_sensor_conf.h"
#include "Camera_Sensor_i2c.h"
#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h"

#include "Driver_Common.h"
#include "delay.h"
#include "base_def.h"

extern ARM_DRIVER_GPIO Driver_GPIO4;

#define CFGSLV_1                            0x4903F000
#define ARX3A0_CAMERA_PIX_CLOCK_SET         0x00140001
#define ARX3A0_SET_BIT_RESET_BITS           0x00
#define ARX3A0_SET_BIT_1                    0x01
#define ARX3A0_SET_BIT_7                    (1U << 7)
#define ARX3A0_CAMERA_SENSOR_SLAVE_ADDR     0x36
#define ARX3A0_SOFTWARE_RESET_REGISTER      0x0103
#define ARX3A0_CHIP_ID_REGISTER             0x3000
#define ARX3A0_CHIP_ID_REGISTER_VALUE       0x0353
#define ARX3A0_MODE_SELECT_REGISTER         0x0100
#define ARX3A0_MIPI_CONFIG_REGISTER         0x31BE
#define ARX3A0_RESET_REGISTER               0x301A

static CAMERA_SENSOR_SLAVE_I2C_CONFIG arx3A0_camera_sensor_i2c_cnfg =
{
  .I3Cx_instance                  = RTE_ARX3A0_CAMERA_SENSOR_I2C_USING_I3Cx_INSTANCE,
  .speed_mode                     = CAMERA_SENSOR_I2C_SPEED_SS_100_KBPS,
  .cam_sensor_slave_addr          = ARX3A0_CAMERA_SENSOR_SLAVE_ADDR,
  .cam_sensor_slave_reg_addr_type = CAMERA_SENSOR_I2C_REG_ADDR_TYPE_16BIT,
};

/* Wrapper function for Delay
 * Delay for millisecond:
 * Provide delay in terms of sleep or wait for millisecond
 * depending on RTOS availability.
 */
#define ARX3A0_DELAY_mSEC(msec)       sleep_or_wait_msec(msec)

/* Wrapper function for i2c read
 * read register value from ARX3A0 Camera Sensor registers
 * using i2c read API @ref camera_sensor_i2c_read
 *
 * for ARX3A0 Camera Sensor specific i2c configurations
 * see @ref ARX3A0_camera_sensor_i2c_cnfg
 */
#define ARX3A0_READ_REG(reg_addr, reg_value, reg_size) \
        camera_sensor_i2c_read(&arx3A0_camera_sensor_i2c_cnfg, \
                                reg_addr,  \
                                reg_value, \
                                reg_size);

/* Wrapper function for i2c write
 * write register value to ARX3A0 Camera Sensor registers
 * using i2c write API @ref camera_sensor_i2c_write.
 *
 * for ARX3A0 Camera Sensor specific i2c configurations
 * see @ref ARX3A0_camera_sensor_i2c_cnfg
 */
#define ARX3A0_WRITE_REG(reg_addr, reg_value, reg_size) \
        camera_sensor_i2c_write(&arx3A0_camera_sensor_i2c_cnfg, \
                                 reg_addr,  \
                                 reg_value, \
                                 reg_size);


static void arx3A0_PIN_configuration(void)
{
	// Camera Reset -> pin P4_5
    // Driver_GPIO4.XXX and PINXXX_Config functions can't really fail so let's ignore the result but assert for invalid configurations
    int32_t err = Driver_GPIO4.Initialize(PIN_NUMBER_5, NULL);
	assert(err == ARM_DRIVER_OK);

	err = Driver_GPIO4.PowerControl(PIN_NUMBER_5,  ARM_POWER_FULL);
	assert(err == ARM_DRIVER_OK);

	err = Driver_GPIO4.SetDirection(PIN_NUMBER_5, GPIO_PIN_DIRECTION_OUTPUT);
	assert(err == ARM_DRIVER_OK);

	// I3C_SDA_B  -> pin P3_8
	// I3C_SCL_B  -> pin P3_9

	/* Configure GPIO Pin : P3_8 as I3C_SDA_B */
	err = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_3);
	assert(err == ARM_DRIVER_OK);

	err = PINPAD_Config (PORT_NUMBER_3, PIN_NUMBER_8, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS)); //SDA
	assert(err == ARM_DRIVER_OK);

	/* Configure GPIO Pin : P3_9 as I3C_SCL_B */
	err = PINMUX_Config (PORT_NUMBER_3, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_4);
	assert(err == ARM_DRIVER_OK);

	err = PINPAD_Config (PORT_NUMBER_3, PIN_NUMBER_9, (0x09 | PAD_FUNCTION_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS)); //SCL
	assert(err == ARM_DRIVER_OK);

	/* Configure P2_7 as pixel clock output */
	err = PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_7, PINMUX_ALTERNATE_FUNCTION_6);
	assert(err == ARM_DRIVER_OK);
}

/**
*  @brief       write array of registers value to ARX3A0 Camera Sensor registers.
*  @param[in]   arx3A0_reg ARX3A0 Camera Sensor Register Array Structure
*  @param[in]   total_num total number of registers (size of array arx3A0_reg)
*  @param[in    register size in bits, @ref CAMERA_SENSOR_I2C_REG_SIZE
*  @return      0 if successful, error code otherwise
*/
static int32_t arx3A0_bulk_write_reg(const struct regval_list_16 arx3A0_reg[],
                                     const uint32_t total_num, const uint32_t reg_size)
{
    uint32_t i  = 0;
    int32_t ret = 0;

    for (i = 0; i < total_num; i++) {
        ret = ARX3A0_WRITE_REG(arx3A0_reg[i].address, arx3A0_reg[i].val, reg_size);

        if (ret != ARM_DRIVER_OK) {
            return ret;
        }
    }

    return ARM_DRIVER_OK;
}

/**
*  @brief  Hard Reset ARX3A0 Camera Sensor
*  @return ARM_DRIVER_OK
*/
static void arx3A0_Camera_Hard_reseten(void)
{
    int32_t err = Driver_GPIO4.SetValue(PIN_NUMBER_5, GPIO_PIN_OUTPUT_STATE_HIGH);
	assert(err == ARM_DRIVER_OK);
	ARX3A0_DELAY_mSEC(2);
	err = Driver_GPIO4.SetValue(PIN_NUMBER_5, GPIO_PIN_OUTPUT_STATE_LOW);
	assert(err == ARM_DRIVER_OK);
	ARX3A0_DELAY_mSEC(2);
	err = Driver_GPIO4.SetValue(PIN_NUMBER_5, GPIO_PIN_OUTPUT_STATE_HIGH);
	assert(err == ARM_DRIVER_OK);
	ARX3A0_DELAY_mSEC(50);
}

/**
*  @brief   Software Reset ARX3A0 Camera Sensor
*  @return  0 if successful, error code otherwise
*/
static int32_t arx3A0_soft_reset(void)
{
    int32_t ret = ARX3A0_WRITE_REG(ARX3A0_SOFTWARE_RESET_REGISTER, ARX3A0_SET_BIT_1, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);

    /* @Observation: more delay is required for Camera Sensor
    *               to setup after Soft Reset.
    */
    ARX3A0_DELAY_mSEC(100);

    return ret;
}

int32_t arx3A0_camera_init(ARX3A0_Camera_Resolution cam_resolution)
{
    uint32_t total_num = 0;
    int32_t  ret = 0;

    /* Configure Camera Sensor Resolution */
    switch(cam_resolution)
    {
        /* Camera Sensor Resolution: VGA 640x480(WxH) */
        case ARX3A0_CAMERA_RESOLUTION_560x560:
            total_num = (sizeof(arx3a0_560_regs) / sizeof(struct regval_list_16));
            ret = arx3A0_bulk_write_reg(arx3a0_560_regs, total_num, CAMERA_SENSOR_I2C_REG_SIZE_16BIT);
            if (ret != ARM_DRIVER_OK) {
                return ret;
            }
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}


int32_t arx3A0_Init()
{
    int32_t  ret = 0;
    uint32_t rcv_data = 0;

    /*
    * The camera pixel clock is wider to P2_6, but this signal is not propagating from the base board
    * P2_7 must be wired on the base board to P2_6 as a workaround!
    *
    * EXTCLK - 20Mhz camera xvclk - P2_7, select 6
    */
    HW_REG_WORD(CFGSLV_1, 0x00) = ARX3A0_CAMERA_PIX_CLOCK_SET; // set camera pix clock//sub system 4

    /* pin configuration */
    arx3A0_PIN_configuration();

    /* camera sensor resten */
    arx3A0_Camera_Hard_reseten();

    /* Initialize i2c using i3c driver instance depending on
    *  ARX3A0 Camera Sensor specific i2c configurations
    *   @ref arx3A0_camera_sensor_i2c_cnfg
    */
    ret = camera_sensor_i2c_init(&arx3A0_camera_sensor_i2c_cnfg);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Soft Reset ARX3A0 Camera Sensor */
    ret = arx3A0_soft_reset();
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Read ARX3A0 Camera Sensor CHIP ID */
    ret = ARX3A0_READ_REG(ARX3A0_CHIP_ID_REGISTER, &rcv_data, CAMERA_SENSOR_I2C_REG_SIZE_16BIT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Proceed only if CHIP ID is correct. */
    if (rcv_data != ARX3A0_CHIP_ID_REGISTER_VALUE) {
        return ARM_DRIVER_ERROR;
    }

    /* @NOTE: By-default after Soft-Reset Camera Sensor will be in streaming state,
    *        As per Hardware Jumper(P2 jumper) settings,
    *        if required then Stop Streaming using \ref arx3A0_stream_stop.
    *
    *        Suspend any stream
    *        ret = arx3A0_stream_stop();
    *        if(ret != ARM_DRIVER_OK)
    *          return ARM_DRIVER_ERROR;
    *        ARX3A0_DELAY_mSEC(10);
    */

    ret = ARX3A0_WRITE_REG(ARX3A0_MODE_SELECT_REGISTER, ARX3A0_SET_BIT_RESET_BITS, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = ARX3A0_READ_REG(ARX3A0_MIPI_CONFIG_REGISTER, &rcv_data, CAMERA_SENSOR_I2C_REG_SIZE_16BIT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    /* Enable LP11 on standby */
    ret = ARX3A0_WRITE_REG(ARX3A0_MIPI_CONFIG_REGISTER, rcv_data | ARX3A0_SET_BIT_7, CAMERA_SENSOR_I2C_REG_SIZE_16BIT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ret = ARX3A0_WRITE_REG(ARX3A0_MODE_SELECT_REGISTER, ARX3A0_SET_BIT_1, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);
    if (ret != ARM_DRIVER_OK) {
        return ret;
    }

    ARX3A0_DELAY_mSEC(200);

    return ARX3A0_WRITE_REG(ARX3A0_MODE_SELECT_REGISTER, ARX3A0_SET_BIT_RESET_BITS, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);
}

int32_t arx3A0_Start(void)
{
    /* Start streaming */
    int32_t ret = ARX3A0_WRITE_REG(ARX3A0_MODE_SELECT_REGISTER, ARX3A0_SET_BIT_1, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);

    /* @Observation: Proper Delay is required for
    *               Camera Sensor Lens to come-out from Shutter and gets steady,
    *               otherwise captured image will not be proper.
    *               adjust delay if captured image is less bright/dull.
    */
    ARX3A0_DELAY_mSEC(200);

    return ret;
}

int32_t arx3A0_Stop(void)
{
  /* Suspend any stream */
  return ARX3A0_WRITE_REG(ARX3A0_MODE_SELECT_REGISTER, ARX3A0_SET_BIT_RESET_BITS, CAMERA_SENSOR_I2C_REG_SIZE_8BIT);
}
