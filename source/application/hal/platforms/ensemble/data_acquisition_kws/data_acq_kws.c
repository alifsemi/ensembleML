/* Project Includes */
#include <Driver_SAI.h>
#include <Driver_PINMUX_AND_PINPAD.h>

#include "RTE_Device.h"
#include "RTE_Components.h"

#include CMSIS_device_header

#include "bsp_core_log.h"

#define I2S_DAC 0               /* DAC I2S Controller 0 */
#define I2S_ADC 2               /* ADC I2S Controller 2 */

extern ARM_DRIVER_SAI ARM_Driver_SAI_(I2S_ADC);

#define AUDIO_SAMPLE_NUM        (5)
#define AUDIO_SAMPLE_SIZE       (1 << 12)	// 4096
#define AUDIO_BUFFER_SIZE       (AUDIO_SAMPLE_NUM*AUDIO_SAMPLE_SIZE)

ARM_DRIVER_SAI      *i2s_drv;
ARM_DRIVER_VERSION   version;
ARM_SAI_CAPABILITIES cap;

uint32_t wlen = 16;
uint32_t sampling_rate = 16000;

//int16_t audio_samples[AUDIO_SAMPLE_NUM][AUDIO_SAMPLE_SIZE] __attribute__((section(".ARM.__at_0x02000000")));

extern  int16_t audio0[64000];

uint32_t volatile i2s_callback_flag;
/**
\fn          void i2s_callback(uint32_t event)
\brief       Callback routine from the i2s driver
\param[in]   event Event for which the callback has been called
*/
void i2s_callback(uint32_t event)
{
    if (event & ARM_SAI_EVENT_RECEIVE_COMPLETE)
    {
        /* Receive Success */
        i2s_callback_flag = 1;
    }
    if (event & ARM_SAI_EVENT_RX_OVERFLOW)
    {

    }
}

int init_data_acq_kws(int idx)
{
    int32_t status = 0;

      /* Configure the adc pins */
   /* Configure P2_1.I2S2_SDI_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_3);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_1, PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_PULL_DOWN | PAD_FUNCTION_READ_ENABLE);

    /* Configure P2_3.I2S2_SCLK_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_3);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_3, PAD_FUNCTION_READ_ENABLE);

    /* Configure P2_3.I2S2_WS_A */
    status |= PINMUX_Config(PORT_NUMBER_2, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_2);
    status |= PINPAD_Config(PORT_NUMBER_2, PIN_NUMBER_4, PAD_FUNCTION_READ_ENABLE);
    if (status)
    {
        printf_err("I2S pinmux configuration failed\n");
        return -1;
    }
    
   /* Use the I2S as Receiver */
    i2s_drv = &ARM_Driver_SAI_(I2S_ADC);

    /* Verify the I2S API version for compatibility*/
    version = i2s_drv->GetVersion();
    info("I2S API version = %d\n", version.api);

    /* Verify if I2S protocol is supported */
    ARM_SAI_CAPABILITIES cap = i2s_drv->GetCapabilities();
    if (!cap.protocol_i2s)
    {
        printf_err("I2S is not supported\n");
        return -1;
    }

    /* Initializes I2S interface */
    status = i2s_drv->Initialize(i2s_callback);
    if (status)
    {
        printf_err("I2S Init failed status = %d\n", status);
        goto error_i2s_initialize;
    }

    /* Enable the power for I2S */
    status = i2s_drv->PowerControl(ARM_POWER_FULL);
    if (status)
    {
        printf_err("I2S Power failed status = %d\n", status);
        goto error_i2s_power;
    }

    /* configure I2S Receiver to Asynchronous Master */
    status = i2s_drv->Control(ARM_SAI_CONFIGURE_RX |
                              ARM_SAI_MODE_MASTER  |
                              ARM_SAI_MONO_MODE    |
                              ARM_SAI_ASYNCHRONOUS |
                              ARM_SAI_PROTOCOL_I2S |
                              ARM_SAI_DATA_SIZE(wlen), wlen*2, sampling_rate);
    if (status)
    {
        printf_err("I2S Control status = %d\n", status);
        goto error_i2s_control;
    }

    return 0;

error_i2s_control:
    i2s_drv->PowerControl(ARM_POWER_OFF);
error_i2s_power:
    i2s_drv->Uninitialize();
error_i2s_initialize:
    return -1;
}

int get_data_kws(int idx)
{
    int32_t status = 0;
    i2s_callback_flag = 0;

    /* Enable Receiver */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 1, 0);
    if (status)
    {
        printf_err("I2S Control RX start status = %d\n", status);
        return -1;
    }

    /* Receive data */
    status = i2s_drv->Receive(&audio0[0], 64000);
    //status = i2s_drv->Receive(&audio0[sample_index*4096], AUDIO_SAMPLE_SIZE);
    //status = i2s_drv->Receive(&audio_samples[sample_index][0], AUDIO_SAMPLE_SIZE);
    if (status)
    {
        printf_err("I2S Receive status = %d\n", status);
        return -1;
    }

    /* Wait for the completion event */
    while(1) {
        /*TODO: Add timeout */
        if (i2s_callback_flag) {
            break;
        }
    }

    /* Stop the RX */
    status = i2s_drv->Control(ARM_SAI_CONTROL_RX, 0, 0);
    if (status)
    {
        printf_err("I2S Control RX stop status = %d\n", status);
        return -1;
    }

    return 0;
}

