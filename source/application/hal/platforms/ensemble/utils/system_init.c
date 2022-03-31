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
#include "system_init.h"

#include "uart_stdout.h"

#include <string.h>
#include <inttypes.h>

#include "RTE_Device.h"
#include "RTE_Components.h"
#include "Driver_PINMUX_AND_PINPAD.h"
#include "Driver_GPIO.h"

#include CMSIS_device_header

#include "lvgl.h"

uint32_t volatile ms_ticks = 0;
static uint64_t cpu_cycle_count = 0;

int Init_SysTick(void);

/**
 * SysTick initialisation
 */
int Init_SysTick(void) 
{
    const uint32_t ticks_1ms = GetSystemCoreClock()/1000 + 1;
    int err = 0;

    /* Reset CPU cycle count value. */
    cpu_cycle_count = 0;

    /* Changing configuration for sys tick => guard from being
     * interrupted. */
    NVIC_DisableIRQ(SysTick_IRQn);

    /* SysTick init - this will enable interrupt too. */
    err = SysTick_Config(ticks_1ms);

    /* Enable interrupt again. */
    NVIC_EnableIRQ(SysTick_IRQn);

    /* Wait for SysTick to kick off */
    while (!err && !SysTick->VAL) {
        __NOP();
    }

    return err;
}

/**
 * @brief   System tick interrupt handler.
 **/
void SysTick_Handler(void)
{
    /* Increment the cycle counter based on load value. */
    cpu_cycle_count += SysTick->LOAD + 1;
    lv_tick_inc(1);
}

/**
 * Gets the current SysTick derived counter value
 */
uint64_t Get_SysTick_Cycle_Count(void)
{
    uint32_t systick_val;

    NVIC_DisableIRQ(SysTick_IRQn);
    systick_val = SysTick->VAL & SysTick_VAL_CURRENT_Msk;
    NVIC_EnableIRQ(SysTick_IRQn);

    return cpu_cycle_count + (SysTick->LOAD - systick_val);
}

extern ARM_DRIVER_GPIO Driver_GPIO1;
extern ARM_DRIVER_GPIO Driver_GPIO3;

int system_init(void)
{

    /* UART init - will enable valid use of printf (stdout
     * re-directed at this UART (UART0) */
    //UartStdOutInit();
    info("Processor internal clock: %" PRIu32 "Hz\n", GetSystemCoreClock());

    // Initialize GPIOs to capture the buttons state

	Driver_GPIO1.Initialize(PIN_NUMBER_12, NULL);
	Driver_GPIO1.PowerControl(PIN_NUMBER_12, ARM_POWER_FULL);
	Driver_GPIO1.SetDirection(PIN_NUMBER_12, GPIO_PIN_DIRECTION_INPUT);
	PINMUX_Config(PORT_NUMBER_1, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config(PORT_NUMBER_1, PIN_NUMBER_12, (PAD_FUNCTION_READ_ENABLE|PAD_FUNCTION_SCHMITT_TRIGGER_ENABLE|PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_HIGH_Z));

	Driver_GPIO3.Initialize(PIN_NUMBER_4, NULL);
	Driver_GPIO3.PowerControl(PIN_NUMBER_4, ARM_POWER_FULL);
	Driver_GPIO3.SetDirection(PIN_NUMBER_4, GPIO_PIN_DIRECTION_INPUT);
	PINMUX_Config(PORT_NUMBER_3, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_0);
	PINPAD_Config(PORT_NUMBER_3, PIN_NUMBER_4, (PAD_FUNCTION_READ_ENABLE|PAD_FUNCTION_SCHMITT_TRIGGER_ENABLE|PAD_FUNCTION_DRIVER_DISABLE_STATE_WITH_HIGH_Z));

    info("%s: complete\n", __FUNCTION__);
    return 0;
}

void system_release(void)
{
    __disable_irq();
}

void system_name(char* name, size_t size)
{
    strncpy(name, DESIGN_NAME, size);
}

static bool do_inference_once = true;
static bool last_btn0 = false;
static bool last_btn1 = false;

bool run_requested(void)
{
    bool ret = true;
    bool new_btn0, new_btn1;
    uint32_t pin_state0, pin_state1;

    // Get new button state (active low)
    Driver_GPIO1.GetValue(PIN_NUMBER_12, &pin_state0); 
    new_btn0 = pin_state0 == 0;
    Driver_GPIO3.GetValue(PIN_NUMBER_4, &pin_state1); 
    new_btn1 = pin_state1 == 0;

    if (do_inference_once)
    {
        // Edge detector - run inference on the positive edge of the button pressed signal
        ret = !last_btn0 && new_btn0;
    }
    if (new_btn1 && last_btn1 != new_btn1)
    {
        // Switch single shot and continuous inference mode
        do_inference_once = !do_inference_once;
    }
    last_btn0 = new_btn0;
    last_btn1 = new_btn1;
    return ret;
}