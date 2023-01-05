/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "grbl/grbllib.h"
#include "main.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
__IO uint32_t uwTick;

void Driver_IncTick (void);
/*******************************************************************************
 * Code
 ******************************************************************************/
 void SysTick_Handler(void)
{
	time_isr();
	uwTick++;
	Driver_IncTick();
//	if(uwTick % 500 == 0)
//		USER_LED_TOGGLE();
}

/*!
 * @brief Main function
 */
int main(void)
{
	//, FSL_SDK_DRIVER_QUICK_ACCESS_ENABLE
	status_t status;
    char ch;
//	gpio_pin_config_t USER_LED_config = 
//	{
//		.direction = kGPIO_DigitalOutput,
//		.outputLogic = 0U,
//		.interruptMode = kGPIO_NoIntmode
//	};
//	GPIO_PinInit(GPIO1, 9U, &USER_LED_config);
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
	
	Set_NVIC_PriorityGroup(Group_4);
	PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
	PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
	PRINTF("OSC:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_OscClk));
	PRINTF("IPG:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_IpgClk));
	PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
	PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
	PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
	PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
	PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
	PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk)); 
	PRINTF("USB1PLL:		 %d Hz\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllClk));
	PRINTF("USB1PDF0:		 %d Hz\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk));
	flash_init();
	
//	TMRn_PWM_Init();
//	TMR_Pulse_Init();
//	while(1)
//	{
//		
//	}
	/* Set systick reload value to generate 1ms interrupt */
	if (SysTick_Config(SystemCoreClock / 1000U))
	{
		while (1)
		{
			
		}
	}
//	SDK_DelayAtLeastUs(100000,BOARD_BOOTCLOCKRUN_CORE_CLOCK);
//	PIT_TIMER_Init();
//	PIT_StartTimer(PIT, PIT_CHANNEL_X);

//	eth_calling();
	grbl_enter();
	
	while(1)
	{
		
	}
}
