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

//FreeRTOS hreader
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "grbl/grbllib.h"
#include "main.h"
#include "RGBLED_DIALOG_BACKEND.h"
#include "core_delay.h"  
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
TaskHandle_t grbl_task_handler;

void time_isr(void);
/*******************************************************************************
 * Code
 ******************************************************************************/
//void SysTick_Handler(void)
//{
//	time_isr();
//	uwTick++;
//	Driver_IncTick();
////	if(uwTick % 500 == 0)
////		USER_LED_TOGGLE();
//}
uint8_t EEPROM_Test(void);
void test_func();


#define MCU_INFI_NAME                   "IMX.RT1052"
#define MCU_INFO_FLASH                  "8M"
#define MCU_INFO_RAM                    "1024K"
#define GRBL_VERSION 					"1.4h"
#define GRBL_VERSION_BUILD 				"20221222"
void grblReprotMcuInfo(void) {

    PRINTF("\r\n/*********************************************************/\r\n");
    PRINTF("*-\\    |    /\r\n");
    PRINTF("* --OpenGRBL--\r\n");
    PRINTF("*-/    |    \\\r\n");
    PRINTF("*-CPU Name:%s\r\n", MCU_INFI_NAME);
    PRINTF("*-CPU Flash:%s\r\n", MCU_INFO_FLASH);
    PRINTF("*-CPU RAM:%s\r\n", MCU_INFO_RAM);
    PRINTF("*-CPU Clock:%ldMHz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk)/1000000);
    PRINTF("*-CPU Step Clock:%ldMHz\r\n", CLOCK_GetFreq(kCLOCK_OscClk)/1000000);
//    PRINTF("*-Flash Info:0x%lx, flash_size:%ldMB\r\n", (uint32_t)sFlash.info.flash_id, (sFlash.info.flash_size / (uint32_t)1024));
    PRINTF("*-BuildVersion:%s\r\n", GRBL_VERSION_BUILD);
    PRINTF("/*********************************************************/\r\n");
}

void enter_grbl_task(void *parg) 
{
	grbl_enter();
}

void grblTaskInit(void) 
{
  xTaskCreate(enter_grbl_task, 
              "grbl task", 
              1024 * 2, 
              NULL, 
              1, 
              &grbl_task_handler
              );

}

/*!
 * @brief Main function
 */
int main(void)
{
    char ch;
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
	
//	PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
//	PRINTF("AHB:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_AhbClk));
//	PRINTF("OSC:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_OscClk));
//	PRINTF("IPG:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_IpgClk));
//	PRINTF("SEMC:            %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
//	PRINTF("SYSPLL:          %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
//	PRINTF("SYSPLLPFD0:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
//	PRINTF("SYSPLLPFD1:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
//	PRINTF("SYSPLLPFD2:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
//	PRINTF("SYSPLLPFD3:      %d Hz\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk)); 
//	PRINTF("USB1PLL:		 %d Hz\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllClk));
//	PRINTF("USB1PDF0:		 %d Hz\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk));
	grblReprotMcuInfo();
	grblTaskInit();
//	TMRn_PWM_Init();
//	TMR_Pulse_Init();
//	while(1)
//	{
//		
//	}
	/* Set systick reload value to generate 1ms interrupt */
//	if (SysTick_Config(SystemCoreClock / 1000U))
//	{
//		while (1)
//		{
//			
//		}
//	}
//	SDK_DelayAtLeastUs(100000,BOARD_BOOTCLOCKRUN_CORE_CLOCK);
//	PIT_TIMER_Init();
//	PIT_StartTimer(PIT, PIT_CHANNEL_X);
//	grbl_enter();
//	FlexPWM_Init();
//	while(1)
//	{
//		for(uint8_t i = 0;i<255;i++)
//		{
//			CPU_TS_Tmr_Delay_MS(10);
//			SetColorValue(i,0,0);
//		}
//		for(uint8_t i = 0;i<255;i++)
//		{
//			CPU_TS_Tmr_Delay_MS(10);
//			SetColorValue(0,i,0);
//		}
//		for(uint8_t i = 0;i<255;i++)
//		{
//			CPU_TS_Tmr_Delay_MS(10);
//			SetColorValue(0,0,i);
//		}
//	}
	return 0;
}
