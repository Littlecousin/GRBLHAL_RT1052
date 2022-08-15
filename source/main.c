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
 * Definitions
 ******************************************************************************/
#define FlexSpiInstance           0U
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE
#define FLASH_SIZE                0x4000000UL /* 64MBytes */
#define FLASH_PAGE_SIZE           512UL       /* 512Bytes */
#define FLASH_SECTOR_SIZE         0x40000UL   /* 256KBytes */
#define FLASH_BLOCK_SIZE          0x40000UL   /* 256KBytes */
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
	uwTick++;
	Driver_IncTick();
	if(uwTick % 500 == 0)
		USER_LED_TOGGLE();
}

void UART_ModeConfig(void)
{
	/*定义串口配置参数结构体变量，用于保存串口的配置信息*/
	lpuart_config_t config;

	/*调用固件库函数得到默认的串口配置参数，在默认的配置参数基础上修改*/
	LPUART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;  //波特率

	config.rxIdleType = kLPUART_IdleTypeStopBit;
	config.rxIdleConfig = kLPUART_IdleCharacter128;

	config.enableRx = true; //是否允许接收数据
	config.enableTx = true;   //是否允许发送数据

//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0U); 
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0U); 
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_LPUART1_TX, 0x10B0U); 
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_LPUART1_RX, 0x10B0U); 
	
	/*调用固件库函数，将修改好的配置信息写入到串口的配置寄存器中*/
	LPUART_Init(LPUART1, &config, BOARD_DebugConsoleSrcFreq());

	/*允许接收中断*/
	LPUART_EnableInterrupts(LPUART1, kLPUART_RxDataRegFullInterruptEnable);
//	LPUART_EnableInterrupts(LPUART1, kLPUART_IdleLineInterruptEnable);
	/*设置中断优先级,*/
//	set_IRQn_Priority(DEBUG_UART_IRQ,Group4_PreemptPriority_6, Group4_SubPriority_0);
	/*使能中断*/
	EnableIRQ(LPUART1_IRQn);
}

void FLEXSPI_NorFlash_GetConfig_Hyperflash(flexspi_nor_config_t *config)
{
    config->memConfig.tag              = FLEXSPI_CFG_BLK_TAG;
    config->memConfig.version          = FLEXSPI_CFG_BLK_VERSION;
    config->memConfig.readSampleClkSrc = kFLEXSPIReadSampleClk_ExternalInputFromDqsPad;
    config->memConfig.serialClkFreq =
        kFLEXSPISerialClk_30MHz; /* Serial Flash Frequencey.See System Boot Chapter for more details */
    config->memConfig.lutCustomSeqEnable = true;
    config->memConfig.sflashA1Size       = FLASH_SIZE;
    config->memConfig.csHoldTime         = 3U;                       /* Data hold time, default value: 3 */
    config->memConfig.csSetupTime        = 3U;                       /* Date setup time, default value: 3 */
    config->memConfig.deviceType     = kFLEXSPIDeviceType_SerialNOR; /* Flash device type default type: Serial NOR */
    config->memConfig.deviceModeType = kDeviceConfigCmdType_Generic;
    config->memConfig.columnAddressWidth  = 3U;
    config->memConfig.deviceModeCfgEnable = 0U;
    config->memConfig.waitTimeCfgCommands = 0U;
    config->memConfig.configCmdEnable     = 0U;
    config->memConfig.busyOffset          = 15U;
    config->memConfig.busyBitPolarity     = 1U;
    /* Always enable Safe configuration Frequency */
    config->memConfig.controllerMiscOption = FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_DiffClkEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_WordAddressableEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_SafeConfigFreqEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_DdrModeEnable);
    config->memConfig.sflashPadType = kSerialFlash_8Pads; /* Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal */
    config->pageSize                = FLASH_PAGE_SIZE;
    config->sectorSize              = FLASH_SECTOR_SIZE;
    config->blockSize               = FLASH_BLOCK_SIZE;
    config->isUniformBlockSize      = true;
    config->ipcmdSerialClkFreq      = kFLEXSPISerialClk_30MHz; /* Clock frequency for IP command */
    config->serialNorType           = kSerialNorType_HyperBus;

    // Read
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READ + 0U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0U, RADDR_DDR, FLEXSPI_8PAD, 0x18U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READ + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, DUMMY_DDR, FLEXSPI_8PAD, 0x06U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READ + 2U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04U, STOP, FLEXSPI_1PAD, 0x0U);

    // Read Status
    // 0
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 2U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 3U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x70U);
    // 1
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 4U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0xA0U, RADDR_DDR, FLEXSPI_8PAD, 0x18U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 5U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, DUMMY_RWDS_DDR, FLEXSPI_8PAD, 0x0BU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_READSTATUS + 6U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(READ_DDR, FLEXSPI_8PAD, 0x04U, STOP, FLEXSPI_1PAD, 0x00U);

    // Write Enable
    // 0
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 2U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 3U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    // 1
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 4U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 5U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 6U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x02U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE + 7U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U);

    // Page Program
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 2U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 3U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xA0U);
    // 1
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 4U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, RADDR_DDR, FLEXSPI_8PAD, 0x18U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 5U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, WRITE_DDR, FLEXSPI_8PAD, 0x80U);

    // Erase Sector
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 1U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 2U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 3U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x80U);
    // 1
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 4U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 5U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 6U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x05U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 7U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0xAAU);
    // 2
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 8U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 9U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 10U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x02U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 11U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, CMD_DDR, FLEXSPI_8PAD, 0x55U);
    // 3
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 12U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00U, RADDR_DDR, FLEXSPI_8PAD, 0x18U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 13U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CADDR_DDR, FLEXSPI_8PAD, 0x10U, CMD_DDR, FLEXSPI_8PAD, 0x00U);
    config->memConfig.lookupTable[4U * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR + 14U] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x30U, STOP, FLEXSPI_1PAD, 0x0U);

    // Erase Chip
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 1] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 2] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 3] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x80);
    // 1
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 4] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 5] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 6] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 7] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
    // 2
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 8] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 9] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 10] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x02);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 11] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x55);
    // 3
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 12] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x00);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 13] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0xAA);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 14] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x05);
    config->memConfig.lookupTable[4 * NOR_CMD_LUT_SEQ_IDX_CHIPERASE + 15] =
        FSL_ROM_FLEXSPI_LUT_SEQ(CMD_DDR, FLEXSPI_8PAD, 0x00, CMD_DDR, FLEXSPI_8PAD, 0x10);

    // LUT customized sequence
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_WRITEENABLE].seqNum = 2U;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_WRITEENABLE].seqId  = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_READSTATUS].seqNum  = 2U;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_READSTATUS].seqId   = NOR_CMD_LUT_SEQ_IDX_READSTATUS;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqNum = 2U;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_PAGEPROGRAM].seqId  = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqNum = 4U;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_ERASESECTOR].seqId  = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqNum   = 4U;
    config->memConfig.lutCustomSeq[NOR_CMD_INDEX_CHIPERASE].seqId    = NOR_CMD_LUT_SEQ_IDX_CHIPERASE;
}

void FLEXSPI_NorFlash_GetConfig_Norflash(flexspi_nor_config_t *config)
{
    config->memConfig.tag              = FLEXSPI_CFG_BLK_TAG;
    config->memConfig.version          = FLEXSPI_CFG_BLK_VERSION;
    config->memConfig.readSampleClkSrc = kFLEXSPIReadSampleClk_LoopbackFromDqsPad;//kFLEXSPIReadSampleClk_LoopbackFromDqsPad kFLEXSPIReadSampleClk_ExternalInputFromDqsPad
    config->memConfig.serialClkFreq =
        kFLEXSPISerialClk_30MHz; /* Serial Flash Frequencey.See System Boot Chapter for more details */
    config->memConfig.lutCustomSeqEnable = true;
    config->memConfig.sflashA1Size       = FLASH_SIZE;
    config->memConfig.csHoldTime         = 3U;                       /* Data hold time, default value: 3 */
    config->memConfig.csSetupTime        = 3U;                       /* Date setup time, default value: 3 */
    config->memConfig.deviceType     = kFLEXSPIDeviceType_SerialNOR; /* Flash device type default type: Serial NOR */
    config->memConfig.deviceModeType = kDeviceConfigCmdType_Generic;
    config->memConfig.columnAddressWidth  = 3U;
    config->memConfig.deviceModeCfgEnable = 0U;
    config->memConfig.waitTimeCfgCommands = 0U;
    config->memConfig.configCmdEnable     = 0U;
    config->memConfig.busyOffset          = 15U;
    config->memConfig.busyBitPolarity     = 1U;
    /* Always enable Safe configuration Frequency */
    config->memConfig.controllerMiscOption = FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_DiffClkEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_WordAddressableEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_SafeConfigFreqEnable) |
                                             FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_DdrModeEnable);
    config->memConfig.sflashPadType = kSerialFlash_8Pads; /* Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal */
    config->pageSize                = FLASH_PAGE_SIZE;
    config->sectorSize              = FLASH_SECTOR_SIZE;
    config->blockSize               = FLASH_BLOCK_SIZE;
    config->isUniformBlockSize      = true;
    config->ipcmdSerialClkFreq      = kFLEXSPISerialClk_133MHz; /* Clock frequency for IP command */
    config->serialNorType           = kSerialNorType_HyperBus;

	// Sequence 0 - Quad Read
	// 0x6B - Fast read Quad Output command, 0x18 - 24 bit address
	config->memConfig.lookupTable[0]  = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x6B,
						   RADDR_SDR, FLEXSPI_1PAD, 0x18);
	// 0x08 - 8 dummy clocks, 0x80 - read 128 bytes
	config->memConfig.lookupTable[1]  = FSL_ROM_FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x08,
						   READ_SDR,  FLEXSPI_4PAD, 0x80);

	// Sequence 1 - Read Status Register
	// 0x05 - Read status register command, 0x4 - read 4 bytes
	config->memConfig.lookupTable[4]  = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x05,
						   READ_SDR,  FLEXSPI_1PAD, 0x04);

	// Sequence 2 - Write Status Register 2
	// 0x31 - Write status register 2 command, 0x1 - write 1 byte
	config->memConfig.lookupTable[8]  = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x31,
						   WRITE_SDR, FLEXSPI_1PAD, 0x01);

	// Sequence 3 - Write enable
	// 0x06 - Write enable command
	config->memConfig.lookupTable[12] = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x06,
						   STOP,      0x00,          0x00);
	//[16] - Seq 4 empty

	// Sequence 5 - 4K Sector erase
	// 0x20 - Sector erase command, 0x18 - 24 bit address
	config->memConfig.lookupTable[20] = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x20,
						   RADDR_SDR, FLEXSPI_1PAD, 0x18);
	//[24] - Seq 6 empty
	//[28] - Seq 7 empty
	//[32] - Seq 8 empty

	// Sequence 9 - Page Program, 256 bytes
	// 0x02 - page program command, 0x18 - 24 bit address
	config->memConfig.lookupTable[36] = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x02,
						   RADDR_SDR, FLEXSPI_1PAD, 0x18),
	// 0x04 - write 4 bytes
	config->memConfig.lookupTable[37] = FSL_ROM_FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04,
						   STOP,      0x00,          0x00);
	//[40] - Seq 10 empty
	//[44] - Seq 11 empty

	// Sequence 12 - Read JEDEC
	// 0x9F - read JEDEC command, 0x04 - read 4 bytes
	config->memConfig.lookupTable[48] = FSL_ROM_FLEXSPI_LUT_SEQ(CMD_SDR,       FLEXSPI_1PAD, 0x9F,
						   READ_SDR,  FLEXSPI_1PAD, 0x04);
	//[52-60] - Seqs 13 - 15 empty
}

#define FlexSpiInstance           0U
flexspi_nor_config_t norConfig;

uint32_t serialNorAddress;        /* Address of the serial nor device location */
uint32_t FlexSPISerialNorAddress; /* Address of the serial nor device in FLEXSPI memory */
uint32_t serialNorTotalSize;
uint32_t serialNorSectorSize;
uint32_t serialNorPageSize;
#define SECTOR_INDEX_FROM_END 1U
static uint8_t s_buffer[FLASH_PAGE_SIZE];
static uint8_t s_buffer_rbc[FLASH_PAGE_SIZE];
void error_trap(void)
{
//    PRINTF("\r\n\r\n\r\n\t---- HALTED DUE TO FLEXSPI NOR ERROR! ----");
    while (1)
    {
    }
}
void flash_test()
{
	status_t status;
    /* Clean up FLEXSPI NOR flash driver Structure */
    memset(&norConfig, 0, sizeof(flexspi_nor_config_t));

    /* Setup FLEXSPI NOR Configuration Block */
    FLEXSPI_NorFlash_GetConfig_Hyperflash(&norConfig);

    /* Initializes the FLEXSPI module for the other FLEXSPI APIs */
    status = ROM_FLEXSPI_NorFlash_Init(FlexSpiInstance, &norConfig);
    if (status == kStatus_Success)
    {
//        PRINTF("\r\n Successfully init FLEXSPI serial NOR flash\r\n ");
    }
    else
    {
//        PRINTF("\r\n Erase sector failure !\r\n");
        error_trap();
    }

    /* Perform software reset after initializing flexspi module */
    ROM_FLEXSPI_NorFlash_ClearCache(FlexSpiInstance);

//#if !defined(XIP_EXTERNAL_FLASH) || (XIP_EXTERNAL_FLASH != 1)
//    /* Read ID-CFI Parameters */
//    status = FLEXSPI_NorFlash_VerifyID(FlexSpiInstance);
//    if (status == kStatus_Success)
//    {
//        PRINTF("\r\n HyperFlash has been found successfully\r\n ");
//    }
//    else
//    {
//        PRINTF("\r\n HyperFlash can not be found!\r\n");
//        error_trap();
//    }
//#endif // XIP_EXTERNAL_FLASH

//    serialNorTotalSize  = norConfig.memConfig.sflashA1Size;
//    serialNorSectorSize = norConfig.sectorSize;
//    serialNorPageSize   = norConfig.pageSize;

//    /* Print HyperFlash information */
//    PRINTF("\r\n HyperFlash Information: ");
//    PRINTF("\r\n Total program flash size:\t%d KB, Hex: (0x%x)", (serialNorTotalSize / 1024U), serialNorTotalSize);
//    PRINTF("\r\n Program flash sector size:\t%d KB, Hex: (0x%x) ", (serialNorSectorSize / 1024U), serialNorSectorSize);
//    PRINTF("\r\n Program flash page size:\t%d B, Hex: (0x%x)\r\n", serialNorPageSize, serialNorPageSize);

///*
// * SECTOR_INDEX_FROM_END = 1 means the last sector,
// * SECTOR_INDEX_FROM_END = 2 means (the last sector - 1) ...
// */
//#ifndef SECTOR_INDEX_FROM_END
//#define SECTOR_INDEX_FROM_END 1U
//#endif
//    /* Erase a sector from target device dest address */
//    serialNorAddress        = 0x800000;//serialNorTotalSize - (SECTOR_INDEX_FROM_END * serialNorSectorSize);
//    FlexSPISerialNorAddress = EXAMPLE_FLEXSPI_AMBA_BASE + serialNorAddress;

//    /* Erase one sector. */
//    PRINTF("\r\n Erasing serial NOR flash over FLEXSPI");
//    status = ROM_FLEXSPI_NorFlash_Erase(FlexSpiInstance, &norConfig, serialNorAddress, serialNorSectorSize);
//    if (status == kStatus_Success)
//    {
//        /* Print message for user. */
//        PRINTF("\r\n Successfully erased one sector of NOR flash device 0x%x -> 0x%x\r\n", serialNorAddress,
//               (serialNorAddress + serialNorSectorSize));
//    }
//    else
//    {
//        PRINTF("\r\n Erase sector failure!\r\n");
//        error_trap();
//    }

//    PRINTF("\r\n Program a buffer to a page of NOR flash");
//    /* Prepare user buffer. */
//    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++)
//    {
//        s_buffer[i] = i;
//    }

//    /* Program user buffer into FLEXSPI NOR flash */
//    status = ROM_FLEXSPI_NorFlash_ProgramPage(FlexSpiInstance, &norConfig, serialNorAddress, (const uint32_t *)s_buffer);
//    if (status != kStatus_Success)
//    {
//        PRINTF("\r\n Page program failure!\r\n");
//        error_trap();
//    }
//	
//    DCACHE_InvalidateByRange(FlexSPISerialNorAddress, sizeof(s_buffer_rbc));
//    /* Verify programming by reading back from FLEXSPI memory directly */
//    memcpy(s_buffer_rbc, (void *)(FlexSPISerialNorAddress), sizeof(s_buffer_rbc));
//    if (memcmp(s_buffer_rbc, s_buffer, sizeof(s_buffer)) == 0)
//    {
//        PRINTF("\r\n Successfully programmed and verified location FLEXSPI memory 0x%x -> 0x%x \r\n",
//               (FlexSPISerialNorAddress), (FlexSPISerialNorAddress + sizeof(s_buffer)));
//    }
//    else
//    {
//        PRINTF("\r\n Program data -  read out data value incorrect!\r\n ");
//        error_trap();
//    }

//	
//	/* Erase one sector. */
//    PRINTF("\r\n Erasing serial NOR flash over FLEXSPI");
//    status = ROM_FLEXSPI_NorFlash_Erase(FlexSpiInstance, &norConfig, serialNorAddress, serialNorSectorSize);
//    if (status == kStatus_Success)
//    {
//        /* Print message for user. */
//        PRINTF("\r\n Successfully erased one sector of NOR flash device 0x%x -> 0x%x\r\n", serialNorAddress,
//               (serialNorAddress + serialNorSectorSize));
//    }
//    else
//    {
//        PRINTF("\r\n Erase sector failure!\r\n");
//        error_trap();
//    }
//	DCACHE_InvalidateByRange(FlexSPISerialNorAddress, sizeof(s_buffer_rbc));
}

/*!
 * @brief Main function
 */
int main(void)
{
	status_t status;
    char ch;
	gpio_pin_config_t USER_LED_config = 
	{
		.direction = kGPIO_DigitalOutput,
		.outputLogic = 0U,
		.interruptMode = kGPIO_NoIntmode
	};
	GPIO_PinInit(GPIO1, 9U, &USER_LED_config);
	
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
//    BOARD_InitDebugConsole();
	flash_test();
	UART_ModeConfig();
	/* Set systick reload value to generate 1ms interrupt */
	if (SysTick_Config(SystemCoreClock / 1000U))
	{
		while (1)
		{
		}
	}
	SDK_DelayAtLeastUs(100000,BOARD_BOOTCLOCKRUN_CORE_CLOCK);
	grbl_enter();
	while(1)
	{
		
	}
}
