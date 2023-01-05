#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H
#include "fsl_romapi.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"

#ifdef __cplusplus
extern "C" {
#endif
	
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FlexSpiInstance           0U
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE
#define FLASH_SIZE                0x4000000UL /* 64MBytes */
#define FLASH_PAGE_SIZE           512UL       /* 512Bytes */
#define FLASH_SECTOR_SIZE         0x40000UL   /* 256KBytes */
#define FLASH_BLOCK_SIZE          0x40000UL   /* 256KBytes */

void flash_init();

#ifdef __cplusplus
}
#endif

#endif