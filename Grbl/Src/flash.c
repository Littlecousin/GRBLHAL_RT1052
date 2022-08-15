/*

  flash.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

  This code reads/writes the whole RAM-based emulated EPROM contents from/to flash

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>

#include "main.h"
#include "grbl/hal.h"

#define FLASH_START_ADDR 0x60400000
void* _EEPROM_Emul_Start = (uint32_t *)FLASH_START_ADDR;
extern uint8_t _EEPROM_Emul_Sector;
extern flexspi_nor_config_t norConfig;
bool memcpy_from_flash (uint8_t *dest)
{
	void *ptr = dest;
	if(dest)
	{
		DCACHE_InvalidateByRange(FLASH_START_ADDR, 2048);
		memcpy(ptr, _EEPROM_Emul_Start, hal.nvs.size);
		return true;
	}
	else
		return false;
}

bool memcpy_to_flash (uint8_t *source)
{
	status_t status;
    status = ROM_FLEXSPI_NorFlash_Erase(0, &norConfig, 0x400000, 0x40000UL);//0x40000UL hal.nvs.size
    if (status == kStatus_Success)
    {
		
    }
    else
    {
		while(1)
		{
			
		}
    }
	DCACHE_InvalidateByRange(FLASH_START_ADDR, 2048);
	for(uint8_t i= 0;i<NVS_SIZE/512;i++)
	{
		status = ROM_FLEXSPI_NorFlash_ProgramPage(0, &norConfig, 0x400000+512*i, (const uint32_t *)(source+512*i));//Ð´ÈëÒ»Ò³
		if (status != kStatus_Success)
		{
			while(1)
			{
				
			}
		}
	}
	DCACHE_InvalidateByRange(FLASH_START_ADDR, 2048);
    return true;//status == HAL_OK;
}
