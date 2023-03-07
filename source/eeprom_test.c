#include "fsl_debug_console.h"
#include "bsp_led.h"
#include "bsp_i2c_eeprom.h"

/*******************************************************************
 * Definition
 *******************************************************************/
#define EEPROM_TEST_NUM           200
#define EEPORM_TEST_START_ADDR    1


/*******************************************************************
 * Variables
 *******************************************************************/
uint8_t EEPROM_Buffer_Write[300];
uint8_t EEPROM_Buffer_Read[300];


/**
  * @brief  I2C(AT24C02)读写测试
  * @param  无
  * @retval 正常返回1 ，不正常返回0
  */
uint8_t EEPROM_Test(void)
{
  uint16_t i;

  EEPROM_INFO("写入的数据");
    
  for ( i=0; i<EEPROM_TEST_NUM; i++ ) //填充缓冲
  {   
    EEPROM_Buffer_Write[i] = i%4;

    PRINTF("0x%02X ", EEPROM_Buffer_Write[i]);
    if((i+1)%10 == 0 || i == (EEPROM_TEST_NUM-1))    
        PRINTF("\r\n");    
   }
  
  //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
  I2C_EEPROM_Buffer_Write(EEPROM_WRITE_ADDRESS_8_BIT,
                             EEPORM_TEST_START_ADDR,
                             EEPROM_Buffer_Write,
                             EEPROM_TEST_NUM);
  
  EEPROM_INFO("写成功");
   
  EEPROM_INFO("读出的数据");
  //将EEPROM读出数据顺序保持到I2c_Buf_Read中
  I2C_EEPROM_Buffer_Read(EEPROM_READ_ADDRESS_8_BIT,
                           EEPORM_TEST_START_ADDR,
                           EEPROM_Buffer_Read,
                           EEPROM_TEST_NUM); 
   
  //将I2c_Buf_Read中的数据通过串口打印
  for (i=0; i<EEPROM_TEST_NUM; i++)
  { 
    if(EEPROM_Buffer_Read[i] != EEPROM_Buffer_Write[i])
    {
      PRINTF("0x%02X ", EEPROM_Buffer_Read[i]);
      RGB_LED_COLOR_RED;
      EEPROM_ERROR("错误:I2C EEPROM写入与读出的数据不一致");
      return 0;
    }
    PRINTF("0x%02X ", EEPROM_Buffer_Read[i]);
    if((i+1)%10 == 0 || i == (EEPROM_TEST_NUM-1))    
        PRINTF("\r\n");
    
  }
  
  CORE_BOARD_LED_ON;
  RGB_LED_COLOR_GREEN;
  EEPROM_INFO("I2C(AT24C02)读写测试成功");
  return 1;
}


/*********************************************END OF FILE**********************/
