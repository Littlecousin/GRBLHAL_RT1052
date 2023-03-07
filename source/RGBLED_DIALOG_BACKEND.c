#include "RGBLED_DIALOG_BACKEND.h"
#include "fsl_debug_console.h"

/**
* @brief  初始化 PWM 使用到的外部引脚  
* @retval 无
*/
void PWM_gpio_config(void)
{
  gpio_pin_config_t PWM_pin_config;
  
   /*设置外部引脚的复用功能*/
  IOMUXC_SetPinMux(PWM4_PWMA00_IOMUXC, 0U);                                      
  IOMUXC_SetPinMux(PWM4_PWMA01_IOMUXC, 0U);                                      
  IOMUXC_SetPinMux(PWM1_PWMA03_IOMUXC, 0U); 
  
  /*设置引脚的 pad 属性 */
  IOMUXC_SetPinConfig(PWM4_PWMA00_IOMUXC, PWM_PAD_CONFIG_DATA);
  IOMUXC_SetPinConfig(PWM4_PWMA01_IOMUXC, PWM_PAD_CONFIG_DATA);
  IOMUXC_SetPinConfig(PWM1_PWMA03_IOMUXC, PWM_PAD_CONFIG_DATA);
  
  /*配置引脚为输出模式，默认电平位高电平*/
  PWM_pin_config.direction = kGPIO_DigitalOutput;
  PWM_pin_config.outputLogic = 1;
  PWM_pin_config.interruptMode = kGPIO_NoIntmode;
   
  GPIO_PinInit(PWM4_PWMA00_GPIO, PWM4_PWMA00_GPIO_PIN, &PWM_pin_config);
  GPIO_PinInit(PWM4_PWMA01_GPIO, PWM4_PWMA01_GPIO_PIN, &PWM_pin_config);
  GPIO_PinInit(PWM1_PWMA03_GPIO, PWM1_PWMA03_GPIO_PIN, &PWM_pin_config);
}

void PWM_config(void)
{
  pwm_config_t pwmConfig;
  
  /*设置时钟*/
//  CLOCK_SetDiv(kCLOCK_AhbDiv, 0x2); /* Set AHB PODF to 2, divide by 3 */
  CLOCK_SetDiv(kCLOCK_IpgDiv, 0x3); /* Set IPG PODF to 3, divede by 4 */
  
  
   /* Set the PWM Fault inputs to a low value */
   XBARA_Init(XBARA1);
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm4Fault0);
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm4Fault1);  
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault0);
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault1);
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1234Fault2);
   XBARA_SetSignalsConnection(XBARA1, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1234Fault3); 
     
   PWM_GetDefaultConfig(&pwmConfig);
   /* Use full cycle reload */
   pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle; 
   /* PWM A & PWM B form a complementary PWM pair */
   pwmConfig.pairOperation = kPWM_Independent;   
   pwmConfig.enableDebugMode = true;
   pwmConfig.prescale = kPWM_Prescale_Divide_128;
   
   /* Initialize submodule 0 */
    if (PWM_Init(PWM4, kPWM_Module_0, &pwmConfig) == kStatus_Fail)
    {
        PRINTF("PWM initialization failed\n");
        //return 1;
    }

    /* Initialize submodule 1 */
    if (PWM_Init(PWM4, kPWM_Module_1, &pwmConfig) == kStatus_Fail)
    {
        PRINTF("PWM initialization failed\n");
        //return 1;
    }
     if (PWM_Init(PWM1, kPWM_Module_3, &pwmConfig) == kStatus_Fail)
    {
        PRINTF("PWM initialization failed\n");
        //return 1;
    }   
   /* Call the init function with demo configuration */
    PWM_DRV_Init3PhPwm();

    /* Set the load okay bit for all submodules to load registers from their buffer */
    PWM_SetPwmLdok(PWM4, kPWM_Control_Module_0 | kPWM_Control_Module_1, true);
    PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, true);
    /* Start the PWM generation from Submodules 0, 1 and 2 */
    PWM_StartTimer(PWM4, kPWM_Control_Module_0 | kPWM_Control_Module_1);
    PWM_StartTimer(PWM1, kPWM_Control_Module_3);
 
}

/*PWM 初始化函数*/
static void PWM_DRV_Init3PhPwm(void)
{
    pwm_signal_param_t pwmSignal[1];
    uint32_t pwmSourceClockInHz;
    uint32_t pwmFrequencyInHz = 8000;

    pwmSourceClockInHz = PWM_SRC_CLK_FREQ;

    
    pwmSignal[0].pwmChannel = kPWM_PwmA;
    pwmSignal[0].level = kPWM_LowTrue;
    pwmSignal[0].dutyCyclePercent = 0; /* 1 percent dutycycle */    
    pwmSignal[0].deadtimeValue = 0;

    PWM_SetupPwm(PWM4, kPWM_Module_0, pwmSignal, 1, kPWM_SignedEdgeAligned, pwmFrequencyInHz,
                 pwmSourceClockInHz);

    PWM_SetupPwm(PWM4, kPWM_Module_1, pwmSignal, 1, kPWM_SignedEdgeAligned, pwmFrequencyInHz,
                 pwmSourceClockInHz);

    PWM_SetupPwm(PWM1, kPWM_Module_3, pwmSignal, 1, kPWM_SignedEdgeAligned, pwmFrequencyInHz,
                 pwmSourceClockInHz);
}

void SetColorValue(uint8_t r, uint8_t g, uint8_t b)
{
  //R
  PWM_UpdatePwmDutycycle(PWM4,kPWM_Module_0, kPWM_PwmA, kPWM_SignedEdgeAligned, r*100/255); //更新占空比
  PWM_SetPwmLdok(PWM4, kPWM_Control_Module_0, true);    //更新有关设置
  //G
  PWM_UpdatePwmDutycycle(PWM4,kPWM_Module_1, kPWM_PwmA, kPWM_SignedEdgeAligned, g*100/255); 
  PWM_SetPwmLdok(PWM4, kPWM_Control_Module_1, true);    
  //B
  PWM_UpdatePwmDutycycle(PWM1,kPWM_Module_3, kPWM_PwmA, kPWM_SignedEdgeAligned, b*100/255); 
  PWM_SetPwmLdok(PWM1, kPWM_Control_Module_3, true);    
}

void FlexPWM_Init(void)
{
  PWM_gpio_config();
  PWM_config();
}

void FlexPWM_DeInit(void)
{
  PWM_Deinit(PWM4, kPWM_Module_0);
  PWM_Deinit(PWM4, kPWM_Module_1);
  PWM_Deinit(PWM1, kPWM_Module_3);
}
