#ifndef _RGBLED_DIALOG_BACKEND_H_
#define _RGBLED_DIALOG_BACKEND_H_

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "pad_config.h"

#define PWM_SRC_CLK_FREQ CLOCK_GetFreq(kCLOCK_IpgClk)

#define PWM4_PWMA00_GPIO      GPIO1
#define PWM4_PWMA00_GPIO_PIN  (24U)
#define PWM4_PWMA00_IOMUXC    IOMUXC_GPIO_AD_B1_08_FLEXPWM4_PWMA00  

#define PWM4_PWMA01_GPIO       GPIO1
#define PWM4_PWMA01_GPIO_PIN   (25U)
#define PWM4_PWMA01_IOMUXC     IOMUXC_GPIO_AD_B1_09_FLEXPWM4_PWMA01   


#define PWM1_PWMA03_GPIO       GPIO1
#define PWM1_PWMA03_GPIO_PIN   (10U)
#define PWM1_PWMA03_IOMUXC     IOMUXC_GPIO_AD_B0_10_FLEXPWM1_PWMA03

#define PWM_PAD_CONFIG_DATA            (SRE_0_SLOW_SLEW_RATE| \
                                        DSE_6_R0_6| \
                                        SPEED_1_MEDIUM_100MHz| \
                                        ODE_0_OPEN_DRAIN_DISABLED| \
                                        PKE_1_PULL_KEEPER_ENABLED| \
                                        PUE_0_KEEPER_SELECTED| \
                                        PUS_0_100K_OHM_PULL_DOWN| \
                                        HYS_0_HYSTERESIS_DISABLED) 

static void PWM_DRV_Init3PhPwm(void);
void SetColorValue(uint8_t r, uint8_t g, uint8_t b);
void FlexPWM_Init(void);
void FlexPWM_DeInit(void);
#endif 
