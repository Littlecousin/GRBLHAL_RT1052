#ifndef __BSP_TMR_H
#define __BSP_TMR_H

#include "fsl_common.h"
#include "fsl_qtmr.h"
#include "pad_config.h" 

///* 定义 TMR 定时器的通道和工作模式instance/channel used for board */
//#define BOARD_QTMR_BASEADDR TMR3
//#define BOARD_FIRST_QTMR_CHANNEL kQTMR_Channel_0
//#define BOARD_SECOND_QTMR_CHANNEL kQTMR_Channel_1
//#define QTMR_ClockCounterOutput kQTMR_ClockCounter0Output

///* 中断号和中断服务函数定义 */
//#define QTMR_IRQ_ID TMR3_IRQn
//#define QTMR_IRQ_HANDLER TMR3_IRQHandler

/* 得到TMR定时器的时钟频率 */
#define QTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_IpgClk)

/* PWM输出引脚的PAD配置 */
#define TMR_PWM_OUTPUT_PAD_CONFIG_DATA       (SRE_0_SLOW_SLEW_RATE| \
                                        DSE_6_R0_6| \
                                        SPEED_2_MEDIUM_100MHz| \
                                        ODE_0_OPEN_DRAIN_DISABLED| \
                                        PKE_1_PULL_KEEPER_ENABLED| \
                                        PUE_1_PULL_SELECTED| \
                                        PUS_2_100K_OHM_PULL_UP| \
                                        HYS_0_HYSTERESIS_DISABLED)   
    /* 配置说明 : */
    /* 转换速率: 转换速率慢
      驱动强度: R0/6 
      带宽配置 : medium(100MHz)
      开漏配置: 关闭 
      拉/保持器配置: 开启
      拉/保持器选择: 保持器
      上拉/下拉选择: 100K上拉
      滞回器配置: 关闭 */  

#define QTMR_CH0_GPIO		GPIO1
#define QTMR_CH0_GPIO_PIN	(16U)
#define QTMR_CH0_IOMUXC		IOMUXC_GPIO_AD_B1_00_QTIMER3_TIMER0
#define QTMR_BASEADDR		TMR3
#define QTMR_PWM_CHANNEL_0	kQTMR_Channel_0
/*定义输出pwm频率和占空比*/
#define TMR1_CH0_PWM_FREQUENCY 50000  //5000 表示频率为5000Hz,
#define TMR1_CH0_PWM_DUTYCYCLE 50      //5  代表占空比为5%

/*
*定义定时时间（单位：ms）
*注意:TMR定时器的计数寄存器是16位，使用kCLOCK_IpgClk时钟，选择最大时钟分频（128分频）
*最多实现63ms的定时。
*/
#define TMR_TIMIER 50

typedef struct _tmr_config_t
{
	TMR_Type *base;
	qtmr_channel_selection_t Channel;
	uint32_t LoadTime;
	uint32_t InterruptMask;
	qtmr_primary_count_source_t primarySource;
	uint8_t Div;
	IRQn_Type InterruptType;
}tmr_config_t;

void TMR_Init(TMR_Type *base, qtmr_channel_selection_t channel,uint32_t intTime);
void TMRn_Init(tmr_config_t *tmr_confog);
void TMRn_PWM_Init();

extern uint32_t  IpFreq;

void TMR_Pulse_Init();

void PlsueCh0Init(void);

void PlsueCh1Init(void);

void PlsueCh2Init(void);


void PlsueCh3Init(void);

void PulseChxFreq(TMR_Type *TIMx,uint32_t Plusefreq,uint8_t TimChanel);

void PulseChxStream_Init(TMR_Type *TIMx,IRQn_Type IRQn,uint32_t Plusefreq,uint16_t PluseData,uint8_t TimChanel,uint8_t OutChanel,uint16_t OutMode);


#endif /* __BSP_TMR_H */


