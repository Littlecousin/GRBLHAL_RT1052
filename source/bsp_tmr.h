#ifndef __BSP_TMR_H
#define __BSP_TMR_H

#include "fsl_common.h"
#include "fsl_qtmr.h"
#include "pad_config.h" 

///* ���� TMR ��ʱ����ͨ���͹���ģʽinstance/channel used for board */
//#define BOARD_QTMR_BASEADDR TMR3
//#define BOARD_FIRST_QTMR_CHANNEL kQTMR_Channel_0
//#define BOARD_SECOND_QTMR_CHANNEL kQTMR_Channel_1
//#define QTMR_ClockCounterOutput kQTMR_ClockCounter0Output

///* �жϺź��жϷ��������� */
//#define QTMR_IRQ_ID TMR3_IRQn
//#define QTMR_IRQ_HANDLER TMR3_IRQHandler

/* �õ�TMR��ʱ����ʱ��Ƶ�� */
#define QTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_IpgClk)

/* PWM������ŵ�PAD���� */
#define TMR_PWM_OUTPUT_PAD_CONFIG_DATA       (SRE_0_SLOW_SLEW_RATE| \
                                        DSE_6_R0_6| \
                                        SPEED_2_MEDIUM_100MHz| \
                                        ODE_0_OPEN_DRAIN_DISABLED| \
                                        PKE_1_PULL_KEEPER_ENABLED| \
                                        PUE_1_PULL_SELECTED| \
                                        PUS_2_100K_OHM_PULL_UP| \
                                        HYS_0_HYSTERESIS_DISABLED)   
    /* ����˵�� : */
    /* ת������: ת��������
      ����ǿ��: R0/6 
      �������� : medium(100MHz)
      ��©����: �ر� 
      ��/����������: ����
      ��/������ѡ��: ������
      ����/����ѡ��: 100K����
      �ͻ�������: �ر� */  

#define QTMR_CH0_GPIO		GPIO1
#define QTMR_CH0_GPIO_PIN	(16U)
#define QTMR_CH0_IOMUXC		IOMUXC_GPIO_AD_B1_00_QTIMER3_TIMER0
#define QTMR_BASEADDR		TMR3
#define QTMR_PWM_CHANNEL_0	kQTMR_Channel_0
/*�������pwmƵ�ʺ�ռ�ձ�*/
#define TMR1_CH0_PWM_FREQUENCY 50000  //5000 ��ʾƵ��Ϊ5000Hz,
#define TMR1_CH0_PWM_DUTYCYCLE 50      //5  ����ռ�ձ�Ϊ5%

/*
*���嶨ʱʱ�䣨��λ��ms��
*ע��:TMR��ʱ���ļ����Ĵ�����16λ��ʹ��kCLOCK_IpgClkʱ�ӣ�ѡ�����ʱ�ӷ�Ƶ��128��Ƶ��
*���ʵ��63ms�Ķ�ʱ��
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


