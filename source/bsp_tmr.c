#include "bsp_nvic.h"
#include "bsp_tmr.h"


volatile uint32_t qtmrIsrFlag = 0;


void TMR_Init(TMR_Type *base, qtmr_channel_selection_t channel)
{

  qtmr_config_t qtmrConfig; /*����TMR ��ʱ����ʼ���ṹ��*/
  
  /*��ʼ��TMR ��ʱ��*/
  QTMR_GetDefaultConfig(&qtmrConfig);
  qtmrConfig.primarySource = kQTMR_ClockDivide_128;
  QTMR_Init(base, channel, &qtmrConfig);
  
  /*�����Զ���װ��ֵ*/
  QTMR_SetTimerPeriod(base, channel, MSEC_TO_COUNT(TMR_TIMIER, (QTMR_SOURCE_CLOCK / 128)));


  /*ʹ�ܱȽ��ж�*/
  QTMR_EnableInterrupts(base, channel, kQTMR_CompareInterruptEnable);
    
  /*�����ж����ȼ�,*/
  set_IRQn_Priority(QTMR_IRQ_ID,Group4_PreemptPriority_6, Group4_SubPriority_0);
  /*ʹ���ж�*/
  EnableIRQ(QTMR_IRQ_ID);

  
  /*����ͨ��2�ļ�ʱ����ʱ�ӵ������ؼ���*/
  QTMR_StartTimer(base, channel, kQTMR_PriSrcRiseEdge);
}



/*TMR��ʱ���жϷ�����*/
void QTMR_IRQ_HANDLER(void)
{
	if(QTMR_GetStatus(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL))
	{
		/* ����жϱ�־λ*/
		QTMR_ClearStatusFlags(BOARD_QTMR_BASEADDR,\
			BOARD_SECOND_QTMR_CHANNEL, kQTMR_CompareFlag);
		/*���ñ�־λ*/
		qtmrIsrFlag++;
	}
  
}
