#include "bsp_nvic.h"
#include "bsp_tmr.h"


volatile uint32_t qtmrIsrFlag = 0;



#define TMR_IRQn_TBALE	{ TMR1_IRQn, TMR2_IRQn, TMR3_IRQn, TMR4_IRQn}

void TMR_Init(TMR_Type *base, qtmr_channel_selection_t channel,uint32_t Time)
{
	qtmr_config_t qtmrConfig; /*定义TMR 定时器初始化结构体*/

	/*初始化TMR 定时器*/
	QTMR_GetDefaultConfig(&qtmrConfig);
	qtmrConfig.primarySource = kQTMR_ClockDivide_128;
	QTMR_Init(base, channel, &qtmrConfig);

	/*设置自动重装载值*/
	QTMR_SetTimerPeriod(base, channel, MSEC_TO_COUNT(Time, (QTMR_SOURCE_CLOCK / 128)));

	/*使能比较中断*/
	QTMR_EnableInterrupts(base, channel, kQTMR_Compare1InterruptEnable);//kQTMR_Compare1Flag kQTMR_CompareInterruptEnable

	/*设置中断优先级,*/
	set_IRQn_Priority(QTMR_IRQ_ID,Group4_PreemptPriority_6, Group4_SubPriority_0);
	/*使能中断*/
	EnableIRQ(QTMR_IRQ_ID);
}

//	QTMR_StartTimer(base, channel, kQTMR_PriSrcRiseEdge);
void TMRn_Init(tmr_config_t *tmr_confog)
{
	qtmr_config_t qtmrConfig; /*定义TMR 定时器初始化结构体*/
	/*初始化TMR 定时器*/
	QTMR_GetDefaultConfig(&qtmrConfig);
	qtmrConfig.primarySource = tmr_confog->primarySource;
	QTMR_Init(tmr_confog->base, tmr_confog->Channel, &qtmrConfig);
	/*设置自动重装载值*/
	QTMR_SetTimerPeriod(tmr_confog->base, tmr_confog->Channel, tmr_confog->LoadTime);
	/*使能比较中断*/
	QTMR_EnableInterrupts(tmr_confog->base, tmr_confog->Channel, tmr_confog->InterruptMask);//kQTMR_Compare1Flag kQTMR_CompareInterruptEnable
	/*设置中断优先级,*/
	set_IRQn_Priority(QTMR_IRQ_ID,Group4_PreemptPriority_6, Group4_SubPriority_0);
	/*使能中断*/
	EnableIRQ(tmr_confog->InterruptType);
}

///*TMR定时器中断服务函数*/
//void QTMR_IRQ_HANDLER(void)
//{
//	if(QTMR_GetStatus(BOARD_QTMR_BASEADDR, BOARD_SECOND_QTMR_CHANNEL))
//	{
//		/* 清除中断标志位*/
//		QTMR_ClearStatusFlags(BOARD_QTMR_BASEADDR,\
//			BOARD_SECOND_QTMR_CHANNEL, kQTMR_CompareFlag);
//		/*设置标志位*/
//		qtmrIsrFlag++;
//	}
//  
//}

