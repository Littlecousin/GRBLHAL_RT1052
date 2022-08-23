#ifndef __BSP_TMR_H
#define __BSP_TMR_H

#include "fsl_common.h"
#include "fsl_qtmr.h"

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


/*定义定时时间（单位：ms）*
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
#endif /* __BSP_TMR_H */


