#include "bsp_nvic.h"
#include "bsp_tmr.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"

volatile uint32_t qtmrIsrFlag = 0;



#define TMR_IRQn_TBALE	{ TMR1_IRQn, TMR2_IRQn, TMR3_IRQn, TMR4_IRQn}

//void TMR_Init(TMR_Type *base, qtmr_channel_selection_t channel,uint32_t Time)
//{
//	qtmr_config_t qtmrConfig; /*定义TMR 定时器初始化结构体*/

//	/*初始化TMR 定时器*/
//	QTMR_GetDefaultConfig(&qtmrConfig);
//	qtmrConfig.primarySource = kQTMR_ClockDivide_128;
//	QTMR_Init(base, channel, &qtmrConfig);

//	/*设置自动重装载值*/
//	QTMR_SetTimerPeriod(base, channel, MSEC_TO_COUNT(Time, (QTMR_SOURCE_CLOCK / 128)));

//	/*使能比较中断*/
//	QTMR_EnableInterrupts(base, channel, kQTMR_Compare1InterruptEnable);//kQTMR_Compare1Flag kQTMR_CompareInterruptEnable

//	/*设置中断优先级,*/
//	set_IRQn_Priority(QTMR_IRQ_ID,Group4_PreemptPriority_6, Group4_SubPriority_0);
//	/*使能中断*/
//	EnableIRQ(QTMR_IRQ_ID);
//}

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
	set_IRQn_Priority(tmr_confog->InterruptType,Group4_PreemptPriority_6, Group4_SubPriority_0);
	/*使能中断*/
	EnableIRQ(tmr_confog->InterruptType);
}

void TMRn_PWM_Init()
{
	/*定义GPIO引脚配置结构体*/
	gpio_pin_config_t gpt_config; 
	
	qtmr_config_t qtmrConfig; /*定义TMR 定时器初始化结构体*/
	/*配置初始化结构体*/
	gpt_config.direction = kGPIO_DigitalOutput; //输出模式
	gpt_config.outputLogic =  1;                //默认高电平
	gpt_config.interruptMode = kGPIO_NoIntmode; //不使用中断
	/*初始化QTMR定时器CH0 外部引脚*/
	IOMUXC_SetPinMux(QTMR_CH0_IOMUXC, 0U);
	IOMUXC_SetPinConfig(QTMR_CH0_IOMUXC, TMR_PWM_OUTPUT_PAD_CONFIG_DATA);
	GPIO_PinInit(QTMR_CH0_GPIO, QTMR_CH0_GPIO_PIN, &gpt_config);
	
	/*初始化TMR 定时器*/
	QTMR_GetDefaultConfig(&qtmrConfig);
	qtmrConfig.primarySource = kQTMR_ClockDivide_2;
	QTMR_Init(QTMR_BASEADDR, QTMR_PWM_CHANNEL_0,  &qtmrConfig);
	/* 创建CH0的PWM输出，并且指定PWM的频率与占空比*/
	QTMR_SetupPwm(QTMR_BASEADDR, QTMR_PWM_CHANNEL_0, TMR1_CH0_PWM_FREQUENCY, TMR1_CH0_PWM_DUTYCYCLE, false, QTMR_SOURCE_CLOCK / 8);
//	QTMR_StartTimer(QTMR_BASEADDR, QTMR_PWM_CHANNEL_0, kQTMR_PriSrcRiseEdge);
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

#include "bsp_nvic.h"
#include "bsp_tmr.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"																			

uint32_t  IpFreq;
void PlsueCh0Init(void)
{
   gpio_pin_config_t gpt_config; 
   gpt_config.direction = kGPIO_DigitalOutput;
   gpt_config.outputLogic =  1;               
   gpt_config.interruptMode = kGPIO_NoIntmode; 
   IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_QTIMER3_TIMER0, 0U);
   IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_QTIMER3_TIMER0, 0xB0B0);
   GPIO_PinInit(GPIO2, 6, &gpt_config);
}

void PlsueCh1Init(void)
{
	gpio_pin_config_t gpt_config; 
	gpt_config.direction = kGPIO_DigitalOutput; 
	gpt_config.outputLogic =  1;                
	gpt_config.interruptMode = kGPIO_NoIntmode; 
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_QTIMER3_TIMER3, 0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_03_QTIMER3_TIMER3, 0xB0B0);
	GPIO_PinInit(GPIO1, 19, &gpt_config);
}

void PlsueCh2Init(void)
{
	gpio_pin_config_t gpt_config; 
	gpt_config.direction = kGPIO_DigitalOutput; 
	gpt_config.outputLogic =  1;                
	gpt_config.interruptMode = kGPIO_NoIntmode; 
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_00_QTIMER1_TIMER0, 0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_00_QTIMER1_TIMER0, 0xB0B0);
	GPIO_PinInit(GPIO2, 0, &gpt_config);
}

void PlsueCh3Init(void)
{
	gpio_pin_config_t gpt_config; 
	gpt_config.direction = kGPIO_DigitalOutput; 
	gpt_config.outputLogic =  1;               
	gpt_config.interruptMode = kGPIO_NoIntmode; 
	IOMUXC_SetPinMux(IOMUXC_GPIO_B1_08_QTIMER1_TIMER3, 0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B1_08_QTIMER1_TIMER3, 0xB0B0);
	GPIO_PinInit(GPIO2, 24, &gpt_config);
}

void PulseChxFreq(TMR_Type *TIMx,uint32_t Plusefreq,uint8_t TimChanel)
{
	 TIMx->CHANNEL[TimChanel].CTRL&=~TMR_CTRL_CM_MASK; 
	 if(Plusefreq<10000)
	 {
		TIMx->CHANNEL[TimChanel].CTRL=0x1E23; 
		TIMx->CHANNEL[TimChanel].COMP1=IpFreq/128/(Plusefreq);
	 }
	 else
	 {
		TIMx->CHANNEL[TimChanel].CTRL=0x1023; 
		TIMx->CHANNEL[TimChanel].COMP1=IpFreq/1/(Plusefreq);	 
	 }
	 TIMx->CHANNEL[TimChanel].SCTRL=0x00;
	 TIMx->CHANNEL[TimChanel].LOAD=0x00; 
	 TIMx->CHANNEL[TimChanel].CSCTRL=0x00; 
	 TIMx->CHANNEL[TimChanel].CNTR=0; 
	 TIMx->CHANNEL[TimChanel].CTRL|=(1<<13); 
}

void PulseChxStream_Init(TMR_Type *TIMx,IRQn_Type IRQn,uint32_t Plusefreq,uint16_t PluseData,uint8_t TimChanel,uint8_t OutChanel,uint16_t OutMode)
{
	TIMx->CHANNEL[OutChanel].CTRL&=~TMR_CTRL_CM_MASK; 
	TIMx->CHANNEL[TimChanel].CTRL&=~TMR_CTRL_CM_MASK; 
	if(Plusefreq<10000)
	{
		TIMx->CHANNEL[TimChanel].CTRL=0x1E23; 
		TIMx->CHANNEL[TimChanel].COMP1=IpFreq/128/(Plusefreq);
	}
	else
	{
		TIMx->CHANNEL[TimChanel].CTRL=0x1023; 
		TIMx->CHANNEL[TimChanel].COMP1=IpFreq/1/(Plusefreq);	 
	}
	TIMx->CHANNEL[TimChanel].SCTRL=0x00;
	TIMx->CHANNEL[TimChanel].LOAD=0x00; 
	/* TMR3_CSCTRL: DBG_EN=0,FAULT=0,ALT_LOAD=0,ROC=0,TCI=0,UP=0,OFLAG=0,
	TCF2EN=0,TCF1EN=0,TCF2=0,TCF1=0,CL2=0,CL1=0 */
	TIMx->CHANNEL[TimChanel].CSCTRL=0x00; 
	/* TMR1_CTRL: CM=0,PCS=7,SCS=0,ONCE=1,LENGTH=1,DIR=0,COINIT=0,OUTMODE=7 */
	TIMx->CHANNEL[OutChanel].CTRL=OutMode; 
	/* TMR1_SCTRL: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,
	Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=1 */
	TIMx->CHANNEL[OutChanel].SCTRL=0x0001;
	TIMx->CHANNEL[OutChanel].CNTR=0x00; 
	TIMx->CHANNEL[OutChanel].LOAD=0x00; 
	TIMx->CHANNEL[OutChanel].COMP1=PluseData; 
	/* TMR1_CSCTRL: DBG_EN=0,FAULT=0,ALT_LOAD=0,ROC=0,TCI=0,UP=0,OFLAG=0,
	TCF2EN=0,TCF1EN=1,TCF2=0,TCF1=0,CL2=0,CL1=0 */
	TIMx->CHANNEL[OutChanel].CSCTRL=0x40; 
	TIMx->CHANNEL[TimChanel].CNTR=0; 
	//   set_IRQn_Priority(IRQn,Group4_PreemptPriority_6, Group4_SubPriority_0);
	//   EnableIRQ(IRQn); 
	TIMx->CHANNEL[TimChanel].CTRL|=(1<<13); 
	TIMx->CHANNEL[OutChanel].CTRL|=(1<<13); 
} 

void TMR_Pulse_Init()
{
	uint32_t i = 0;
	qtmr_config_t qtmrConfig;
    gpio_pin_config_t gpt_config;
    gpt_config.direction = kGPIO_DigitalOutput;
    gpt_config.outputLogic =  0;
    gpt_config.interruptMode = kGPIO_NoIntmode;
	GPIO_PinInit(GPIO1, 16, &gpt_config);
	QTMR_GetDefaultConfig(&qtmrConfig);
	IpFreq=CLOCK_GetFreq(kCLOCK_IpgClk)/2;
    /* Initial the input channel. */
    qtmrConfig.primarySource = kQTMR_ClockCounter0Output;
    QTMR_Init(TMR3, kQTMR_Channel_1, &qtmrConfig);
	PulseChxStream_Init(TMR3,TMR3_IRQn,200000,5,1,0,0x0A67);
    for(i=0;i<100000000;i++);

    PulseChxStream_Init(TMR3,TMR3_IRQn,200000,10,1,0,0x0A67);
    for(i=0;i<100000000;i++);

    PulseChxStream_Init(TMR3,TMR3_IRQn,200000,15,1,0,0x0A67);
    for(i=0;i<100000000;i++);

    PulseChxStream_Init(TMR3,TMR3_IRQn,200000,20,1,0,0x0A67);
}

void TMR3_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    QTMR_ClearStatusFlags(TMR3, kQTMR_Channel_1, kQTMR_CompareFlag);

    qtmrIsrFlag = true;
    SDK_ISR_EXIT_BARRIER;
}
