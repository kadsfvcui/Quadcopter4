#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "Delay.h"
#include "PPM.h"

uint16_t PPM_Time = 0;
uint8_t PPM_CNT = 0;

void PPM_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TimeBase_InitStructure;
	TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBase_InitStructure.TIM_Prescaler = 84 - 1;		//PSC
	TimeBase_InitStructure.TIM_Period = 65535;		//ARR
	TimeBase_InitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TimeBase_InitStructure);
	
	TIM_ICInitTypeDef IC_InitStructure;
	IC_InitStructure.TIM_Channel = TIM_Channel_1;	//选择CH1
	IC_InitStructure.TIM_ICFilter = 0xF;	//滤波
	IC_InitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿触发
	IC_InitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;		//捕获单元预分频
	IC_InitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	//直连
	TIM_ICInit(TIM3, &IC_InitStructure);
	
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);	//触发源选择TI1FP1
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);		//从模式选择复位计数器
	
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);		//开启TIM3输入捕获中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStrcuture;
	NVIC_InitStrcuture.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStrcuture.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStrcuture.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStrcuture.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStrcuture);
	
	TIM_Cmd(TIM3, ENABLE);		//开启定时器
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		PPM_Time = TIM3->CCR1;
		if(PPM_Time <= 2100)
		{
			PPM[PPM_CNT++] = PPM_Time;
		}
		else 
		{
			PPM_CNT = 0;
		}			
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
}
