#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"

// 初始化延时函数
void Delay_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM2_InitStructure;
	TIM2_InitStructure.TIM_Prescaler = 84 - 1;
	TIM2_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_InitStructure.TIM_Period = 0xFFFFFFFF;
	TIM2_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM2_InitStructure);
	// 禁用TIM2
	TIM_Cmd(TIM2, DISABLE);
	// 将TIM2计数器清零
	TIM2->CNT = 0;
}

// 延时函数，延时xus微秒
void Delay_us(uint16_t xus)
{
	// 使能定时器2
	TIM_Cmd(TIM2, ENABLE);
	// 循环等待定时器2计数器达到xus
	while(TIM2->CNT < xus)
	{
	    //空循环
	}
	// 禁用定时器2
	TIM_Cmd(TIM2, DISABLE);
	// 将定时器2计数器清零
	TIM2->CNT = 0;
}

void Delay_ms(uint16_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}

void Delay_s(uint16_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
}
