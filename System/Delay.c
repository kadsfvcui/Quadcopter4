#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"

RCC_ClocksTypeDef Clocks;

void Delay_us(uint16_t xus)
{
	RCC_GetClocksFreq(&Clocks);
	
	SysTick->LOAD = xus * (Clocks.HCLK_Frequency / 1000000);		//������װ��ֵ	
	SysTick->VAL = 0x00;		//VAL����
	SysTick->CTRL |= 0x00000005;		//HCLKʱ��1��Ƶ������SysTick
	while(!(SysTick->CTRL & (1<<16)));		//�ȴ�VAL����COUNTFLAGλ��1
	SysTick->CTRL = 0x00000004;		//�ر�SysTick
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
