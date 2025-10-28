#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include "usart.h"
#include "Delay.h"
#include <stdio.h>

void USART_GPIO_Init(void)
{ 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//开启GPIOB时钟
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//开启USART1时钟
	
	GPIOB->AFR[0] |= GPIO_AF_USART1<<24 | GPIO_AF_USART1<<28;		//复用USART1
	GPIOB->MODER |= GPIO_Mode_AF<<12 | GPIO_Mode_AF<<14;		//复用模式
	GPIOB->OTYPER |= GPIO_OType_PP<<6 | GPIO_OType_PP<<7;		//推挽输出
	GPIOB->OSPEEDR |= GPIO_High_Speed<<12 | GPIO_High_Speed<<14;	//速度
	GPIOB->PUPDR |= GPIO_PuPd_NOPULL<<12 | GPIO_PuPd_NOPULL<<14;	//浮空
}

void USART_Config(uint32_t bps)
{
	USART_GPIO_Init();
	
	USART1->CR1 &= ~(USART_CR1_UE);		//USART关闭
	USART1->CR1 &= ~USART_CR1_OVER8;		//16倍过采样
	USART1->CR1 &= ~USART_CR1_M;		//8位字长
	USART1->CR1 &= ~USART_CR1_PCE;		//禁止奇偶校验位
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;		//发送、接收使能
	
	USART1->CR2 &= ~USART_CR2_STOP;		//1个停止位
	
	float USARTDIV;
	static uint16_t DIV_M;			//整数部分
	uint16_t DIV_F;			//小数部分
	if((USART1->CR1 & USART_CR1_OVER8) == SET)			
	{
		USARTDIV = 84000000.0/8/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*8 + 0.5f;		//OVER8为1
	}
	else
	{
		USARTDIV = 84000000.0/16/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*16 + 0.5f;		//OVER8为0
	}
	USART1->BRR = DIV_M<<4 | DIV_F;
	
	USART1->CR1 |= USART_CR1_RXNEIE;		//开启USART中断
	
	SCB->AIRCR |= 5<<SCB_AIRCR_PRIGROUP_Pos;		//设置NVIC中断优先级分组
	NVIC->ISER[1] |= 1<<5;		//使能USART1到NVIC中断
	NVIC->IP[36] = 5;		//设置USART1中断响应优先级和抢占优先级
	
	USART1->CR1 |= USART_CR1_UE;		//开启USART
}

void USART_SendByte(uint8_t Byte)
{
	while(!(USART1->SR & USART_SR_TXE));		//等待上一次发送完成
	USART1->DR = Byte;
}

void USART_SendArray(int8_t *array, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        USART_SendByte(array[i]);
    }
	while(!(USART1->SR & USART_SR_TC));
}

int fputc(int ch, FILE *f)
{
    USART_SendByte(ch);
    return ch;
}

/**
 * @brief  发送数据到匿名上位机
 * @note   数据格式: 帧头(0xAA) + 帧头(0xFF) + 功能码 + 数据长度 + 数据 + 校验和 + 附加校验和
 * @param  data: 要发送的数据数组
 * @param  len:  数据长度
 * @param  id:   功能码/标识码
 * @retval None
 */
void Ano_SendData(int8_t *data, uint8_t len, uint8_t id)
{
    uint8_t sum_check = 0, add_check = 0;
    int8_t MyData[len + 6];
    MyData[0] = 0xaa;
    MyData[1] = 0xff;
    MyData[2] = id;
    MyData[3] = len;
    
	for(int i = 4; i < (len + 4); ++i){
        MyData[i] = data[i - 4];
    }

    for(int i = 0; i < (len + 4); ++i){
        sum_check += MyData[i];
        add_check += sum_check;
    }

    MyData[len + 4] = sum_check;
    MyData[len + 5] = add_check;
    
    USART_SendArray(MyData, len + 6);
} 

void USART_SendString(char *str)
{
	while(*str != '\0')
	{
		USART_SendByte(*str++);
	}
}

//uint8_t USART_ReceiveByte(void)
//{
//	uint8_t RxData;
//	while((USART1->SR & USART_SR_RXNE) == RESET);		//等待接收到数据
//	RxData = USART1->DR;
//	return RxData;
//}

void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_RXNE)		//DR寄存器不为空
	{
		RxData = USART1->DR;		//读取DR
		USART1->SR &= ~USART_SR_RXNE;		//清除RXNE标志位
	}		
}
