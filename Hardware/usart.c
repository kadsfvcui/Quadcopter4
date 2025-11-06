#include "usart.h"

uint8_t RxData;

// 初始化USART的GPIO
void USART_GPIO_Init(void)
{ 
	// 使能GPIOB的时钟
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		
	// 使能USART1的时钟
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		
	
	// 将PB6和PB7配置为USART1的复用功能
	GPIOB->AFR[0] |= GPIO_AF_USART1<<24 | GPIO_AF_USART1<<28;		
	// 将PB6和PB7配置为复用功能
	GPIOB->MODER |= GPIO_Mode_AF<<12 | GPIO_Mode_AF<<14;		
	// 将PB6和PB7配置为推挽输出
	GPIOB->OTYPER |= GPIO_OType_PP<<6 | GPIO_OType_PP<<7;		
	// 将PB6和PB7配置为高速
	GPIOB->OSPEEDR |= GPIO_High_Speed<<12 | GPIO_High_Speed<<14;	
	// 将PB6和PB7配置为上拉
	GPIOB->PUPDR |= GPIO_PuPd_NOPULL<<12 | GPIO_PuPd_NOPULL<<14;	
}

void USART_Config(uint16_t bps)
{
	// 初始化USART的GPIO口
	USART_GPIO_Init();
	
	// 关闭USART1
	USART1->CR1 &= ~(USART_CR1_UE);		
	// 关闭USART1的8倍过采样
	USART1->CR1 &= ~USART_CR1_OVER8;		
	// 关闭USART1的奇偶校验
	USART1->CR1 &= ~USART_CR1_M;		
	// 关闭USART1的奇偶校验使能
	USART1->CR1 &= ~USART_CR1_PCE;		
	// 使能USART1的发送和接收
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;		
	
	// 设置USART1的停止位
	USART1->CR2 &= ~USART_CR2_STOP;		
	
	// 计算USART1的波特率
	float USARTDIV;
	static uint16_t DIV_M;			
	uint16_t DIV_F;			
	if((USART1->CR1 & USART_CR1_OVER8) == SET)			
	{
		// 如果USART1的8倍过采样使能，则计算波特率
		USARTDIV = 84000000.0/8/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*8 + 0.5f;	
	}
	else
	{
		// 如果USART1的8倍过采样未使能，则计算波特率
		USARTDIV = 84000000.0/16/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*16 + 0.5f;	
	}
	// 设置USART1的波特率
	USART1->BRR = DIV_M<<4 | DIV_F;
	
	// 使能USART1的接收中断
	USART1->CR1 |= USART_CR1_RXNEIE;		
	
	// 设置NVIC优先级分组
	SCB->AIRCR |= 5<<SCB_AIRCR_PRIGROUP_Pos;		
	// 使能USART1的接收中断
	NVIC->ISER[1] |= 1<<5;		
	// 设置USART1的接收中断优先级
	NVIC->IP[36] = 5;		
	
	// 使能USART1
	USART1->CR1 |= USART_CR1_UE;	
}

// 函数：USART_SendByte
// 功能：发送一个字节
// 参数：Byte：要发送的字节
void USART_SendByte(uint8_t Byte)
{
	// 等待发送完成
	while(!(USART1->SR & USART_SR_TC));		
	// 发送字节
	USART1->DR = Byte;
}

int fputc(int ch, FILE *f)
{
    USART_SendByte(ch);
    return ch;
}

//uint8_t USART_ReceiveByte(void)
//{
//	uint8_t RxData;
//	while((USART1->SR & USART_SR_RXNE) == RESET);		
//	RxData = USART1->DR;
//	return RxData;
//}

void USART1_IRQHandler(void)
{
	// 进入中断服务程序
	OSIntEnter();
	// 检查USART1的状态寄存器，看是否有接收数据
	if(USART1->SR & USART_SR_RXNE)		
	{
		// 读取接收到的数据
		RxData = USART1->DR;		
		// 清除接收中断标志位
		USART1->SR &= ~USART_SR_RXNE;	
	}		
	// 退出中断服务程序
	OSIntExit();
}
