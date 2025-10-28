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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;		//����GPIOBʱ��
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;		//����USART1ʱ��
	
	GPIOB->AFR[0] |= GPIO_AF_USART1<<24 | GPIO_AF_USART1<<28;		//����USART1
	GPIOB->MODER |= GPIO_Mode_AF<<12 | GPIO_Mode_AF<<14;		//����ģʽ
	GPIOB->OTYPER |= GPIO_OType_PP<<6 | GPIO_OType_PP<<7;		//�������
	GPIOB->OSPEEDR |= GPIO_High_Speed<<12 | GPIO_High_Speed<<14;	//�ٶ�
	GPIOB->PUPDR |= GPIO_PuPd_NOPULL<<12 | GPIO_PuPd_NOPULL<<14;	//����
}

void USART_Config(uint32_t bps)
{
	USART_GPIO_Init();
	
	USART1->CR1 &= ~(USART_CR1_UE);		//USART�ر�
	USART1->CR1 &= ~USART_CR1_OVER8;		//16��������
	USART1->CR1 &= ~USART_CR1_M;		//8λ�ֳ�
	USART1->CR1 &= ~USART_CR1_PCE;		//��ֹ��żУ��λ
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;		//���͡�����ʹ��
	
	USART1->CR2 &= ~USART_CR2_STOP;		//1��ֹͣλ
	
	float USARTDIV;
	static uint16_t DIV_M;			//��������
	uint16_t DIV_F;			//С������
	if((USART1->CR1 & USART_CR1_OVER8) == SET)			
	{
		USARTDIV = 84000000.0/8/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*8 + 0.5f;		//OVER8Ϊ1
	}
	else
	{
		USARTDIV = 84000000.0/16/bps;
		DIV_M = (uint32_t)USARTDIV;
		DIV_F = (USARTDIV - DIV_M)*16 + 0.5f;		//OVER8Ϊ0
	}
	USART1->BRR = DIV_M<<4 | DIV_F;
	
	USART1->CR1 |= USART_CR1_RXNEIE;		//����USART�ж�
	
	SCB->AIRCR |= 5<<SCB_AIRCR_PRIGROUP_Pos;		//����NVIC�ж����ȼ�����
	NVIC->ISER[1] |= 1<<5;		//ʹ��USART1��NVIC�ж�
	NVIC->IP[36] = 5;		//����USART1�ж���Ӧ���ȼ�����ռ���ȼ�
	
	USART1->CR1 |= USART_CR1_UE;		//����USART
}

void USART_SendByte(uint8_t Byte)
{
	while(!(USART1->SR & USART_SR_TXE));		//�ȴ���һ�η������
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
 * @brief  �������ݵ�������λ��
 * @note   ���ݸ�ʽ: ֡ͷ(0xAA) + ֡ͷ(0xFF) + ������ + ���ݳ��� + ���� + У��� + ����У���
 * @param  data: Ҫ���͵���������
 * @param  len:  ���ݳ���
 * @param  id:   ������/��ʶ��
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
//	while((USART1->SR & USART_SR_RXNE) == RESET);		//�ȴ����յ�����
//	RxData = USART1->DR;
//	return RxData;
//}

void USART1_IRQHandler(void)
{
	if(USART1->SR & USART_SR_RXNE)		//DR�Ĵ�����Ϊ��
	{
		RxData = USART1->DR;		//��ȡDR
		USART1->SR &= ~USART_SR_RXNE;		//���RXNE��־λ
	}		
}
