#ifndef __USART_H__
#define __USART_H__

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "misc.h"
#include <stdio.h>
#include "ucos_ii.h"

extern uint8_t RxData;

void USART_GPIO_Init(void);
void USART_Config(uint16_t bps);
void USART_SendByte(uint8_t Byte);
//uint8_t USART_ReceiveByte(void);

#endif
