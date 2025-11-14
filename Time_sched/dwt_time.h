#ifndef __DWT_TIME_H__
#define __DWT_TIME_H__

#include "stm32f4xx.h"

void DWT_Init(void);
float DWT_GetTime(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
