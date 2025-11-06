#ifndef __PPM_H__
#define __PPM_H__

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "Delay.h"
#include "ucos_ii.h"

extern uint16_t PPM[8];

void PPM_Init(void);

#endif
