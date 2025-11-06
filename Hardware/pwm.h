#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "Delay.h"
#include "ucos_ii.h"

void GPIO_Config(void);
void TIM1_Config(void);
void PWM_SetCompare(int motor, uint16_t compare);
void MOTOR_Init(void);
void MOTOR_SetSpeed(int moter,int8_t Speed);
void Unlock_ESC(int motor);
void Unlock_ALL_ESC(void);

#endif
