#ifndef __PWM_H__
#define __PWM_H__

void GPIO_Config(void);
void TIM1_Config(void);
void PWM_SetCompare(int motor, uint16_t compare);
void MOTOR_Init(void);
void MOTOR_SetSpeed(int moter,int8_t Speed);
void Unlock_ESC(int motor);
void Unlock_ALL_ESC(void);

#endif
