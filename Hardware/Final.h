#ifndef __SERVICE_H__
#define __SERVICE_H__

#include "stm32f4xx.h"                  // Device header
#include "GY86.h"
#include "pwm.h"
#include "usart.h"
#include "OLED.h"
#include "PPM.h"
#include "Madgwick.h"
#include "PID.h"
#include "Delay.h"
#include <stdio.h>
#include <stdbool.h>

extern MPU6050_DataTypeDef MPU6050_Data;
extern HMC5883L_DataTypeDef HMC_Data;
extern SEQTypeDef q;
extern Quad_TypeDef quad;

void Board_Init(void);
void GY86_Show(void);
void test(void);

#endif
