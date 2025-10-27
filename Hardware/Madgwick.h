#ifndef __MADGWICK_H__
#define __MADGWICK_H__

#include "GY86.h"
#include "usart.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>

typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
}SEQTypeDef;

void MadgwickUpdate(MPU6050_AccDataTypeDef *accdata, MPU6050_GyroDataTypeDef *gyrodata, HMC5883L_DataTypeDef *magdata, SEQTypeDef *Q);
void Quaternion2Euler(SEQTypeDef *Q, float *roll, float *pitch, float *yaw);

#endif
