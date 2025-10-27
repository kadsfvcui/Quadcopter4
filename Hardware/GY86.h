#ifndef __GY86_H__
#define __GY86_H__

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "GY86_Reg.h"
#include "MS5611_CMD.h"
#include "Delay.h"

typedef struct
{
	int16_t Acc_X;
	int16_t Acc_Y;
	int16_t Acc_Z;
}MPU6050_AccDataTypeDef;

typedef struct
{
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
}MPU6050_GyroDataTypeDef;

typedef struct
{
	int16_t Mag_X;
	int16_t Mag_Y;
	int16_t Mag_Z;
}HMC5883L_DataTypeDef;

typedef struct
{
	int16_t reserve;
	int16_t C[6];
	int16_t crc;
}MS5611_Prom_DataTypeDef;

typedef struct
{
	int32_t TEMP;
	int32_t P;
}MS5611_ResultTypeDef;

void GY86_Init(void);
void SCL_W(uint8_t BitValue);
void SDA_W(uint8_t BitValue);
BitAction SDA_R(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t Byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveAck(void);
void MyI2C_Init(void);

void GY86_WriteReg(uint8_t SADDR, uint8_t RegAddr, uint8_t Data);
uint8_t GY86_ReadReg(uint8_t SADDR, uint8_t RegAddr);

void MPU6050_Init(void);
void MPU6050_GetAccData(MPU6050_AccDataTypeDef* DataStruct);
void MPU6050_GetGyroData(MPU6050_GyroDataTypeDef* DataStruct);

void HMC5883L_Init(void);
void HMC5883L_GetData(HMC5883L_DataTypeDef* DataStruct);

void GY86_MS5611_SendCMD(uint8_t CMD);
void MS5611_Reset(void);
void MS5611_ReadProm(MS5611_Prom_DataTypeDef* DataStruct);
uint32_t MS5611_ReadTempPress(uint8_t OSR);
void MS5611_Calculate(MS5611_Prom_DataTypeDef* Prom, uint32_t Dtemp, uint32_t Dp, MS5611_ResultTypeDef* DataStruct);

#endif
