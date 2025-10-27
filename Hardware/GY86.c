#define MPU6050_ADDRESS 0xD0		//1101 000
#define HMC5883L_ADDRESS 0x3C		//0011 110
#define MS5611_ADDRESS 0xEE		//1110 111

#include "GY86.h"

void SCL_W(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
	Delay_us(10);
}

void SDA_W(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction)BitValue);
	Delay_us(10);
}

BitAction SDA_R(void)
{
	uint8_t BitValue;
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	Delay_us(10);
	return (BitAction)BitValue;
}

void MyI2C_Start(void)
{
    SDA_W(1);
	SCL_W(1);
	SDA_W(0);
	SCL_W(0);
}

void MyI2C_Stop(void)
{
    SDA_W(0);
	SCL_W(1);
	SDA_W(1);
}

void MyI2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    for(i=0;i<8;i++)
	{
		SDA_W(Byte & (0x80>>i));
	    SCL_W(1);
	    SCL_W(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i,Byte=0x00;
	SDA_W(1);
	for(i=0;i<8;i++)
	{
		SCL_W(1);
	    if(SDA_R()==1){Byte |= (0x80>>i);}
	    SCL_W(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)
{
    SDA_W(AckBit);
	SCL_W(1);
	SCL_W(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
	SDA_W(1);
	SCL_W(1);
	AckBit = SDA_R();
	SCL_W(0);
	return AckBit;
}

void GY86_WriteReg(uint8_t SADDR, uint8_t RegAddr, uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(SADDR);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t GY86_ReadReg(uint8_t SADDR, uint8_t RegAddr)
{
	uint8_t RxData;
	
	MyI2C_Start();
	MyI2C_SendByte(SADDR);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(SADDR | 0x01);
	MyI2C_ReceiveAck();
	RxData = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return RxData;
}

void MyI2C_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIOInitStructure;
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIOInitStructure.GPIO_OType = GPIO_OType_OD;
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
	GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOB, &GPIOInitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_10);
}


void MPU6050_Init(void)
{
	MyI2C_Init();
	
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);		//解除睡眠
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);		//六轴传感器不待机
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x09);		//采样率分频
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);		//配置低通滤波器
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x08);		//配置陀螺仪量程 ±500°/s
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x08);		//配置加速度计量程 ±4g
}

void MPU6050_GetAccData(MPU6050_AccDataTypeDef* DataStruct)
{
	DataStruct->Acc_X = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L);
	DataStruct->Acc_Y = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L);
	DataStruct->Acc_Z = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L);
}

void MPU6050_GetGyroData(MPU6050_GyroDataTypeDef* DataStruct)
{
	DataStruct->Gyro_X = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L);
	DataStruct->Gyro_Y = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L);
	DataStruct->Gyro_Z = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L);
}

void HMC5883L_Init(void)
{
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x02);		//开启MPU6050旁路模式
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x00);
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_CONFIGA, 0x50);
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_CONFIGB, 0x20);		//磁力计量程 ±1.3Gass
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
}

void HMC5883L_GetData(HMC5883L_DataTypeDef* DataStruct)
{
	DataStruct->Mag_X = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_XOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_XOUT_L);
	DataStruct->Mag_Y = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_YOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_YOUT_L);
	DataStruct->Mag_Z = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_ZOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_ZOUT_L);
}

void GY86_MS5611_SendCMD(uint8_t CMD)
{
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(CMD);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

void MS5611_Reset(void)
{
	GY86_MS5611_SendCMD(MS5611_CMD_RESET);
}

void MS5611_ReadProm(MS5611_Prom_DataTypeDef* DataStruct)
{
	GY86_MS5611_SendCMD(MS5611_CMD_PROM_1);		//发送读取命令
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	DataStruct->reserve = MyI2C_ReceiveByte()<<8;		//读取接收到的字节
	MyI2C_SendAck(0);
	
	DataStruct->reserve |= MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	
	for(int i = 0; i<6; i++)
	{
		GY86_MS5611_SendCMD(MS5611_CMD_PROM_2 + 2*i);
		
		MyI2C_Start();
		MyI2C_SendByte(MS5611_ADDRESS | 0x01);
		MyI2C_ReceiveAck();
		
		DataStruct->C[i] = MyI2C_ReceiveByte()<<8;
		MyI2C_SendAck(0);
		
		DataStruct->C[i] |= MyI2C_ReceiveByte();
		MyI2C_SendAck(1);
	}
	
	GY86_MS5611_SendCMD(MS5611_CMD_PROM_7);		//发送读取命令
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	DataStruct->crc = MyI2C_ReceiveByte()<<8;
	MyI2C_SendAck(0);
	
	DataStruct->crc |= MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
}

uint32_t MS5611_ReadTempPress(uint8_t OSR)
{
	uint32_t data = 0;
	
	GY86_MS5611_SendCMD(OSR);		//发送转换命令
	Delay_ms(50);
	
	GY86_MS5611_SendCMD(MS5611_CMD_ADC_READ);		//发送读取命令
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	data = MyI2C_ReceiveByte()<<16;
	MyI2C_SendAck(0);
	
	data = MyI2C_ReceiveByte()<<8;
	MyI2C_SendAck(0);
	
	data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	
	return data;
}

/*	A * 2^n = A << n
	A / 2^n = A >> n	*/
void MS5611_Calculate(MS5611_Prom_DataTypeDef* Prom, uint32_t Dtemp, uint32_t Dp, MS5611_ResultTypeDef* DataStruct)
{
	int32_t dT = 0, TEMP = 0, P = 0;
	int64_t OFF = 0, SENS = 0;
	dT = Dtemp - (Prom->C[4] << 8);
	TEMP = 2000 + ((dT * Prom->C[5]) >> 23);
	OFF = (Prom->C[1] << 16) + ((Prom->C[3] * dT) >> 7);
	SENS = (Prom->C[0] << 15) + ((Prom->C[2] * dT) >> 8);
	P = (((Dp * SENS) >> 21) - OFF) >> 15;
	
	if(TEMP < 2000)		//温度低于20℃
	{
		int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
		T2 = dT*dT >> 31;
		OFF2 = 5*(TEMP - 2000)*(TEMP - 2000) >> 1;
		SENS2 = 5*(TEMP - 2000)*(TEMP - 2000) >> 2;
		if(TEMP < -1500)		//温度低于-15℃
		{
			OFF2 = OFF2 + 7*(TEMP + 1500)*(TEMP + 1500);
			SENS2 = SENS2 + (11*(TEMP + 1500)*(TEMP + 1500) >> 1);
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}
	
	DataStruct->P = P;
	DataStruct->TEMP = TEMP;
}

void GY86_Init(void)
{
	MPU6050_Init();
	HMC5883L_Init();
	MS5611_Reset();
}
