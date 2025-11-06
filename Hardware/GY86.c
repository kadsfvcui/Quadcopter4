#include "GY86.h"

#define MPU6050_ADDRESS 0xD0		//1101 000
#define HMC5883L_ADDRESS 0x3C		//0011 110
#define MS5611_ADDRESS 0xEE		//1110 111

// 函数：SCL_W
// 功能：设置SCL线电平
// 参数：BitValue，要设置的电平值
void SCL_W(uint8_t BitValue)
{
	// 设置GPIOB的GPIO_Pin_10引脚的电平为BitValue
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)BitValue);
	// 延时10微秒
	Delay_us(10);
}

// 函数：SDA_W
// 功能：向SDA线写入一个位
// 参数：BitValue，要写入的位值
void SDA_W(uint8_t BitValue)
{
	// 向GPIOB的GPIO_Pin_3位写入BitValue
	GPIO_WriteBit(GPIOB, GPIO_Pin_3, (BitAction)BitValue);
	// 延时10微秒
	Delay_us(10);
}

// 函数：读取SDA引脚的状态
// 参数：无
// 返回值：BitAction类型，表示SDA引脚的状态
BitAction SDA_R(void)
{
	// 定义一个变量BitValue，用于存储SDA引脚的状态
	uint8_t BitValue;
	// 读取GPIOB端口上GPIO_Pin_3引脚的状态，并将其存储到BitValue变量中
	BitValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
	// 延时10微秒
	Delay_us(10);
	// 返回BitValue的值，表示SDA引脚的状态
	return (BitAction)BitValue;
}

// 函数：发送起始信号
// 功能：启动I2C通信
void MyI2C_Start(void)
{
    // 将SDA线拉高
    SDA_W(1);
    // 将SCL线拉高
	SCL_W(1);
    // 将SDA线拉低
	SDA_W(0);
    // 将SCL线拉低
	SCL_W(0);
}

// 函数：发送停止信号
void MyI2C_Stop(void)
{
    // 将SDA线拉低
    SDA_W(0);
    // 将SCL线拉高
	SCL_W(1);
    // 将SDA线拉高
	SDA_W(1);
}

// 函数：MyI2C_SendByte
// 功能：通过I2C发送一个字节
// 参数：Byte：要发送的字节
void MyI2C_SendByte(uint8_t Byte)
{
    // 定义一个变量i，用于循环
    uint8_t i;
    // 循环8次，每次发送一个位
    for(i=0;i<8;i++)
	{
		// 将Byte的第i位发送到SDA线上
		SDA_W(Byte & (0x80>>i));
	    // 将SCL线拉高
	    SCL_W(1);
	    // 将SCL线拉低
	    SCL_W(0);
	}
}

// 函数：MyI2C_ReceiveByte
// 功能：通过I2C接收一个字节
// 返回值：Byte：接收到的字节
uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i,Byte=0x00;
	// 设置SDA为高电平
	SDA_W(1);
	// 循环8次，接收8位数据
	for(i=0;i<8;i++)
	{
		// 设置SCL为高电平
		SCL_W(1);
		// 如果SDA为高电平，则将Byte的第i位设置为1
	    if(SDA_R()==1){Byte |= (0x80>>i);}
	    // 设置SCL为低电平
	    SCL_W(0);
	}
	// 返回接收到的数据
	return Byte;
}

// 函数：MyI2C_SendAck
// 功能：发送应答位
// 参数：AckBit：应答位
void MyI2C_SendAck(uint8_t AckBit)
{
    // 设置SDA引脚的电平
    SDA_W(AckBit);
	// 设置SCL引脚的电平为高
	SCL_W(1);
	// 设置SCL引脚的电平为低
	SCL_W(0);
}

// 函数：MyI2C_ReceiveAck
// 功能：接收应答位
// 返回值：应答位
uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t AckBit;
	// 将SDA引脚设置为高电平
	SDA_W(1);
	// 将SCL引脚设置为高电平
	SCL_W(1);
	// 读取SDA引脚的电平
	AckBit = SDA_R();
	// 将SCL引脚设置为低电平
	SCL_W(0);
	// 返回应答位
	return AckBit;
}

// 函数：GY86_WriteReg
// 功能：指定地址写入数据
// 参数：SADDR：I2C从机地址，RegAddr：寄存器地址，Data：要写入的数据
void GY86_WriteReg(uint8_t SADDR, uint8_t RegAddr, uint8_t Data)
{
	// 发送I2C起始信号
	MyI2C_Start();
	// 发送GY86传感器的地址
	MyI2C_SendByte(SADDR);
	// 等待GY86传感器的应答
	MyI2C_ReceiveAck();
	// 发送要写入的寄存器地址
	MyI2C_SendByte(RegAddr);
	// 等待GY86传感器的应答
	MyI2C_ReceiveAck();
	// 发送要写入的数据
	MyI2C_SendByte(Data);
	// 等待GY86传感器的应答
	MyI2C_ReceiveAck();
	// 发送I2C停止信号
	MyI2C_Stop();
}

// 函数：GY86_ReadReg
// 功能：指定地址读取GY86寄存器的值
// 参数：SADDR：I2C从机地址，RegAddr：寄存器地址
// 返回值：读取到的数据
uint8_t GY86_ReadReg(uint8_t SADDR, uint8_t RegAddr)
{
	uint8_t RxData; // 定义一个变量用于存储读取到的数据
	
	MyI2C_Start(); // 发送I2C起始信号
	MyI2C_SendByte(SADDR); // 发送I2C从机地址
	MyI2C_ReceiveAck(); // 接收I2C从机应答信号
	MyI2C_SendByte(RegAddr); // 发送寄存器地址
	MyI2C_ReceiveAck(); // 接收I2C从机应答信号
	
	MyI2C_Start(); // 发送I2C起始信号
	MyI2C_SendByte(SADDR | 0x01); // 发送I2C从机地址，加上读操作
	MyI2C_ReceiveAck(); // 接收I2C从机应答信号
	RxData = MyI2C_ReceiveByte(); // 读取数据
	MyI2C_SendAck(1); // 发送I2C从机应答信号
	MyI2C_Stop(); // 发送I2C停止信号
	
	return RxData; // 返回读取到的数据
}

// 初始化I2C
void MyI2C_Init(void)
{
	// 使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	// 定义GPIO初始化结构体
	GPIO_InitTypeDef GPIOInitStructure;
	// 设置GPIO模式为输出
	GPIOInitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// 设置GPIO输出类型为开漏
	GPIOInitStructure.GPIO_OType = GPIO_OType_OD;
	// 设置GPIO引脚为3和10
	GPIOInitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10;
	// 设置GPIO上拉/下拉为不使用
	GPIOInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// 设置GPIO速度为25MHz
	GPIOInitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	// 初始化GPIOB
	GPIO_Init(GPIOB, &GPIOInitStructure);
	
	// 设置GPIO引脚3和10为高电平
	GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_10);
}


// 初始化MPU6050
void MPU6050_Init(void)
{
	// 初始化I2C
	MyI2C_Init();
	
	// 设置MPU6050的电源管理寄存器，解除睡眠
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, 0x01);		
	// 六轴传感器不待机
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2, 0x00);		
	// 设置MPU6050的采样率分频器
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0x09);		
	// 设置MPU6050的配置寄存器，配置低通滤波器
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_CONFIG, 0x06);		
	// 设置MPU6050的陀螺仪配置寄存器，配置陀螺仪量程
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, 0x18);		
	// 设置MPU6050的加速度计配置寄存器，配置加速度计量程
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, 0x18);		
}

// 函数：获取MPU6050数据
// 参数：MPU6050_DataTypeDef* DataStruct，数据结构体指针
void MPU6050_GetData(MPU6050_DataTypeDef* DataStruct)
{
	// 读取加速度计X轴数据
	DataStruct->Acc_X = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_L);
	// 读取加速度计Y轴数据
	DataStruct->Acc_Y = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_YOUT_L);
	// 读取加速度计Z轴数据
	DataStruct->Acc_Z = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_ACCEL_ZOUT_L);
	// 读取陀螺仪X轴数据
	DataStruct->Gyro_X = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_XOUT_L);
	// 读取陀螺仪Y轴数据
	DataStruct->Gyro_Y = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_YOUT_L);
	// 读取陀螺仪Z轴数据
	DataStruct->Gyro_Z = GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_H)<<8 | GY86_ReadReg(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_L);
}

// 初始化HMC5883L传感器
void HMC5883L_Init(void)
{
	// 设置MPU6050的INT_PIN_CFG寄存器，开启MPU6050旁路模式
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 0x02);		
	// 设置MPU6050的USER_CTRL寄存器，将I2C主模式关闭
	GY86_WriteReg(MPU6050_ADDRESS, MPU6050_USER_CTRL, 0x00);
	// 设置HMC5883L的CONFIGA寄存器，设置采样频率为15Hz
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_CONFIGA, 0x50);
	// 设置HMC5883L的CONFIGB寄存器，设置增益为2.5Ga
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_CONFIGB, 0x00);
	// 设置HMC5883L的MODE寄存器，将传感器设置为连续测量模式
	GY86_WriteReg(HMC5883L_ADDRESS, HMC5883L_MODE, 0x00);
}

// 函数：获取HMC5883L磁力计的数据
// 参数：HMC5883L_DataTypeDef类型的指针DataStruct，用于存储读取的数据
void HMC5883L_GetData(HMC5883L_DataTypeDef* DataStruct)
{
	// 读取X轴磁力计数据，并将其存储到DataStruct的Mag_X成员中
	DataStruct->Mag_X = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_XOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_XOUT_L);
	// 读取Y轴磁力计数据，并将其存储到DataStruct的Mag_Y成员中
	DataStruct->Mag_Y = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_YOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_YOUT_L);
	// 读取Z轴磁力计数据，并将其存储到DataStruct的Mag_Z成员中
	DataStruct->Mag_Z = GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_ZOUT_H)<<8 | GY86_ReadReg(HMC5883L_ADDRESS, HMC5883L_ZOUT_L);
}

// 函数：向MS5611传感器发送命令
// 参数：CMD，要发送的命令
void GY86_MS5611_SendCMD(uint8_t CMD)
{
	// 发送I2C起始信号
	MyI2C_Start();
	// 发送MS5611传感器的地址
	MyI2C_SendByte(MS5611_ADDRESS);
	// 等待MS5611传感器回应
	MyI2C_ReceiveAck();
	// 发送命令
	MyI2C_SendByte(CMD);
	// 等待MS5611传感器回应
	MyI2C_ReceiveAck();
	// 发送I2C停止信号
	MyI2C_Stop();
}

// 函数：MS5611_Reset
// 功能：重置MS5611传感器
void MS5611_Reset(void)
{
	// 发送重置命令
	GY86_MS5611_SendCMD(MS5611_CMD_RESET);
}

// 函数：读取MS5611的PROM数据
void MS5611_ReadProm(MS5611_Prom_DataTypeDef* DataStruct)
{
	// 发送读取 PROM 命令
	GY86_MS5611_SendCMD(MS5611_CMD_PROM_1);		
	
	// 开始 I2C 通信
	MyI2C_Start();
	// 发送 MS5611 地址
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	// 等待应答
	MyI2C_ReceiveAck();
	
	// 读取接收到的第一个字节
	DataStruct->reserve = MyI2C_ReceiveByte()<<8;		
	// 发送应答0，继续读取
	MyI2C_SendAck(0);
	
	// 读取接收到的第二个字节
	DataStruct->reserve |= MyI2C_ReceiveByte();
	// 发送应答1，停止读取
	MyI2C_SendAck(1);
	
	// 循环读取 6 个 PROM 数据
	for(int i = 0; i<6; i++)
	{
		// 发送读取 PROM 命令
		GY86_MS5611_SendCMD(MS5611_CMD_PROM_2 + 2*i);
		
		// 开始 I2C 通信
		MyI2C_Start();
		// 发送 MS5611 地址
		MyI2C_SendByte(MS5611_ADDRESS | 0x01);
		// 等待应答
		MyI2C_ReceiveAck();
		
		// 读取接收到的第一个字节
		DataStruct->C[i] = MyI2C_ReceiveByte()<<8;
		MyI2C_SendAck(0);
		
		// 读取接收到的第二个字节
		DataStruct->C[i] |= MyI2C_ReceiveByte();
		MyI2C_SendAck(1);
	}
	// 发送读取命令
	GY86_MS5611_SendCMD(MS5611_CMD_PROM_7);		
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	DataStruct->crc = MyI2C_ReceiveByte()<<8;
	MyI2C_SendAck(0);
	
	DataStruct->crc |= MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
}

// 读取MS5611的温度和压力数据
uint32_t MS5611_ReadTempPress(uint8_t OSR)
{
	uint32_t data = 0;
	
	// 发送转换命令
	GY86_MS5611_SendCMD(OSR);		
	Delay_ms(50);
	
	// 发送命令，读取ADC数据
	GY86_MS5611_SendCMD(MS5611_CMD_ADC_READ);		
	
	// 开始I2C通信
	MyI2C_Start();
	// 发送MS5611的地址
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	// 等待应答
	MyI2C_ReceiveAck();
	
	// 读取高8位数据
	data = MyI2C_ReceiveByte()<<16;
	// 发送应答
	MyI2C_SendAck(0);
	
	// 读取中8位数据
	data = MyI2C_ReceiveByte()<<8;
	// 发送应答
	MyI2C_SendAck(0);
	
	// 读取低8位数据
	data = MyI2C_ReceiveByte();
	// 发送应答
	MyI2C_SendAck(1);
	
	// 返回数据
	return data;
}

/*	A * 2^n = A << n
	A / 2^n = A >> n	*/
void MS5611_Calculate(MS5611_Prom_DataTypeDef* Prom, uint32_t Dtemp, uint32_t Dp, MS5611_ResultTypeDef* DataStruct)
{
	// 计算温度和压力
	int32_t dT = 0, TEMP = 0, P = 0;
	int64_t OFF = 0, SENS = 0;
	// 计算温度偏差
	dT = Dtemp - (Prom->C[4] << 8);
	// 计算温度
	TEMP = 2000 + ((dT * Prom->C[5]) >> 23);
	// 计算压力偏差
	OFF = (Prom->C[1] << 16) + ((Prom->C[3] * dT) >> 7);
	// 计算压力敏感度
	SENS = (Prom->C[0] << 15) + ((Prom->C[2] * dT) >> 8);
	// 计算压力
	P = (((Dp * SENS) >> 21) - OFF) >> 15;
	
	// 如果温度低于20℃
	if(TEMP < 2000)		
	{
		int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
		// 计算温度二次偏差
		T2 = dT*dT >> 31;
		// 计算压力偏差二次偏差
		OFF2 = 5*(TEMP - 2000)*(TEMP - 2000) >> 1;
		// 计算压力敏感度二次偏差
		SENS2 = 5*(TEMP - 2000)*(TEMP - 2000) >> 2;
		// 如果温度低于-15℃
		if(TEMP < -1500)		
		{
			// 计算压力偏差二次偏差
			OFF2 = OFF2 + 7*(TEMP + 1500)*(TEMP + 1500);
			// 计算压力敏感度二次偏差
			SENS2 = SENS2 + (11*(TEMP + 1500)*(TEMP + 1500) >> 1);
		}
		// 更新温度
		TEMP -= T2;
		// 更新压力偏差
		OFF -= OFF2;
		// 更新压力敏感度
		SENS -= SENS2;
	}
	
	// 将计算结果存储到结构体中
	DataStruct->P = P;
	DataStruct->TEMP = TEMP;
}

// 初始化GY86传感器
void GY86_Init(void)
{
	// 初始化MPU6050传感器
	MPU6050_Init();
	// 初始化HMC5883L传感器
	HMC5883L_Init();
	// 重置MS5611传感器
	MS5611_Reset();
}
