#include "Service.h"

uint16_t PPM[8];
uint8_t RxData;
int8_t data[100];

MPU6050_AccDataTypeDef Acc_Data;
MPU6050_GyroDataTypeDef Gyro_Data;
HMC5883L_DataTypeDef HMC_Data;
SEQTypeDef q = {1, 0, 0, 0};
float roll, pitch, yaw;

void ALL_Init(void)
{
	MOTOR_Init();
//	Unlock_ALL_ESC();
	GY86_Init();
	OLED_Init();
	PPM_Init();
	USART_Config(9600);
}

void GY86_Show(void)	
{
	MPU6050_GetAccData(&Acc_Data);
	MPU6050_GetGyroData(&Gyro_Data);
	HMC5883L_GetData(&HMC_Data);
	printf("mx: %d, my: %d, mz: %d\r\n", HMC_Data.Mag_X, HMC_Data.Mag_Y, HMC_Data.Mag_Z);
	switch(RxData)
	{
		case 01:
		{
			OLED_ShowString(1, 1, "AccX: ");
			OLED_ShowString(2, 1, "AccY: ");
			OLED_ShowString(3, 1, "AccZ: ");
			OLED_ShowSignedNum(1, 7, Acc_Data.Acc_X, 5);
			OLED_ShowSignedNum(2, 7, Acc_Data.Acc_Y, 5);
			OLED_ShowSignedNum(3, 7, Acc_Data.Acc_Z, 5);
			break;
		}
		case 02:
		{
			OLED_ShowString(1, 1, "GyroX:");
			OLED_ShowString(2, 1, "GyroY:");
			OLED_ShowString(3, 1, "GyroZ:");
			OLED_ShowSignedNum(1, 7, Gyro_Data.Gyro_X, 5);
			OLED_ShowSignedNum(2, 7, Gyro_Data.Gyro_Y, 5);
			OLED_ShowSignedNum(3, 7, Gyro_Data.Gyro_Z, 5);
			break;
		}
		case 03:
		{
			OLED_ShowString(1, 1, "MagX: ");
			OLED_ShowString(2, 1, "MagY: ");
			OLED_ShowString(3, 1, "MagZ: ");
			OLED_ShowSignedNum(1, 7, HMC_Data.Mag_X, 5);
			OLED_ShowSignedNum(2, 7, HMC_Data.Mag_Y, 5);
			OLED_ShowSignedNum(3, 7, HMC_Data.Mag_Z, 5);
			break;
		}
	}
}

void test(void)
{
	GY86_Show();
	PWM_SetCompare(1, PPM[2]);
	PWM_SetCompare(2, PPM[2]);
	PWM_SetCompare(3, PPM[2]);
	PWM_SetCompare(4, PPM[2]);
}

void Madgwick_Test(void)
{
    MPU6050_GetAccData(&Acc_Data);
	MPU6050_GetGyroData(&Gyro_Data);
	HMC5883L_GetData(&HMC_Data);

	MadgwickUpdate(&Acc_Data, &Gyro_Data, &HMC_Data, &q);


//	Quaternion2Euler(&q, &roll, &pitch, &yaw);

//  printf("roll: %.2f, pitch: %.2f, yaw: %.2f\n", roll, pitch, yaw);

//	int16_t roll_int = roll * 100 * 180.0f / 3.1415926f;
//	int16_t pitch_int = pitch * 100 * 180.0f / 3.1415926f;
//	int16_t yaw_int = yaw * 100 * 180.0f / 3.1415926f;

//	data[0] = (roll_int & 0xff);
//	data[1] = (roll_int >> 8);
//	data[2] = (pitch_int & 0xff);
//	data[3] = (pitch_int >> 8);
//	data[4] = (yaw_int & 0xff);
//	data[5] = (yaw_int >> 8);
//	data[6] = 0;

	int16_t q1_int = q.q1 * 10000;
	int16_t q2_int = q.q2 * 10000;
	int16_t q3_int = q.q3 * 10000;
	int16_t q4_int = q.q4 * 10000;

	data[0] = (q1_int & 0xff);
	data[1] = (q1_int >> 8);
	data[2] = (q2_int & 0xff);
	data[3] = (q2_int >> 8);
	data[4] = (q3_int & 0xff);
	data[5] = (q3_int >> 8);
	data[6] = (q4_int & 0xff);
	data[7] = (q4_int >> 8);
	data[8] = 0;

	Ano_SendData(data, 9, 0x04);

//		Ano_SendData(data, 7, 0x03);
	
	Delay_ms(10);
}
