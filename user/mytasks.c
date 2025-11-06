#include "mytasks.h"

OS_STK Task_StartupStk[TASK_STK_SIZE];
OS_STK Task_AngleStk[TASK_STK_SIZE];
OS_STK Task_MotorStk[TASK_STK_SIZE];
OS_STK Task_COMStk[TASK_STK_SIZE];
//OS_STK Task_DisplayStk[TASK_STK_SIZE];

OS_EVENT *GY86DataMutex; // GY86数据互斥信号量
INT8U err; // 错误码

#define COM_BUF_SIZE 128
static char com_buf[COM_BUF_SIZE];

void Mutex_Init(void)
{
    GY86DataMutex = OSSemCreate(1); // 创建互斥信号量
}

//启动任务，创建其他任务
void Task_Startup(void *p_arg)
{
    INT8U Task_Startup_err;
    OSTaskNameSet(TASK_STARTUP_PRIO, (INT8U *)"Task_Startup", &Task_Startup_err);

    Board_Init();

    OSTaskCreate(Task_Angle, (void *)0, (OS_STK *)&Task_AngleStk[TASK_STK_SIZE-1], TASK_ANGLE_PRIO);
    OSTaskCreate(Task_Motor, (void *)0, (OS_STK *)&Task_MotorStk[TASK_STK_SIZE-1], TASK_MOTOR_PRIO);
    OSTaskCreate(Task_COM, (void *)0, (OS_STK *)&Task_COMStk[TASK_STK_SIZE-1], TASK_COM_PRIO);
//	OSTaskCreate(Task_Display, (void *)0, (OS_STK *)&Task_DisplayStk[TASK_STK_SIZE-1], TASK_DISPLAY_PRIO);
	Mutex_Init();

    OSTaskDel(TASK_STARTUP_PRIO);
}

//姿态解算任务
void Task_Angle(void *p_arg)
{
    INT8U Task_Angle_err;
    OSTaskNameSet(TASK_ANGLE_PRIO, (INT8U *)"Task_Angle", &Task_Angle_err);

    while(1)
    {
		OSSemPend(GY86DataMutex, 0, &err);
        MPU6050_GetData(&MPU6050_Data);
        HMC5883L_GetData(&HMC_Data);
		OSSemPost(GY86DataMutex);
		Display();
        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

//电机控制任务
void Task_Motor(void *p_arg)
{
    INT8U Task_Motor_err;
    OSTaskNameSet(TASK_MOTOR_PRIO, (INT8U *)"Task_Motor", &Task_Motor_err);

    while(1)
    {
        PWM_SetCompare(1, PPM[2]);
        PWM_SetCompare(2, PPM[2]);
        PWM_SetCompare(3, PPM[2]);
        PWM_SetCompare(4, PPM[2]);
        OSTimeDlyHMSM(0, 0, 0, 20);
    }
}

//蓝牙通信任务
void Task_COM(void *p_arg)
{
    INT8U Task_COM_err;
    OSTaskNameSet(TASK_COM_PRIO, (INT8U *)"Task_COM", &Task_COM_err);

    while(1)
    {
		OSSemPend(GY86DataMutex, 0, &err);
		// 单次格式化所有传感器数据
        int len = snprintf(com_buf, COM_BUF_SIZE,
            "Acc:[%d,%d,%d] Gyro:[%d,%d,%d] Mag:[%d,%d,%d]\r\n",
            MPU6050_Data.Acc_X, MPU6050_Data.Acc_Y, MPU6050_Data.Acc_Z,
            MPU6050_Data.Gyro_X, MPU6050_Data.Gyro_Y, MPU6050_Data.Gyro_Z,
            HMC_Data.Mag_X, HMC_Data.Mag_Y, HMC_Data.Mag_Z);

        for(int i = 0; i<len; i++)
		{
			USART_SendByte(com_buf[i]);
		}
		OSSemPost(GY86DataMutex);

        OSTimeDlyHMSM(0, 0, 0, 30);
    }
}

//oled显示任务
//void Task_Display(void *p_arg)
//{
//    INT8U Task_Display_err;
//    OSTaskNameSet(TASK_DISPLAY_PRIO, (INT8U *)"Task_Display", &Task_Display_err);

//    while(1)
//    {
//	   	OSSemPend(GY86DataMutex, 0, &err);
//	    Display();
//		OSSemPost(GY86DataMutex);
//        OSTimeDlyHMSM(0, 0, 0, 50);
//    }
//}

void Display(void)
{
	switch(RxData)
	{
		case 01:
		{
			OLED_ShowString(1, 1, "AccX: ");
			OLED_ShowString(2, 1, "AccY: ");
			OLED_ShowString(3, 1, "AccZ: ");
			OLED_ShowSignedNum(1, 7, MPU6050_Data.Acc_X, 5);
			OLED_ShowSignedNum(2, 7, MPU6050_Data.Acc_Y, 5);
			OLED_ShowSignedNum(3, 7, MPU6050_Data.Acc_Z, 5);
			break;
		}
		case 02:
		{
			OLED_ShowString(1, 1, "GyroX:");
			OLED_ShowString(2, 1, "GyroY:");
			OLED_ShowString(3, 1, "GyroZ:");
			OLED_ShowSignedNum(1, 7, MPU6050_Data.Gyro_X, 5);
			OLED_ShowSignedNum(2, 7, MPU6050_Data.Gyro_Y, 5);
			OLED_ShowSignedNum(3, 7, MPU6050_Data.Gyro_Z, 5);
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
		case 04:
		{
			OLED_ShowNum(1, 1, PPM[0], 4);
			OLED_ShowNum(2, 1, PPM[1], 4);
			OLED_ShowNum(3, 1, PPM[2], 4);
			OLED_ShowNum(4, 1, PPM[3], 4);
			OLED_ShowString(1, 5, "        ");
			OLED_ShowString(2, 5, "        ");
			OLED_ShowString(3, 5, "        ");
			OLED_ShowString(4, 5, "        ");
			break;
		}
	}
}
