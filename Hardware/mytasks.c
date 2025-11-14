#include "mytasks.h"

OS_STK Task_StartupStk[TASK_STK_SIZE];
OS_STK Task_AttitudeStk[TASK_STK_SIZE];
OS_STK Task_OuterStk[TASK_STK_SIZE];
OS_STK Task_InnerStk[TASK_STK_SIZE];
OS_STK Task_COMStk[TASK_STK_SIZE];
//OS_STK Task_DisplayStk[TASK_STK_SIZE];

OS_EVENT *Sem_InnerLoop;    
OS_EVENT *Sem_OuterLoop;
OS_EVENT *Sem_Attitude;

OS_EVENT *GY86DataMutex; // GY86数据互斥信号量
OS_EVENT *SEQDataMutex; // 四元数互斥信号量
OS_EVENT *QuadDataMutex; // 四轴数据互斥信号量
INT8U err; // 错误码



#define COM_BUF_SIZE 128

void Sem_Init(void)
{
    Sem_InnerLoop = OSSemCreate(0); // 创建内环控制信号量
    Sem_OuterLoop = OSSemCreate(0); // 创建外环控制信号量
    Sem_Attitude = OSSemCreate(0); // 创建姿态解算信号量

    GY86DataMutex = OSSemCreate(1); // 创建GY86互斥信号量
    SEQDataMutex = OSSemCreate(1); // 创建四元数互斥信号量
    QuadDataMutex = OSSemCreate(1); // 创建四轴数据互斥信号量
}

//启动任务，创建其他任务
void Task_Startup(void *p_arg)
{
    INT8U Task_Startup_err;
    OSTaskNameSet(TASK_STARTUP_PRIO, (INT8U *)"Task_Startup", &Task_Startup_err);

    Board_Init();

    OSTaskCreate(Task_Attitude, (void *)0, (OS_STK *)&Task_AttitudeStk[TASK_STK_SIZE-1], TASK_ATTITUDE_PRIO);
    OSTaskCreate(Task_Outer, (void *)0, (OS_STK *)&Task_OuterStk[TASK_STK_SIZE-1], TASK_OUTER_PRIO);
    OSTaskCreate(Task_Inner, (void *)0, (OS_STK *)&Task_InnerStk[TASK_STK_SIZE-1], TASK_INNER_PRIO);
    OSTaskCreate(Task_COM, (void *)0, (OS_STK *)&Task_COMStk[TASK_STK_SIZE-1], TASK_COM_PRIO);
//	OSTaskCreate(Task_Display, (void *)0, (OS_STK *)&Task_DisplayStk[TASK_STK_SIZE-1], TASK_DISPLAY_PRIO);
	Sem_Init();

    OSTaskDel(TASK_STARTUP_PRIO);
}

//姿态解算任务
void Task_Attitude(void *p_arg)
{
    INT8U Task_Attitude_err;
    OSTaskNameSet(TASK_ATTITUDE_PRIO, (INT8U *)"Task_Attitude", &Task_Attitude_err);

    float lastTime = DWT_GetTime();
    float currentTime, dt;

    while(1)
    {
        OSSemPend(Sem_Attitude, 0, &err);
//        printf("Task_Attitude Running...\n");
		OSSemPend(GY86DataMutex, 0, &err);
        MPU6050_GetData(&MPU6050_Data);
        HMC5883L_GetData(&HMC_Data);
		OSSemPost(GY86DataMutex);

        currentTime = DWT_GetTime(); // 获取系统时间并转换成s
        dt = currentTime - lastTime;
        lastTime = currentTime;

        OSSemPend(SEQDataMutex, 0, &err);
		MadgwickUpdate(&MPU6050_Data, &HMC_Data, &q, dt);
//        Display();
		OSSemPost(SEQDataMutex);
    }
}

//外环控制任务
void Task_Outer(void *p_arg)
{
    INT8U Task_Outer_err;
    OSTaskNameSet(TASK_OUTER_PRIO, (INT8U *)"Task_Outer", &Task_Outer_err);
    
    // 初始化PID控制器
    QuadPID_Init(&quad);
    
    // 定义姿态变量
    float roll, pitch, yaw;
    float lastTime = DWT_GetTime();
    float currentTime, dt;
    
    while(1)
    {
        OSSemPend(Sem_OuterLoop, 0, &err);
//        printf("Task_Outer Running...\n");
        // 获取当前姿态角
        OSSemPend(SEQDataMutex, 0, &err);
        Quaternion2Euler(&q, &roll, &pitch, &yaw);
        OSSemPost(SEQDataMutex);
        
        // 计算时间间隔
        currentTime = DWT_GetTime(); // 获取系统时间
        dt = currentTime - lastTime;
        lastTime = currentTime;
        
        // 更新PID控制器
        OSSemPend(QuadDataMutex, 0, &err);
        OuterLoop_Update(&quad, roll, pitch, yaw, PPM, dt);
        OSSemPost(QuadDataMutex);
    }
}

void Task_Inner(void *p_arg)
{
    INT8U Task_Inner_err;
    OSTaskNameSet(TASK_INNER_PRIO, (INT8U *)"Task_Inner", &Task_Inner_err);

    float lastTime = DWT_GetTime();
    float currentTime, dt;

    while(1)
    {
        OSSemPend(Sem_InnerLoop, 0, &err);
//        printf("Task_Inner Running...\n");
        // 计算时间间隔
        currentTime = DWT_GetTime(); // 获取系统时间(秒)
        dt = currentTime - lastTime;
        lastTime = currentTime;

        // 获取IMU数据
        OSSemPend(GY86DataMutex, 0, &err);
        MPU6050_DataTypeDef imu = MPU6050_Data;
        OSSemPost(GY86DataMutex);

        // 更新PID控制器
        OSSemPend(QuadDataMutex, 0, &err);
        InnerLoop_Update(&quad, &imu, dt);
        QuadPID_MotorControl(&quad);
        OSSemPost(QuadDataMutex);
    }
}

//蓝牙通信任务
void Task_COM(void *p_arg)
{
    INT8U Task_COM_err;
    OSTaskNameSet(TASK_COM_PRIO, (INT8U *)"Task_COM", &Task_COM_err);

    while(1)
    {
//        printf("Task_COM Running...\n");
		OSSemPend(SEQDataMutex, 0, &err);
		printf("%f, %f, %f, %f\n", q.q1, q.q2, q.q3, q.q4);
		OSSemPost(SEQDataMutex);

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
