#include "mytasks.h"

OS_STK Task_StartupStk[TASK_STK_SIZE];
OS_STK Task_AngleStk[TASK_STK_SIZE];
OS_STK Task_OuterStk[TASK_STK_SIZE];
OS_STK Task_InnerStk[TASK_STK_SIZE];
OS_STK Task_COMStk[TASK_STK_SIZE];
//OS_STK Task_DisplayStk[TASK_STK_SIZE];

OS_EVENT *GY86DataMutex; // GY86数据互斥信号量
OS_EVENT *SEQDataMutex; // 四元数互斥信号量
OS_EVENT *QuadDataMutex; // 四轴数据互斥信号量
INT8U err; // 错误码



#define COM_BUF_SIZE 128

void Mutex_Init(void)
{
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

    OSTaskCreate(Task_Angle, (void *)0, (OS_STK *)&Task_AngleStk[TASK_STK_SIZE-1], TASK_ANGLE_PRIO);
    OSTaskCreate(Task_Outer, (void *)0, (OS_STK *)&Task_OuterStk[TASK_STK_SIZE-1], TASK_OUTER_PRIO);
    OSTaskCreate(Task_Inner, (void *)0, (OS_STK *)&Task_InnerStk[TASK_STK_SIZE-1], TASK_INNER_PRIO);
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

    float lastTime = OSTimeGet() / 1000.0f;
    float currentTime, dt;

    while(1)
    {
//        printf("Task_Angle Running...\n");
		OSSemPend(GY86DataMutex, 0, &err);
        MPU6050_GetData(&MPU6050_Data);
        HMC5883L_GetData(&HMC_Data);
		OSSemPost(GY86DataMutex);

        currentTime = OSTimeGet() / 1000.0f; // 获取系统时间并转换成s
        dt = currentTime - lastTime;
        lastTime = currentTime;

        OSSemPend(SEQDataMutex, 0, &err);
		MadgwickUpdate(&MPU6050_Data, &HMC_Data, &q, 0.01f);
//        Display();
		OSSemPost(SEQDataMutex);
        OSTimeDlyHMSM(0, 0, 0, 10);
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
    float lastTime = 0;
    float currentTime, dt;
    
    while(1)
    {
//        printf("Task_Outer Running...\n");
        // 获取当前姿态角
        OSSemPend(SEQDataMutex, 0, &err);
        Quaternion2Euler(&q, &roll, &pitch, &yaw);
        OSSemPost(SEQDataMutex);
        
        // 计算时间间隔
        currentTime = OSTimeGet() / 1000.0f; // 获取系统时间并转换为s
        dt = currentTime - lastTime;
        lastTime = currentTime;
        
        // 防止dt过大
        if(dt > 0.1f) dt = 0.01f;
        if(dt <= 0) dt = 0.01f;
        
        // 更新PID控制器
        OSSemPend(QuadDataMutex, 0, &err);
        OuterLoop_Update(&quad, roll, pitch, yaw, PPM, dt);
        OSSemPost(QuadDataMutex);

        OSTimeDlyHMSM(0, 0, 0, 10);
    }
}

void Task_Inner(void *p_arg)
{
    INT8U Task_Inner_err;
    OSTaskNameSet(TASK_INNER_PRIO, (INT8U *)"Task_Inner", &Task_Inner_err);

    float lastTime = 0;
    float currentTime, dt;

    while(1)
    {
//        printf("Task_Inner Running...\n");
        // 计算时间间隔
        currentTime = OSTimeGet() / 1000.0f; // 获取系统时间并转换为秒
        dt = currentTime - lastTime;
        lastTime = currentTime;

        // 防止dt过大或过小
        if(dt > 0.1f) dt = 0.01f;
        if(dt <= 0) dt = 0.01f;

        // 更新PID控制器
        OSSemPend(QuadDataMutex, 0, &err);
        OSSemPend(GY86DataMutex, 0, &err);
        MPU6050_GetData(&MPU6050_Data);
        InnerLoop_Update(&quad, &MPU6050_Data, dt);
        OSSemPost(GY86DataMutex);
        QuadPID_MotorControl(&quad);
        OSSemPost(QuadDataMutex);

        OSTimeDlyHMSM(0, 0, 0, 1);
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
