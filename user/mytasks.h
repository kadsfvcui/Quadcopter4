#ifndef __MYTASKS_H__
#define __MYTASKS_H__

#include "Final.h"
#include "ucos_ii.h"

#define  TASK_STARTUP_PRIO  4   
#define  TASK_ANGLE_PRIO    5
#define  TASK_MOTOR_PRIO    6
#define  TASK_COM_PRIO      7
//#define  TASK_DISPLAY_PRIO  8
#define  TASK_STK_SIZE      128

extern OS_STK Task_StartupStk[TASK_STK_SIZE];
extern OS_STK Task_AngleStk[TASK_STK_SIZE];
extern OS_STK Task_MotorStk[TASK_STK_SIZE];
extern OS_STK Task_COMStk[TASK_STK_SIZE];
//extern OS_STK Task_DisplayStk[TASK_STK_SIZE];

void Mutex_Init(void);
void Task_Startup(void *p_arg);
void Task_Angle(void *p_arg);
void Task_Motor(void *p_arg);
void Task_COM(void *p_arg);
//void Task_Display(void *p_arg);
void Display(void);

#endif
