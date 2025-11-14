#ifndef __MYTASKS_H__
#define __MYTASKS_H__

#include "Final.h"
#include "ucos_ii.h"

#define  TASK_STARTUP_PRIO  8   
#define  TASK_ATTITUDE_PRIO    5
#define  TASK_COM_PRIO      7
#define  TASK_INNER_PRIO    4
#define  TASK_OUTER_PRIO    6
//#define  TASK_DISPLAY_PRIO  8
#define  TASK_STK_SIZE      128

extern OS_STK Task_StartupStk[TASK_STK_SIZE];
extern OS_STK Task_AttitudeStk[TASK_STK_SIZE];
extern OS_STK Task_MotorStk[TASK_STK_SIZE];
extern OS_STK Task_COMStk[TASK_STK_SIZE];
//extern OS_STK Task_DisplayStk[TASK_STK_SIZE];

void Sem_Init(void);
void Task_Startup(void *p_arg);
void Task_Attitude(void *p_arg);
void Task_Outer(void *p_arg);
void Task_Inner(void *p_arg);
void Task_COM(void *p_arg);
//void Task_Display(void *p_arg);
void Display(void);

#endif
