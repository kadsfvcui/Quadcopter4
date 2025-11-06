#include "mytasks.h"

int main(void)
{
	
	OS_CPU_ExceptStkBase = &OS_CPU_ExceptStk[OS_CPU_EXCEPT_STK_SIZE - 1];	//设置MSP地址
	OS_CPU_SysTickInit(84000);	//初始化Systick，设置tick为1ms
	OS_TRACE_INIT();	//初始化SystemView
	OS_TRACE_START();	//启动SystemView
	OSInit();	//初始化ucOS
	OSTaskCreate(Task_Startup, (void *)0, (OS_STK *)&Task_StartupStk[TASK_STK_SIZE - 1], TASK_STARTUP_PRIO);
	OSStart();	//启动ucOS
	while(1)
	{
		
	}
}
