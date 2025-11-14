#include "tim_sched.h"

extern OS_EVENT *Sem_InnerLoop;
extern OS_EVENT *Sem_Attitude;
extern OS_EVENT *Sem_OuterLoop;

// TIM2中断处理函数
void TIM2_IRQHandler(void)
{
    // 进入中断服务程序
    OSIntEnter();

    static uint8_t cnt = 0;

    // 检查TIM2更新中断是否发生
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        OSSemPost(Sem_InnerLoop);
        cnt++;
        if(cnt == 10){
            cnt = 0;
            OSSemPost(Sem_Attitude);
            OSSemPost(Sem_OuterLoop);
        }

        // 清除TIM2更新中断标志位
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }

    // 退出中断服务程序
    OSIntExit();
}

// TIM2定时器初始化函数
// 配置为1ms定时中断
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 设置定时器基本参数
    TIM_TimeBaseStructure.TIM_Period = 50 - 1; // 1ms定时
    TIM_TimeBaseStructure.TIM_Prescaler = 167; // 84MHz计数频率 (168MHz/168)
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 使能更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 使能TIM2定时器
    TIM_Cmd(TIM2, ENABLE);
}
