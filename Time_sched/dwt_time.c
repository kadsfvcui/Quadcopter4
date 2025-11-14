#include "dwt_time.h"

#define SystemCoreClock 84000000

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 获取当前时间（秒）
 * @note 通过DWT（数据观察与跟踪）计数器获取系统运行时间
 * @return float 返回从系统启动开始经过的时间（秒）
 */
float DWT_GetTime(void)
{
    return (float)DWT->CYCCNT / SystemCoreClock;  // 将DWT计数器的值转换为秒
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tick = (SystemCoreClock / 1000000) * us;
    while ((DWT->CYCCNT - start) < tick);
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tick = (SystemCoreClock / 1000) * ms;
    while ((DWT->CYCCNT - start) < tick);
}

void Delay_s(uint32_t s)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t tick = SystemCoreClock * s;
    while ((DWT->CYCCNT - start) < tick);
}
