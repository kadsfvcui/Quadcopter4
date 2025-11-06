#include "system.h"

void SystemInit(void) {
    // 1. 启用HSE并禁用旁路模式
    RCC_CR |= (1 << 16);       // 开启HSE
    RCC_CR &= ~(1 << 18);      // 禁用旁路模式
    while (!(RCC_CR & (1 << 17))); // 等待HSE就绪

    // 2. 设置Flash等待周期
    FLASH_ACR |= 0x02;         // 2等待周期

    // 3. 配置PLL参数
    RCC_PLLCFGR = 0;
    RCC_PLLCFGR |= (8 << 0);      // PLL_M=8
    RCC_PLLCFGR |= (336 << 6);    // PLL_N=336
    RCC_PLLCFGR &= ~(0x3 << 16);  // 清除PLLP旧值
    RCC_PLLCFGR |= (0x1 << 16);   // PLL_P=4分频
    RCC_PLLCFGR |= (1 << 22);     // PLL源选择HSE

    // 4. 启用PLL并等待锁定
    RCC_CR |= (1 << 24);       // 开启PLL
    while (!(RCC_CR & (1 << 25))); // 等待PLL就绪

    // 5. 配置总线分频
    RCC_CFGR &= ~(0xF << 4);   // AHB分频=1（84MHz）
    RCC_CFGR |= (4 << 10);      // APB1分频=2（42MHz）
    RCC_CFGR &= ~(0x7 << 13);   // APB2分频=1（84MHz）

    // 6. 切换系统时钟到PLL
    RCC_CFGR &= ~0x03;         // 清除SW位
    RCC_CFGR |= 0x02;          // SW=0b10（PLL作为系统时钟）
    while ((RCC_CFGR & 0x0C) != 0x08); // 等待切换完成
}
