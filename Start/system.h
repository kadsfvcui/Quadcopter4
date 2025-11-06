#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#define RCC_BASE 0x40023800
#define RCC_CR (*(volatile unsigned int *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR (*(volatile unsigned int *)(RCC_BASE + 0x04))
#define RCC_CFGR (*(volatile unsigned int *)(RCC_BASE + 0x08))

#define FLASH_BASE 0x40023C00
#define FLASH_ACR (*(volatile unsigned int *)(FLASH_BASE + 0x00))

#define STK_BASE 0xE000E010
#define STK_CTRL (*(volatile unsigned int *)(STK_BASE + 0x00))
#define STK_LOAD (*(volatile unsigned int *)(STK_BASE + 0x04))
#define STK_VAL (*(volatile unsigned int *)(STK_BASE + 0x08))
#define STK_CALIB (*(volatile unsigned int *)(STK_BASE + 0x0C))

#define SCB_SHPR3 (*(volatile unsigned int *)(0xE000ED20))

void SystemInit(void);

#endif
