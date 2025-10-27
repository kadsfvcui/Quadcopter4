#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "Delay.h"


//PA8,9,10,11
void GPIO_Config(void){
    RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;//使能
    
    GPIOA->MODER &=~(GPIO_MODER_MODER8|GPIO_MODER_MODER9|GPIO_MODER_MODER10|GPIO_MODER_MODER11);//清零
    GPIOA->MODER |=GPIO_MODER_MODER8_1|GPIO_MODER_MODER9_1|GPIO_MODER_MODER10_1|GPIO_MODER_MODER11_1;//复用
    GPIOA->OTYPER &=~(GPIO_OTYPER_OT_8|GPIO_OTYPER_OT_9|GPIO_OTYPER_OT_10|GPIO_OTYPER_OT_11);//推挽
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR8|GPIO_OSPEEDER_OSPEEDR9|GPIO_OSPEEDER_OSPEEDR10|GPIO_OSPEEDER_OSPEEDR11); // 清零
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1|GPIO_OSPEEDER_OSPEEDR9_1|GPIO_OSPEEDER_OSPEEDR10_1|GPIO_OSPEEDER_OSPEEDR11_1; // 高速
    GPIOA->AFR[1] &= ~((0xF << (4 * 0)) | (0xF << (4 * 1)) | (0xF << (4 * 2)) | (0xF << (4 * 3)));
    GPIOA->AFR[1] |= (1 << (4 * 0)) | (1 << (4 * 1)) | (1 << (4 * 2)) | (1 << (4 * 3)); // 设置为 AF1（TIM1）
}

void TIM1_Config(void){
    RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;//使能时钟
    
    TIM1->PSC=100-1;
    TIM1->ARR=20000-1;
    TIM1->RCR=0;
    
    
    TIM1->CR1&=~TIM_CCER_CC1E;
    TIM1->CR1&=~TIM_CR1_DIR;//向上计数
    TIM1->CR1 |= TIM_CR1_ARPE;//使能ARR
    
    TIM1->CCMR1 &= ~TIM_CCMR1_CC1S;//输出
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE; //使能预装载
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M; 
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2; //PWM1
    
    TIM1->CCMR1 &= ~TIM_CCMR1_CC2S;//输出
    TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
    TIM1->CCMR1 &= ~TIM_CCMR1_OC2M; 
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2; //PWM1

    TIM1->CCMR2 &=~TIM_CCMR2_CC3S; 
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM1->CCMR2 &=~TIM_CCMR2_OC3M;
    TIM1->CCMR2 |=TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2;
    
    TIM1->CCMR2 &=~TIM_CCMR2_CC4S;
    TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
    TIM1->CCMR2 &=~TIM_CCMR2_OC4M;
    TIM1->CCMR2 |=TIM_CCMR2_OC4M_1|TIM_CCMR2_OC4M_2;

    TIM1->CCR1 = 1500; 
    TIM1->CCR2 = 1500;
    TIM1->CCR3 = 1500;
    TIM1->CCR4 = 1500;


    TIM1->CCER &=~(TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC3P|TIM_CCER_CC4P);//高电平有效
    TIM1->CCER|=TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;//使能输出
    
	TIM1->BDTR |= TIM_BDTR_MOE;   // 使能主输出（高级定时器需要此步骤）
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;     // 启动定时器

}

void PWM_SetCompare(int motor, uint16_t compare) {
    switch (motor) {
        case 1:
            TIM1->CCR1 = compare;
            break;
        case 2:
            TIM1->CCR2 = compare;
            break;
        case 3:
            TIM1->CCR3 = compare;
            break;
        case 4:
            TIM1->CCR4 = compare;
            break;
        default:
            break;
    }
}


void MOTOR_Init(void){
    GPIO_Config();
    TIM1_Config();
}


void MOTOR_SetSpeed(int moter,int8_t Speed)  
{
    uint16_t compare;
    
    // 将Speed从 -100 到 100 映射到占空比范围 1000 (5%) 到 2000 (10%)
    if (Speed > 100) Speed = 100;
    if (Speed < 0) Speed = 0;

    // 将速度范围线性映射到占空比范围
    compare = 1000 + (Speed * 10); // 将Speed映射到1000到2000

    PWM_SetCompare(moter, compare); 
}

void Unlock_ESC(int motor){
    PWM_SetCompare(motor,2000);
    Delay_s(5);
    PWM_SetCompare(motor,1000);
    Delay_s(1);
}

void Unlock_ALL_ESC(void)
{
	PWM_SetCompare(1, 2000);
	PWM_SetCompare(2, 2000);
	PWM_SetCompare(3, 2000);
	PWM_SetCompare(4, 2000);
	Delay_s(5);
	PWM_SetCompare(1, 1000);
	PWM_SetCompare(2, 1000);
	PWM_SetCompare(3, 1000);
	PWM_SetCompare(4, 1000);
	Delay_s(1);
}
