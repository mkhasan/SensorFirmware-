
#include "stm32f1xx_hal.h"

/* Include my libraries here */
#include "defines.h"
//nclude "tm_stm32_disco.h"
//nclude "tm_stm32_delay.h"
 
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

volatile unsigned int Timer2_Counter=0;
 
 
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2,TIM_IT_CC1 ) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 ); // Clear the interrupt flag
        Timer2_Counter++;
    }
}
 
void init_Timer2()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
     
    /* TIM2 Clock Enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
     
    /* Enable TIM2 Global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
     
    /* TIM2 Initialize */   
    TIM_TimeBaseStructure.TIM_Period=1000-1; // 1kHz
    TIM_TimeBaseStructure.TIM_Prescaler=24-1; // 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision=0;
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
     
    /* TIM2 Enale */
    TIM_Cmd(TIM2,ENABLE);
    TIM_ITConfig(TIM2,TIM_IT_CC1 , ENABLE); // interrupt enable
}
 
void delay_ms(unsigned int del)
{
    Timer2_Counter=0;
    while(Timer2_Counter < del);
}
 
int main()
{
    SystemInit();
    //init_port();
    init_Timer2();
     
    while(1)
    {
        //GPIOA->BRR = GPIO_Pin_0;  // PA0 OFF
        delay_ms(500);
        //GPIOA->BSRR = GPIO_Pin_0; // PA0 ON
        delay_ms(500);
    }


}