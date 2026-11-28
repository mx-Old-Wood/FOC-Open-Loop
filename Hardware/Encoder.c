#include "stm32f4xx.h"                  // Device header
#include "Encoder.h"

void Encoder_Init(void)
{
    RCC_AHB1PeriphClockCmd(ENCODER_GPIO_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(ENCODER_TIM_RCC, ENABLE);

    GPIO_PinAFConfig(ENCODER_GPIO_PORT, ENCODER_A_PINSOURCE, ENCODER_GPIO_AF);
    GPIO_PinAFConfig(ENCODER_GPIO_PORT, ENCODER_B_PINSOURCE, ENCODER_GPIO_AF);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = ENCODER_PIN_A | ENCODER_PIN_B;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ENCODER_GPIO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = ENCODER_TIM_ARR;
    TIM_TimeBaseInit(ENCODER_TIM, &TIM_TimeBaseInitStructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = ENCODER_IC_FILTER;

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(ENCODER_TIM, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInit(ENCODER_TIM, &TIM_ICInitStructure);

    TIM_EncoderInterfaceConfig(ENCODER_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
    TIM_SetCounter(ENCODER_TIM, 0);
    TIM_Cmd(ENCODER_TIM, ENABLE);
}

uint16_t Encoder_GetCount(void)
{
    return (uint16_t)TIM_GetCounter(ENCODER_TIM);
}
