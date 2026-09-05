#include "stm32f4xx.h"                  // Device header
#include "PWM.h"

void PWM_Init(void)
{
    RCC_AHB1PeriphClockCmd(PWM_GPIO_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(PWM_TIM_RCC, ENABLE);

    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH1N_PINSOURCE, PWM_GPIO_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH1_PINSOURCE, PWM_GPIO_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH2N_PINSOURCE, PWM_GPIO_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH2_PINSOURCE, PWM_GPIO_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH3N_PINSOURCE, PWM_GPIO_AF);
    GPIO_PinAFConfig(PWM_GPIO_PORT, PWM_CH3_PINSOURCE, PWM_GPIO_AF);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = PWM_TIM_CH1_PIN | PWM_TIM_CH1N_PIN |
                                  PWM_TIM_CH2_PIN | PWM_TIM_CH2N_PIN |
                                  PWM_TIM_CH3_PIN | PWM_TIM_CH3N_PIN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(PWM_GPIO_PORT, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;                                 // 时钟分割
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;                 // 中心对齐计数模式，下溢时产生事件更新
    TIM_TimeBaseInitStructure.TIM_Period = PWM_TIM_PERIOD - 1;                                  // 自动重装载寄存器周期的值
    TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_TIM_PRESCALER - 1;                            // 时钟预分频数
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_Pulse = 0;                                                          // 初始比较值；启动PWM前应更新CCR，PWM2模式下0不等同于0%占空比
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                                           // 使用PWM2模式，与后续SVPWM占空比计算方式保持一致
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;                                   // 主通道高电平为有效电平
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;                                 // 互补通道高电平有效，反相和死区由TIM1硬件完成
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                               // 允许CHx主通道输出
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                             // 允许CHxN互补通道输出
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;                                // MOE关闭时，主通道空闲状态设为低电平
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;                              // MOE关闭时，互补通道空闲状态设为低电平
	TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);                                                 // 初始化U相主输出和互补输出
    TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);                                                 // 初始化V相主输出和互补输出
    TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure);                                                 // 初始化W相主输出和互补输出
    TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);                                                       // 保持MOE关闭，防止初始化完成后立即驱动功率管

    TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);                                        // CCR1先写入影子寄存器，在更新事件时统一生效
    TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);                                        // CCR2同步更新，避免三相占空比更新时间不同
    TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);                                        // CCR3同步更新，避免输出产生瞬时跳变
    TIM_ARRPreloadConfig(PWM_TIM, ENABLE);                                                      // ARR在更新事件时生效，避免周期中途改变

    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Disable;                                // 运行时关闭MOE后，不再由定时器维持输出电平
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Disable;                                // 定时器停止时，不强制定时器保持空闲输出
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;                                    // 不锁定BDTR，便于调试期间修改死区等参数
    TIM_BDTRInitStructure.TIM_DeadTime = 100;                                                   // 插入上下桥死区；168MHz、CKD_DIV1时约为595ns
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                                        // 暂不使用BKIN硬件刹车输入
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;                           // BKIN高电平有效；Break关闭时暂不起作用
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;                    // 故障解除后允许硬件自动恢复MOE
    TIM_BDTRConfig(PWM_TIM, &TIM_BDTRInitStructure);                                            // 将死区、刹车及输出控制参数写入TIM1

    TIM_Cmd(PWM_TIM, ENABLE);
}

void PWM_OutputEnable(void)
{
    TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
}

void PWM_OutputDisable(void)
{
    TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
}

void PWM_SetCompare_CH1(uint16_t value)
{
    if (value >= PWM_TIM_PERIOD)
        value = PWM_TIM_PERIOD - 1;
    TIM_SetCompare1(PWM_TIM, value);
}

void PWM_SetCompare_CH2(uint16_t value)
{
    if (value >= PWM_TIM_PERIOD)
        value = PWM_TIM_PERIOD - 1;
    TIM_SetCompare2(PWM_TIM, value);
}

void PWM_SetCompare_CH3(uint16_t value)
{
    if (value >= PWM_TIM_PERIOD)
        value = PWM_TIM_PERIOD - 1;
    TIM_SetCompare3(PWM_TIM, value);
}

void PWM_SetCompareAll(uint16_t value)
{
    if (value >= PWM_TIM_PERIOD)
        value = PWM_TIM_PERIOD - 1;
    TIM_SetCompare1(PWM_TIM, value);
    TIM_SetCompare2(PWM_TIM, value);
    TIM_SetCompare3(PWM_TIM, value);
}
