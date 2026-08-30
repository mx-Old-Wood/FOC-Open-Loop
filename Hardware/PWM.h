#ifndef __PWM_H__
#define __PWM_H__

#define PWM_TIM             TIM1                        // PWM使用的定时器
#define PWM_TIM_RCC         RCC_APB2Periph_TIM1         // PWM使用的定时器时钟
#define PWM_GPIO_PORT       GPIOE                       // PWM使用的定时器GPIO端口
#define PWM_GPIO_RCC        RCC_AHB1Periph_GPIOE        // PWM使用的定时器GPIO时钟
#define PWM_GPIO_AF         GPIO_AF_TIM1                // PWM使用的定时器GPIO复用功能
#define PWM_TIM_CH1_PIN     GPIO_Pin_9                  // PWM通道1主输出引脚
#define PWM_TIM_CH1N_PIN    GPIO_Pin_8                  // PWM通道1互补输出引脚
#define PWM_TIM_CH2_PIN     GPIO_Pin_11                 // PWM通道2主输出引脚
#define PWM_TIM_CH2N_PIN    GPIO_Pin_10                 // PWM通道2互补输出引脚
#define PWM_TIM_CH3_PIN     GPIO_Pin_13                 // PWM通道3主输出引脚
#define PWM_TIM_CH3N_PIN    GPIO_Pin_12                 // PWM通道3互补输出引脚
#define PWM_CH1_PINSOURCE   GPIO_PinSource9             // PWM通道1主输出引脚源
#define PWM_CH1N_PINSOURCE  GPIO_PinSource8             // PWM通道1互补输出引脚源
#define PWM_CH2_PINSOURCE   GPIO_PinSource11            // PWM通道2主输出引脚源
#define PWM_CH2N_PINSOURCE  GPIO_PinSource10            // PWM通道2互补输出引脚源
#define PWM_CH3_PINSOURCE   GPIO_PinSource13            // PWM通道3主输出引脚源
#define PWM_CH3N_PINSOURCE  GPIO_PinSource12            // PWM通道3互补输出引脚源
#define PWM_TIM_PERIOD      8400                        // PWM使用的定时器周期
#define PWM_TIM_PRESCALER   1                           // PWM使用的定时器预分频值

void PWM_Init(void);
void PWM_OutputEnable(void);
void PWM_OutputDisable(void);
void PWM_SetCompare_CH1(uint16_t value);
void PWM_SetCompare_CH2(uint16_t value);
void PWM_SetCompare_CH3(uint16_t value);

#endif
