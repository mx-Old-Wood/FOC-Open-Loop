#ifndef __ENCODER_H__
#define __ENCODER_H__

#define ENCODER_TIM         TIM3                    // 编码器使用的定时器
#define ENCODER_TIM_RCC     RCC_APB1Periph_TIM3     // 编码器定时器时钟
#define ENCODER_GPIO_RCC    RCC_AHB1Periph_GPIOA    // 编码器GPIO时钟
#define ENCODER_GPIO_PORT   GPIOA                   // 编码器GPIO端口
#define ENCODER_GPIO_AF     GPIO_AF_TIM3            // 编码器GPIO复用功能
#define ENCODER_PIN_A       GPIO_Pin_6              // A相信号引脚
#define ENCODER_PIN_B       GPIO_Pin_7              // B相信号引脚
#define ENCODER_A_PINSOURCE GPIO_PinSource6         // A相引脚源
#define ENCODER_B_PINSOURCE GPIO_PinSource7         // B相引脚源
#define ENCODER_TIM_ARR     65535                   // 16位计数器自动重装值
#define ENCODER_IC_FILTER   7                       // A/B两路输入滤波配置

void Encoder_Init(void);
uint16_t Encoder_GetCount(void);

#endif
