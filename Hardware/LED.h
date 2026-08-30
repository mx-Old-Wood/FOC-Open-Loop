#ifndef __LED_H__
#define __LED_H__

#define LED_RCC         RCC_AHB1Periph_GPIOD            // LED使用的GPIO时钟
#define LED_PORT        GPIOD                           // LED使用的GPIO端口
#define LED_PIN_BLUE    GPIO_Pin_11                     // 蓝色LED引脚
#define LED_PIN_GREEN   GPIO_Pin_12                     // 绿色LED引脚
#define LED_PIN_RED     GPIO_Pin_13                     // 红色LED引脚

typedef enum color                                      // LED颜色枚举类型
{
    NONE = 0,
    RED,
    GREEN,
    BLUE,
    CYAN,
    PURPLE,
    YELLOW,
    WHITE
} Color;

void LED_Init(void);
void LED_Set(Color color);

#endif
