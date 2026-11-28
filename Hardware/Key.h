#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>

#define KEY_HOLD            0x01    					//按住
#define KEY_DOWN            0x02    					//按下
#define KEY_UP              0x04    					//松开
#define KEY_SINGLE          0x08    					//单击
#define KEY_DOUBLE          0x10    					//双击
#define KEY_LONG            0x20    					//长按
#define KEY_REPEAT          0x40    					//重复

#define KEY_PRESSED         1       					//按键按下
#define KEY_RELEASE         0       					//按键松开

#define KEY_SCAN_TIME       30      					//按键扫描时间

#define KEY_TIME_DOUBLE     180     					//双击判定等待时间
#define KEY_TIME_LONG       1800    					//长按判定等待时间
#define KEY_TIME_REPEAT     120     					//重复触发间隔时间

typedef enum
{
    KEY1 = 0,                                           //Key1
    KEY2,                                               //Key2
    KEY3,                                               //Key3
    KEY_POWER,                                          //电源按键
    KEY_COUNT                                           //按键总数
} Key_ID;

typedef struct
{
    volatile uint8_t Key_Flag;                  	    //按键标志位
    uint8_t CurrState;                 				    //当前引脚状态
    uint8_t KeyState;                  				    //按键状态机状态
    uint16_t KeyTime;                  				    //按键计时
    uint8_t Acc;                    				    //每键独立的连发加速计数
} KEYS;

static KEYS Key[KEY_COUNT];                             //按键结构体数组
typedef struct
{
    uint32_t GPIO_RCC;
    GPIO_TypeDef *GPIO_Port;
    uint32_t GPIO_Pin;
} Key_GPIO;

static const Key_GPIO Key_GPIO_List[KEY_COUNT] = 
{
    [KEY1] = {RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_12},
    [KEY2] = {RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_13},
    [KEY3] = {RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_14},
    [KEY_POWER] = {RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_11},
};

void Key_Init(void);
uint8_t Key_Check(uint8_t KeyIdx, uint8_t Flag);
void Key_Clear(void);
void Key_Tick(void);

#endif
