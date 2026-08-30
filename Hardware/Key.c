#include "stm32f4xx.h"                  // Device header
#include "Key.h"

static KEYS Key[KEY_COUNT];             //按键结构体数组

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

void Key_Init(void)
{
    uint32_t GPIO_Rcc = 0;
    for (uint8_t i = 0; i < KEY_COUNT; i++)
        GPIO_Rcc |= Key_GPIO_List[i].GPIO_RCC;
    RCC_AHB1PeriphClockCmd(GPIO_Rcc, ENABLE);
    
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        GPIO_InitStructure.GPIO_Pin = Key_GPIO_List[i].GPIO_Pin;
        GPIO_Init(Key_GPIO_List[i].GPIO_Port, &GPIO_InitStructure);
    }
    Key_Clear();
}

static uint8_t Key_GetState(uint8_t KeyIdx)
{
    if (KeyIdx >= KEY_COUNT)
        return 0;
    if (GPIO_ReadInputDataBit(Key_GPIO_List[KeyIdx].GPIO_Port, Key_GPIO_List[KeyIdx].GPIO_Pin) == 0)
        return KEY_PRESSED;

    return KEY_RELEASE;
}

uint8_t Key_Check(uint8_t KeyIdx, uint8_t Flag)
{
    uint8_t result = 0;

    if (KeyIdx >= KEY_COUNT)
        return 0;

    if (Key[KeyIdx].Key_Flag & Flag)
    {
        result = 1;
        if (Flag != KEY_HOLD)
            Key[KeyIdx].Key_Flag &= (uint8_t)~Flag;
    }
    return result;
}

void Key_Clear(void)
{

    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        Key[i].Key_Flag   = 0;
        Key[i].CurrState  = KEY_RELEASE;
        Key[i].KeyState   = 0;
        Key[i].KeyTime    = 0;
        Key[i].Acc        = 0;
    }
}

void Key_Tick(void)
{
    uint8_t i;
    static uint8_t Count;

    for (i = 0; i < KEY_COUNT; i++)
    {
        if(Key[i].KeyTime)
            Key[i].KeyTime--;
    }

    Count++;
    if (Count >= KEY_SCAN_TIME)
    {
        Count = 0;
        for (i = 0; i < KEY_COUNT; i++)
        {
            Key[i].CurrState = Key_GetState(i);
            switch (Key[i].KeyState)
            {
                case 0:
                    if (Key[i].CurrState == KEY_PRESSED)
                    {
                        Key[i].KeyTime = KEY_TIME_LONG;
                        Key[i].Key_Flag |= KEY_HOLD;
                        Key[i].Key_Flag |= KEY_DOWN;
                        Key[i].KeyState = 1;
                    }
                    else
                        Key[i].Key_Flag &= ~KEY_HOLD;
                    break;
                case 1:
                    if (Key[i].CurrState == KEY_RELEASE)
                    {
                        Key[i].KeyTime = KEY_TIME_DOUBLE;
                        Key[i].Key_Flag &= ~KEY_HOLD;
                        Key[i].Key_Flag |= KEY_UP;
                        Key[i].KeyState = 2;
                    }
                    else if (Key[i].KeyTime == 0)
                    {
                        Key[i].KeyTime = KEY_TIME_REPEAT;
                        Key[i].Key_Flag |= KEY_LONG;
                        Key[i].KeyState = 4;
                    }
                    break;
                case 2:
                    if (Key[i].CurrState == KEY_PRESSED)
                    {
                        Key[i].Key_Flag |= KEY_HOLD;
                        Key[i].Key_Flag |= KEY_DOWN;
                        Key[i].Key_Flag |= KEY_DOUBLE;
                        Key[i].KeyState = 3;
                    }
                    else if (Key[i].KeyTime == 0)
                    {
                        Key[i].Key_Flag &= ~KEY_HOLD;
                        Key[i].Key_Flag |= KEY_SINGLE;
                        Key[i].KeyState = 0;
                    }
                    break;
                case 3:
                    if (Key[i].CurrState == KEY_RELEASE)
                    {
                        Key[i].Key_Flag &= ~KEY_HOLD;
                        Key[i].Key_Flag |= KEY_UP;
                        Key[i].KeyState = 0;
                    }
                    break;
                case 4:
                    if (Key[i].CurrState == KEY_RELEASE)
                    {
                        Key[i].Key_Flag &= ~KEY_HOLD;
                        Key[i].Key_Flag |= KEY_UP;
                        Key[i].KeyState = 0;
                        Key[i].Acc = 0;
                    }
                    else if (Key[i].KeyTime == 0)
                    {
                        Key[i].KeyTime = KEY_TIME_REPEAT - Key[i].Acc * KEY_SCAN_TIME;
                        Key[i].Key_Flag |= KEY_REPEAT;
                        Key[i].Acc++;
                        Key[i].Acc %= 4;
                    }
                    break;
                default:
                    break;
            }        
        }

    }
}
