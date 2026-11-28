#include "stm32f4xx.h"                  // Device header
#include "LED.h"


/**
  *  @brief		初始化LED
  *  @param		无
  *  @retval	无
  *  @memo		无
  **/
  
void LED_Init(void)
{
	RCC_AHB1PeriphClockCmd(LED_GPIO_RCC, ENABLE); 

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = LED_PIN_BLUE | LED_PIN_GREEN | LED_PIN_RED;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(LED_PORT, LED_PIN_BLUE);
    GPIO_SetBits(LED_PORT, LED_PIN_GREEN);
    GPIO_SetBits(LED_PORT, LED_PIN_RED);
}


/**
  *  @brief		设置LED颜色
  *  @param		color: LED颜色
  *  @retval	无
  *  @memo		无
  **/
  
void LED_Set(Color color)
{
	switch(color)
	{
		case NONE:
			GPIO_SetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_SetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_SetBits(LED_PORT, LED_PIN_RED);
			break;
		case RED:
			GPIO_SetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_SetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_ResetBits(LED_PORT, LED_PIN_RED);
			break;
		case GREEN:
			GPIO_SetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_ResetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_SetBits(LED_PORT, LED_PIN_RED);
			break;
		case BLUE:
			GPIO_ResetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_SetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_SetBits(LED_PORT, LED_PIN_RED);
			break;
		case CYAN:
			GPIO_ResetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_ResetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_SetBits(LED_PORT, LED_PIN_RED);
			break;
		case PURPLE:
			GPIO_ResetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_SetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_ResetBits(LED_PORT, LED_PIN_RED);
			break;
		case YELLOW:
			GPIO_SetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_ResetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_ResetBits(LED_PORT, LED_PIN_RED);
			break;
		case WHITE:
			GPIO_ResetBits(LED_PORT, LED_PIN_BLUE);
			GPIO_ResetBits(LED_PORT, LED_PIN_GREEN);
			GPIO_ResetBits(LED_PORT, LED_PIN_RED);
			break;
		default:
			break;
	}
}
