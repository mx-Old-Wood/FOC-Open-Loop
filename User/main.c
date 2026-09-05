#include "stm32f4xx.h"
#include "Delay.h"
#include "VOFA.h"
#include "LED.h"
#include "PWM.h"

static const uint16_t pwm_test_compare[] = 
{
	PWM_TIM_PERIOD * 90 / 100,
	PWM_TIM_PERIOD * 75 / 100,
	PWM_TIM_PERIOD * 50 / 100,
	PWM_TIM_PERIOD * 25 / 100,
	PWM_TIM_PERIOD * 10 / 100
};

int main(void)
{
	Delay_Init(168);//初始化延时函数（F407主频168MHz）
	VOFA_Init(115200);//初始化VOFA串口通信（波特率115200）
	LED_Init();//初始化LED
	PWM_Init();//初始化PWM

	float data[10] = {0};
	uint32_t tick = 0;

	PWM_SetCompareAll(PWM_TIM_PERIOD / 2);
	Delay_ms(1000);
	PWM_OutputEnable();
	LED_Set(GREEN);

	while(1)
	{
		VOFA_Process();//处理VOFA串口接收数据
		if (tick % 100 == 0) //每隔一段时间发送一次数据
		{
			data[0] = (float)TIM1 -> CNT;							// 当前计数值
			data[1] = (float)TIM1 -> ARR;							// 自动重装载值
			data[2] = (float)TIM1 -> CCR1;							// 通道1比较值
			data[3] = (float)TIM1 -> CCR2;							// 通道2比较值
			data[4] = (float)TIM1 -> CCR3;							// 通道3比较值
			data[5] = (TIM1 -> CR1 & TIM_CR1_CEN) ? 1.0f : 0.0f;	// 定时器使能
			data[6] = (TIM1 -> BDTR & TIM_BDTR_MOE) ? 1.0f : 0.0f;	// 主输出使能
			data[7] = (float)(TIM1 -> BDTR & 0xFF);					// 死区发生器DTG配置值
			data[8] = (float)TIM1 -> CCER;							// 通道1、2、3使能
			VOFA(data, 9);//发送数据
		}
		if (tick % 2000 == 0 && tick != 0)
		{
			PWM_SetCompareAll(pwm_test_compare[tick / 2000 - 1]);
			if (tick / 2000 >= 5)
				tick = 0;
		}

		tick++;
		Delay_ms(1); //延时1ms
	}
}
