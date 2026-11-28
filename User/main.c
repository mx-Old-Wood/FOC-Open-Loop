#include "stm32f4xx.h"
#include "FocMath.h"
#include "Encoder.h"
#include "Delay.h"
#include "VOFA.h"
#include "LED.h"
#include "PWM.h"
#include "ADC.h"
#include "PID.h"

/*
static const uint16_t pwm_test_compare[] = 
{
	PWM_TIM_PERIOD * 90 / 100,
	PWM_TIM_PERIOD * 75 / 100,
	PWM_TIM_PERIOD * 50 / 100,
	PWM_TIM_PERIOD * 25 / 100,
	PWM_TIM_PERIOD * 10 / 100
};
*/

PID_Pos pi_id = {.p = 1.0f, .i = 0.05f, .d = 0.0f, .set = 0.0f, .actual = 0.25f, .point_limits = 10.0f};
PID_Pos pi_iq = {.p = 1.0f, .i = 0.05f, .d = 0.0f, .set = 0.5f, .actual = 0.125f, .point_limits = 10.0f};
const float sim_ts = 0.0001f;
const float sim_r = 1.0f;
const float sim_l = 0.01f;
uint16_t test_step = 0;

int main(void)
{
	Delay_Init(168);		//初始化延时函数（F407主频168MHz）
	VOFA_Init(115200);		//初始化VOFA串口通信（波特率115200）
	LED_Init();				//初始化LED
	//PWM_Init();				//初始化PWM
	AD_Init();				//初始化ADC
    Encoder_Init();

	float data[14] = {0};
	uint32_t tick = 0;

	IPark voltage = {0};
	SVPWM modulation = {0};

	/*
	PWM_SetCompareAll(PWM_TIM_PERIOD / 2);
	Delay_ms(1000);
	PWM_OutputEnable();
	LED_Set(GREEN);
	*/

	while(1)
	{
		VOFA_Process();//处理VOFA串口接收数据
		pi_id.set = 0.25f;
		pi_iq.set = (test_step < 200) ? 0.5f : -0.5f;
		const float id_feedback = pi_id.actual;
		const float iq_feedback = pi_iq.actual;
		PID_Position(&pi_id);
		PID_Position(&pi_iq);

		voltage.Ds = pi_id.out;
		voltage.Qs = pi_iq.out;
		voltage.Theta = 0;
		IParkCale(&voltage);

		modulation.Ualpha = voltage.Alpha;
		modulation.Ubeta = voltage.Beta;
		const int valid = SvpwmCale(&modulation, 12.0f);

		if (valid)
		{
			const float duty_mean = (modulation.Ta + modulation.Tb + modulation.Tc) / 3.0f;
			const float vd_applied = 12 * (modulation.Ta - duty_mean);
			const float vq_applied = 12 * (modulation.Tb - modulation.Tc) * 0.577350269f;
			pi_id.actual = id_feedback + (sim_ts / sim_l) * (vd_applied - sim_r * id_feedback);
			pi_iq.actual = iq_feedback + (sim_ts / sim_l) * (vq_applied - sim_r * iq_feedback);
		}

		const int32_t count = (int32_t)Encoder_GetCount;

		data[0] = (float)count;
		data[1] = (float)(count - 32768);
		VOFA(data, 2);

        // data[0] = pi_id.set;
        // data[1] = id_feedback;
        // data[2] = pi_iq.set;
		// data[3] = iq_feedback;
		// data[4] = pi_id.out;
		// data[5] = pi_iq.out;
		// data[6] = (float)valid;
		// data[7] = (float)test_step * sim_ts;
		// VOFA(data, 8);

		++test_step;

		if (tick % 100 == 0) //每隔一段时间发送一次数据
		{
//			data[0] = (float)TIM1 -> CNT;							// 当前计数值
//			data[1] = (float)TIM1 -> ARR;							// 自动重装载值
//			data[2] = (float)TIM1 -> CCR1;							// 通道1比较值
//			data[3] = (float)TIM1 -> CCR2;							// 通道2比较值
//			data[4] = (float)TIM1 -> CCR3;							// 通道3比较值
//			data[5] = (TIM1 -> CR1 & TIM_CR1_CEN) ? 1.0f : 0.0f;	// 定时器使能
//			data[6] = (TIM1 -> BDTR & TIM_BDTR_MOE) ? 1.0f : 0.0f;	// 主输出使能
//			data[7] = (float)(TIM1 -> BDTR & 0xFF);					// 死区发生器DTG配置值
//			data[8] = (float)TIM1 -> CCER;							// 通道1、2、3使能
//			data[9] = (float)AD_GetValue(POT);
//			data[10] = (float)AD_GetValue(I_W);
//			data[11] = (float)AD_GetValue(Ubus);
//			data[12] = (float)AD_GetValue(Temp);
//			data[13] = (float)AD_GetValue(I_U);
//			VOFA(data, 14);//发送数据
		}
		/*
		if (tick % 2000 == 0 && tick != 0)
		{
			PWM_SetCompareAll(pwm_test_compare[tick / 2000 - 1]);
			if (tick / 2000 >= 5)
				tick = 0;
		}
		*/

		tick++;
		Delay_ms(1); //延时1ms
	}
}
