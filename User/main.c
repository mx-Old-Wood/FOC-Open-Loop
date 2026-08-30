#include "stm32f4xx.h"
#include "Delay.h"
#include "VOFA.h"
#include "LED.h"

int main(void)
{
	Delay_Init(168);//初始化延时函数（F407主频168MHz）
	VOFA_Init(115200);//初始化VOFA串口通信（波特率115200）
	LED_Init();//初始化LED

	float data[10] = {1.0f, 2.0f, 3.0f, 4.0f};
	uint32_t tick = 0;
	
	while(1)
	{
		VOFA_Process();//处理VOFA串口接收数据
		const VOFA_CmdStatus* cmd_status = VOFA_GetCmdStatus();
		static uint32_t last_total_count = 0;
		if (tick++ % 100 == 0) //每隔一段时间发送一次数据
		{
			data[0] += 1.0f;
			if (data[0] > 10.0f)
				data[0] = -10.0f;
			data[1] += 0.5f;
			data[2] = data[0] * 2.0f;
			data[3] = data[1] * 0.2f;
			data[4] = cmd_status -> total_count;
			data[5] = cmd_status -> success_count;
			data[6] = cmd_status -> invalid_length_count;
			data[7] = cmd_status -> invalid_command_count;
			data[8] = cmd_status -> invalid_value_count;
			data[9] = cmd_status -> unknown_error_count;
			VOFA(data, 10);//发送数据
		}
		tick++;

		if (cmd_status -> total_count != last_total_count)
		{
			last_total_count = cmd_status -> total_count;
			if (cmd_status -> last_result == VOFA_CMD_RESULT_OK)
			{
				LED_Set(GREEN); //命令执行成功，点亮绿色LED
			}
			else if (cmd_status -> last_result == VOFA_CMD_RESULT_INVALID_LENGTH)
			{
				LED_Set(YELLOW); //命令执行失败，点亮黄色LED
			}
			else if (cmd_status -> last_result == VOFA_CMD_RESULT_INVALID_COMMAND)
			{
				LED_Set(PURPLE); //命令执行失败，点亮紫色LED
			}
			else if (cmd_status -> last_result == VOFA_CMD_RESULT_INVALID_VALUE)
			{
				LED_Set(CYAN); //命令执行失败，点亮青色LED
			}
			else if (cmd_status -> last_result == VOFA_CMD_RESULT_UNKNOWN_ERROR)
			{
				LED_Set(BLUE); //命令执行失败，点亮蓝色LED
			}
			else
			{
				LED_Set(RED); //命令执行失败，点亮红色LED
			}
		}
		Delay_ms(1); //延时10ms
	}
}
