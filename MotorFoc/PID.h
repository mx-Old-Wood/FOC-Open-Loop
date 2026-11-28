#ifndef __PID_H__
#define __PID_H__

/*===================================增量式PID(标准)===================================*/
typedef struct PID_Increment
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
}PID_Inc;

/*===================================增量式PID-积分分离===================================*/
typedef struct PID_Increment_Separation
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
	float err_threshold;         	// 积分分离阈值
}PID_Inc_Sep;

/*===================================增量式PID-梯形积分===================================*/
typedef struct PID_Increment_Trapezoidal
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
}PID_Inc_Trap;

/*===================================增量式PID-变速积分===================================*/
typedef struct PID_Increment_VariableIntegral
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
	float err_threshold_max;     	// 积分关闭上阈值
	float err_threshold_min;     	// 积分全开下阈值
}PID_Inc_VarInt;

/*===================================增量式PID-抗积分饱和===================================*/
typedef struct PID_Increment_AntiWindup
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
	float out_max;               	// 输出上限
	float out_min;               	// 输出下限
}PID_Inc_AW;

/*===================================增量式PID-不完全微分===================================*/
typedef struct PID_Increment_IncompleteDerivative
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float d_coeff;               	// 微分滤波系数, 取值(0, 1]
	float set;                   	// 设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float l_differential;        	// 上次微分项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
}PID_Inc_ID;

/*===================================增量式PID-前馈===================================*/
typedef struct PID_Increment_FeedForward
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float kf;                    	// 前馈系数
	float set;                   	// 设定值
	float l_set;                 	// 上次设定值
	float actual;                	// 实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float ll_err;                	// 上上次误差
	float increment;             	// 输出增量
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float feedforward;           	// 前馈项
	float out;                   	// 当前输出
	float point_limits;          	// 单次增量限幅
}PID_Inc_FF;

/*===================================位置式PID===================================*/
typedef struct PID_Positional
{
	float p;                     	// 比例系数
	float i;                     	// 积分系数
	float d;                     	// 微分系数
	float integration_K;         	// 积分速率系数
	float set;                   	// 设定值
	float actual;                	// 实际值
	float l_actual;              	// 上次实际值
	float err;                   	// 当前误差
	float l_err;                 	// 上次误差
	float increment;             	// 误差累加值
	float proportion;            	// 比例项
	float integral;              	// 积分项
	float differential;          	// 微分项
	float out;                   	// 当前输出
	float point_limits;          	// 积分累加限幅
}PID_Pos;

void PID_Increment(PID_Inc *PID);
void PID_Increment_Separation(PID_Inc_Sep *PID);
void PID_Increment_Trapezoidal(PID_Inc_Trap *PID);
void PID_Increment_VariableIntegral(PID_Inc_VarInt *PID);
void PID_Increment_AntiWindup(PID_Inc_AW *PID);
void PID_Increment_IncompleteDerivative(PID_Inc_ID *PID);
void PID_Increment_FeedForward(PID_Inc_FF *PID);
void PID_Position(PID_Pos *PID);
void PID_Position_DM(PID_Pos *PID);
float limit_value(float value, float positive_value, float negative_value);

#endif
