#include "PID.h"

/*
 * PID控制算法库
 * 包含增量式、位置式共9种PID算法实现及通用限幅函数
 *
 * 【增量式PID通用公式】(每个周期)
 *   err = set - actual
 *   proportion  = p * (err - l_err)
 *   integral    = i * err
 *   differential= d * (err - 2*l_err + ll_err)
 *   increment   = proportion + integral + differential
 *   out        += increment
 *
 *   各变体仅修改上述公式中的 integral / differential / increment 行, 其余逻辑保持不变
 *   变体间结构体独立, 可互相参考组合使用
 *
 * 【位置式PID通用公式】
 *   err = set - actual
 *   increment  += err
 *   out = p*err + i*increment + d*(err - l_err)
 */

/* ================================================================
   增量式PID系列
   ================================================================ */

/**
 * @brief  增量式PID(标准) — 所有变体的基线
 * @note   积分: i * err (矩形积分)
 *         微分: d * (err - 2*l_err + ll_err) (完全微分)
 *         输出: out += increment (增量累加)
 */
void PID_Increment(PID_Inc *PID)//括号内是指将PID_Inc结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	PID -> integral = PID -> i * PID -> err;                                          				 //积分 = PID控制系数i(积分) * 误差

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量合成 = 比例 + 积分 + 微分

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅(防止单次增量过大)

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代: l_err -> ll_err
    
	PID -> l_err = PID -> err;																		 //误差迭代: err  -> l_err
}

/**
 * @brief  增量式PID-积分分离
 * @diff   标准: integral = i * err (无条件)
 *         本变体: |err| > err_threshold 时 integral = 0 (退化为PD)
 *                |err| <= err_threshold 时 integral = i * err
 * @why    启动/变速时误差大, 积分累积导致超调; 误差进入阈值后才开启积分消除静差
 */
void PID_Increment_Separation(PID_Inc_Sep *PID)//括号内是指将PID_Inc_Sep结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	if ((PID -> err) > (PID -> err_threshold) || (PID -> err) < -(PID -> err_threshold))			 //|误差| > 积分分离阈值 → 关闭积分项
	{
		PID -> integral = 0;																		 //积分项清零, 系统退化为PD控制
	}
	else
	{
		PID -> integral = PID -> i * PID -> err;                                                     //|误差| <= 阈值 → 正常积分消除静差
	}

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量合成

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
}

/**
 * @brief  增量式PID-梯形积分
 * @diff   标准: integral = i * err (矩形积分, 只取当前点)
 *         本变体: integral = i * (err + l_err) / 2 (梯形积分, 取两点平均)
 * @why    矩形积分低估/高估积分面积; 梯形积分用两点均值更逼近真实积分曲线, 累积更平滑
 */
void PID_Increment_Trapezoidal(PID_Inc_Trap *PID)//括号内是指将PID_Inc_Trap结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	PID -> integral = PID -> i * (PID -> err + PID -> l_err) / 2.0f;                 				 //积分 = i * 梯形积分((当前误差 + 上次误差) / 2)

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量合成

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
}

/**
 * @brief  增量式PID-变速积分
 * @diff   标准: integral = i * err (积分速度恒定)
 *         积分分离: integral = 0 或 i*err (只有开/关)
 *         本变体: integral = i * err * coef
 *                 coef 在 [0, 1] 间连续变化, 误差越小积分越快
 *                 |err| >= err_threshold_max → coef = 0
 *                 |err| <= err_threshold_min → coef = 1
 *                 中间区间 → coef 线性插值
 * @why    比积分分离更平滑, 阈值附近无抖振
 */
void PID_Increment_VariableIntegral(PID_Inc_VarInt *PID)//括号内是指将PID_Inc_VarInt结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	float abs_err;    //误差的绝对值,用于计算积分系数
	float coef;       //积分系数,范围[0, 1]

	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	abs_err = (PID -> err > 0) ? (PID -> err) : (-(PID -> err));									 //计算|误差|绝对值(C标准库abs在部分平台占用额外ROM)

	if (abs_err >= PID -> err_threshold_max)														 //|误差| >= 上限 → 积分系数为0, 完全关闭积分
	{
		coef = 0;
	}
	else if (abs_err <= PID -> err_threshold_min)													 //|误差| <= 下限 → 积分系数为1, 全速积分
	{
		coef = 1.0f;
	}
	else
	{
		//线性区间: coef = (max - |err|) / (max - min)
		//误差越小 → coef越接近1 → 积分越快
		coef = (float)(PID -> err_threshold_max - abs_err) / (float)(PID -> err_threshold_max - PID -> err_threshold_min);						 
	}

	PID -> integral = PID -> i * PID -> err * coef;                                                  //积分 = i * 误差 * 变速系数

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量合成

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
}

/**
 * @brief  增量式PID-抗积分饱和(Anti-Windup)
 * @diff   标准: integral = i * err, 不关心输出是否已饱和
 *         本变体: 当 out >= out_max 且 increment > 0 时, 清零积分项 ← 阻止正向饱和堆积
 *                当 out <= out_min 且 increment < 0 时, 清零积分项 ← 阻止负向饱和堆积
 *                执行机构已顶到极限时继续积分是无意义的, 只会导致恢复滞后
 *         与 limit_value 的区别: limit_value 事后截增量, AntiWindup 事前冻积分
 * @why    执行机构(如电机PWM占空比)有物理上限, 输出饱和后积分持续累加会导致:
 *         (1) 反向调节时需先"还清"堆积的积分, 响应严重滞后
 *         (2) 积分项过大导致输出在饱和边界来回振荡
 */
void PID_Increment_AntiWindup(PID_Inc_AW *PID)//括号内是指将PID_Inc_AW结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	PID -> integral = PID -> i * PID -> err;                                          				 //积分 = PID控制系数i(积分) * 误差

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量初次合成

	if ((PID -> out) >= (PID -> out_max) && (PID -> increment) > 0)								 	 //输出已达上限 且 增量仍为正 → 积分饱和, 清零积分项
	{
		PID -> integral = 0;
		PID -> increment = PID -> proportion + PID -> differential;								 	 //重算增量(不含积分项)
	}
	else if ((PID -> out) <= (PID -> out_min) && (PID -> increment) < 0)							 //输出已达下限 且 增量仍为负 → 积分饱和, 清零积分项
	{
		PID -> integral = 0;
		PID -> increment = PID -> proportion + PID -> differential;								     //重算增量(不含积分项)
	}

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅(二次保护)

	PID -> out += PID -> increment; 																 //输出累加

	if ((PID -> out) > (PID -> out_max)) PID -> out = PID -> out_max;								 //输出最终钳位(上位)
    
	if ((PID -> out) < (PID -> out_min)) PID -> out = PID -> out_min;								 //输出最终钳位(下位)

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
}

/**
 * @brief  增量式PID-不完全微分(Incomplete Derivative)
 * @diff   标准: differential = d * (err - 2*l_err + ll_err) (完全微分, 噪声等比例放大)
 *         本变体: raw_diff = d * (err - 2*l_err + ll_err)     ← 先算原始微分
 *                differential = d_coeff * raw_diff + (1-d_coeff) * l_differential ← 一阶低通滤波
 * @why    纯微分对噪声敏感, 传感器抖动经微分放大后可能淹没真实信号
 *         一阶低通滤波器的截止频率 = d_coeff / (2*PI*T), d_coeff越小滤波越强
 *         d_coeff = 1.0 → 退化为标准完全微分; d_coeff → 0 → 微分几乎不响应
 */
void PID_Increment_IncompleteDerivative(PID_Inc_ID *PID)//括号内是指将PID_Inc_ID结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	float raw_diff;   //原始微分值(滤波前), 临时变量

	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	PID -> integral = PID -> i * PID -> err;                                          				 //积分 = PID控制系数i(积分) * 误差

	raw_diff = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err);           				 //原始微分 = d * (err - 2*l_err + ll_err)

	PID -> differential = PID -> d_coeff * raw_diff + (1.0f - PID -> d_coeff) * (PID -> l_differential); //一阶低通: 新微分 = α*原始微分 + (1-α)*上次微分

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential;     				 //增量合成

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
    
	PID -> l_differential = PID -> differential;													 //微分项迭代(供下次滤波使用)
}

/**
 * @brief  增量式PID-前馈(Feedforward)
 * @diff   标准: increment = proportion + integral + differential
 *         本变体: feedforward = kf * (set - l_set)               ← 新增前馈项
 *                increment = proportion + integral + differential + feedforward
 * @why    PID是"看着误差追", 前馈是"预见变化抢跑"
 *         设定值变化时前馈立即输出, 不必等误差建立, 大幅提高跟踪响应速度
 *         kf = 0 → 退化为标准增量式PID
 */
void PID_Increment_FeedForward(PID_Inc_FF *PID)//括号内是指将PID_Inc_FF结构体变量指向指针变量PID,所以下属代码使用->指向指针变量的子个体,而不是使用正常结构体所使用的.来指向结构体
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> proportion = PID -> p * (PID -> err - PID -> l_err);                        				 //比例 = PID控制系数p(比例) * (误差 - 上次的误差)

	PID -> integral = PID -> i * PID -> err;                                          				 //积分 = PID控制系数i(积分) * 误差

	PID -> differential = PID -> d * (PID -> err - 2 * PID -> l_err + PID -> ll_err); 				 //微分 = PID控制系数d(微分) * (误差 - 2 * 上次的误差 + 上上次的误差)

	PID -> feedforward = PID -> kf * (PID -> set - PID -> l_set);									 //前馈 = kf * 设定值变化量 (set变化越大, 前馈越强)

	PID -> increment = PID -> proportion + PID -> integral + PID -> differential + PID -> feedforward; //增量 = PID三项 + 前馈

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //增量限幅

	PID -> out += PID -> increment; 																 //输出累加

	PID -> ll_err = PID -> l_err;   																 //误差迭代
    
	PID -> l_err = PID -> err;
    
	PID -> l_set = PID -> set;																		 //设定值迭代(供下次计算变化量使用)
}

/* ================================================================
   位置式PID系列
   ================================================================ */

/**
 * @brief  位置式PID(标准)
 * @note   直接计算输出绝对值, 非增量累加
 *         积分由 increment(误差累加和) 实现, 而非每个周期单独计算
 *         适用于舵机角度、阀门开度等需要绝对位置指令的执行机构
 */
void PID_Position(PID_Pos *PID)
{
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> increment += PID -> err;                                                                  //误差累加(位置式PID的积分来源)

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //积分累加限幅(防止积分饱和)

	PID -> proportion = PID -> p * PID -> err;                                                       //比例项 = p * 误差

	PID -> integral = PID -> i * PID -> increment;													 //积分项 = i * 误差累加和

	PID -> differential = PID -> d * (PID -> err - PID -> l_err);									 //微分项 = d * (误差 - 上次的误差)

	PID -> l_err = PID -> err;																		 //误差迭代

	PID -> out = PID -> proportion + PID -> integral + PID -> differential;							 //输出 = 比例 + 积分 + 微分
}

/**
 * @brief  位置式PID-微分先行(Derivative on Measurement)
 * @diff   标准位置式: differential = d * (err - l_err)                ← 对误差微分
 *         本变体:     differential = -(actual - l_actual)              ← 对实际值微分(取反)
 * @why    设定值突变时, err也会突变, 导致微分项产生冲击(spike)
 *         对实际值微分则不受设定值影响, 仅响应系统真实动态
 *         此外通过 integration_K 控制积分速度: integration_K=0时暂停误差累加, 已有积分仍参与输出, integration_K>0时启用
 */
void PID_Position_DM(PID_Pos *PID)                                                                   //微分先行: 对实际值微分, 不对误差微分
{																									 //设定值频繁变化时效果显著优于标准位置式
	PID -> err = PID -> set - PID -> actual;                                          				 //误差 = 设置 - 实际

	PID -> increment += PID -> err * PID -> integration_K;                                           //误差累加 * 积分速率系数 (integration_K=0时关闭积分)

	PID -> increment = limit_value(PID -> increment, PID -> point_limits, (-PID -> point_limits));	 //积分累加限幅

	PID -> proportion = PID -> p * PID -> err;                                                       //比例项 = p * 误差

	PID -> integral = PID -> i * PID -> increment;													 //积分项 = i * 误差累加和

	PID -> differential = -PID->d * (PID -> actual - PID -> l_actual);                                        //微分项 = -d*(实际值变化量) — 关键差异: 对实际值微分

	PID -> l_err = PID -> err;																		 //误差迭代
    
	PID -> l_actual = PID -> actual;																 //实际值迭代(供下次微分计算)

	PID -> out = PID -> proportion + PID -> integral + PID -> differential;							 //输出 = 比例 + 积分 + 微分
}

/* ================================================================
   通用工具函数
   ================================================================ */

/**
 * @brief  限幅函数
 * @param  value          待限幅的值
 * @param  positive_value 正向限幅上限
 * @param  negative_value 负向限幅下限(通常为 -positive_value)
 * @retval 限幅后的值, 保证在 [negative_value, positive_value] 区间内
 */
float limit_value(float value, float positive_value, float negative_value)
{
	if (value > positive_value)
		value = positive_value;
	if (value < negative_value)
		value = negative_value;
	return value;
}
