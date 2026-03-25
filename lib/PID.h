#ifndef GENERAL_PID_H
#define GENERAL_PID_H
#include "stdint.h"

struct PIDstruct;

// PID参数结构体
typedef struct
{
	float Kp, Ki, Kd;							// PID基本参数
	float output_limit;							// 输出限幅
	float integral_limit;						// 积分限幅

	/*以下变量需要在improve_options配置才能生效*/

	uint32_t exception_countMAX;				// 异常计数
	void (*exception_call)(struct PIDstruct *); // 异常回调函数

	float DeadBand; // 死区

	float integral_rate_A;	// 变速积分满速区
	float integral_rate_B;	// 变速积分范围
	float integral_rate_AB; // 分离区为A+B

	float Derivative_Filter_Coefficient; // 微分滤波系数

	float Output_Filter_Coefficient; // 输出滤波系数
} PID_param_t;

// PID结构体
typedef struct PIDstruct
{
	uint8_t enable;		// 使能控制器
	PID_param_t *param; // 参数结构体指针

	float target;					   // 目标值
	float measure, last_measure;	   // 测量值
	float error, last_error;		   // 偏差值
	float Pout, Iout, Dout, last_Dout; // 各项输出值
	float Iterm;					   // 本周期积分增量
	float output, last_output;		   // 输出值
	uint32_t exception_count;		   // 异常计数
} PID_t;



float angle_PID_Calculate(PID_t *pid, float measure, float target);

#endif

