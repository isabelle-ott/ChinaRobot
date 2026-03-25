#include "PID.h"
#include "fast_math.h"
#include "bsp.h"

// #ifndef __ABS
// #define __ABS(var) (((var) < 0) ? (-(var)) : (var))
// #endif

/**
 * @brief PID的模板，需根据具体需求删减内容制定专用函数
 * @param pid pid参数结构体指针
 * @param measure 测量值
 * @param target 目标值
 * @return 控制值
 */
float Template_PID_Calculate(PID_t *pid, float measure, float target)
{
	uint8_t option = 1; // 优化选项，请删除这一行

	if (!pid->enable) return 0;
	// 保留之前的数据
	pid->last_measure = pid->measure;
	pid->last_output = pid->output;
	pid->last_Dout = pid->Dout;
	pid->last_error = pid->error;
	// 更新当前的数据
	pid->measure = measure;
	pid->target = target;
	pid->error = pid->target - pid->measure;

	if (option) /* 【可选】堵转检测--------------------------------------------------------------------- */
	{
		if (pid->output >= pid->param->output_limit * 0.01f)
		{ // 低功率不检测堵转
			// 测量值与目标差值太大时进行堵转计数
			if (_abs(pid->error / pid->target) > 0.9f)
			{
				if (pid->exception_count <= pid->param->exception_countMAX)
					pid->exception_count++;
			}
			else
				pid->exception_count = 0;
		}
		// 计数太多时认定为堵转
		if (pid->exception_count > pid->param->exception_countMAX)
		{
			if (pid->param->exception_call != 0) pid->param->exception_call(pid);
			return pid->output = 0.0f;
		}
	}
	if (option) /* 【可选】死区-------------------------------------------------------------------------- */
	{
		if (_abs(pid->error) <= pid->param->DeadBand) return pid->output = 0;
	}
	/*********************************** 比例处理 **********************************/
	pid->Pout = pid->param->Kp * pid->error;
	/*********************************** 微分处理 **********************************/
	pid->Dout = pid->param->Kd * (pid->error - pid->last_error);
	if (option) /* 【可选】微分先行------------------------------------------------------------------------ */
	{
		pid->Dout = pid->param->Kd * (pid->last_measure - pid->measure);
	}
	if (option) /* 【可选】微分作用滤波--------------------------------------------------------------------- */
	{
		pid->Dout = pid->Dout * pid->param->Derivative_Filter_Coefficient +
					pid->last_Dout * (1 - pid->param->Derivative_Filter_Coefficient);
	}
	/*********************************** 积分处理 **********************************/
	if (option) /* 【可选】梯形积分------------------------------------------------------------------------- */
	{
		pid->Iterm = pid->param->Ki * ((pid->error + pid->last_error) / 2);
	}
	else // 普通积分
	{
		pid->Iterm = pid->param->Ki * pid->error;
	}
	if (option) /* 【可选】变速积分------------------------------------------------------------------------- */
	{
		// 积分作用将要变大，且误差较大时，对本周期积分积累速度进行变速
		if ((pid->error * pid->Iout > 0) && (_abs(pid->error) > pid->param->integral_rate_A))
		{
			if (_abs(pid->error) >= pid->param->integral_rate_AB) // 分离区，积分分离
				pid->Iterm = 0;
			else // 变速区，减弱积分积累
				pid->Iterm *= (pid->param->integral_rate_AB - _abs(pid->error)) / pid->param->integral_rate_B;
		}
	}
	// 输出值到达极限，且积分作用将要变大时，取消积分积累
	if ((_abs(pid->output) >= pid->param->output_limit) && (pid->error * pid->Iout > 0))
		pid->Iterm = 0;
	pid->Iout += pid->Iterm;
	pid->Iout = _limit(pid->Iout, -(pid->param->integral_limit), pid->param->integral_limit); // 积分限幅
	/*********************************** 输出处理 **********************************/
	pid->output = pid->Pout + pid->Iout + pid->Dout;
	if (option) /* 【可选】输出滤波------------------------------------------------------------------------ */
	{
		pid->output = pid->output * pid->param->Output_Filter_Coefficient +
					  pid->last_output * (1 - pid->param->Output_Filter_Coefficient);
	}
	pid->output = _limit(pid->output, -(pid->param->output_limit), pid->param->output_limit); // 输出限幅
	return pid->output;
}

void PID_init(PID_t *pid, PID_param_t *pid_param)
{
	pid->param = pid_param;
	pid->enable = 1;
	pid->target = 0.0f;
	pid->measure = pid->last_measure = 0.0f;
	pid->error = pid->last_error = 0.0f;
	pid->Pout = pid->Iout = pid->Dout = pid->last_Dout = 0.0f;
	pid->Iterm = 0.0f;
	pid->output = pid->last_output = 0.0f;
	pid->exception_count = 0;
}

/**
 * @brief 底盘角度环PID
 */
float angle_PID_Calculate(PID_t *pid, float measure, float target)
{
	// 保留之前的数据
	pid->last_measure = pid->measure;
	pid->last_output = pid->output;
	pid->last_Dout = pid->Dout;
	pid->last_error = pid->error;
	// 更新当前的数据
	pid->measure = measure;
	pid->target = target;
	pid->error = limit_angle(pid->target - pid->measure);

	/* 死区 */
	// if (_abs(pid->error) <= pid->param->DeadBand) return pid->output = 0.0f;
	/*********************************** 比例处理 **********************************/
	pid->Pout = pid->param->Kp * pid->error;
	/*********************************** 微分处理 **********************************/
	pid->Dout = pid->param->Kd * (pid->error - pid->last_error);
	/*********************************** 积分处理 **********************************/
	/* 梯形积分 */
	pid->Iterm = pid->param->Ki * ((pid->error + pid->last_error) / 2);
	// 输出值到达极限，且积分作用将要变大时，取消积分积累
	if ((_abs(pid->output) >= pid->param->output_limit) && (pid->error * pid->Iout > 0))
		pid->Iterm = 0;
	pid->Iout += pid->Iterm;
	pid->Iout = _limit(pid->Iout, -(pid->param->integral_limit), pid->param->integral_limit); // 积分限幅
	/*********************************** 输出处理 **********************************/
	pid->output = pid->Pout + pid->Iout + pid->Dout;
	pid->output = _limit(pid->output, -(pid->param->output_limit), pid->param->output_limit); // 输出限幅
	// print_f("%g,%g,%g,%g,%g\n", pid->error, pid->output, pid->Pout, pid->Iout, pid->Dout);
	return pid->output;
}
