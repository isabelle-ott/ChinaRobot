/**
 * @file application_LZ.c
 * @brief 立桩相关代码
 */
#include "application_LZ.h"
#include "application_common.h"
#include "integraler.h"
#include "math.h"

/**
 * @brief 定位立桩的球
 * @param R 绕桩半径
 * @return 1定位完成 0定位超时或数据超时
 */
uint8_t LZ_positioning(int32_t R)
{
	const float MV_R = 300.0f;			 // 定位画面上的圆弧半径，实现定位过程中Y轴沿圆弧运动
	const int32_t K_p = 12;				 // 比例增益系数
	const int32_t MAX_Speed = 200;		 // 最大速度
	const int32_t Position_X_Limit = 15; // X轴位置容差
	const int32_t Position_Y_Limit = 5;	 // Y轴位置容差
	const uint32_t MAX_Time = 5000;		 // 最大定位时间为5s
	const uint32_t L_frame_TIME = 500;	 // 丢帧的时间

	uint32_t start_time = OS_tick();	   // 记录开始时间
	uint32_t Lost_frame_time = start_time; // 记录丢帧时间
	uint8_t positioning_complete = 0;	   // 是否定到位

	while ((OS_tick() - start_time) < MAX_Time)
	{
		// 获取视觉数据
		MV_DATA_t MV;
		if (openMV_get_data(openMV_mode_LZ, &MV) == 0)
		{
			if ((OS_tick() - Lost_frame_time) > L_frame_TIME)
			{
				break; // 连续一段时间没有openMV的数据就放弃本次定位
			}
			continue;
		}
		Lost_frame_time = OS_tick();

		// 检查是否达到目标位置
		if (_abs(MV.x) <= Position_X_Limit && _abs(MV.y) <= Position_Y_Limit)
		{
			positioning_complete = 1;
			break;
		}

		// 计算控制输出
		int32_t x_error = MV.x;
		int32_t y_error = MV.y - (MV_R - sqrtf(MV_R * MV_R - MV.x * MV.x));
		int32_t x_speed = K_p * x_error;
		int32_t y_speed = K_p * y_error;
		// 限制速度范围
		x_speed = _limit(x_speed, -MAX_Speed, MAX_Speed);
		y_speed = _limit(y_speed, -MAX_Speed, MAX_Speed);
		// 设置速度
		set_speed_RXY(R, x_speed, y_speed);
	}

	set_speed_xyZ(0, 0, 0); // 停止运动

	if (positioning_complete)
	{
		log_info("立桩定位结束");
	}
	else
	{
		log_info("立桩定位超时");
	}
	return positioning_complete;
}

void LZ_towardsCenter(void)
{
	while (get_HW(2) != get_HW(3))
	{
		if (get_HW(3))
			set_speed_xyZ(0, 0, -150);
		else
			set_speed_xyZ(0, 0, 150);
	}
	set_speed_xyZ(0, 0, 0);
}

void LZ_getWhite(void)
{
	LZ_towardsCenter();
	servo_action_w(ServoAG_LZ_getWhite, 1);

	openMV_set_mode(openMV_mode_IDLE);
	servo_action_w(ServoAG_ARM_reset, 1);
}

/**
 * @brief 绕桩抓球
 */
void LZ_scan(void)
{
	/* 参数 */
	const uint32_t get_interval = 2000;		// snum: 两次抓球的最小时间间隔
	const int32_t angle_count_limit = 3600; // snum: 最大旋转角度计数(单位0.1度)
	const int32_t Vx = 520;					// snum: 绕桩速度
	const int32_t default_R = 55;			// snum: 默认绕桩半径
	const float R_Ki = 0.05f;				// snum: 绕桩半径的积分速度
	const int32_t R_limit = 20;				// snum: 绕桩半径的范围

	/* 准备动作 */
	servo_action_w(ServoAG_LZ_scan, 1);
	openMV_set_mode(openMV_mode_LZ);
	set_Angle_enable(0);
	openMV_clear_data();

	/* 定义变量 */
	MV_DATA_t MV_data = {0, 0, 0};											 // 用于接收openMV数据
	int32_t last_angle = imu_get_angle(), angle_count = 0;					 // 用于记录绕过的角度
	int32_t stop_angle = limit_angle(get_targetAngle() + angle_count_limit); // 计算停止的角度
	uint32_t last_get_t = 0;												 // 记录抓球时间，避免连续抓球
	uint8_t is_ok = 0, catch_times = 0;										 // 记录是否抓到白球，以及抓到几个红蓝球

	integraler_t R_Handler; // 绕桩半径处理句柄
	integral_init(&R_Handler, default_R, default_R - R_limit, default_R + R_limit);

	/* 开始扫描 */
	while (angle_count_limit > abs(angle_count))
	{
		/* 累计绕过的角度 */
		int32_t angle = imu_get_angle();
		angle_count += limit_angle(angle - last_angle);
		last_angle = angle;

		/* 绕桩算法 */
		float R_error = 0;
		if (get_HW(2) == get_HW(3))
			R_error = 0.0f;
		else if (get_HW(3))
			R_error = -R_Ki;
		else
			R_error = R_Ki;
		integral_add(&R_Handler, R_error);
		set_speed_RXY(integral_get(&R_Handler), Vx, 0);
		// print_f("R:%d", (int32_t)integral_get(&R_Handler));

		/* 扫描球 */
		if (!is_ok && openMV_get_data(openMV_mode_LZ, &MV_data) && (OS_tick() - last_get_t > get_interval))
		{
			log_info("尝试定位立桩球");
			last_get_t = OS_tick();
			uint8_t ret = LZ_positioning(default_R);
			if (ret == 1)
			{
				// 定位到中心
				log_info("立桩定位成功，开始抓球");
				servo_action_w(ServoAG_LZ_get, 1);
				servo_action_w(ServoAG_LZ_scan, 1);
				catch_times++;
				if (catch_times >= 3)
				{
					is_ok = 1;
					LZ_getWhite();
				}
			}
			openMV_clear_data();
		}
	}
	if (!is_ok) LZ_getWhite(); // 如果没抓到3个球就要在绕圈完成之后再抓白球
	set_speed_xyZ(0, 0, 0);
	set_Angle_enable(1);
	set_angle_w(stop_angle); // 保证停止时的角度
}

/**
 * @brief 调整姿态准备绕立桩
 */
void LZ_ready(void)
{
	while (get_HW(2) != get_HW(3))
	{
		if (get_HW(3))
			set_speed_xy(150, 0);
		else
			set_speed_xy(-150, 0);
	}
	set_speed_xy(0, 0);
}

void LZ_run(void)
{
	LZ_ready();
	LZ_scan();
}
MAKE_CMD(void, LZ_run, void);
