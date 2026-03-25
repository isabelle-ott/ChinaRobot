/**
 * @file application_common.c
 * @brief 应用层通用运动控制，比如循迹等代码
 */

#include "application_common.h"

/**
 * @brief 进行一次计算，控制Y轴速度，保持循迹板在线上，同时计算陀螺仪零漂补偿
 * @param p 线的目标位置，单位mm，范围[12,96]
 * @param CalibrateIMU 是否校准陀螺仪 -1完全不校准，0两循迹板都有线时校准，1单循迹板有线就校准
 * @param use_M 是否使用中循迹板参与计算
 * @param Vy Y轴速度指针，用于输出需要的Y轴速度，NULL表示不需要输出直接设置
 * @return 是否进入微调阶段
 */
uint8_t keep_line(uint8_t p, int8_t CalibrateIMU, uint8_t use_M, int32_t *Vy)
{
	const int32_t v_Kp = 10;		// snum: 控制前后速度的P值
	const float speedZ = 0.5f;		// snum: 旋转速度因子，控制陀螺仪调整的幅度
	const int32_t dead_deltaY = 6;	// snum: 前后死区判断
	const int32_t dead_deltaZ = 25; // snum: 旋转死区判断
	const int32_t v_default = 300;	// snum: 缓慢前进的速度

	set_Angle_enable(1);

	uint8_t L = TrackBoard_to_mm(TrackBoard_read_L());
	uint8_t R = TrackBoard_to_mm(TrackBoard_read_R());
	uint8_t M = use_M ? TrackBoard_to_mm(TrackBoard_read_M()) : 0;

	/* 找到线：同时调整旋转和前后 */
	if (L || R || M)
	{
		/* 前后的调整逻辑：以左右循迹板的最大值为准，调整至目标值 */
		uint8_t m = max(M, _max(L, R)); // 取最大值
		int32_t delta_y = (p - m);		// 计算Y轴误差
		int32_t v = delta_y * v_Kp;		// P控制器,计算速度
		if (Vy)
			*Vy = v;
		else
			set_speed_y(v);

		/* 旋转的调整逻辑：根据循迹板算出当前姿态，调整陀螺仪的零漂补偿 */
		int32_t delta_z = 0;

		if (CalibrateIMU == 1 || (CalibrateIMU == 0 && (L && R)) || (CalibrateIMU == 0 && use_M && (M && (L || R))))
		{
			delta_z = use_M ? TrackBoardMLR_to_angle(M, L, R) : TrackBoardLR_to_angle(L, R);
			int32_t delta_z_step = (float)delta_z * speedZ;
			imu_set_angle(get_targetAngle() + delta_z_step);
		}

		/* 死区判断，告诉调用者 "差不多行了" */
		if (_abs(delta_y) < dead_deltaY && _abs(delta_z) < dead_deltaZ &&
			((CalibrateIMU == -1 && (!!L) + (!!M) + (!!R) >= 1) || /* -1需要至少有1个板找到线才能判定死区 */
			 (CalibrateIMU == 0 && (!!L) + (!!M) + (!!R) >= 1) ||  /* 0 需要至少有1个板找到线才能判定死区 */
			 (CalibrateIMU == 1 && (!!L) + (!!M) + (!!R) >= 2)))   /* 1 需要至少有2个板找到线才能判定死区 */
			return 1;
	}
	/* 没有找到线：缓慢前进 */
	else
	{
		if (Vy)
			*Vy = v_default;
		else
			set_speed_y(v_default);
	}
	return 0;
}

/**
 * @brief 向前找到线并校准陀螺仪
 * @note 需要确保只需要前进或稍微旋转就能使两个循迹板同时在线上
 * @param speed 向前速度
 * @param p 线的目标位置，单位mm，范围[12,96]
 * @param time 微调时间，准确来说是在白线上待的时间，-1表示默认值
 */
void find_line(int32_t speed, int8_t p, int32_t time)
{
	if (time < 0) time = 3000; // snum:时间的默认值

	/* 高速向前，直到任何一个循迹板碰到线 */
	set_speed_xy(0, speed);
	while (!(TrackBoard_read_L() || TrackBoard_read_R())) __NOP();

	/* 校准陀螺仪 */
	uint32_t time_count = 0;	 // 记录识别到白线的时间
	uint32_t last_t = OS_tick(); // 记录上次循环的时间
	while (time_count < time)
	{
		uint8_t L = TrackBoard_read_L(), R = TrackBoard_read_R();

		// 计算识别到白线的时间
		uint32_t tick = OS_tick();
		if (L || R) time_count += tick - last_t;
		last_t = tick;

		if (keep_line(p, 1, 0, NULL)) break; // 进入死区就结束
	}
	set_speed_xy(0, 0);
}
MAKE_CMD(void, find_line, int32_t, int8_t, int32_t);

/**
 * @brief 高精度校正车身，可能会耗时较久
 * @warning 不会用到中循迹板，需要确保左右循迹板都在线上
 * @note 校准的是底盘的目标角度，并不是陀螺仪的零偏量，使用绝对角度需要注意
 */
void CalibrateAngle_super(void)
{
	const float Kp_z = 1.0f; // snum: Kp

	uint8_t state = 0;

	set_speed_xyZ(0, 0, 0); // 关闭陀螺仪，接管控制底盘
	uint32_t Calibrates_startTick = OS_tick(), check_startTick = 0;
	while (OS_tick() - Calibrates_startTick > 10000)
	{
		int32_t L = TrackBoard_to_mm(TrackBoard_read_L());
		int32_t R = TrackBoard_to_mm(TrackBoard_read_R());
		switch (state)
		{
		case 0: // 状态0：向前找线
			if (L > 12 || R > 12)
			{
				set_speed_xyZ(0, 0, 0);
				state = 1;
				break;
			}
			set_speed_xyZ(0, 500, 0);
			break;
		case 1: // 状态1：矫正车身
			if (L == R)
			{
				set_speed_xyZ(0, 0, 0);
				if (L == 0) // 两边都没有线就打回状态1
				{
					state = 0;
				}
				else // 两侧值相等代表车正了
				{
					check_startTick = OS_tick();
					state = 2;
				}
				break;
			}
			set_speed_xyZ(0, 0, Kp_z * (L - R));
			break;
		case 2: // 状态2：到位检测
			if (OS_tick() - check_startTick >= 200)
				state = 3; // 在状态2坚持了一段时间，判定为确实到位
			if (L != R || L == 0)
				state = 1; // 有异常就打回
			set_speed_xyZ(0, 0, 0);
		}
		if (state == 3) break;
	}
	set_speed_xyZ(0, 0, 0);
	set_Angle_enable(1); // 开启陀螺仪，自动将当前角度设置为目标角度
}
MAKE_CMD(void, CalibrateAngle_super, void);

// 单元测试
void keep_line_allTime(uint8_t p, int8_t CalibrateIMU)
{
	set_speed_x(0);
	while (1)
	{
		keep_line(p, CalibrateIMU, 0, NULL);
	}
}
MAKE_CMD(void, keep_line_allTime, uint8_t, int8_t);
