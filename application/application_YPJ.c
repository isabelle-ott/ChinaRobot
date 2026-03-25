/**
 * @file application_YPJ.c
 * @brief 圆盘机相关代码
 */
#include "application_YPJ.h"
#include "application_common.h"

static void find_ender(int32_t Vx, uint8_t condition, int32_t read_counter_MAX)
{
	uint8_t (*TrackBoard_read_ender)(void) = is_site(SITE_RED) ? TrackBoard_read_L : TrackBoard_read_R;
	int32_t read_counter = 0; // 目标计数，到达某个值后就认为到达目标了
	set_speed_x(Vx);
	uint32_t last_read_tick = OS_tick();
	while (read_counter < read_counter_MAX)
	{
		// 计算距离上次循环的时间
		uint32_t tick = OS_tick(), t = tick - last_read_tick;
		last_read_tick = tick;
		// 如果循迹板是目标状态就累计时间，否则清空累计
		if ((!!TrackBoard_read_ender()) == (!!condition))
			read_counter += t;
		else
			read_counter = 0;
		// 保持巡线
		keep_line(54, 0, 0, NULL);
	}
	set_speed_xy(0, 0);
}

/**
 * @brief 在圆盘机前，调整姿态进入圆盘机中点
 */
void YPJ_ready(void)
{
	const int32_t Vy_high = 1200;			 // snum: 高速向前找线的速度
	const int32_t Vx_high = 1200;			 // snum: 粗略找尽头的速度
	const int32_t read_counter_MAX = 20;	 // snum: 粗略找线下的滤波，避免噪声导致错误
	const int32_t Vx_low = 400;				 // snum: 低速精准找尽头的速度，单位[0.1rpm]
	const int32_t Distance_toCenter_B = 133; // snum: 尽头距离中点的距离，单位[mm]
	const int32_t Distance_toCenter_R = 133; // snum: 尽头距离中点的距离，单位[mm]
	const int32_t Vmax_toCenter = 1200;		 // snum: 向中点移动的速度，单位[0.1rpm]
	const int32_t keep_line_val_R = 36;
	const int32_t keep_line_val_B = 24;

	const uint32_t YPJready_startTick = OS_tick();
	log_info("YPJ_ready\n");
	/* 向前找线: 以较高的速度向前运动，直到任意循迹板碰到白线 */
	set_speed_xy(0, Vy_high);
	while (!(TrackBoard_read_L() || TrackBoard_read_R() || TrackBoard_read_M())) osDelay(2);
	log_info("微调");
	while (1)
	{
		uint8_t online = keep_line(54, 0, 1, NULL); // 前后调整，使线在循迹板中间

		int32_t L = TrackBoard_to_mm(TrackBoard_read_L());
		int32_t R = TrackBoard_to_mm(TrackBoard_read_R());
		int32_t M = TrackBoard_to_mm(TrackBoard_read_M());
		if (L == 0 && (45 < R && R < 65)) // 只有右侧循迹板有线
			set_speed_x(Vx_high);
		else if (R == 0 && (45 < L && L < 65)) // 只有左侧循迹板有线
			set_speed_x(-Vx_high);
		else
			set_speed_x(0);
		if (L && R && online) break; // 两侧都识别到线且keep_line进入死区就算找到线
	}
	set_speed_xy(0, 0);
	log_info("找到线，开始找尽头");

	/* 找线的尽头: 分别向左右高速移动找尽头进行粗略定位，再低速找一遍精准定位 */
	int32_t find_end_dir = is_site(SITE_RED) ? (-1) : (1);
	uint8_t (*TrackBoard_read_ender)(void) = is_site(SITE_RED) ? TrackBoard_read_L : TrackBoard_read_R;
	// 向远离出发点方向移动，直到循迹板在线上
	find_ender(-find_end_dir * Vx_high, 1, read_counter_MAX);
	// 向靠近出发点方向移动，直到循迹板不在线上
	find_ender(find_end_dir * Vx_high, 0, read_counter_MAX);
	// 向远离出发点方向低速移动，直到循迹板在线上，此时就到达尽头了
	set_speed_x(-find_end_dir * Vx_low);
	while (1)
	{
		keep_line(54, 0, 0, NULL);
		if (TrackBoard_read_ender()) break;
	}
	/* 找到尽头了，最后再移动固定的距离就到中点了 */
	reset_odom();

	/* 匀加减速的准备计算 */
	int32_t Distance_toCenter = is_site(SITE_RED) ? Distance_toCenter_R : Distance_toCenter_B;
	// 距离的单位换算：从 [mm] 转为 [0.1rpm · ms] 即 [(1/600000)round]
	const float Distance_tmp = (float)Distance_toCenter / (float)chassis_mmpr * 600000.0f;
	// 计算时间中点: 在次之前做匀加速，在此之后做匀减速
	const uint32_t t1_toCenter = Distance_tmp / (float)(Vmax_toCenter + Vx_low);
	/* 实现匀加减速，效果：先从Vx_low开始匀加速到Vmax，再匀减速到Vx_low，两个阶段加速度相同 */
	uint32_t toCenter_startTick = OS_tick();
	while (get_odom_distance() < Distance_toCenter)
	{
		keep_line(54, 0, 0, NULL); // 保持巡线

		uint32_t t = OS_tick();
		int32_t Vx = Vmax_toCenter - (abs((int32_t)(t - toCenter_startTick - t1_toCenter)) * (Vmax_toCenter - Vx_low) / t1_toCenter);
		Vx = max(Vx_low, Vx);
		set_speed_x(-find_end_dir * Vx);
		// print_f("%d", Vx);
	}
	set_speed_xy(0, 0);

	servo_action(ServoAG_YPJ_scan, 1);

	/* 调整前后距离 */
	uint32_t keep_line_val = is_site(SITE_RED) ? keep_line_val_R : keep_line_val_B;
	uint32_t tick = OS_tick();
	while (OS_tick() - tick < 3000 && !keep_line(keep_line_val, 1, 0, NULL)) osDelay(2); // snum:超时时间
	log_info_f("YPJ_ready OK: %dms", OS_tick() - YPJready_startTick);
}
MAKE_CMD(void, YPJ_ready, void);

/**
 * @brief 扫描抓球
 */
void YPJ_scan(void)
{
	const uint32_t min_time = 12000, timeout = 20000; // snum:最小时间和超时时间

	set_speed_xyZ(0, 0, 0); // 确保停止，同时关闭陀螺仪
	openMV_set_mode(openMV_mode_YPJ);
	// servo_action_w(ServoAG_YPJ_scan, 1);

	MV_DATA_t MV_data; // 用于存储openMV数据

	uint32_t count_Y = 0, count_RB = 0; // 对抓到的球计数
	openMV_clear_data();
	uint8_t expectation_color = 0; // 要抓的颜色。0表示什么颜色都不抓，1表示什么颜色都抓

	uint32_t start_tick = OS_tick();
	while (OS_tick() - start_tick < 1000)
	{
		if (!openMV_get_data(openMV_mode_YPJ, &MV_data)) continue;

		if (expectation_color == 0)
		{
			/* 开始扫描的一段时间内都没有球，就没必要跳过第一个球了 */
			if (OS_tick() - start_tick > 500) break;
			/* 跳过第一个颜色 */
			expectation_color = (MV_data.type == 0x52) ? 0x51 : 0x52;
		}
		if (expectation_color == MV_data.type)
		{
			expectation_color = 2; //标记一下，代表可以马上抓
			break;
		}
	}
	start_tick = OS_tick();
	while (OS_tick() - start_tick < timeout)
	{
		if (!(expectation_color == 2 || openMV_get_data(openMV_mode_YPJ, &MV_data))) continue;
		expectation_color = 0;
		if (MV_data.type == 0x52) // 黄球
		{
			servo_action(ServoAG_YPJ_get_Y_Ball, 0);
			count_Y++;
			osDelay(300);
			openMV_clear_data();
		}
		else if (MV_data.type == 0x51) // 红蓝球
		{
			servo_action(ServoAG_YPJ_get_RB_Ball, 0);
			count_RB++;
			osDelay(300);
			openMV_clear_data();
		}
		// 抓够且大于最小时间，就可以提前结束
		if (count_RB >= 4 && count_Y >= 4 && OS_tick() - start_tick > min_time) break;
	}
	openMV_set_mode(openMV_mode_IDLE);
	servo_action_w(ServoAG_ready, 1);
	servo_action_w(ServoAG_ARM_reset, 1);
	set_Angle_enable(1); // 开启陀螺仪
	log_info("YPJ_scan OK");
}
MAKE_CMD(void, YPJ_scan, void);

void YPJ_run(void)
{
	YPJ_ready();
	YPJ_scan();
	// 结束的时候再校准一次陀螺仪
	CalibrateAngle_super();
}

MAKE_CMD(void, YPJ_run, void);
