/**
 * @file application_JT.c
 * @brief 阶梯平台相关代码
 */
#include "application_JT.h"
#include "application_common.h"
#include "my_tasks.h"
#include "QR_code.h"

enum
{
	Min_height,				 // 最低高度阶梯
	Mid_height,				 // 中间高度阶梯
	Max_height,				 // 最高高度阶梯
	None_height				 // 阶梯平台结束
} volatile e_current_height; // 记录当前处于哪个高度的阶梯

int32_t prohibit_switch = 0; // 禁止切换区域，X轴里程计小于该值时不判断是否切换高度。避免频繁切换高度
int32_t prohibit_scan = 0;	 // 禁止扫描区域，X轴里程计小于该值时不扫描物体。避免高度切换后一小段距离内的混乱状态
int32_t height_switch = 0;	 // 高度切换点，X轴里程计大于该值时进行高度切换。此变量用于实现延迟切换高度以应对高度传感器的安装位置问题

static const ServoAction_e scan_action[] = {[Min_height] = ServoAG_JT_scanL,
											[Mid_height] = ServoAG_JT_scanM,
											[Max_height] = ServoAG_JT_scanH,
											[None_height] = ServoAG_JT_scanH};
static const ServoAction_e get_action[] = {[Min_height] = ServoAG_JT_getL,
										   [Mid_height] = ServoAG_JT_getM,
										   [Max_height] = ServoAG_JT_getH,
										   [None_height] = ServoAG_JT_getH};
/* 自动根据当前高度选择对应的扫描状态的动作组 */
static void auto_arm_scan(void)
{
	set_speed_xy(0, 0);
	servo_action_w(scan_action[e_current_height], 1);
}
/* 自动根据当前高度选择对应的抓取动作组 */
static void auto_arm_get(void)
{
	set_speed_xy(0, 0);
	servo_action_w(get_action[e_current_height], 1);
}

static uint8_t height_switch_processer(void)
{
	/* 进行高度切换 */
	if (height_switch != 0 && get_odom_X_abs() >= height_switch)
	{
		set_speed_x(0);
		switch (e_current_height)
		{
		case Min_height:
			e_current_height = Max_height;
			prohibit_scan = get_odom_X_abs() + 50; // 切换到最高高度之后，一段距离内不扫描
			break;
		case Max_height:
			e_current_height = Mid_height;
			prohibit_scan = get_odom_X_abs() + 5;
			break;
		case Mid_height:
			e_current_height = None_height;
			prohibit_scan = get_odom_X_abs() + 100;
			break;
		}
		height_switch = 0;
		auto_arm_scan();
		print_f("切换高度到%d\n", e_current_height);
		openMV_clear_data();
		My_QR_Handle._Set_QR_Data_Zero();
		return 1;
	}
	return 0;
}

uint8_t JT_positioning(void)
{
	// 使用常量而不是静态变量，避免多任务环境中的问题
	const int32_t K_p = 10;				 // 比例增益系数
	const uint32_t MAX_Time = 5000;		 // 最大定位时间为5s
	const int32_t MAX_Speed = 200;		 // 最大速度
	const int32_t Position_X_Limit = 10; // X轴位置容差
	const uint32_t L_frame_TIME = 500;	 // 丢帧的时间

	uint32_t start_time = OS_tick();	   // 记录开始时间
	uint32_t Lost_frame_time = start_time; // 记录丢帧时间
	uint8_t positioning_complete = 0;	   // 是否定到位
	MV_DATA_t MV;

	while ((OS_tick() - start_time) < MAX_Time)
	{
		// 在阶梯最高处，可能在定位过程中移动到中间处，此时需要进行高度切换。如果发生切换就退出本次定位
		if (e_current_height == Max_height && height_switch_processer()) return 0;

		// 获取最新的视觉数据
		if (openMV_get_data(openMV_mode_JT, &MV) == 0)
		{
			if ((OS_tick() - Lost_frame_time) > L_frame_TIME)
			{
				break; // 连续一段时间没有openMV的数据就放弃本次定位
			}
			continue;
		}
		Lost_frame_time = OS_tick();

		// 检查是否达到目标位置
		if (_abs(MV.x) <= Position_X_Limit)
		{
			positioning_complete = 1;
			break;
		}

		// 计算控制输出
		int32_t x_speed = K_p * MV.x;
		x_speed = _limit(x_speed, -MAX_Speed, MAX_Speed);
		// 设置速度
		set_speed_x(x_speed);
	}

	// 停止运动
	set_speed_x(0);

	if (positioning_complete)
	{
		int32_t Positioning_result = 0;
		start_time = OS_tick();
		while ((OS_tick() - start_time) < 500)
		{
			if (!openMV_get_data(openMV_mode_JT, &MV)) continue;
			Positioning_result += (MV.type == 0x54) ? 1 : (MV.type == 0x53 ? -1 : 0);
		}

		/* 如果二维码模块读取到，就不可能是方块和圆环，判定为误识别 */
		if (My_QR_Handle.QR_data != 0) positioning_complete = 4;

		if (Positioning_result > 0)
			positioning_complete = 2; // 方块
		else
			positioning_complete = 3; // 圆环
		log_info("JT定位结束\n");
	}
	else
	{
		log_info("JT定位超时\n");
	}
	return positioning_complete;
}
MAKE_CMD(uint8_t, JT_positioning, void);

/****************************************************************************************/

uint8_t track_isOnLine = 0;
/**
 * @brief 这个函数会在 my_tasks.c 中被 JT_TrackTask 循环调用(周期3ms)
 */
void JT_TrackTask_process(void)
{
	// 巡线，只有在最高阶梯时才强力校准陀螺仪
	if (is_site(SITE_RED))
		track_isOnLine = keep_line(36, (e_current_height == Max_height) ? 1 : 0, 0, NULL);
	else
		track_isOnLine = keep_line(24, (e_current_height == Max_height) ? 1 : 0, 0, NULL);

	/* 处理移动过程中阶梯的的高度变化 */
	int32_t X = get_odom_X_abs();
	// 如果不处于禁止判断高度切换的区域，则进行判断，判断结果存储在 height_switch 中
	// 如果height_switch不为0则说明已经有在准备切换高度了，不再进行判断
	if (X > prohibit_switch && height_switch == 0)
	{
		// 在最低的阶梯，左红外识别到木板就进行高度切换
		if (e_current_height == Min_height && get_HW(5))
		{
			height_switch = X + 30; // snum: 延迟高度切换
		}
		// 在最高的阶梯，右红外识别到不木板就进行高度切换
		else if (e_current_height == Max_height && !get_HW(1))
		{
			height_switch = X + 0; // snum: 延迟高度切换
		}
		// 在中间的阶梯，红外识别不到就进行高度切换
		else if (e_current_height == Mid_height && !get_HW(1))
		{
			height_switch = X + 160;
		}
	}
}

static void wait_online(uint32_t timeout)
{
	JT_Track_start();
	set_speed_x(0);
	uint32_t start = OS_tick();
	while (OS_tick() - start < timeout && (!track_isOnLine)) osDelay(2);
}

static void GET_it(uint8_t isBlock)
{
	log_info("巡线调整");
	wait_online(3000); // 确保在线上
	if (isBlock)
		log_info("抓取方块");
	else
		log_info("抓取圆环");
	JT_Track_stop();
	auto_arm_get();				  // 抓取起来
	move(-200, -90, 100, 10, 10); // 后退一点
	// 放到存储仓
	servo_action_w(isBlock ? ServoAG_JT_putBlock : ServoAG_JT_putRing, 1);
	wait_online(3000); // 确保在线上
	auto_arm_scan();   // 准备继续扫描
	JT_Track_start();
}

/**
 * @brief 扫描抓取
 */
void JT_scan(void)
{
	const int32_t Vx = 150; // snum:前进速度
	/* 初始化变量 */
	e_current_height = Min_height;
	prohibit_scan = prohibit_switch = height_switch = 0;
	reset_odom();
	openMV_set_mode(openMV_mode_JT);

	MV_DATA_t MV_data;
	auto_arm_scan();
	JT_Track_start();
	openMV_clear_data();
	My_QR_Handle._Set_QR_Data_Zero();
	int32_t last_get_odom = -1000;
	while (e_current_height != None_height)
	{
		height_switch_processer();
		// 如果两个循迹板都没有线就不要继续移动了，找到线再动
		set_speed_x((TrackBoard_read_L() || TrackBoard_read_R()) ? Vx : 0);

		if (get_odom_X_abs() < prohibit_scan) // 如果处于禁止扫描区域就不扫描
		{
			My_QR_Handle._Set_QR_Data_Zero();
			openMV_clear_data();
			continue;
		}

		if (My_QR_Handle._Get_QR_Data() == 1)
		{
			set_speed_x(0);
			log_info("抓二维码");
			GET_it(1);
			My_QR_Handle._Set_QR_Data_Zero();
			openMV_clear_data();
		}
		else if (openMV_get_data(openMV_mode_JT, &MV_data))
		{
			set_speed_x(0);
			uint32_t positioning_result = JT_positioning(); // 定位
			if (positioning_result == 2 || positioning_result == 3)
			{
				GET_it((positioning_result == 2) ? 1 : 0);

				if (abs(get_odom_X_abs() - last_get_odom) < 30)
				{
					// 在同一个地方抓了第二次，就跳过这个物料
					prohibit_scan = get_odom_X_abs() + 150;
				}
				last_get_odom = get_odom_X_abs();
			}

			/* 这里对误识别的处理 */
			if (positioning_result == 4)
			{
				if (My_QR_Handle._Get_QR_Data() == 1)
				{
					set_speed_x(0);
					log_info("抓二维码");
					GET_it(1);
				}
				else
					prohibit_scan = get_odom_X_abs() + 50;
			}

			My_QR_Handle._Set_QR_Data_Zero();
			openMV_clear_data();
		}
	}
	JT_Track_stop();
	set_speed_xy(0, 0);
	openMV_set_mode(openMV_mode_IDLE);
	servo_action_w(ServoAG_ARM_reset, 1);
	log_info("JT_scan OK");
}
MAKE_CMD(void, JT_scan, void);

/* 在阶梯平台前，调整姿态进入阶梯平台的一侧 */
void JT_ready(void)
{
	const int32_t X = 0; // snum:左移距离

	find_line(800, 36, -1);
	reset_odom();
	int32_t stop_x = 5000; // 停止位置，超过这个值就会停止左移，一开始给一个很大的数，检测红外之后再更新

	set_speed_x(-300); // snum:左移速度
	while (get_odom_X_abs() < stop_x)
	{
		if (stop_x == 5000 && !get_HW(4)) stop_x = get_odom_X_abs() + X; // 第一次检测到左侧红外不被遮挡，记录停止位置
		keep_line(24, 0, 0, NULL);
	}
	set_speed_xy(0, 0);
	log_info("JT_ready OK");
}
MAKE_CMD(void, JT_ready, void);

void JT_run(void)
{
	JT_ready();
	JT_scan();
}
MAKE_CMD(void, JT_run, void);
