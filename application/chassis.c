/*
LF RF
LB RB
X轴正方向为-->
Y轴正方向为^ ,与机械臂方向相同
Z轴正方向为逆时针转
电机旋转正方向为向前转
*/
#include "chassis.h"
#include "PID.h"
#include "fast_math.h"
#include "bsp.h"
#include "imu.h"
#include "motor_application.h"
#include "math.h"
#include "miniCMD_makeCmd.h"

/*
走固定距离的方案：
不用传感器，直接生成速度曲线，实现加减速，每个控制周期发送速度给电机
*/

// 控制周期(ms)
#define T_control 10
const float RAD_TO_DEG = 57.295779513f; // 180/PI

// 轮子周长
const int32_t mmpr = chassis_mmpr;

// 角度环PID参数
PID_param_t angle_pid_param = {.Kp = 10.0f, .Ki = 0, .Kd = 0, .DeadBand = 0, .output_limit = 2000, .integral_limit = 50};
PID_t angle_pid = {.param = &angle_pid_param};
static void set_angle_pid(float Kp, float Ki, float Kd, float DeadBand, float output_limit, float integral_limit)
{
	angle_pid_param.Kp = Kp;
	angle_pid_param.Ki = Ki;
	angle_pid_param.Kd = Kd;
	angle_pid_param.DeadBand = DeadBand;
	angle_pid_param.output_limit = output_limit;
	angle_pid_param.integral_limit = integral_limit;
	print_f("set: Kp=%g,Ki=%g,Kd=%g,DeadBand=%g,output_limit=%g,integral_limit=%g",
			angle_pid_param.Kp,
			angle_pid_param.Ki,
			angle_pid_param.Kd,
			angle_pid_param.DeadBand,
			angle_pid_param.output_limit,
			angle_pid_param.integral_limit);
}
MAKE_CMD(void, set_angle_pid, float, float, float, float, float, float);

static SemaphoreHandle_t chassisMutex;
static StaticSemaphore_t chassisMutex_buffer;
#define MutexTake xSemaphoreTakeRecursive(chassisMutex, 3 * T_control)
#define MutexGive xSemaphoreGiveRecursive(chassisMutex)

struct
{
	enum chassis_control_mode_e control_mode; // 控制模式
	int32_t odom_X, odom_Y;					  // 里程计，记录xy方向上的位移，单位(0.1rpm * 10ms)

	// 在chassis_control中,速度使用极坐标系表示
	int32_t V;			 // 速度大小（距离模式下自动由 calculate_V 计算） [单位: 0.1·rpm]
	int32_t Vdir;		 // 速度方向 [单位: 0.1·deg 范围: +-1800]
	int32_t Vz;			 // 角速度（角度模式下自动由PID计算） [单位: 0.1·rpm]
	int32_t targt_angle; // 角度环PID目标值，角度闭环(control_mode & Mode_Angle == 1)时有效 [单位:0.1·rpm]
	// 直角坐标系表示的速度，距离模式下禁止使用
	int32_t use_Vxy, Vx, Vy;

	/* 距离模式使用的参数 */
	int32_t Vmax, V_start, V_const, V_end;						  // 距离模式下的最大速度[单位:0.1·rpm]
	int32_t accel, decel, accel_start, accel_end;				  // 加速度 减速度 [单位: (0.1·rpm)/s 范围<1000000]
	uint32_t start_timeStamp;									  // 开始运动的时间戳
	int32_t acc_time, const_time, dec_time, start_time, end_time; // 加速、匀速、减速的时间 [ms]
} chassis;

static inline void calculate_V(uint32_t timeStamp)
{
	if (timeStamp < chassis.start_timeStamp) return;

	int32_t t = timeStamp - chassis.start_timeStamp;
	/* 第一加速度阶段 */
	if (t < chassis.start_time)
	{
		chassis.V = chassis.V_start + (float)(t * chassis.accel_start) / 1000.0f;
	}
	/* 匀速阶段 */
	else if (t < chassis.start_time + chassis.const_time)
	{
		chassis.V = chassis.V_const;
	}
	/* 第二加速度阶段 */
	else if (t < chassis.start_time + chassis.const_time + chassis.end_time)
	{
		chassis.V = chassis.V_const + (float)((t - chassis.start_time - chassis.const_time) * chassis.accel_end) / 1000.0f;
	}
	/* 到位 */
	else
	{
		chassis.V = chassis.V_end;
		chassis.control_mode &= (~Mode_Distance);
	}
}

/**
 * @brief 底盘控制实现，需要循环调用
 */
void chassis_control(void)
{
	MutexTake;
	uint32_t t = OS_tick(); // 实现周期性

	/* 距离模式或距离+角度模式下，计算速度V, 实现匀加减速和到位检测 */
	if (chassis.control_mode & Mode_Distance)
	{
		calculate_V(t);
		// print_f("%d", chassis.V);
	}
	// log_info_f("%d,%d,%d,%d", chassis.odom_Y, chassis.odom_X, chassis.odom_dis);

	/* 角度闭环下，计算角速度Vz，并优化速度方向 */
	int32_t dir = chassis.Vdir;			   // 速度方向，角度闭环下会被优化
	if (chassis.control_mode & Mode_Angle) // 角度闭环
	{
		int32_t imu_angle = imu_get_angle(); // 陀螺仪角度
		// 角速度由PID计算
		chassis.Vz = angle_PID_Calculate(&angle_pid, imu_angle, chassis.targt_angle);
		// print_f("%d,%d,%d\n", chassis.targt_angle, imu_angle, chassis.Vz);
		// 优化速度方向，实现边自转边走直线
		// dir += chassis.targt_angle - imu_angle;
	}

	// 极坐标系转到直角坐标系
	if (!chassis.use_Vxy)
	{
		float Vdir_sin, Vdir_cos;
		fast_sincos(dir, &Vdir_sin, &Vdir_cos);
		chassis.Vx = chassis.V * Vdir_cos;
		chassis.Vy = chassis.V * Vdir_sin;
	}

	// 运动逆解算
	Motor_Set_Speed(chassis.Vx, chassis.Vy, chassis.Vz);
	// print_f("%d,%d,%d\n", chassis.Vx, chassis.Vy, chassis.Vz);

	// 软件里程计
	chassis.odom_X += chassis.Vx;
	chassis.odom_Y += chassis.Vy;

	MutexGive;

	/* 实现周期性 */
	osDelayUntil(&t, T_control);
}

/**
 * @brief 底盘初始化
 */
void chassis_init(void)
{
	chassisMutex = xSemaphoreCreateRecursiveMutexStatic(&chassisMutex_buffer);
	Motor_Init();
	chassis_reset();
	imu_set_angle(0);
	set_Angle_enable(1);
}

/********************************* 处理模式切换 *********************************/

// 终止底盘运动（不会关闭角度环）
void chassis_stop(void)
{
	MutexTake;
	chassis.control_mode &= ~Mode_Distance; // 关闭距离模式
	chassis.V = chassis.Vx = chassis.Vy = chassis.Vz = 0;
	chassis.use_Vxy = 0;
	MutexGive;
}

// 重置底盘状态，关闭所有模式，清零里程计
void chassis_reset(void)
{
	MutexTake;
	chassis.control_mode = Mode_OpenLoop; // 关闭所有模式
	chassis.odom_X = chassis.odom_Y = 0;  // 里程计清零
	chassis_stop();
	MutexGive;
	log_info("底盘状态重置\n");
}

// 开关角度环（可能会终止距离模式）
void set_Angle_enable(uint8_t enable)
{
	if ((!!enable) == (!!(chassis.control_mode & Mode_Angle))) return; // 无需切换
	MutexTake;
	/* 在距离模式中开关角度环，视为不正常的调用 */
	if (chassis.control_mode & Mode_Distance)
		log_error("开关角度环时处于距离模式\n");

	/* 避免切换模式后突变 */
	chassis_stop();
	if (enable && (!(chassis.control_mode & Mode_Angle)))
		chassis.targt_angle = imu_get_angle(); // 开启角度环之前，使当前角度为目标角度

	/* 切换模式 */
	chassis.control_mode = enable ? Mode_Angle : Mode_OpenLoop;
	MutexGive;
}
MAKE_CMD(void, set_Angle_enable, uint8_t);

enum chassis_control_mode_e get_control_mode(void)
{
	return chassis.control_mode;
}

/********************************* 开环控制API *********************************/
// todo: 需要重新整理一下分类
// 底盘速度控制（极坐标系），全开环
void set_speed_polarZ(int32_t V, int32_t dir, int32_t Vz)
{
	MutexTake;
	set_Angle_enable(0); // 关闭角度环和距离模式
	chassis.V = V;
	chassis.Vdir = dir;
	chassis.Vz = Vz;
	chassis.use_Vxy = 0;
	MutexGive;
}

// 底盘速度控制（直角坐标系），全开环
void set_speed_xyZ(int32_t Vx, int32_t Vy, int32_t Vz)
{
	MutexTake;
	set_Angle_enable(0); // 关闭角度环和距离模式
	chassis.Vx = Vx;
	chassis.Vy = Vy;
	chassis.Vz = Vz;
	chassis.use_Vxy = 1;
	MutexGive;
}
MAKE_CMD(void, set_speed_xyZ, int32_t, int32_t, int32_t);

void set_speed_RXY(int32_t r, int32_t Vx, int32_t Vy)
{
	MutexTake;
	set_Angle_enable(0); // 关闭角度环和距离模式
	chassis.Vx = Vx;
	chassis.Vy = Vy;
	chassis.Vz = 0.013f * Vx * r;
	chassis.use_Vxy = 1;
	MutexGive;
}
MAKE_CMD(void, set_speed_RXY, int32_t, int32_t, int32_t);

void reset_odom(void)
{
	MutexTake;
	chassis.odom_X = chassis.odom_Y = 0;
	MutexGive;
}
// 单位转换：把速度单位 0.1rpm 的每10ms的积分转为mm
int32_t get_odom_X(void)
{
	// 即 (chassis.odom_X / 10) / 60 / 100 * mmpr
	// -> chassis.odom_X / (60000 / mmpr)
	// -> chassis.odom_X / 255.31914893617
	// -> chassis.odom_X / 256
	return chassis.odom_X / 256;
}
// 单位mm
int32_t get_odom_Y(void)
{
	return chassis.odom_Y / 256;
}

int32_t get_odom_X_abs(void)
{
	return abs(get_odom_X());
}

int32_t get_odom_distance(void)
{
	float X = chassis.odom_X / 256.0f, Y = chassis.odom_Y / 256.0f;
	return sqrtf(X * X + Y * Y);
}

/********************************* 角度闭环API *********************************/

// 底盘速度控制（极坐标系），角度环开启
void set_speed_polar(int32_t V, int32_t dir)
{
	MutexTake;
	set_Angle_enable(1); // 自动开启角度环
	chassis.V = V;
	chassis.Vdir = dir;
	chassis.use_Vxy = 0;
	MutexGive;
}

// 底盘速度控制(直角坐标系)，角度锁定
void set_speed_xy(int32_t Vx, int32_t Vy)
{
	MutexTake;
	set_Angle_enable(1); // 自动开启角度环
	chassis.Vx = Vx;
	chassis.Vy = Vy;
	chassis.use_Vxy = 1;
	MutexGive;
}

/**
 * @brief Y轴底盘速度控制(直角坐标系)
 * @param Vy Y轴速度
 */
void set_speed_y(int32_t Vy)
{
	MutexTake;
	chassis.Vy = Vy;
	chassis.use_Vxy = 1;
	MutexGive;
}

/**
 * @brief X轴底盘速度控制(直角坐标系)
 * @param Vx X轴速度
 */
void set_speed_x(int32_t Vx)
{
	MutexTake;
	chassis.Vx = Vx;
	chassis.use_Vxy = 1;
	MutexGive;
}

void set_speed_z(int32_t Vz)
{
	MutexTake;
	chassis.Vz = Vz;
	MutexGive;
}

// 转过一定角度
void turn_angle(int32_t angle)
{
	MutexTake;
	set_Angle_enable(1); // 自动开启角度环
	chassis.targt_angle = limit_angle(chassis.targt_angle + angle);
	MutexGive;
}
MAKE_CMD(void, turn_angle, int32_t);

void wait_angle(void)
{
	uint32_t startTick = OS_tick();
	while (abs(chassis.targt_angle - imu_get_angle()) > 20)
	{
		if (OS_tick() - startTick > 8000) break; // snum: 等待超时
		osDelay(3);
	}
}

// 转过一定角度
void turn_angle_w(int32_t angle)
{
	turn_angle(angle);
	wait_angle();
}
MAKE_CMD(void, turn_angle_w, int32_t);

// 转到指定角度，不建议使用，绝对角度不利于模块调试
void set_angle(int32_t angle)
{
	MutexTake;
	set_Angle_enable(1); // 自动开启角度环
	chassis.targt_angle = angle;
	MutexGive;
}

void set_angle_w(int32_t angle)
{
	set_angle(angle);
	wait_angle();
}

// 获取目标角度
int32_t get_targetAngle(void)
{
	return chassis.targt_angle;
}

/********************************* 距离模式API *********************************/

/**
 * @brief 不是对外提供的API，完全指定距离模式下运动速度曲线
 * 可能出现以下情况:
 * 1. 正常3段速度（只要运动距离足够长都是这种情况）
 *    - 速度从 [初速] 以 [accel_start] 加/减速到 [匀速] ，匀速运动一段时间，后以 [accel_end] 加/减速到 [V_end]
 *    - 当匀速=初速时只有后面两个阶段，当匀速=末速时只有前面两个阶段
 * 2. 运动距离过短（在第1中情况下，随着运动距离减小，匀速时间会减小，直到没有匀速阶段，此时再减小距离就会使两段匀加减速冲突）
 *    - 改变匀速值，在严格按照指定加减速度的情况下，使两段加减速恰好无缝衔接
 * 3. 即使调整匀速值，依然无法在距离内以指定加减速度达到末速度
 *    - 改变加减速度，使速度恰好是一条直线
 * @param V_start (mm/s) 初速，可以比V_const大，此时accel_start是减速度而不是加速度；范围[0,3000]
 * @param V_const (mm/s) 匀速，匀速运动时的速度；范围[1,3000]
 * @param V_end (mm/s) 末速；范围[0,3000]
 * @param dir (deg) 移动方向
 * @param distance (mm) 移动距离；范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 第一段加速度大小，从初速到匀速的加速度；范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 第三段加速度大小，从匀速到末速的减速度；范围[1,3000]
 */
void move_3para(int32_t V_start, int32_t V_const, int32_t V_end, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end)
{
	/* 参数合法化处理 */
	V_start = limit(_abs(V_start), 0, 3000);
	V_const = limit(_abs(V_const), 1, 3000);
	V_end = limit(_abs(V_end), 0, 3000);
	dir = limit_angle(dir * 10);
	distance = limit(_abs(distance), 1, 10000);
	accel_start = limit(_abs(accel_start), 1, 3000);
	accel_end = limit(_abs(accel_end), 1, 3000);

	if (V_const < V_start && V_const < V_end) V_const = _max(V_start, V_end); // 对匀速值初步调整，先减速再加速的运动没有意义

	/* 加速度单位从 ((mm/s)/10ms) 转为 ((mm/s)/s) , 方便计算, 范围变为[1,300000] */
	accel_start *= 100;
	accel_end *= 100;
	/* 判断加速度符号 */
	if (V_const < V_start) accel_start = -accel_start;
	if (V_end < V_const) accel_end = -accel_end;

	int32_t start_time = 0, end_time = 0, const_time = 0;

	float d1 = (float)(V_const * V_const - V_start * V_start) / (float)(2 * accel_start); // 正常完成第一阶段的距离(需要绝对值)
	float d3 = (float)(V_end * V_end - V_const * V_const) / (float)(2 * accel_end);		  // 正常完成第三阶段的距离(需要绝对值)
	float d2 = distance - d1 - d3;
	if (d2 >= 0.0f)
	{
		log_info("正常三段运动");
		V_const = V_const;
		accel_start = accel_start;
		accel_end = accel_end;
		start_time = (float)(1000 * (V_const - V_start)) / (float)accel_start;
		end_time = (float)(1000 * (V_end - V_const)) / (float)accel_end;
		const_time = (1000.0f * d2) / (float)V_const;
	}
	else
	{
		/*
		无法进入匀速段，尝试调整匀速值使 d1 + d3 = distance
		d1 + d3 = (V_const^2-V_start^2)/2*a1 + (V_end^2-V_const^2)/2*a2 = distance
		整理得
		V_const^2 = (2*a1*a2*distance + a2*V_start^2 - a1*V_end^2) / (a2 - a1)
		*/
		float vv = -1.0f;
		if (accel_end != accel_start) // 避免除零
		{
			vv = ((2.0f * (float)accel_start * (float)accel_end * (float)distance) +
				  ((float)accel_end * (float)(V_start * V_start)) -
				  ((float)accel_start * (float)(V_end * V_end))) /
				 (float)(accel_end - accel_start);
		}

		if (vv >= 0.0f)
		{
			V_const = sqrtf(vv); // 计算合适的匀速
			log_info_f("调整匀速值 %d", V_const);
			accel_start = accel_start;
			accel_end = accel_end;
			start_time = (float)(1000 * (V_const - V_start)) / (float)accel_start;
			end_time = (float)(1000 * (V_end - V_const)) / (float)accel_end;
			const_time = 0; // 没有第二阶段
		}
		else
		{
			// 极端情况，调整加速度，使速度成为一条直线, 只有第三阶段
			V_const = V_start;
			accel_end = (float)(V_end * V_end - V_start * V_start) / (float)(2 * distance); // 计算合适的加速度
			log_info_f("调整加速度 %d", accel_end);
			accel_start = 300000;
			start_time = 0;
			const_time = 0;
			end_time = (float)(1000 * (V_end - V_start)) / (float)accel_end;
		}
	}

	/* 单位转换 */
	// 速度单位从 (mm/s) 转为 (0.1rpm)
	V_start = (float)(V_start * 600) / (float)mmpr;
	V_const = (float)(V_const * 600) / (float)mmpr;
	V_end = (float)(V_end * 600) / (float)mmpr;
	// 加速度单位从 ((mm/s)/s) 转为 (0.1rpm/s)
	accel_start = (float)accel_start * 600.0f / mmpr;
	accel_end = (float)accel_end * 600.0f / mmpr;

	/* 最终计算结果 */
	chassis.use_Vxy = 0;
	chassis.Vdir = dir;					 // 方向
	chassis.V_start = V_start;			 // 初速
	chassis.V_const = V_const;			 // 匀速
	chassis.V_end = V_end;				 // 末速
	chassis.accel_start = accel_start;	 // 第一阶段的加速度
	chassis.accel_end = accel_end;		 // 第三阶段的加速度
	chassis.start_time = start_time;	 // 第一阶段需要的时间
	chassis.const_time = const_time;	 // 第二阶段需要的时间
	chassis.end_time = end_time;		 // 第三阶段需要的时间
	chassis.start_timeStamp = OS_tick(); // 开始的时间戳
	chassis.control_mode |= Mode_Distance;

	log_info_f("V_start=%d, V_const=%d, V_end=%d\naccel_start=%d, accel_end=%d\nstart_time=%d, const_time=%d, end_time=%d\n",
			   chassis.V_start, chassis.V_const, chassis.V_end,
			   chassis.accel_start, chassis.accel_end,
			   chassis.start_time, chassis.const_time, chassis.end_time);
}

/**
 * @brief 指定末速度的距离模式，以当前速度在目标方向上的投影作为初速度
 * @param V_const (mm/s) 匀速，匀速运动时的速度；范围[1,3000]
 * @param V_end (mm/s) 末速；范围[0,3000]
 * @param dir (deg) 移动方向
 * @param distance (mm) 移动距离；范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 第一段加速度大小，从初速到匀速的加速度；范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 第三段加速度大小，从匀速到末速的减速度；范围[1,3000]
 */
inline void move_Ve(int32_t V_const, int32_t V_end, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end)
{
	int32_t V_start;
	if (chassis.use_Vxy)
	{
		float V_start_x, V_start_y; // 目标方向的单位向量
		fast_sincos(dir, &V_start_y, &V_start_x);
		V_start = chassis.Vx * V_start_x + chassis.Vy * V_start_y; // 点积就是投影长度
	}
	else
	{
		V_start = chassis.V * fast_cos(dir - chassis.Vdir);
	}
	V_start = (float)(V_start * mmpr) / 600.0f;
	move_3para(V_start, V_const, V_end, dir, distance, accel_start, accel_end);
}

/**
 * @brief 距离模式，以当前速度在目标方向上的投影作为初速度，末速度为0
 * @param V_const (mm/s) 匀速，匀速运动时的速度；范围[1,3000]
 * @param dir (deg) 移动方向
 * @param distance (mm) 移动距离；范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 第一段加速度大小，从初速到匀速的加速度；范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 第三段加速度大小，从匀速到末速的减速度；范围[1,3000]
 */
void move(int32_t V_const, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end)
{
	move_Ve(V_const, 0, dir, distance, accel_start, accel_end);
}
MAKE_CMD(void, move, int32_t, int32_t, int32_t, int32_t, int32_t);

void wait_move_ok(void)
{
	while (chassis.control_mode & Mode_Distance) osDelay(2);
}

/**
 * @brief 平移一定距离，不会开关角度环
 * @param V 最大速度 (mm/s) 范围[1,3000]
 * @param dir 运动方向 (deg) 范围[-180,180]
 * @param distance 移动距离 (mm) 范围[1,10000]
 * @param accel 加速度 ((mm/s)/10ms) 范围[1,3000]
 * @param decel 减速度 ((mm/s)/10ms) 范围[1,3000]
 */
void move_w(int32_t V, int32_t dir, int32_t distance, int32_t accel, int32_t decel)
{
	move(V, dir, distance, accel, decel);
	wait_move_ok();
}
MAKE_CMD(void, move_w, int32_t, int32_t, int32_t, int32_t, int32_t);

/**
 * @brief  直角坐标系下的平移一定距离
 * @param V 最大速度 (mm/s) 范围[1,3000]
 * @param d_X X轴方向的距离 (mm) 范围[-10000,10000]
 * @param d_Y Y轴方向的距离 (mm) 范围[-10000,10000]
 * @param accel 加速度 ((mm/s)/10ms) 范围[1,3000]
 * @param decel 减速度 ((mm/s)/10ms) 范围[1,3000]
 */
void move_XY_w(int32_t V, int32_t d_X, int32_t d_Y, int32_t accel, int32_t decel)
{
	move(V, atan2f((float)d_Y, (float)d_X) * RAD_TO_DEG, sqrt(d_X * d_X + d_Y * d_Y), accel, decel);
	wait_move_ok();
}
MAKE_CMD(void, move_XY_w, int32_t, int32_t, int32_t, int32_t, int32_t);

void move_XY_w_v_end(int32_t V,int32_t V_end,int32_t d_X, int32_t d_Y, int32_t accel, int32_t decel)
{
	move_Ve(V, V_end, atan2f((float)d_Y, (float)d_X) * RAD_TO_DEG, sqrt(d_X * d_X + d_Y * d_Y), accel, decel);
	wait_move_ok();
}


