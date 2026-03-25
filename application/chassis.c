/*
LF RF : 左前轮 右前轮
LB RB : 左后轮 右后轮
X轴正方向向右 -->
Y轴正方向向前 ^ , 逆时针为正方向
Z轴正方向为逆时针旋转
底盘运动学解算与控制模块
*/
#include "chassis.h"
#include "PID.h"
#include "fast_math.h"
#include "bsp.h"
#include "imu.h"
#include "motor_application.h"
#include "math.h"
#include "miniCMD_makeCmd.h"
#include "encoder_math.h"
#include "encoder_wheel.h"

/*
底盘控制相关参数设置
用于实现底盘的闭环控制、里程计积分以及梯形加减速轨迹规划
*/

// 控制周期(ms)
#define T_control 10
const float RAD_TO_DEG = 57.295779513f; // 弧度转角度系数 180/PI

// 轮子每圈对应的毫米数 (Millimeters Per Revolution)
const int32_t mmpr = chassis_mmpr;

// 角度闭环PID参数初始化
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
    enum chassis_control_mode_e control_mode; // 底盘当前控制模式
    int32_t odom_X, odom_Y;                  // 底盘里程计XY坐标累计位置 单位(0.1rpm * 10ms)

    // 在 chassis_control 任务中，会被解算器覆盖更新
    int32_t V;        // 极坐标系下的设定线速度，由 calculate_V 计算 [单位: 0.1rpm]
    int32_t Vdir;      // 极坐标系下的速度方向角 [单位: 0.1deg 范围: +-1800]
    int32_t Vz;           // 自旋速度，通常由角度PID计算得出 [单位: 0.1rpm]
    int32_t targt_angle; // 目标偏航角，在启用角度闭环(control_mode & Mode_Angle == 1)时有效 [单位:0.1rpm]
    // 是否使用直角坐标系速度控制标志位，以及XY正交速度分量
    int32_t use_Vxy, Vx, Vy;

    /* 梯形加减速曲线参数 */
    int32_t Vmax, V_start, V_const, V_end;                  // 速度段参数规划 [单位:0.1rpm]
    int32_t accel, decel, accel_start, accel_end;             // 加速度、减速度参数 [单位: (0.1rpm)/s 范围<1000000]
    uint32_t start_timeStamp;                            // 运动轨迹开始时间戳
    int32_t acc_time, const_time, dec_time, start_time, end_time; // 各个运动阶段的持续时间 [ms]
} chassis;

static inline void calculate_V(uint32_t timeStamp)
{
    if (timeStamp < chassis.start_timeStamp) return;

    int32_t t = timeStamp - chassis.start_timeStamp;
    /* 加速阶段计算 */
    if (t < chassis.start_time)
    {
       chassis.V = chassis.V_start + (float)(t * chassis.accel_start) / 1000.0f;
    }
    /* 匀速阶段计算 */
    else if (t < chassis.start_time + chassis.const_time)
    {
       chassis.V = chassis.V_const;
    }
    /* 减速阶段计算 */
    else if (t < chassis.start_time + chassis.const_time + chassis.end_time)
    {
       chassis.V = chassis.V_const + (float)((t - chassis.start_time - chassis.const_time) * chassis.accel_end) / 1000.0f;
    }
    /* 运动完成/停止阶段 */
    else
    {
       chassis.V = chassis.V_end;
       chassis.control_mode &= (~Mode_Distance);
    }
}

/**
 * @brief 底盘控制主循环核心任务
 */
void chassis_control(void)
{
    MutexTake;
    uint32_t t = OS_tick(); // 获取当前系统时间戳

    BSP_Encoder_Update();

    /* 如果处于距离控制模式，计算当前时刻设定的速度V，用于加减速曲线规划 */
    if (chassis.control_mode & Mode_Distance)
    {
       calculate_V(t);
       // print_f("%d", chassis.V);
    }
    // log_info_f("%d,%d,%d,%d", chassis.odom_Y, chassis.odom_X, chassis.odom_dis);

    /* 角度控制模式下，计算需要补偿的自旋速度Vz */
    int32_t dir = chassis.Vdir;             // 暂存极坐标方向角
    if (chassis.control_mode & Mode_Angle) // 处于角度闭环模式
    {
       int32_t imu_angle = imu_get_angle(); // 获取当前IMU偏航角
       // 通过PID计算需要补偿的旋转速度Vz
       chassis.Vz = angle_PID_Calculate(&angle_pid, imu_angle, chassis.targt_angle);
       // print_f("%d,%d,%d\n", chassis.targt_angle, imu_angle, chassis.Vz);
       // 可选：将角度偏差补偿到运动方向中
       // dir += chassis.targt_angle - imu_angle;
    }

    // 如果不使用直角坐标控制，则将极坐标速度分解为XY正交速度
    if (!chassis.use_Vxy)
    {
       float Vdir_sin, Vdir_cos;
       fast_sincos(dir, &Vdir_sin, &Vdir_cos);
       chassis.Vx = chassis.V * Vdir_cos;
       chassis.Vy = chassis.V * Vdir_sin;
    }

    // 设置电机底层速度输出
    Motor_Set_Speed(chassis.Vx, chassis.Vy, chassis.Vz);
    // print_f("%d,%d,%d\n", chassis.Vx, chassis.Vy, chassis.Vz);

    // 积分计算里程计坐标
    chassis.odom_X += chassis.Vx;
    chassis.odom_Y += chassis.Vy;

    MutexGive;

    /* 延时以保证绝对的控制周期 */
    osDelayUntil(&t, T_control);
}

/**
 * @brief 底盘系统初始化
 */
void chassis_init(void)
{
    chassisMutex = xSemaphoreCreateRecursiveMutexStatic(&chassisMutex_buffer);
    Motor_Init();
    BSP_Encoder_Init();
    chassis_reset();
    imu_set_angle(0);
    set_Angle_enable(1);
}

/********************************* 模式控制与切换 *********************************/

// 紧急停止底盘运动并清除控制状态
void chassis_stop(void)
{
    MutexTake;
    chassis.control_mode &= ~Mode_Distance; // 清除距离控制模式
    chassis.V = chassis.Vx = chassis.Vy = chassis.Vz = 0;
    chassis.use_Vxy = 0;
    MutexGive;
}

// 复位底盘状态、里程计与控制模式
void chassis_reset(void)
{
    MutexTake;
    chassis.control_mode = Mode_OpenLoop; // 恢复为开环模式
    chassis.odom_X = chassis.odom_Y = 0;  // 清零里程计
    chassis_stop();
    MutexGive;
    log_info("底盘状态已复位\n");
}

// 开启或关闭底盘角度闭环控制
void set_Angle_enable(uint8_t enable)
{
    if ((!!enable) == (!!(chassis.control_mode & Mode_Angle))) return; // 状态相同则直接返回
    MutexTake;
    /* 处于距离控制模式下不允许直接切换角度闭环状态，以防发生失控 */
    if (chassis.control_mode & Mode_Distance)
       log_error("距离模式下禁止切换角度控制\n");

    /* 切换状态前先停止运动 */
    chassis_stop();
    if (enable && (!(chassis.control_mode & Mode_Angle)))
       chassis.targt_angle = imu_get_angle(); // 开启闭环时，以当前角度为初始目标角度

    /* 更新控制模式标志位 */
    chassis.control_mode = enable ? Mode_Angle : Mode_OpenLoop;
    MutexGive;
}
MAKE_CMD(void, set_Angle_enable, uint8_t);

enum chassis_control_mode_e get_control_mode(void)
{
    return chassis.control_mode;
}

/********************************* 开环速度设定 API *********************************/
// todo: 完善并测试这些开环接口
// 设定极坐标系速度及自旋速度 (开环)
void set_speed_polarZ(int32_t V, int32_t dir, int32_t Vz)
{
    MutexTake;
    set_Angle_enable(0); // 关闭角度闭环
    chassis.V = V;
    chassis.Vdir = dir;
    chassis.Vz = Vz;
    chassis.use_Vxy = 0;
    MutexGive;
}

// 设定直角坐标系正交速度及自旋速度 (开环)
void set_speed_xyZ(int32_t Vx, int32_t Vy, int32_t Vz)
{
    MutexTake;
    set_Angle_enable(0); // 关闭角度闭环
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
    set_Angle_enable(0); // 关闭角度闭环
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
// 将里程计内部单位(0.1rpm*10ms)转换为实际位移毫米(mm)
int32_t get_odom_X(void)
{
    // 推导: (chassis.odom_X / 10) / 60 / 100 * mmpr
    // -> chassis.odom_X / (60000 / mmpr)
    // -> chassis.odom_X / 255.31914893617
    // -> chassis.odom_X / 256
    return chassis.odom_X / 256;
}
// 获取Y轴里程，单位mm
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

/********************************* 闭环速度设定 API *********************************/

// 设定极坐标系速度 (开启角度闭环保持航向)
void set_speed_polar(int32_t V, int32_t dir)
{
    MutexTake;
    set_Angle_enable(1); // 开启角度闭环
    chassis.V = V;
    chassis.Vdir = dir;
    chassis.use_Vxy = 0;
    MutexGive;
}

// 设定直角坐标系速度 (开启角度闭环保持航向)
void set_speed_xy(int32_t Vx, int32_t Vy)
{
    MutexTake;
    set_Angle_enable(1); // 开启角度闭环
    chassis.Vx = Vx;
    chassis.Vy = Vy;
    chassis.use_Vxy = 1;
    MutexGive;
}

/**
 * @brief 设定Y轴移动速度 (开启角度闭环)
 * @param Vy Y轴速度分量
 */
void set_speed_y(int32_t Vy)
{
    MutexTake;
    chassis.Vy = Vy;
    chassis.use_Vxy = 1;
    MutexGive;
}

/**
 * @brief 设定X轴移动速度 (开启角度闭环)
 * @param Vx X轴速度分量
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

// 增量式旋转指定角度
void turn_angle(int32_t angle)
{
    MutexTake;
    set_Angle_enable(1); // 开启角度闭环
    chassis.targt_angle = limit_angle(chassis.targt_angle + angle);
    MutexGive;
}
MAKE_CMD(void, turn_angle, int32_t);

void wait_angle(void)
{
    uint32_t startTick = OS_tick();
    while (abs(chassis.targt_angle - imu_get_angle()) > 20)
    {
       if (OS_tick() - startTick > 8000) break; // 超时退出防死锁机制
       osDelay(3);
    }
}

// 阻塞式旋转指定角度 (旋转完毕后再返回)
void turn_angle_w(int32_t angle)
{
    turn_angle(angle);
    wait_angle();
}
MAKE_CMD(void, turn_angle_w, int32_t);

// 绝对式设定目标偏航角度
void set_angle(int32_t angle)
{
    MutexTake;
    set_Angle_enable(1); // 开启角度闭环
    chassis.targt_angle = angle;
    MutexGive;
}

void set_angle_w(int32_t angle)
{
    set_angle(angle);
    wait_angle();
}

// 获取当前设定的目标偏航角
int32_t get_targetAngle(void)
{
    return chassis.targt_angle;
}

/********************************* 轨迹规划 API *********************************/

/**
 * @brief 三段式速度规划移动API，实现梯形加减速曲线控制
 * 运动轨迹规划数学推导说明:
 * 1. 运动分为3个阶段：加速段、匀速段、减速段。
 * - 从 [V_start] 以 [accel_start] 的加速度加速至 [V_const]，再以 [accel_end] 的减速度减速至 [V_end]
 * - 实际运行总距离 = 加速段位移 + 匀速段位移 + 减速段位移
 * 2. 如果设定的距离过短，不足以达到设定的最高匀速，则会重新计算能达到的最大速度，此时匀速段距离为0。
 * - 代码内部会通过求解一元二次方程自动修正最高速度。
 * 3. 传入的参数会进行限幅保护，防止输入非法的控制参数。
 * - 自动将毫米等国际单位转换为内部编码器计算单位。
 * @param V_start (mm/s) 起始起步速度，目标速度恒大于该值，范围[0,3000]
 * @param V_const (mm/s) 目标匀速阶段的恒定速度，范围[1,3000]
 * @param V_end (mm/s) 运动结束时的末速度，范围[0,3000]
 * @param dir (deg) 极坐标系下的移动方向角
 * @param distance (mm) 移动的总距离设定值，范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 起步加速阶段的加速度绝对值，范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 刹车减速阶段的减速度绝对值，范围[1,3000]
 */
void move_3para(int32_t V_start, int32_t V_const, int32_t V_end, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end)
{
    /* 参数限幅与异常值处理 */
    V_start = limit(_abs(V_start), 0, 3000);
    V_const = limit(_abs(V_const), 1, 3000);
    V_end = limit(_abs(V_end), 0, 3000);
    dir = limit_angle(dir * 10);
    distance = limit(_abs(distance), 1, 10000);
    accel_start = limit(_abs(accel_start), 1, 3000);
    accel_end = limit(_abs(accel_end), 1, 3000);

    if (V_const < V_start && V_const < V_end) V_const = _max(V_start, V_end); // 修正逻辑：如果设定的匀速值反而最小，将其提升至起步和结束的最高值

    /* 加速度单位统一转换 ((mm/s)/10ms) 转换为 ((mm/s)/s) , 乘以100进行放大，范围变成[1,300000] */
    accel_start *= 100;
    accel_end *= 100;
    /* 判断加减速符号逻辑 */
    if (V_const < V_start) accel_start = -accel_start;
    if (V_end < V_const) accel_end = -accel_end;

    int32_t start_time = 0, end_time = 0, const_time = 0;

    float d1 = (float)(V_const * V_const - V_start * V_start) / (float)(2 * accel_start); // 计算加速段理论所需距离 (基于运动学公式 v^2 - v0^2 = 2ax)
    float d3 = (float)(V_end * V_end - V_const * V_const) / (float)(2 * accel_end);         // 计算减速段理论所需距离 (基于运动学公式 v^2 - v0^2 = 2ax)
    float d2 = distance - d1 - d3;
    if (d2 >= 0.0f)
    {
       log_info("规划成功，存在匀速段");
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
       如果设定的总距离不足以完成完整的梯形加速，则匀速段距离变为0，此时 d1 + d3 = distance
       d1 + d3 = (V_const^2-V_start^2)/2*a1 + (V_end^2-V_const^2)/2*a2 = distance
       求解上述方程推导出实际可达到的最大速度平方为:
       V_const^2 = (2*a1*a2*distance + a2*V_start^2 - a1*V_end^2) / (a2 - a1)
       */
       float vv = -1.0f;
       if (accel_end != accel_start) // 加速度不同时的公式求解
       {
          vv = ((2.0f * (float)accel_start * (float)accel_end * (float)distance) +
               ((float)accel_end * (float)(V_start * V_start)) -
               ((float)accel_start * (float)(V_end * V_end))) /
              (float)(accel_end - accel_start);
       }

       if (vv >= 0.0f)
       {
          V_const = sqrtf(vv); // 计算出实际能达到的最大速度
          log_info_f("修正后的最高速度为 %d", V_const);
          accel_start = accel_start;
          accel_end = accel_end;
          start_time = (float)(1000 * (V_const - V_start)) / (float)accel_start;
          end_time = (float)(1000 * (V_end - V_const)) / (float)accel_end;
          const_time = 0; // 匀速段时间为0
       }
       else
       {
          // 异常情况兜底处理，直接以固定减速度进行刹车
          V_const = V_start;
          accel_end = (float)(V_end * V_end - V_start * V_start) / (float)(2 * distance); // 重新计算实际减速度
          log_info_f("异常修正，重设减速度为 %d", accel_end);
          accel_start = 300000;
          start_time = 0;
          const_time = 0;
          end_time = (float)(1000 * (V_end - V_start)) / (float)accel_end;
       }
    }

    /* 单位量纲转换 */
    // 将线速度单位 (mm/s) 转换为底盘内部速度控制单位 (0.1rpm)
    V_start = (float)(V_start * 600) / (float)mmpr;
    V_const = (float)(V_const * 600) / (float)mmpr;
    V_end = (float)(V_end * 600) / (float)mmpr;
    // 将加速度单位 ((mm/s)/s) 转换为内部控制单位 (0.1rpm/s)
    accel_start = (float)accel_start * 600.0f / mmpr;
    accel_end = (float)accel_end * 600.0f / mmpr;

    /* 将计算好的参数填入底盘结构体准备执行 */
    chassis.use_Vxy = 0;
    chassis.Vdir = dir;                 // 目标极角
    chassis.V_start = V_start;        // 起始速度
    chassis.V_const = V_const;        // 匀速段速度
    chassis.V_end = V_end;           // 末尾速度
    chassis.accel_start = accel_start;  // 加速段实际加速度
    chassis.accel_end = accel_end;     // 减速段实际减速度
    chassis.start_time = start_time;    // 规划加速持续时间
    chassis.const_time = const_time;    // 规划匀速持续时间
    chassis.end_time = end_time;       // 规划减速持续时间
    chassis.start_timeStamp = OS_tick(); // 记录运动轨迹起步时间戳
    chassis.control_mode |= Mode_Distance; // 开启距离控制模式

    log_info_f("V_start=%d, V_const=%d, V_end=%d\naccel_start=%d, accel_end=%d\nstart_time=%d, const_time=%d, end_time=%d\n",
             chassis.V_start, chassis.V_const, chassis.V_end,
             chassis.accel_start, chassis.accel_end,
             chassis.start_time, chassis.const_time, chassis.end_time);
}

/**
 * @brief 带有结束速度的移动API封装，允许设置运动结束时的非零速度
 * @param V_const (mm/s) 目标匀速阶段的恒定速度，范围[1,3000]
 * @param V_end (mm/s) 运动结束时的末速度，范围[0,3000]
 * @param dir (deg) 极坐标系下的移动方向角
 * @param distance (mm) 移动的总距离设定值，范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 起步加速阶段的加速度绝对值，范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 刹车减速阶段的减速度绝对值，范围[1,3000]
 */
inline void move_Ve(int32_t V_const, int32_t V_end, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end)
{
    int32_t V_start;
    if (chassis.use_Vxy)
    {
       float V_start_x, V_start_y; // 提取当前的XY轴速度进行合成
       fast_sincos(dir, &V_start_y, &V_start_x);
       V_start = chassis.Vx * V_start_x + chassis.Vy * V_start_y; // 根据当前运行状态计算出当前速度作为起点速度
    }
    else
    {
       V_start = chassis.V * fast_cos(dir - chassis.Vdir);
    }
    V_start = (float)(V_start * mmpr) / 600.0f;
    move_3para(V_start, V_const, V_end, dir, distance, accel_start, accel_end);
}

/**
 * @brief 常规移动API，默认运动到终点后完全静止(结束速度为0)
 * @param V_const (mm/s) 目标匀速阶段的恒定速度，范围[1,3000]
 * @param dir (deg) 极坐标系下的移动方向角
 * @param distance (mm) 移动的总距离设定值，范围[1,10000]
 * @param accel_start ((mm/s)/10ms) 起步加速阶段的加速度绝对值，范围[1,3000]
 * @param accel_end ((mm/s)/10ms) 刹车减速阶段的减速度绝对值，范围[1,3000]
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
 * @brief 阻塞式常规移动API，移动完成前挂起当前任务
 * @param V 目标匀速 (mm/s) 范围[1,3000]
 * @param dir 移动方向角 (deg) 范围[-180,180]
 * @param distance 移动距离 (mm) 范围[1,10000]
 * @param accel 起步加速度 ((mm/s)/10ms) 范围[1,3000]
 * @param decel 刹车减速度 ((mm/s)/10ms) 范围[1,3000]
 */
void move_w(int32_t V, int32_t dir, int32_t distance, int32_t accel, int32_t decel)
{
    move(V, dir, distance, accel, decel);
    wait_move_ok();
}
MAKE_CMD(void, move_w, int32_t, int32_t, int32_t, int32_t, int32_t);

/**
 * @brief  阻塞式直角坐标位移API，指定XY偏移量进行运动
 * @param V 目标匀速 (mm/s) 范围[1,3000]
 * @param d_X X轴相对偏移距离 (mm) 范围[-10000,10000]
 * @param d_Y Y轴相对偏移距离 (mm) 范围[-10000,10000]
 * @param accel 起步加速度 ((mm/s)/10ms) 范围[1,3000]
 * @param decel 刹车减速度 ((mm/s)/10ms) 范围[1,3000]
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

int32_t get_driven_wheel_distance(void)
{
    int32_t raw_pulses = BSP_Encoder_Get_Total_Pulses();
    return Math_Pulses_To_mm(raw_pulses);
}