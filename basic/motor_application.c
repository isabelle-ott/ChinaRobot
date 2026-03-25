#include "motor_application.h"
#include "can.h"
#include "logger.h"
#include "motor_basic.h"
#include "mycan.h"
#include "miniCMD_makeCmd.h"

typedef struct
{
	int32_t set_speed;
	int32_t get_Angle;
	MOTOR_BASIC_T motor_self;
} MOTOR_APPLICATION_T;

// 左后
static MOTOR_APPLICATION_T LB = {.motor_self =
									 {.ID = 2 << 8, .hcan = &hcan2, .RxFifo = CAN_RX_FIFO0, .FilterBank = 11}};
// 左前
static MOTOR_APPLICATION_T LF = {.motor_self =
									 {.ID = 1 << 8, .hcan = &hcan1, .RxFifo = CAN_RX_FIFO0, .FilterBank = 01}};
// 右后
static MOTOR_APPLICATION_T RB = {.motor_self =
									 {.ID = 3 << 8, .hcan = &hcan1, .RxFifo = CAN_RX_FIFO1, .FilterBank = 02}};
// 右前
static MOTOR_APPLICATION_T RF = {.motor_self =
									 {.ID = 4 << 8, .hcan = &hcan2, .RxFifo = CAN_RX_FIFO1, .FilterBank = 12}};

/**
 * @brief 电机相关的所有初始化
 */
void Motor_Init(void)
{
	/* 配置CAN */
	MyCan_Init(&(LB.motor_self), &(LF.motor_self), &(RB.motor_self), &(RF.motor_self));
	LB.set_speed = LF.set_speed = RB.set_speed = RF.set_speed = 0;

	/* 通信协议的初始化 */
	Motor_Basic_Init(&(LB.motor_self));
	Motor_Basic_Init(&(LF.motor_self));
	Motor_Basic_Init(&(RB.motor_self));
	Motor_Basic_Init(&(RF.motor_self));
}

// 麦轮解算
static inline void Motor_calculation(int vx, int vy, int w)
{
	//   机械臂方向 ^（Y轴正方向）
	//    LF     RF
	//     |      |
	//        ^
	//     |      |
	//    LB     RB
	LB.set_speed = (vx - vy + w);
	LF.set_speed = -(vx + vy - w);
	RB.set_speed = (vx + vy + w);
	RF.set_speed = -(vx - vy - w);
}

void Motor_Set_Speed(int vx, int vy, int w)
{
	Motor_calculation(vx, vy, w);
	Motor_abord_All_TX();
	// snum:设定加速度
	Motor_Basic_Set_Speed_Var(&(LB.motor_self), LB.set_speed, 250);
	Motor_Basic_Set_Speed_Var(&(LF.motor_self), LF.set_speed, 250);
	Motor_Basic_Set_Speed_Var(&(RB.motor_self), RB.set_speed, 250);
	Motor_Basic_Set_Speed_Var(&(RF.motor_self), RF.set_speed, 250);
	// Motor_Send_Togather();
}
MAKE_CMD(void, Motor_Set_Speed, int, int, int);
/*
void Motor_Set_Position(int px, int py, int w, int speed)
{
	Motor_calculation(px, py, w);
	Motor_Basic_Set_Position_Var_Trapezium(&(LB.motor_self), LB.set_speed, 1, 1, speed);
	Motor_Basic_Set_Position_Var_Trapezium(&(LF.motor_self), LF.set_speed, 1, 1, speed);
	Motor_Basic_Set_Position_Var_Trapezium(&(RB.motor_self), RB.set_speed, 1, 1, speed);
	Motor_Basic_Set_Position_Var_Trapezium(&(RF.motor_self), RF.set_speed, 1, 1, speed);
	Motor_Send_Togather();
}

// 请求获取电机角度
void Motor_require_Angle(void)
{
	Motor_Basic_require_Angle(&(LB.motor_self));
	Motor_Basic_require_Angle(&(LF.motor_self));
	Motor_Basic_require_Angle(&(RB.motor_self));
	Motor_Basic_require_Angle(&(RF.motor_self));
}

// 获取角度差，作为两次读取之间的平均速度
void Motor_read_deltaAngle(int32_t *VLB, int32_t *VLF, int32_t *VRB, int32_t *VRF)
{
	int32_t last_LB = LB.get_Angle, last_LF = LF.get_Angle, last_RB = RB.get_Angle, last_RF = RF.get_Angle;
	Motor_Basic_read_Angle(&(LB.motor_self), &(LB.get_Angle));
	Motor_Basic_read_Angle(&(LF.motor_self), &(LF.get_Angle));
	Motor_Basic_read_Angle(&(RB.motor_self), &(RB.get_Angle));
	Motor_Basic_read_Angle(&(RF.motor_self), &(RF.get_Angle));
	*VLB = LB.get_Angle - last_LB;
	*VLF = LF.get_Angle - last_LF;
	*VRB = RB.get_Angle - last_RB;
	*VRF = RF.get_Angle - last_RF;
	// log_info_f("%d,%d,%d,%d",*VLB,*VLF,*VRB,*VRF);
}
*/

/******************************************* 以下是电机单元测试 *******************************************/
void Motor_LB_Set_Speed(int v)
{
	Motor_Basic_Set_Speed_Var(&(LB.motor_self), v, 250);
}
MAKE_CMD(void, Motor_LB_Set_Speed, int);

void Motor_LF_Set_Speed(int v)
{
	Motor_Basic_Set_Speed_Var(&(LF.motor_self), v, 250);
}
MAKE_CMD(void, Motor_LF_Set_Speed, int);

void Motor_RB_Set_Speed(int v)
{
	Motor_Basic_Set_Speed_Var(&(RB.motor_self), v, 250);
}
MAKE_CMD(void, Motor_RB_Set_Speed, int);

void Motor_RF_Set_Speed(int v)
{
	Motor_Basic_Set_Speed_Var(&(RF.motor_self), v, 250);
}
MAKE_CMD(void, Motor_RF_Set_Speed, int);

/*
void test_Motor_speed(int32_t speed)
{
	Motor_Basic_Set_Speed_Var(&(LB.motor_self), speed, 0xffff);
	int32_t last_angle = 0, angle = 0;
	while (1)
	{
		Motor_Basic_require_Angle(&(LB.motor_self));
		if (Motor_Basic_read_Angle(&(LB.motor_self), &angle))
		{
			print_f("%d,%d\n", speed, 60000 * (angle - last_angle) / 3600);
			last_angle = angle;
		}
		HAL_Delay(10);
	}
}
MAKE_CMD(void, test_Motor_speed, int32_t);

int32_t test_Motor_getAngle(void)
{
	int32_t angle = 1010;
	Motor_Basic_require_Angle(&(LB.motor_self));
	osDelay(20); // 实测响应速度是25us
	if (Motor_Basic_read_Angle(&(LB.motor_self), &angle) == 0)
	{
		print("nore\n");
	}
	return angle;
}
MAKE_CMD(int32_t, test_Motor_getAngle, void);

void test_track(void)
{
	int speed = 0, step = 1, last_angle = 0, angle = 0;
	while (1)
	{
		if (speed > 200)
			step = -1;
		else if (speed < -200)
			step = 1;
		speed += step;
		Motor_Basic_Set_Speed_Var(&(LB.motor_self), speed, 0xffff);
		Motor_Basic_require_Angle(&(LB.motor_self));
		if (Motor_Basic_read_Angle(&(LB.motor_self), &angle))
		{
			print_f("%d,%d\n", speed, 60000 * (angle - last_angle) / 3600);
			last_angle = angle;
		}
		HAL_Delay(10);
	}
}
	*/
