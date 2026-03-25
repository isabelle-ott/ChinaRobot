#ifndef CHASSIS_H
#define CHASSIS_H
#include "stdint.h"

enum chassis_control_mode_e // 底盘控制模式，可以用位运算提取Mode_Angle和Mode_Distance
{
	Mode_OpenLoop = 0,	   // 全开环控制模式                                          00
	Mode_Angle = 1,		   // 角度模式，进行角度闭环                                   01 
	Mode_Distance = 2,	   // 距离模式，到位后自动转到Mode_Speed                       10
	Mode_AngleDistance = 3 // 距离模式（包括角度闭环，所以是3），到位后自动转Mode_Angle  11
};

// 轮子周长
#define chassis_mmpr 235

void chassis_init(void);
void chassis_control(void);

void chassis_stop(void);

void chassis_reset(void);

void set_Angle_enable(uint8_t enable);

void set_speed_polarZ(int32_t V, int32_t dir, int32_t Vz);

void set_speed_xyZ(int32_t Vx, int32_t Vy, int32_t Vz);

void reset_odom(void);

int32_t get_odom_X(void);

int32_t get_odom_Y(void);

int32_t get_odom_X_abs(void);

int32_t get_odom_distance(void);

void set_speed_polar(int32_t V, int32_t dir);

void set_speed_xy(int32_t Vx, int32_t Vy);

void set_speed_RXY(int32_t r, int32_t Vx, int32_t Vy);

void set_speed_y(int32_t Vy);

void set_speed_x(int32_t Vx);

void set_speed_z(int32_t Vz);

void turn_angle(int32_t angle);

void turn_angle_w(int32_t angle);

void set_angle(int32_t angle);

void set_angle_w(int32_t angle);

int32_t get_targetAngle(void);

void move_Ve(int32_t V_const, int32_t V_end, int32_t dir, int32_t distance, int32_t accel_start, int32_t accel_end);

void move(int32_t V, int32_t dir, int32_t distance, int32_t accel, int32_t decel);

void wait_move_ok(void);

void move_w(int32_t V, int32_t dir, int32_t distance, int32_t accel, int32_t decel);

void move_XY_w(int32_t V, int32_t d_X, int32_t d_Y, int32_t accel, int32_t decel);

void move_XY_w_v_end(int32_t V, int32_t V_end, int32_t d_X, int32_t d_Y, int32_t accel, int32_t decel);

#endif


