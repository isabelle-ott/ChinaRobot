#ifndef __MOTOR_APPLICATION_H
#define __MOTOR_APPLICATION_H

#include "main.h"
void Motor_Init(void);

void Motor_read_deltaAngle(int32_t *VLB, int32_t *VLF, int32_t *VRB, int32_t *VRF);

void test_Motor_speed(int32_t speed);

int32_t test_Motor_getAngle(void);

void test_track(void);

void Motor_Set_Speed(int vx, int vy, int w);

void Motor_Set_Position(int px, int py, int w, int speed);

void Motor_require_Angle(void);

#endif
