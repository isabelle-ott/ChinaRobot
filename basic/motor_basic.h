#ifndef __MOTOR_BASIC_H
#define __MOTOR_BASIC_H

#include "main.h"


/* 电机的外设配置 */
typedef struct
{
	uint32_t ID;			 // 电机ID（左移8位之后的结果）
	CAN_HandleTypeDef *hcan; // 电机对应CAN的句柄
	uint32_t FilterBank;	 // 电机对应的过滤器编号，10号及以上为CAN2
	uint32_t RxFifo;		 // 电机对应的接收FIFO
} MOTOR_BASIC_T;

void Motor_Basic_Init(MOTOR_BASIC_T *aim_motor);

void Motor_Enable(MOTOR_BASIC_T *aim_motor);

void Motor_Send_Togather(void);

void Motor_abord_All_TX(void);

void Motor_Basic_Set_Speed_Var(MOTOR_BASIC_T *aim_motor, int aim_speed, uint16_t aim_acceleration);

void Motor_Basic_require_Angle(MOTOR_BASIC_T *aim_motor);

int8_t Motor_Basic_read_Angle(MOTOR_BASIC_T *motor, int32_t *Angle);

#endif
