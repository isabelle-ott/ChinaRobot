#ifndef __MYCAN_H
#define __MYCAN_H

#include "main.h"
#include "motor_basic.h"

void MyCan_Init(MOTOR_BASIC_T *motor_1, MOTOR_BASIC_T *motor_2, MOTOR_BASIC_T *motor_3, MOTOR_BASIC_T *motor_4);
void MyCan_Transmit(CAN_HandleTypeDef *hcan, uint32_t data_id, uint8_t Length, uint8_t *Data);

void MyCan_abort_Transmit(CAN_HandleTypeDef *hcan);

int8_t MyCan_Get_RxData(MOTOR_BASIC_T *motor, uint8_t *data_ptr);


#endif

