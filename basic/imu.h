#ifndef IMU_H
#define IMU_H
//#include "general.h"
#include "main.h"
#include "fast_math.h"
void imu_init(void);
void imu_itCall(uint16_t Size);
void imu_error_process(void);
void imu_set_angle(int32_t angle);
int32_t imu_get_angle(void);

#endif
