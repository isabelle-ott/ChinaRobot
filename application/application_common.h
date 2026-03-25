#ifndef __APPLICATION_COMMON_H__
#define __APPLICATION_COMMON_H__

#include "bsp.h"
#include "chassis.h"
#include "miniCMD_makeCmd.h"
#include "servo.h"
#include "openmv.h"
#include "TrackBoard.h"
#include "imu.h"
#include "HW.h"
#include "fast_math.h"


uint8_t keep_line(uint8_t p, int8_t CalibrateIMU, uint8_t use_M, int32_t *Vy);

void find_line(int32_t speed, int8_t p, int32_t time);

void CalibrateAngle_super(void);

#endif // __APPLICATION_COMMON_H__
