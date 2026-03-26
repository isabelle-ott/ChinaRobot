#ifndef ENCODER_MATH_H
#define ENCODER_MATH_H

#include "stdint.h"

//编码器线数
#define ENCODER_LINE_LENGTH 400.0f
//编码器倍频
#define ENCODER_MULTIPLER   4.0f
//编码轮直径(单位：毫米)
#define ENCODER_WHEEL_DIAMETER 75.0f
//圆周率
#define PI 3.14159265f

int32_t Math_Pulses_To_mm(int32_t pulses);

#endif