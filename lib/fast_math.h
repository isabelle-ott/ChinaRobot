/**
 * @file fast_math.h
 * @brief 一些比较通用的计算函数可以丢这里，不一定是fast
 */
#ifndef FAST_MATH_H
#define FAST_MATH_H
#include "main.h"
int32_t limit_angle(int32_t deg);
float fast_sin(int deg);
float fast_cos(int deg);
void fast_sincos(int32_t deg, float *sin_ptr, float *cos_ptr);
int32_t limit(int32_t x, int32_t min, int32_t max);
int32_t abs(int32_t x);
int32_t max(int32_t a, int32_t b);

#define _abs(x) (((x) >= 0) ? (x) : (-(x)))
#define _limit(x, a, b) (((x) < (a)) ? (a) : (((x) > (b)) ? (b) : (x)))
#define _max(a, b) (((a) > (b)) ? (a) : (b))
#define _min(a, b) (((a) < (b)) ? (a) : (b))
// int sqrt(int num);

#endif
