#include "encoder_math.h"

int32_t Math_Pulses_To_mm(int32_t pulses)
{
    float total_pulses_per_rev = ENCODER_LINE_LENGTH * ENCODER_MULTIPLER;
    float perimeter = PI * ENCODER_WHEEL_DIAMETER;

    // 计算并返回整型毫米数
    return (int32_t)(((float)pulses / total_pulses_per_rev) * perimeter);
}