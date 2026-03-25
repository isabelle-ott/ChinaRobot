#include "encoder_wheel.h"
//本文件是后期添加编码轮子所写
#include "tim.h" // 包含 htim2 的定义

static int16_t last_counter = 0;
static int32_t total_pulses = 0; // 用 32 位整数无限累加脉冲，防止长距离溢出

/**
 * @brief 初始化并启动编码器定时器
 */
void BSP_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    total_pulses = 0;
}

/**
 * @brief 更新编码器数据 (需在 FreeRTOS 任务中以固定频率调用，如 10ms)
 */
void BSP_Encoder_Update(void)
{
    int16_t current_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

    // 利用 int16_t 的自然溢出特性计算差值
    int16_t delta = current_counter - last_counter;

    total_pulses += delta; // 累加到全局 32 位变量中
    last_counter = current_counter;
}

/**
 * @brief 获取当前累计的脉冲总数
 */
int32_t BSP_Encoder_Get_Total_Pulses(void)
{
    return total_pulses;
}

/**
 * @brief 编码器计数值清零 (用于每次抢点发车前)
 */
void BSP_Encoder_Reset(void)
{
    total_pulses = 0;
    last_counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
}