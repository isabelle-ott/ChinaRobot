#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "stdint.h"

void BSP_Encoder_Init(void);
void BSP_Encoder_Update(void);          // 放在 FreeRTOS 任务中定期调用
int32_t BSP_Encoder_Get_Total_Pulses(void); // 获取累计的总脉冲数
void BSP_Encoder_Reset(void);           // 抢点开始前清零位置

#endif