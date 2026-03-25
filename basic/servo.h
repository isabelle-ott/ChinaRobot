#ifndef __SERVO_H__
#define __SERVO_H__

#include "bsp.h"

/* 舵控板动作组的名称，对应的数字不是实际的动作组编号 */
typedef enum
{
	ServoAG_ready,	   // 机械臂举高准备进入任务
	ServoAG_ARM_reset, // 机械臂收回

	ServoAG_YPJ_scan,		 // 圆盘机扫描状态
	ServoAG_YPJ_get_RB_Ball, // 圆盘机拨红蓝球
	ServoAG_YPJ_get_Y_Ball,	 // 圆盘机拨黄球

	ServoAG_JT_scanL,	 // 阶梯平台最低处扫描
	ServoAG_JT_scanM,	 // 阶梯平台中间处扫描
	ServoAG_JT_scanH,	 // 阶梯平台最高处扫描
	ServoAG_JT_getL,	 // 阶梯平台最低处抓取
	ServoAG_JT_getM,	 // 阶梯平台中间处抓取
	ServoAG_JT_getH,	 // 阶梯平台最高处抓取
	ServoAG_JT_putBlock, // 阶梯平台放置方块
	ServoAG_JT_putRing,	 // 阶梯平台放置圆环
	ServoAG_JT_reset,	 // 阶梯平台收回机械臂准备离开

	ServoAG_LZ_getWhite,  // 立桩抓取白球
	ServoAG_LZ_scan,	  // 立桩扫球
	ServoAG_LZ_get,		  // 立桩抓球

	ServoAG_CK_dump_Y_Ball,	 // 仓库倒黄球
	ServoAG_CK_dump_RB_Ball, // 仓库倒红蓝球
	ServoAG_CK_putRing,		 // 仓库放圆环
	ServoAG_CK_getRing,		 // 仓库拿圆环
	ServoAG_CK_reset,		 // 仓库舵机复位
} ServoAction_e;

void servo_uart_IRQHandler(uint16_t Size);

void servo_inti(void);

void servo_action(ServoAction_e action, int32_t check_ACK);

uint8_t servo_waitAction(void);

void servo_action_w(ServoAction_e action, int32_t check_ACK);

#endif // __SERVO_H__
