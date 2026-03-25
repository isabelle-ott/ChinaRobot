/**
 * @file servo.c
 * @brief 舵控板通信
 */
#include "servo.h"
#include "FreeRTOS.h"
#include "bsp.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include "miniCMD_makeCmd.h"

UART_HandleTypeDef *const servo_uart = &huart7; // 舵控板串口句柄

/***************************** 动作组配置，[0]为红区，[1]为蓝区，[2]为超时时间 *****************************/
const uint32_t ActionGroup[][3] = {
	[ServoAG_ready] = {2, 2, 3000},		  // 机械臂举高准备进入圆盘机
	[ServoAG_ARM_reset] = {24, 24, 3000}, // 机械臂收回

	[ServoAG_YPJ_scan] = {3, 3, 3000},		  // 圆盘机扫描状态
	[ServoAG_YPJ_get_RB_Ball] = {5, 5, 1000}, // 圆盘机拨红蓝球
	[ServoAG_YPJ_get_Y_Ball] = {4, 4, 1000},  // 圆盘机拨黄球

	[ServoAG_JT_scanL] = {15, 15, 3000},	// 阶梯平台最低处扫描
	[ServoAG_JT_scanM] = {19, 19, 3000},	// 阶梯平台中间处扫描
	[ServoAG_JT_scanH] = {17, 17, 3000},	// 阶梯平台最高处扫描
	[ServoAG_JT_getL] = {16, 16, 5000},		// 阶梯平台最低处抓取
	[ServoAG_JT_getM] = {20, 20, 5000},		// 阶梯平台中间处抓取
	[ServoAG_JT_getH] = {18, 27, 5000},		// 阶梯平台最高处抓取
	[ServoAG_JT_putBlock] = {21, 21, 6000}, // 阶梯平台放置方块
	[ServoAG_JT_putRing] = {22, 22, 6000},	// 阶梯平台放置圆环

	[ServoAG_LZ_getWhite] = {11, 11, 10000}, // 立桩抓取白球
	[ServoAG_LZ_scan] = {12, 12, 3000},		 // 立桩扫球
	[ServoAG_LZ_get] = {13, 13, 5000},		 // 立桩抓球

	[ServoAG_CK_dump_Y_Ball] = {6, 6, 5000},  // 仓库倒黄球
	[ServoAG_CK_dump_RB_Ball] = {7, 7, 5000}, // 仓库倒红蓝球
	[ServoAG_CK_reset] = {23, 23, 3000},	  // 仓库倒方块
	[ServoAG_CK_getRing] = {26, 26, 5000},	  // 仓库拿圆环
	[ServoAG_CK_putRing] = {9, 8, 5000},	  // 仓库放圆环
};

/************************************* 对信号量的操作的封装,用于中断与API的通信 *************************************/

volatile uint32_t servo_state = 0; // 0:接收到应答信号 1:接收到完成信号

/**
 * @brief 在中断中置事件标志位
 * @param event 1应答信号ACK; 2完成信号DONE;
 */
static inline void state_set(uint32_t event)
{
	servo_state |= event;
}

/**
 * @brief 等待舵控板返回信号
 * @param event 1应答信号ACK; 2完成信号DONE;
 * @param timeout 超时时间
 * @return 1正常 0超时
 */
static inline uint8_t state_wait(uint32_t event, uint32_t timeout)
{
	uint32_t start = OS_tick();
	while ((servo_state & event) == 0)
	{
		if (OS_tick() - start > timeout) return 0; // 超时
		osDelay(1);
	}
	return 1;
}

/**
 * @brief 清除舵控板信号
 * @param event 1应答信号ACK; 2完成信号DONE;
 */
static inline void state_clear(uint32_t event)
{
	servo_state &= ~event;
}

/**************************************** 对串口的封装 ****************************************/
// 傻逼舵控板发送回来的一帧数据可能会随机在中间断开3ms，所以数据的接收这一块会比较奇怪

/* 发送动作组指令 */
static inline void uart_send_action(uint8_t num)
{
	static uint8_t send_buf[7] = {0x55, 0x55, 0x05, 0x06, 0, 0x01, 0x00};
	send_buf[4] = num;
	taskENTER_CRITICAL();
	HAL_UART_Transmit_DMA(servo_uart, send_buf, sizeof(send_buf));
	taskEXIT_CRITICAL();
	log_info_f("ActionGroup-%d-", num);
	/* 等待发送完毕 */
	uint32_t t = OS_tick();
	while (servo_uart->gState != HAL_UART_STATE_READY)
	{
		if (OS_tick() - t > 300)
		{
			log_error("动作组发送超时!");
			break;
		}
	}
}
MAKE_CMD(void, uart_send_action, uint8_t);

static uint8_t RX_buf[32]; // 接收缓冲区，长度必须是2的幂
/* 实现循环访问，此算法依赖于数组长度为2的幂 */

#define auto_idx(idx) (((int32_t)(idx)) & (int32_t)(sizeof(RX_buf) - 1))

/* 串口空闲中断回调, DMA是循环模式, 所以不用重新发起接收 */
void servo_uart_IRQHandler(uint16_t Size)
{
	/* 格式匹配
	{0x55, 0x55, 0x05, 0x08/0x06, 0, 0x01, 0x00}
	*/
	int32_t last_data = Size - 1; // 最新的数据的索引
	if (Size > 0 &&
		(RX_buf[auto_idx(last_data - 6)] == 0x55) &&
		(RX_buf[auto_idx(last_data - 5)] == 0x55) &&
		(RX_buf[auto_idx(last_data - 4)] == 0x05) &&
		(RX_buf[auto_idx(last_data - 1)] == 0x01) &&
		(RX_buf[auto_idx(last_data - 0)] == 0x00))
	{
		if (RX_buf[auto_idx(last_data - 3)] == 0x08) // DONE信号
			state_set(2);
		else if (RX_buf[auto_idx(last_data - 3)] == 0x06) // ACK信号
			state_set(1);
	}
}

/**************************************** 舵控板API *******************************************/

static uint32_t servo_actionOK_timeout = 0;

static SemaphoreHandle_t servo_TX_protect; // 避免多个任务同时调用API
static StaticQueue_t _servo_TX_protect_buffer;

/* 初始化 */
void servo_inti(void)
{
	/* 创建信号量 */
	servo_TX_protect = xSemaphoreCreateRecursiveMutexStatic(&_servo_TX_protect_buffer);

	/* 初始化串口接收 */
	taskENTER_CRITICAL();
	{
		HAL_UARTEx_ReceiveToIdle_DMA(servo_uart, RX_buf, sizeof(RX_buf));
		// 禁止DMA传输中断，DMA只要负责一直搬运就好了，而串口空闲中断要考虑的就多了
		__HAL_DMA_DISABLE_IT(servo_uart->hdmarx, DMA_IT_HT & DMA_IT_TC);
	}
	taskEXIT_CRITICAL();
}

/**
 * @brief 舵控板运行动作组(自动根据红蓝区选择对应的动作组)
 * @param action 动作
 * @param check_ack 是否进行应答校验, 大于零则默认会重发, 0则只发送一次不检查ACK, 小于零则指定重试次数
 */
void servo_action(ServoAction_e action, int32_t check_ACK)
{
	uint8_t retry_count = check_ACK > 0 ? 10 : -check_ACK; // snum: 默认重发次数
	uint8_t GroupNumber = ActionGroup[action][(is_site(SITE_RED)) ? 0 : 1];

	if (check_ACK && ((HAL_UART_GetState(servo_uart) & HAL_UART_STATE_BUSY_RX) != HAL_UART_STATE_BUSY_RX))
	{
		log_error_f("串口状态错误-%d-尝试恢复", HAL_UART_GetState(servo_uart));
		taskENTER_CRITICAL();
		HAL_UART_AbortReceive_IT(servo_uart);
		HAL_UARTEx_ReceiveToIdle_DMA(servo_uart, RX_buf, sizeof(RX_buf));
		__HAL_DMA_DISABLE_IT(servo_uart->hdmarx, DMA_IT_HT & DMA_IT_TC);
		taskEXIT_CRITICAL();
		log_error_f("串口状态-%d-", HAL_UART_GetState(servo_uart));
	}

	xSemaphoreTakeRecursive(servo_TX_protect, portMAX_DELAY);
	state_clear(2 | 1);
	for (uint8_t i = retry_count; i > 0 || check_ACK == 0; i--)
	{
		uart_send_action(GroupNumber);
		servo_actionOK_timeout = OS_tick() + ActionGroup[action][2]; // 设置超时时间

		if (check_ACK == 0 || state_wait(1, 100)) // snum: 重发间隔
		{
			break;
		}
		else if (i == 1)
		{
			log_error("servo_action无应答");
		}
	}
	xSemaphoreGiveRecursive(servo_TX_protect);
}
MAKE_CMD(void, servo_action, ServoAction_e, int32_t);

/* 等待动作完成 */
uint8_t servo_waitAction(void)
{
	xSemaphoreTakeRecursive(servo_TX_protect, portMAX_DELAY);
	uint8_t ret = 1;
	uint32_t t = OS_tick();
	uint32_t timeout = (servo_actionOK_timeout > t) ? (servo_actionOK_timeout - t) : 0;
	if (!state_wait(2, timeout))
	{
		ret = 0;
		log_error_f("waitAction超时 UART_State %d", HAL_UART_GetState(servo_uart));
		log_error_f("S%d", HAL_UART_GetState(servo_uart));
	}
	xSemaphoreGiveRecursive(servo_TX_protect);
	osDelay(200); // 再等一会，避免连续动作导致舵控板处理不过来（辣鸡舵控板 ^_^）
	return ret;
}

void servo_action_w(ServoAction_e action, int32_t check_ACK)
{
	xSemaphoreTakeRecursive(servo_TX_protect, portMAX_DELAY);
	servo_action(action, check_ACK);
	servo_waitAction();
	xSemaphoreGiveRecursive(servo_TX_protect);
}
MAKE_CMD(void, servo_action_w, ServoAction_e, int32_t);
