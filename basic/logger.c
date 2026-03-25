#include "logger.h"
#include "string.h" // strlen
#include "usart.h"
#include <stdarg.h> // 用于处理变长参数
#include <stdio.h>	// vsprintf

#define uart huart5
static char TX_buff[300];
static char RX_buff[150];

static inline void wait_uart_ready(void)
{
	uint32_t t = HAL_tick();
	while (uart.gState != HAL_UART_STATE_READY)
	{
		if (HAL_tick() - t > 300) // snum:超时时间
		{
			beebee(1);
			return;
		}
	}
}

static inline void uart_send(uint8_t *data, uint16_t len)
{
	HAL_UART_Transmit_DMA(&uart, data, len);
}

static inline void uart_receive(void)
{
	taskENTER_CRITICAL();
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&uart, (uint8_t *)RX_buff, sizeof(RX_buff));
		__HAL_DMA_DISABLE_IT(uart.hdmarx, DMA_IT_HT); // 关闭半满中断
	}
	taskEXIT_CRITICAL();
}

static StaticSemaphore_t TXmutex_buffer, RXmutex_buffer;    //信号量静态内存结构
static SemaphoreHandle_t TX_Mutex = NULL, RX_Semap = NULL;  //信号量操作句柄

void _logger_print_init(void)
{
	TX_Mutex = xSemaphoreCreateMutexStatic(&TXmutex_buffer);
	RX_Semap = xSemaphoreCreateBinaryStatic(&RXmutex_buffer);
	uart_receive();
}

// 实现打印到串口，之所以不用printf的重定向是为了避免多任务抢占
void xxx_print_log(const char *str)
{
	if (xSemaphoreTake(TX_Mutex, 300) == pdTRUE)
	{
		wait_uart_ready();
		uart_send((uint8_t *)str, strlen(str)); // 发送数据到串口
		xSemaphoreGive(TX_Mutex);
	}
	else
		beebee(1);
}

// 格式化打印到串口
void xxx_print_log_f(const char *format, ...)
{
	va_list args;			// 用于存储变长参数
	va_start(args, format); // 初始化变长参数列表
	if (xSemaphoreTake(TX_Mutex, 300) == pdTRUE)
	{
		wait_uart_ready();
		int len = vsnprintf(TX_buff, sizeof(TX_buff), format, args);
		uart_send((uint8_t *)TX_buff, len); // 发送数据到串口
		xSemaphoreGive(TX_Mutex);
	}
	else
		beebee(1);
	va_end(args); // 结束变长参数处理
}

void logger_restart_RX(void)
{
	HAL_UART_AbortReceive_IT(&uart);
	xSemaphoreTake(RX_Semap, 0);
	uart_receive();
}

/**
 * @brief 从串口获取字符
 */
char *logger_getToIDLE(void)
{
	logger_restart_RX();

	xSemaphoreTake(RX_Semap, osWaitForever);
	RX_buff[sizeof(RX_buff) - 1] = 0;
	return RX_buff;
}

char *logger_get(void)
{
	if (xSemaphoreTake(RX_Semap, 0) == pdPASS)
	{
		RX_buff[sizeof(RX_buff) - 1] = 0;
		return RX_buff;
	}
	return NULL;
}

/* 串口空闲接收中断回调 */
void logger_itCall(uint16_t Size, BaseType_t *pxHigherPriorityTaskWoken)
{
	RX_buff[Size] = 0;
	BaseType_t t = pdFALSE;
	xSemaphoreGiveFromISR(RX_Semap, &t);
	if (t == pdTRUE) *pxHigherPriorityTaskWoken = pdTRUE;
	HAL_UARTEx_ReceiveToIdle_DMA(&uart, (uint8_t *)RX_buff, sizeof(RX_buff));
	__HAL_DMA_DISABLE_IT(uart.hdmarx, DMA_IT_HT); // 关闭半满中断
}
