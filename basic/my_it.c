/**
 * @brief hal库的中断回调函数统一放在这里，方便管理
 */
#include "my_it.h"
#include "bsp.h"
#include "imu.h"
#include "usart.h"
#include "openmv.h"
#include "servo.h"
#include "QR_code.h"

/* 串口空闲中断 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	BaseType_t HigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART2) // imu
	{
		imu_itCall(Size);
	}
	else if (huart->Instance == UART5) // logger
	{
		logger_itCall(Size, &HigherPriorityTaskWoken);
	}
	else if (huart->Instance == UART8) // openMV
	{
		openMV_itCall(Size);
	}
	else if (huart->Instance == UART7) // 舵控板
	{
		servo_uart_IRQHandler(Size);
	}
	portYIELD_FROM_ISR(HigherPriorityTaskWoken);
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == My_QR_Handle.QR_huart->Instance)
	{
		My_QR_Handle._QR_USART_DATA_Processing(1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	LED(0, 1, 0);
	if (huart->Instance == USART2) // imu
	{
		imu_error_process();
	}
	if (huart->Instance == My_QR_Handle.QR_huart->Instance)
	{
		QR_USART_Error_Processing();
	}
}
