#include "imu.h"
#include "bsp.h"
#include "miniCMD_makeCmd.h"
#include "usart.h"

UART_HandleTypeDef *huart = &huart2;

static uint8_t RX_buff[25];
static volatile int32_t imu_raw_angle = 1; // 陀螺仪返回的角度, 范围[-1800,1800]
static int32_t imu_offset = 0;			   // 陀螺仪的零点偏移，用于消除陀螺仪的累计误差, +-180度
static volatile uint32_t error_count = 0;  // 调试的时候用，用于查看线缆连接质量或干扰情况等

void imu_init(void)
{
	taskENTER_CRITICAL();
	{
		HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)RX_buff, sizeof(RX_buff));
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半满中断
	}
	taskEXIT_CRITICAL();
}

/* 串口空闲中断 */
//可以利用error_count来判断连接质量，若error_count增加，说明连接质量下降
void imu_itCall(uint16_t Size)
{
	/* 数据解析 */
	if (Size == 11 && RX_buff[0] == 0x55 && RX_buff[1] == 0x53)
	{
		uint8_t sum = RX_buff[0] + RX_buff[1] + RX_buff[2] + RX_buff[3] + RX_buff[4] +
					  RX_buff[5] + RX_buff[6] + RX_buff[7] + RX_buff[8] + RX_buff[9];
		if (RX_buff[10] == sum)
		{
			imu_raw_angle = (((int16_t)(RX_buff[7] << 8 | RX_buff[6])) * 1800) >> 15;
		}
		else
			error_count++;
	}
	else
		error_count++;
	HAL_UARTEx_ReceiveToIdle_DMA(huart, RX_buff, sizeof(RX_buff));
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半满中断
}

void imu_error_process(void)
{
	HAL_UART_Abort_IT(huart);
	HAL_UARTEx_ReceiveToIdle_DMA(huart, RX_buff, sizeof(RX_buff));
	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半满中断
}

void imu_set_angle(int32_t angle)
{
	imu_offset = limit_angle(angle - imu_raw_angle);
}
MAKE_CMD(void, imu_set_angle, int32_t);

int32_t imu_get_angle(void)
{
	int32_t angle = imu_raw_angle + imu_offset;
	return limit_angle(angle);
}
MAKE_CMD(int32_t, imu_get_angle, void);

uint32_t imu_get_errorCount(void)
{
	return error_count;
}
MAKE_CMD(uint32_t, imu_get_errorCount, void);
