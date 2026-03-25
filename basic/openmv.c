#include "openmv.h"
#include "usart.h"
#include "miniCMD_makeCmd.h"

#define huart huart8
static uint8_t RX_buf[128];		   // 串口接收缓冲区
static uint32_t RX_errorCount = 0; // 解码错误计数

/* 只是用于set_mode的回应处理 */
static volatile SITE_e current_site = SITE_RED;
static volatile openMV_mode_e current_mode = openMV_mode_IDLE;

volatile uint8_t my_RX_sem = 0;
static volatile MV_DATA_t RX_data = {0, 0, 0};

void openMV_init(void)
{
	taskENTER_CRITICAL();
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart, RX_buf, sizeof(RX_buf));
		__HAL_DMA_DISABLE_IT(huart.hdmarx, DMA_IT_HT); // 关闭半满中断
	}
	taskEXIT_CRITICAL();
	openMV_set_mode(openMV_mode_IDLE);
}

/*
发送格式：[0:2]表示模式，[3:3]表示红蓝区，[4:7]为[0:3]取反用于校验
*/
void openMV_set_mode(openMV_mode_e mode)
{
	uint8_t send_data = 0;
	send_data |= mode;								// 低3位：模式
	send_data |= (is_site(SITE_BLUE) ? 1 : 0) << 3; // 第4位：红蓝
	send_data |= (~(send_data & 0x0f)) << 4;		// 高4位：校验

	print_f("MV send:%x\n", send_data);

	for (uint8_t i = 0; i < 10; i++) // snum:重试次数
	{
		openMV_clear_data();
		my_RX_sem = 0;
		taskENTER_CRITICAL();
		HAL_UART_Transmit(&huart, &send_data, 1, 100);
		taskEXIT_CRITICAL();
		uint32_t start_t = OS_tick();
		while (my_RX_sem == 0 && OS_tick() - start_t < 300);
		if (current_mode == mode && current_site == get_site()) return;
	}

	log_error("MV set mode ERROR"); // 发送失败，没有回应
	current_mode = mode;
	current_site = get_site();
}
MAKE_CMD(void, openMV_set_mode, openMV_mode_e);

/**
 * @brief 获取接收到的openMV数据
 * @param mode 用于筛选数据类型
 * @param data 用于存储数据
 * @param clean 是否先清空
 * @param timeout 超时时间
 * @return 是否成功获取数据
 */
uint8_t openMV_get_data(openMV_mode_e mode, MV_DATA_t *data)
{
	/* 根据模式匹配期待的type */
	uint8_t expect_type[2];
	switch (mode)
	{
	case openMV_mode_YPJ:
		expect_type[0] = 0x51;
		expect_type[1] = 0x52;
		break;
	case openMV_mode_JT:
		expect_type[0] = 0x53;
		expect_type[1] = 0x54;
		break;
	case openMV_mode_LZ:
		expect_type[0] = 0x55;
		expect_type[1] = 0x55;
		break;
	case openMV_mode_LZ_WHITE:
		expect_type[0] = 0x56;
		expect_type[1] = 0x56;
		break;
	default:
		break;
	}

	if (my_RX_sem)
	{
		// print_f("%d,%d\n",RX_buf[0],RX_buf[1],expect_type[0],expect_type[1]);
		if (RX_data.type == expect_type[0] || RX_data.type == expect_type[1])
		{
			data->type = RX_data.type;
			data->x = RX_data.x;
			data->y = RX_data.y;
			my_RX_sem = 0;
			return 1;
		}
	}
	my_RX_sem = 0;
	return 0;
}

uint8_t DBG_openMV_get_data(openMV_mode_e mode, uint32_t timeout)
{
	MV_DATA_t tmp = {0, 0, 0};
	return openMV_get_data(mode, &tmp);
}
MAKE_CMD(uint8_t, DBG_openMV_get_data, openMV_mode_e, uint32_t);

void openMV_clear_data(void)
{
	my_RX_sem = 0;
}

/* 串口数据解码 */
static uint8_t decode(uint16_t Size)
{
	if (Size == 1)
	{
		if (RX_buf[0] != 0x51 && RX_buf[0] != 0x52 && RX_buf[0] != 0x55) return 0;
		RX_data.type = RX_buf[0];
		return 1;
	}
	if (Size == 4)
	{
		if (RX_buf[3] != (uint8_t)(RX_buf[0] + RX_buf[1] + RX_buf[2])) return 0; // 校验

		switch (RX_buf[0])
		{
		case 0x50:
			if (RX_buf[1] > 4 || RX_buf[2] > 1)
			{
				return 0;
			}
			current_mode = (openMV_mode_e)RX_buf[1];
			current_site = (RX_buf[2] == 0) ? SITE_RED : SITE_BLUE;
			return 1;
		case 0x53:
		case 0x54:
		case 0x55:
		case 0x56:
			RX_data.type = RX_buf[0];
			RX_data.x = (int8_t)RX_buf[1];
			RX_data.y = (int8_t)RX_buf[2];
			return 1;
		}
	}
	return 0;
}

/* 中断回调函数 */
void openMV_itCall(uint16_t Size)
{
	if (decode(Size))
	{
		my_RX_sem = 1;
	}
	else
	{
		RX_errorCount++;
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart, RX_buf, sizeof(RX_buf));
	__HAL_DMA_DISABLE_IT(huart.hdmarx, DMA_IT_HT); // 关闭半满中断
}

uint32_t openMV_get_errorCount(void)
{
	return RX_errorCount;
}
