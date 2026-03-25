/* 实现张大头电机协议 */
#define firmware_Emm // 选择电机固件(firmware_X,firmware_Emm)

#include "motor_basic.h"
#include "mycan.h"
#include "can.h"

// 通信协议的初始化(需要确保调用前CAN外设已配置好)
void Motor_Basic_Init(MOTOR_BASIC_T *aim_motor)
{
	// 电机使能
	// Motor_Enable(aim_motor);
}

// 使能电机
void Motor_Enable(MOTOR_BASIC_T *aim_motor)
{
	//                          {0xF3, 0xAB, 状态,  同步,  校验}
	static uint8_t cmd_data[] = {0xF3, 0xAB, 0x01, 0x00, 0x6B};
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data), cmd_data);
}

#ifdef firmware_X
// X固件速度模式
void Motor_Basic_Set_Speed_Var(MOTOR_BASIC_T *aim_motor, int aim_speed, uint16_t aim_acceleration)
{
	//                          {0xF6, 方向,加速度H,加速度L,速度H,速度L, 同步, 校验}
	static uint8_t cmd_data[] = {0xF6, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B};
	// 判断速度值的正负,同时取绝对值
	if (aim_speed >= 0)
	{
		cmd_data[1] = 0x00;
	}
	else
	{
		cmd_data[1] = 0x01;
		aim_speed = -aim_speed;
	}
	cmd_data[2] = (uint8_t)(aim_acceleration >> 8);
	cmd_data[3] = (uint8_t)(aim_acceleration & 0xFF);
	cmd_data[4] = (uint8_t)(aim_speed >> 8);
	cmd_data[5] = (uint8_t)(aim_speed & 0xFF);
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data), cmd_data);
}
#else
// Emm固件速度模式
void Motor_Basic_Set_Speed_Var(MOTOR_BASIC_T *aim_motor, int aim_speed, uint16_t aim_acceleration)
{
	//                          {0xF6, 方向, 速度H, 速度L,加速度, 同步, 校验}
	static uint8_t cmd_data[] = {0xF6, 0x01, 0x00, 0x00, 0x00, 0x00, 0x6B};
	// 判断速度值的正负,同时取绝对值
	if (aim_speed >= 0)
	{
		cmd_data[1] = 0x00;
	}
	else
	{
		cmd_data[1] = 0x01;
		aim_speed = -aim_speed;
	}
	cmd_data[2] = (uint8_t)(aim_speed >> 8);
	cmd_data[3] = (uint8_t)(aim_speed & 0xFF);
	cmd_data[4] = (uint8_t)aim_acceleration;
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data), cmd_data);
}
#endif

// 请求获取角度
void Motor_Basic_require_Angle(MOTOR_BASIC_T *aim_motor)
{
	static uint8_t cmd_data[] = {0x36, 0x6B};
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data), cmd_data);
}

/**
 * @brief X固件，读取电机返回的角度
 * @return 1:成功读取到角度数据，0:没有读取到数据
 */
int8_t Motor_Basic_read_Angle(MOTOR_BASIC_T *motor, int32_t *Angle)
{
	uint8_t data[9];
	uint8_t is_get = 0;			// 成功读取到角度数据
	for (int i = 0; i < 3; i++) // 最多读取3次
	{
		if (MyCan_Get_RxData(motor, data) <= 0) return is_get; // FIFO中没有数据可读了

		/* 校验并提取角度数据 */
		if (data[0] != 7 || data[1] != 0x36 || data[7] != 0x6B) continue; // 数据非法
		*Angle = (((int32_t)data[3]) << 24) | (((int32_t)data[4]) << 16) | (((int32_t)data[5]) << 8) | ((int32_t)data[6]);
		if (data[2] == 1) *Angle = 0 - (*Angle);
		is_get = 1; // 读取成功
	}
	return is_get;
}

// 发送多机同步指令
void Motor_Send_Togather(void)
{
	static uint8_t cmd_data[] = {0xFF, 0x66, 0x6B};
	MyCan_Transmit(&hcan1, 0x00, sizeof(cmd_data), cmd_data);
	MyCan_Transmit(&hcan2, 0x00, sizeof(cmd_data), cmd_data);
}

void Motor_abord_All_TX(void)
{
	MyCan_abort_Transmit(&hcan1);
	MyCan_abort_Transmit(&hcan2);
}

/************************* 由于暂时用不到，以下代码未经过测试，慎用 *************************/
/*
// 发送检测标志位的指令
void Motor_require_stateFlag(MOTOR_BASIC_T *aim_motor)
{
	static uint8_t cmd_data[] = {0x3A, 0x6B};
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data), cmd_data);
}

// 梯形加减速位置模式
void Motor_Basic_Set_Position_Var_Trapezium(MOTOR_BASIC_T *aim_motor, uint32_t aim_position, uint16_t add_acceleration, uint16_t subtract_acceleration, int16_t max_speed)
{
	//                            {0xFD,方向,加速度H,加速度L,减速度H,减速度L,速度H,速度L}
	static uint8_t cmd_data_1[] = {0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	//                            {0xFD,角度HH,角度HL,角度LH,角度LL,相对, 同步, 校验}
	static uint8_t cmd_data_2[] = {0xFD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B};
	// 判断速度值的正负,同时取绝对值
	if (max_speed >= 0)
	{
		cmd_data_1[1] = 0x00;
	}
	else
	{
		cmd_data_1[1] = 0x01;
		max_speed = -max_speed;
	}
	// 加速度
	cmd_data_1[2] = (uint8_t)(add_acceleration >> 8);
	cmd_data_1[3] = (uint8_t)(add_acceleration & 0xFF);
	// 减速度
	cmd_data_1[4] = (uint8_t)(subtract_acceleration >> 8);
	cmd_data_1[5] = (uint8_t)(subtract_acceleration & 0xFF);
	// 速度
	cmd_data_1[6] = (uint8_t)(max_speed >> 8);
	cmd_data_1[7] = (uint8_t)(max_speed & 0xFF);
	// 发送第一个帧
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID, sizeof(cmd_data_1), cmd_data_1);
	// 角度
	cmd_data_2[1] = (uint8_t)(aim_position >> 24);
	cmd_data_2[2] = (uint8_t)(aim_position >> 16);
	cmd_data_2[3] = (uint8_t)(aim_position >> 8);
	cmd_data_2[4] = (uint8_t)(aim_position);
	// 发送第二个帧
	MyCan_Transmit(aim_motor->hcan, aim_motor->ID + 1, sizeof(cmd_data_2), cmd_data_2);
}
*/
