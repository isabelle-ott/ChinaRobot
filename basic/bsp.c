#include "bsp.h"

#include "miniCMD_makeCmd.h"
/* 红蓝区 */
static SITE_e site = SITE_RED;
/* 设置红蓝区 */
void set_site(SITE_e s)
{
	site = s;
	log_info_f("设置红蓝区:%d(0红 1蓝)", site);
	switch (site)
	{
	case SITE_RED:
		log_info("设置：红区");
		break;
	case SITE_BLUE:
		log_info("设置：蓝区");
		break;
	default:
		log_error("区域设置错误");
		break;
	}
}
MAKE_CMD(void, set_site, SITE_e);

/* 获取当前红蓝区 */
SITE_e get_site(void)
{
	return site;
}
/* 判断当前红蓝区 */
uint8_t is_site(SITE_e s)
{
	return (site == s);
}

/**
 * @brief 蜂鸣器控制
 * @param state 0:关闭，1:开启，-1:翻转，其它:不变
 */
void beebee(int8_t state)
{
	switch (state)
	{
	case 0:
		HAL_GPIO_WritePin(beebee_GPIO_Port, beebee_Pin, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(beebee_GPIO_Port, beebee_Pin, GPIO_PIN_RESET);
		break;
	case -1:
		HAL_GPIO_TogglePin(beebee_GPIO_Port, beebee_Pin);
		break;
	}
}
MAKE_CMD(void, beebee, int8_t);

/**
 * @brief LED控制
 * @param led_R 0:关闭，1:开启，-1:翻转，其它:不变
 * @param led_G 0:关闭，1:开启，-1:翻转，其它:不变
 * @param led_B 0:关闭，1:开启，-1:翻转，其它:不变
 */
void LED(int8_t led_R, int8_t led_G, int8_t led_B)
{
	if (led_R == 0)
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	else if (led_R == 1)
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	else if (led_R == -1)
		HAL_GPIO_TogglePin(LEDR_GPIO_Port, LEDR_Pin);

	if (led_G == 0)
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	else if (led_G == 1)
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
	else if (led_G == -1)
		HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);

	if (led_B == 0)
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	else if (led_B == 1)
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	else if (led_B == -1)
		HAL_GPIO_TogglePin(LEDB_GPIO_Port, LEDB_Pin);
}
MAKE_CMD(void, LED, int8_t, int8_t, int8_t);

uint8_t Button1_Read(void)
{
	return (HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_SET);
}
uint8_t Button2_Read(void)
{
	return (HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin) == GPIO_PIN_SET);
}
uint8_t Button3_Read(void)
{
	return (HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin) == GPIO_PIN_SET);
}

/**
 * @brief 等待按键按下(无消抖,按下立即触发)
 * @param button 允许用或，以同时等待多个按键其中的一个
 * @return 被按下的按键
 */
BUTTON_e Button_waitPress(int32_t button)
{
	while (1)
	{
		if ((button & BUTTON_1) && Button1_Read()) return BUTTON_1;
		if ((button & BUTTON_2) && Button2_Read()) return BUTTON_2;
		if ((button & BUTTON_3) && Button3_Read()) return BUTTON_3;
		osDelay(2);
	}
}
