#include "HW.h"
#include "miniCMD_makeCmd.h"

uint8_t get_HW(uint8_t num)
{
	switch (num)
	{
	case 1:
		return HAL_GPIO_ReadPin(gpio_HW_1_GPIO_Port, gpio_HW_1_Pin) == GPIO_PIN_RESET;
	case 2:
		return HAL_GPIO_ReadPin(gpio_HW_2_GPIO_Port, gpio_HW_2_Pin) == GPIO_PIN_RESET;
	case 3:
		return HAL_GPIO_ReadPin(gpio_HW_3_GPIO_Port, gpio_HW_3_Pin) == GPIO_PIN_RESET;
	case 4:
		return HAL_GPIO_ReadPin(gpio_HW_4_GPIO_Port, gpio_HW_4_Pin) == GPIO_PIN_RESET;
	case 5:
		return HAL_GPIO_ReadPin(gpio_HW_5_GPIO_Port, gpio_HW_5_Pin) == GPIO_PIN_RESET;
	}
	return 0;
}

void HW_test(void)
{
	while (1)
	{
		print_f("%d,%d,%d,%d,%d\n", get_HW(1), get_HW(2), get_HW(3), get_HW(4), get_HW(5));
		HAL_Delay(100);
	}
}

MAKE_CMD(void, HW_test, void);
