/**
 * @author TanES (tan_163mail@163.com)
 * @brief 通用的头文件，包含类型定义以及蜂鸣器控制等简单常用的函数
 */
#ifndef GENERAL_H
#define GENERAL_H
#include "cmsis_os.h"
#include "logger.h" // log库
#include "main.h"

typedef enum
{
	SITE_RED,
	SITE_BLUE
} SITE_e;

typedef enum
{
	LED_R = 1,
	LED_G = 2,
	LED_B = 4,
	LED_ALL = LED_R | LED_G | LED_B
}
LED_e;

typedef enum
{
	BUTTON_1 = 1,
	BUTTON_2 = 2,
	BUTTON_3 = 4,
	BUTTON_ALL = BUTTON_1 | BUTTON_2 | BUTTON_3
}
BUTTON_e;

#define HAL_tick() HAL_GetTick()
#define OS_tick() xTaskGetTickCount()


void set_site(SITE_e s);
SITE_e get_site(void);
uint8_t is_site(SITE_e s);
void beebee(int8_t state); // 蜂鸣器控制

void LED(int8_t led_R, int8_t led_G, int8_t led_B);

uint8_t Button1_Read(void);

uint8_t Button2_Read(void);

uint8_t Button3_Read(void);

BUTTON_e Button_waitPress(int32_t button);

#endif
