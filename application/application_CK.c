/**
 * @file application_CK.c
 * @brief 仓库任务
 */

#include "application_CK.h"
#include "application_common.h"

const int32_t Vx_high = 300; // snum: 高速位移
const int32_t Vx_low = 150;	 // snum: 低速位移

void CK_dump(uint8_t Y_ball, uint8_t RB_ball)
{
	if (!(Y_ball || RB_ball)) return;
	/* 高速左移粗略定位边缘 */
	set_speed_x(Vx_high);
	while (get_HW(1))
	{
		keep_line(18, 0, 0, NULL);
		osDelay(3);
	}
	/* 低速右移精准定位 */
	set_speed_x(-Vx_low);
	while (!get_HW(1))
	{
		keep_line(18, 0, 0, NULL);
		osDelay(3);
	}

	/* 传感器判定点和实际到位点是有一段距离的 */
	set_speed_x(Vx_low);
	reset_odom();
	while (get_odom_X_abs() < 60)
	{
		keep_line(18, 0, 0, NULL);
		osDelay(3);
	}
	set_speed_xy(0, 0);

	/* 倒红蓝球 */
	if (RB_ball)
	{
		move_w(100, 0, 80, 5, 5);
		servo_action(ServoAG_CK_dump_RB_Ball, 1);
		osDelay(5000);
		servo_action_w(ServoAG_CK_reset, 1);
	}

	/* 倒黄球 */
	if (Y_ball)
	{
		move_w(100, 180, 80, 5, 5);
		servo_action(ServoAG_CK_dump_Y_Ball, 1);
		osDelay(5000);
		servo_action_w(ServoAG_CK_reset, 1);
	}
	
}

void CK_put_Ring(void)
{
	/* 高速移动一小段距离缩短时间 */
	reset_odom();
	set_speed_x(-Vx_high);
	while (get_odom_X_abs() < 150) keep_line(18, 0, 0, NULL);
	set_speed_xy(0, 0);

	/* 拿起圆环 */
	servo_action_w(ServoAG_CK_getRing, 1);

	/* 继续低速移动，找到边缘 */
	set_speed_x(-Vx_low);
	while (get_HW(4))
	{
		keep_line(18, 0, 0, NULL);
		osDelay(3);
	}
	/* 多走一点距离就到了 */
	reset_odom();
	int32_t dis = (is_site(SITE_BLUE)) ? 23 : 20;
	while (get_odom_X_abs() < dis)
	{
		keep_line(18, 0, 0, NULL);
		osDelay(3);
	}
	set_speed_xy(0, 0);
	if (is_site(SITE_BLUE))
	{
		/* 后退一点 */
		move_w(100, -90, 15, 30, 30);
	}
	/* 放圆环就ok了 */
	servo_action_w(ServoAG_CK_putRing, 1);
	servo_action_w(ServoAG_ARM_reset, 1);
}

void CK_ready(void)
{
	find_line(300, 12, 3000);
}
MAKE_CMD(void, CK_ready, void);

void CK_run(uint8_t is_dump_Y, uint8_t is_dump_RB, uint8_t is_put_Ring)
{
	CK_ready();
	CK_dump(is_dump_Y, is_dump_RB);
	if (is_put_Ring) CK_put_Ring();
}
MAKE_CMD(void, CK_run, uint8_t, uint8_t, uint8_t);
