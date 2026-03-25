/**
 * @file application_Route.c
 * @brief 路线相关
 */
#include "application_Route.h"
#include "application_common.h"

/* 从基地到圆盘机前 */
void from_HOME_to_YPJ(void)
{
	uint32_t startTick = OS_tick();
	if (is_site(SITE_RED))
	{
		move_XY_w_v_end(1800, 1800, 1472, 600, 30, 30);
		// move_XY_w(1800, 1472, 550, 30, 30);

		move(2600, 0, 2400, 20, 20);
		servo_action(ServoAG_ready, 1);
		wait_move_ok();
	}
	else
	{
		move_XY_w_v_end(1800, 1800, -1472, 600, 30, 30);
		// move_XY_w(1800, -1472, 550, 30, 30);

		move(2600, 180, 2400, 20, 20);
		servo_action(ServoAG_ready, 1);
		wait_move_ok();
	}
	log_info_f("HOME_to_YPJ: %dms", OS_tick() - startTick);
}
MAKE_CMD(void, from_HOME_to_YPJ, void);

/* 从仓库到基地前 */
void from_CK_to_HOME(void)
{
	if (is_site(SITE_RED))
	{
		move_XY_w(800, -1800, -600, 10, 10);
	}
	else
	{
		move_XY_w(800, 2200, -600, 10, 10);
	}
}
MAKE_CMD(void, from_CK_to_HOME, void);

/* 从圆盘机到避障 */
void from_YPJ_to_BZ(void)
{
	if (is_site(SITE_RED))
	{
		move_XY_w(800, -600, -400, 20, 20);
		osDelay(500);
	}
	else
	{
		move_XY_w(800, 600, -400, 20, 20);
		osDelay(500);
	}
}
MAKE_CMD(void, from_YPJ_to_BZ, void);

/* 从避障到阶梯平台 */
void from_BZ_to_JT(void)
{
		if (is_site(SITE_RED))
		{
			move_XY_w(800, -450, 0, 20, 20);
			move_XY_w(800, -800, -1100, 20, 20);
			turn_angle_w(1800);
		}
		else
		{
			move_XY_w(800, 450, 0, 20, 20);
			move_XY_w(800, 800, -1100, 20, 20);
			turn_angle_w(1800);
		}
}
MAKE_CMD(void, from_BZ_to_JT, void);
/* 从立桩到仓库 */
void from_LZ_to_CK(void)
{
	move_XY_w(800, 550, 0, 10, 10);
	move_XY_w(800, 0, 900, 10, 10);
	move_XY_w(800, -550, 0, 10, 10);
}
MAKE_CMD(void, from_LZ_to_CK, void);

/* 从阶梯平台到立桩 */
void from_JT_to_LZ(void)
{
	move_XY_w(800, -400, -220, 10, 10);
	turn_angle_w(1800);
	move_XY_w(500, 0, 300, 10, 10);
}
MAKE_CMD(void, from_JT_to_LZ, void);
