/**
 * @file application_HOME.c
 * @brief 基地相关代码
 */
#include "application_HOME.h"
#include "application_common.h"

/**
 * @brief 调整姿态使投影对准基地
 */
void HOME_run(void)
{
	int32_t upper_signe = is_site(SITE_RED) ? 1 : -1; // 靠近仓库一侧，对于x轴的方向

	/* 使upper循迹在基地上边缘上 */
	set_speed_xy(upper_signe * -400, 0); // 往地图边缘走，直到上循迹板在基地上
	while (!TrackBoard_read_HomeUpper()) __NOP();
	set_speed_xy(upper_signe * 200, 0); // 往仓库方向走，直到循迹板不在基地上
	while (TrackBoard_read_HomeUpper()) __NOP();
	set_speed_xy(0, 0);

	/* 使lower循迹在基地外边缘上 */
	while (TrackBoard_read_HomeLower())
	{
		int32_t Vx = 0;
		if (TrackBoard_read_HomeUpper())
			Vx = upper_signe * 200;
		set_speed_xy(Vx, -200);
	}
	set_speed_xy(0, 0);

	if (is_site(SITE_RED))
		move_w(50, 90, 40, 5, 5);
	else
		move_w(50, 90, 40, 5, 5);

	/* 使upper循迹在基地上边缘上 */
	set_speed_xy(upper_signe * 100, 0); // 往仓库方向走，直到循迹板不在基地上
	while (TrackBoard_read_HomeUpper()) __NOP();
	set_speed_xy(upper_signe * -100, 0); // 往地图边缘走，直到上循迹板在基地上
	while (!TrackBoard_read_HomeUpper()) __NOP();

	if (is_site(SITE_RED))
		move_w(50, 180, 45, 5, 5);
	else
		move_w(50, 0, 45, 5, 5);
}
MAKE_CMD(void, HOME_run, void);
