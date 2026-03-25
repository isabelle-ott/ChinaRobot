#include "TrackBoard.h"
#include "bsp.h"
#include "imu.h"
#include "miniCMD_makeCmd.h"

/* 线的位置和车身角度的关系 */
#define delta_LR_MM_ANGLE 116 / 72 // 当两个循迹板识别到的线位置之差为72mm时，车身偏移11.6度，此数据前提是两板距离350mm
#define delta_ML_MM_ANGLE 135 / 42 // 当中间和左侧循迹板识别到的线位置之差为42mm时，车身偏移13.5度，此数据前提是距离175mm
#define delta_MR_MM_ANGLE 135 / 42 // 当中间和右侧循迹板识别到的线位置之差为42mm时，车身偏移13.5度，此数据前提是距离175mm

/* 快速读取，避免调用函数带来的开销以及函数内部的if分支开销 */
#define GPIO_FastRead(GPIOx, GPIO_Pin) (((GPIOx)->IDR & (GPIO_Pin)) != 0)
#define GPIO_TrackBoard(board, x) GPIO_FastRead(gpio_TrackBoard##board##_##x##_GPIO_Port, gpio_TrackBoard##board##_##x##_Pin)

// 循迹板123对应底盘左右中。调整位移顺序，使低位在底盘外面，高位靠近底盘里面
inline uint8_t TrackBoard_read_L(void)
{
	return (((uint8_t)GPIO_TrackBoard(1, 1) << 7) |
			((uint8_t)GPIO_TrackBoard(1, 2) << 6) |
			((uint8_t)GPIO_TrackBoard(1, 3) << 5) |
			((uint8_t)GPIO_TrackBoard(1, 4) << 4) |
			((uint8_t)GPIO_TrackBoard(1, 5) << 3) |
			((uint8_t)GPIO_TrackBoard(1, 6) << 2) |
			((uint8_t)GPIO_TrackBoard(1, 7) << 1) |
			((uint8_t)GPIO_TrackBoard(1, 8) << 0));
}

inline uint8_t TrackBoard_read_R(void)
{
	return (((uint8_t)GPIO_TrackBoard(2, 1) << 0) |
			((uint8_t)GPIO_TrackBoard(2, 2) << 1) |
			((uint8_t)GPIO_TrackBoard(2, 3) << 2) |
			((uint8_t)GPIO_TrackBoard(2, 4) << 3) |
			((uint8_t)GPIO_TrackBoard(2, 5) << 4) |
			((uint8_t)GPIO_TrackBoard(2, 6) << 5) |
			((uint8_t)GPIO_TrackBoard(2, 7) << 6) |
			((uint8_t)GPIO_TrackBoard(2, 8) << 7));
}

inline uint8_t TrackBoard_read_M(void)
{
	 return (((uint8_t)GPIO_TrackBoard(3, 1) << 0) |
	 		((uint8_t)GPIO_TrackBoard(3, 2) << 1) |
	 		((uint8_t)GPIO_TrackBoard(3, 3) << 2) |
	 		((uint8_t)GPIO_TrackBoard(3, 4) << 3) |
	 		((uint8_t)GPIO_TrackBoard(3, 5) << 4) |
	 		((uint8_t)GPIO_TrackBoard(3, 6) << 5) |
	 		((uint8_t)GPIO_TrackBoard(3, 7) << 6) |
	 		((uint8_t)GPIO_TrackBoard(3, 8) << 7));
}

void TrackBoard_text()
{
	while (1)
	{
		uint8_t M = TrackBoard_read_R    ();
		print_f("M:%d,%d,%d,%d,%d,%d,%d,%d\n", ((uint8_t)GPIO_TrackBoard(2, 1) ),
			   (uint8_t)GPIO_TrackBoard(2, 2) ,
			   (uint8_t)GPIO_TrackBoard(2, 3) ,
			   (uint8_t)GPIO_TrackBoard(2, 4) ,
			   (uint8_t)GPIO_TrackBoard(2, 5) ,
			   (uint8_t)GPIO_TrackBoard(2, 6) ,
			   (uint8_t)GPIO_TrackBoard(2, 7) ,
			   (uint8_t)GPIO_TrackBoard(2, 8) );
		osDelay(100);
	}
}
MAKE_CMD(void, TrackBoard_text, void);

/* 读取靠近仓库一侧的基地循迹 */
uint8_t TrackBoard_read_HomeUpper(void)
{
	if (is_site(SITE_RED))
		return (HAL_GPIO_ReadPin(gpio_TrackBoard_Home_R_GPIO_Port, gpio_TrackBoard_Home_R_Pin) == GPIO_PIN_SET);
	else
		return (HAL_GPIO_ReadPin(gpio_TrackBoard_Home_L_GPIO_Port, gpio_TrackBoard_Home_L_Pin) == GPIO_PIN_SET);
}
/* 读取靠近场地边缘一侧的基地循迹 */
uint8_t TrackBoard_read_HomeLower(void)
{
	if (is_site(SITE_RED))
		return (HAL_GPIO_ReadPin(gpio_TrackBoard_Home_L_GPIO_Port, gpio_TrackBoard_Home_L_Pin) == GPIO_PIN_SET);
	else
		return (HAL_GPIO_ReadPin(gpio_TrackBoard_Home_R_GPIO_Port, gpio_TrackBoard_Home_R_Pin) == GPIO_PIN_SET);
}

/**************************************** 一些计算 ****************************************/

/**
 * @brief 计算线的位置(加权平均)
 * @param sensor 传感器数值
 * @return 单位[mm]，只有最外面的传感器有线12，只有最里面的传感器有线96，多条线则返回平均值
 */
inline uint32_t TrackBoard_to_mm(uint8_t sensor)
{
	if (sensor == 0) return 0;//lyk：实际操作的时候如果会出现速度过快导致循迹板全部识别不到线的情况，有可能猛的打转向，可以保持原先误差（意思就是寻迹在直角转弯的时候有问题可以看看是不是这里的问题）
	/* SWAR 并行计统计1的个数 */
	uint8_t n = sensor - ((sensor >> 1) & 0x55); // 每 2 位计数
	n = (n & 0x33) + ((n >> 2) & 0x33);			 // 每 4 位计数
	n = (n + (n >> 4)) & 0x0F;					 // 每 8 位计数
	/* 加权 */
	uint32_t sum = ((uint32_t)((sensor >> 0) & 1) * (12 * 1)) +
				   ((uint32_t)((sensor >> 1) & 1) * (12 * 2)) +
				   ((uint32_t)((sensor >> 2) & 1) * (12 * 3)) +
				   ((uint32_t)((sensor >> 3) & 1) * (12 * 4)) +
				   ((uint32_t)((sensor >> 4) & 1) * (12 * 5)) +
				   ((uint32_t)((sensor >> 5) & 1) * (12 * 6)) +
				   ((uint32_t)((sensor >> 6) & 1) * (12 * 7)) +
				   ((uint32_t)((sensor >> 7) & 1) * (12 * 8));
	/* 加权平均 */
	return sum / n;
}

/**
 * @brief 根据循迹板计算车身偏移角度
 * @param L_mm 左循迹板的加权平均值
 * @param R_mm 右循迹板的加权平均值
 * @return 单位[0.1deg]，正值为车身偏逆时针，负值表示车身偏顺时针
 */
int32_t TrackBoardLR_to_angle(uint8_t L_mm, uint8_t R_mm)
{
	return (int32_t)(R_mm - L_mm) * delta_LR_MM_ANGLE; // 理论上应该用arctan，这里由于数值不大就认为是线性
}

/**
 * @brief 根据循迹板计算车身偏移角度
 * @param M_mm 中循迹板的加权平均值
 * @param L_mm 左循迹板的加权平均值
 * @param R_mm 右循迹板的加权平均值
 * @return 单位[0.1deg]，正值为车身偏逆时针，负值表示车身偏顺时针
 */
int32_t TrackBoardMLR_to_angle(uint8_t M_mm, uint8_t L_mm, uint8_t R_mm)
{
	// 两侧都有线时，优先使用两侧数据
	if (L_mm != 0 && R_mm != 0) return TrackBoardLR_to_angle(L_mm, R_mm);
	// 中间没有线时，无法判断
	if (M_mm == 0) return 0;
	// 只有中间和一侧有线时，使用中间和该侧数据
	if (L_mm != 0) return (int32_t)(M_mm - L_mm) * delta_ML_MM_ANGLE;
	if (R_mm != 0) return (int32_t)(R_mm - M_mm) * delta_MR_MM_ANGLE;
	// 中间有线，左右都没有线时，无法判断
	return 0;
}

/********************************* 单元测试 *********************************/
void DBG_TrackBoard_print(void)
{
	while (1)
	{
		uint8_t L = TrackBoard_read_L(), R = TrackBoard_read_R(), M = TrackBoard_read_M();
		log_info_f("L:%d,%d,%d,%d,%d,%d,%d,%d R:%d,%d,%d,%d,%d,%d,%d,%d M:%d,%d,%d,%d,%d,%d,%d,%d",
				   (uint8_t)(L >> 7) & 1, (uint8_t)(L >> 6) & 1, (uint8_t)(L >> 5) & 1, (uint8_t)(L >> 4) & 1,
				   (uint8_t)(L >> 3) & 1, (uint8_t)(L >> 2) & 1, (uint8_t)(L >> 1) & 1, (uint8_t)(L >> 0) & 1,
				   (uint8_t)(R >> 7) & 1, (uint8_t)(R >> 6) & 1, (uint8_t)(R >> 5) & 1, (uint8_t)(R >> 4) & 1,
				   (uint8_t)(R >> 3) & 1, (uint8_t)(R >> 2) & 1, (uint8_t)(R >> 1) & 1, (uint8_t)(R >> 0) & 1,
				   (uint8_t)(M >> 7) & 1, (uint8_t)(M >> 6) & 1, (uint8_t)(M >> 5) & 1, (uint8_t)(M >> 4) & 1,
				   (uint8_t)(M >> 3) & 1, (uint8_t)(M >> 2) & 1, (uint8_t)(M >> 1) & 1, (uint8_t)(M >> 0) & 1);
		osDelay(5);
	}
}
MAKE_CMD(void, DBG_TrackBoard_print, void);
