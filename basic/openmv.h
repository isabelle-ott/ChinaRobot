#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "bsp.h"

typedef enum
{
    openMV_mode_IDLE,
    openMV_mode_YPJ,
    openMV_mode_JT,
    openMV_mode_LZ,
    openMV_mode_LZ_WHITE
} openMV_mode_e;

typedef struct
{
	uint8_t type;	 // 数据类型 (0x50回应,0x51红篮球,0x52黄球,0x53圆环,0x54方块,0x55红篮球,0x56白球)
	int8_t x, y;	 // 坐标 (单位是像素)
} MV_DATA_t;

void openMV_init(void);

void openMV_set_mode(openMV_mode_e mode);

uint8_t openMV_get_data(openMV_mode_e mode, MV_DATA_t *data);

void openMV_clear_data(void);

void openMV_itCall(uint16_t Size);

uint32_t openMV_get_errorCount(void);

#endif // __OPENMV_H__

