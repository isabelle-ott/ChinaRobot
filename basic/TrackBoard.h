#ifndef __TRACKBOARD_H__
#define __TRACKBOARD_H__
#include "bsp.h"

uint8_t TrackBoard_read_L(void);

uint8_t TrackBoard_read_R(void);

uint8_t TrackBoard_read_M(void);

uint8_t TrackBoard_read_HomeUpper(void);

uint8_t TrackBoard_read_HomeLower(void);

uint32_t TrackBoard_to_mm(uint8_t sensor);

int32_t TrackBoardLR_to_angle(uint8_t L_mm, uint8_t R_mm);

int32_t TrackBoardMLR_to_angle(uint8_t M, uint8_t L_mm, uint8_t R_mm);

#endif // __TRACKBOARD_H__

