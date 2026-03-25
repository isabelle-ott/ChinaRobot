#ifndef __QR_CODE_H
#define __QR_CODE_H

#include "main.h"
#include "bsp.h"

#define QR_DATA_SIZE 1 // 2维码的数据长度

typedef struct QR_T
{
	UART_HandleTypeDef *QR_huart;
	int QR_data;
	uint8_t *QR_Buf;
	void (*_QR_Init)(void);
	void (*_QR_USART_DATA_Processing)(uint8_t);
	int (*_Get_QR_Data)(void);
	void (*_Set_QR_Data_Zero)(void);
} QR_T;

extern QR_T My_QR_Handle;
void QR_USART_Error_Processing(void);

#endif
