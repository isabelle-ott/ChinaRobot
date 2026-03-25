#include "mycan.h"
#include "can.h"

// ?? MyCan_Init ????????????????????????
static inline HAL_StatusTypeDef Mycan_Init_canFilter(MOTOR_BASIC_T *motor)
{
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // ??????
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; // 32锟斤拷??????锟斤拷??
	sFilterConfig.FilterMaskIdHigh = 0xFFFF;		   // ?????????锟斤拷???????16锟斤拷?
	sFilterConfig.FilterMaskIdLow = 0xF804;			   // ?????????锟斤拷???????16锟斤拷?
	sFilterConfig.FilterActivation = ENABLE;		   // ????????
	sFilterConfig.SlaveStartFilterBank = 10;		   // ???????????10???

	sFilterConfig.FilterBank = motor->FilterBank;
	sFilterConfig.FilterFIFOAssignment = motor->RxFifo;
	sFilterConfig.FilterIdHigh = ((motor->ID << 3) | 4) >> 16;
	sFilterConfig.FilterIdLow = ((motor->ID << 3) | 4);
	return HAL_CAN_ConfigFilter(motor->hcan, &sFilterConfig);
}

/**
 * @brief CAN?????????
 * @param motor_x ????4????????????????????????????????
 */
void MyCan_Init(MOTOR_BASIC_T *motor_1, MOTOR_BASIC_T *motor_2, MOTOR_BASIC_T *motor_3, MOTOR_BASIC_T *motor_4)
{
	/* ???锟斤拷????? */
	Mycan_Init_canFilter(motor_1);
	Mycan_Init_canFilter(motor_2);
	Mycan_Init_canFilter(motor_3);           
	Mycan_Init_canFilter(motor_4);
	/* ????CAN */
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
}

void MyCan_Transmit(CAN_HandleTypeDef *hcan, uint32_t data_id, uint8_t Length, uint8_t *Data)
{
	static uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxMessage = {.IDE = CAN_ID_EXT, .RTR = CAN_RTR_DATA};// ???ID?????(???ID)+???????
	TxMessage.ExtId = data_id;
	TxMessage.DLC = Length;
	/* ???? */
	HAL_CAN_AddTxMessage(hcan, &TxMessage, Data, &TxMailbox);
}

void MyCan_abort_Transmit(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
}

/**
 * @brief ??FIFO?锟斤拷??????
 * @param data_ptr ?????9??????,?锟斤拷?????????,??0锟斤拷?????,??8锟斤拷??????
 * @return ????????FIFO?锟斤拷????
 */
int8_t MyCan_Get_RxData(MOTOR_BASIC_T *motor, uint8_t *data_ptr)
{
	CAN_RxHeaderTypeDef RxHeader;
	int8_t FIFO_len = HAL_CAN_GetRxFifoFillLevel(motor->hcan, motor->RxFifo);
	if (FIFO_len == 0) return FIFO_len;
	HAL_CAN_GetRxMessage(motor->hcan, motor->RxFifo, &RxHeader, data_ptr + 1);
	data_ptr[0] = RxHeader.DLC;
	return FIFO_len;
}

/*
// ??????锟斤拷?
// FIFO0?????????
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static CAN_RxHeaderTypeDef CAN_RxHeader;
	while (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN_RxHeader, (uint8_t *)CAN1_Rx_data) != HAL_OK);
	if (CAN_RxHeader.IDE == CAN_ID_STD)
	{
		// ????????
	}
	else if (CAN_RxHeader.IDE == CAN_ID_EXT)
	{
		// ????????
	}
}

// FIFO1?????????
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	static CAN_RxHeaderTypeDef CAN_RxHeader;
	while (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &CAN_RxHeader, (uint8_t *)CAN2_Rx_data) != HAL_OK);
	if (CAN_RxHeader.IDE == CAN_ID_STD)
	{
		// ????????
	}
	else if (CAN_RxHeader.IDE == CAN_ID_EXT)
	{
		// ????????
	}
}
*/
