/**
 * @file bsp_can.h
 * @author your name (you@domain.com)
 * @brief CAN bus interface definitions
 * @version 0.1
 * @date 2025-12-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

// clang-format off
#define BOARD_DATA_ANY     ((uint16_t)0x500)
#define BOARD_DATA_UINT16  ((uint16_t)0x600)
// clang-format on

typedef CAN_HandleTypeDef hcan_t;

typedef struct __CanCtrlData
{
    hcan_t * hcan;
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
} CanCtrlData_s;

extern hcan_t hcan1;
extern hcan_t hcan2;

extern void can_filter_init(void);

extern void CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data);
#endif
