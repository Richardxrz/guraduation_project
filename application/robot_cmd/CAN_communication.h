/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_communication.c/h
  * @brief      CAN通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信时stdid的内容如下
data_type + (data_id << 4) + target_id

bit 0-3: target_id
bit 4-7: data_id
bit 8-11: data_type

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include "CAN_cmd_dji.h"
#include "CAN_receive.h"

extern void CanSendRcDataToBoard(uint8_t can, uint16_t target_id, uint16_t index);

extern void CanSendGimbalDataToBoard(uint8_t can, uint16_t target_id, uint16_t index);

#endif  // CAN_COMMUNICATION_H
