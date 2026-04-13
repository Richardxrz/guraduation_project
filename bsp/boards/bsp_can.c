/**
 * @file bsp_can.c
 * @author your name (you@domain.com)
 * @brief CAN bus interface implementation
 * @version 0.1
 * @date 2025-12-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "bsp_can.h"

/**
 * @brief          CAN滤波器初始化
 * @param[in]      none
 * @return         none
 */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief          发送控制数据（旧版本 - 已注释）
 * @param[in]      can_handle 选择CAN1或CAN2
 * @param[in]      tx_header  CAN发送数据header
 * @param[in]      tx_data    发送数据
 * @return         none
 */
void CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
{
    uint32_t send_mail_box;
    uint8_t cnt = 20;  // 重复检测次数

    uint32_t free_TxMailbox =
        HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);  // 检测是否有空闲邮箱
    while (free_TxMailbox < 3 && cnt--) {                      // 等待空闲邮箱数达到3
        free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
    }
    HAL_CAN_AddTxMessage(
        can_ctrl_data->hcan, &can_ctrl_data->tx_header, can_ctrl_data->tx_data, &send_mail_box);
}

/**
 * @brief          发送控制数据（新版本 - 带错误检查）
 * @param[in]      can_ctrl_data CAN控制数据结构体指针
 * @return         HAL_StatusTypeDef HAL_OK=成功, HAL_ERROR=失败
 */
// void CAN_SendTxMessage(CanCtrlData_s * can_ctrl_data)
// {
//     uint32_t send_mail_box;
//     uint8_t cnt = 100;  // 增加重试次数到100次
//     HAL_StatusTypeDef status;

//     // 等待有空闲邮箱
//     uint32_t free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
//     while (free_TxMailbox == 0 && cnt--) {  // 修改条件：只要有至少1个空闲邮箱就行
//         free_TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(can_ctrl_data->hcan);
//     }

//     // 如果还是没有空闲邮箱，返回错误（但旧接口是void，所以只能继续尝试）
//     if (free_TxMailbox == 0) {
//         // TODO: 添加错误日志
//         return;  // 直接返回，不发送
//     }

//     // 发送消息并检查返回值
//     status = HAL_CAN_AddTxMessage(
//         can_ctrl_data->hcan, &can_ctrl_data->tx_header,
//         can_ctrl_data->tx_data, &send_mail_box);

//     // 如果发送失败，可以添加错误处理
//     if (status != HAL_OK) {
//         // TODO: 添加错误日志或LED指示
//     }
// }
