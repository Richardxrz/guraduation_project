/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       communication.c/h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *  V1.1.0     2025-03-10      Harry_Wong      1. 初步完成自定义内容通信
  *  V1.2.0     Apr-01-2025     Penguin         1. 重构与优化
  *
  @verbatim
  ==============================================================================
  关于Usart1 和 Uart2 之间的关系：Usart1是内部配置，Uart2是C板上的外侧标注，两者为对应关系。

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "communication.h"

#include "CRC8_CRC16.h"
#include "bsp_uart.h"
#include "bsp_usart.h"
#include "detect_task.h"
#include "fifo.h"
#include "robot_param.h"
#include "signal_generator.h"
#include "uart2_typedef.h"
/*******************************************************************************/
/* Macro Definitions                                                           */
/*******************************************************************************/

#define UART2_OFFLINE_TIME 200  // ms

#define USART_RX_BUF_LENGHT 512
#define USART1_FIFO_BUF_LENGTH 1024

/**
 * @brief 数据包初始化宏定义
 */
#define UART2DataInit(data_name)                                                            \
    {                                                                                       \
        memset(&(Send_Data_##data_name##), 0, sizeof(Data_##data_name##_s));                \
        Send_Data_##data_name##.frame_header.sof = UART2_COMMUNICATE_SOF;                   \
        Send_Data_##data_name##.frame_header.len = sizeof(Send_Data_##data_name##.data);    \
        Send_Data_##data_name##.frame_header.id = Uart2_Data_##data_name##_ID;              \
        Send_Data_##data_name##.frame_header.type = 0;                                      \
        append_CRC8_check_sum(                                                              \
            (uint8_t *)(&(Send_Data_##data_name##.frame_header)), UART2_FRAME_HEADER_SIZE); \
        LastSendTime.Data_##data_name## = 0;                                                \
    }

/**
 * @brief 对于数据发送进行宏定义
 */
#define Uart2CheckDurationAndSend(data_name)                                                       \
    {                                                                                              \
        if (HAL_GetTick() - LastSendTime.Data_##data_name## >=                                     \
            Uart2_Data_##data_name##_Duration) {                                                   \
            LastSendTime.Data_##data_name## = HAL_GetTick();                                       \
            Data##data_name##Renew();                                                              \
            UartSendTxMessage(                                                                     \
                &huart1, (uint8_t *)(&(Send_Data_##data_name##)), sizeof(Send_Data_##data_name##), \
                UART2_COMMUNICATE_TIMEOUT);                                                        \
        }                                                                                          \
    }

/**
 * @brief 对于接收数据进行存储
 */
#define UART2DataSave(data_name)                                                          \
    {                                                                                     \
        uint16_t crc_ok = verify_CRC16_check_sum(received, sizeof(Data_##data_name##_s)); \
        if (crc_ok) {                                                                     \
            memcpy(&Receive_Data_##data_name##, received, sizeof(Data_##data_name##_s));  \
        }                                                                                 \
    }

/*******************************************************************************/
/* Variable Definitions                                                        */
/*******************************************************************************/
uint32_t TASK_DURATION;
uint32_t LAST_TASK_TIME;

// clang-format off
// data time record
LastTime_t LastSendTime;
LastTime_t LastReceiveTime;

// send data
Data_Test_s   Send_Data_Test;
Data_Rc_s     Send_Data_Rc;
Data_Gimbal_s Send_Data_Gimbal;

// receive data
Data_Test_s   Receive_Data_Test;
Data_Rc_s     Receive_Data_Rc;
Data_Gimbal_s Receive_Data_Gimbal;

// receive data buffer
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t usart1_fifo;
uint8_t usart1_fifo_buf[USART1_FIFO_BUF_LENGTH];
UnpackData_t usart1_unpack_obj;
// clang-format on

/*******************************************************************************/
/* Main Functions                                                              */
/*     Uart2SendDataInit                                                       */
/*     Usart1Init                                                              */
/*     USART1_IRQHandler                                                       */
/*******************************************************************************/

// uart数据包初始化
void Uart2SendDataInit(void)
{
    UART2DataInit(Test);
    UART2DataInit(Rc);
    UART2DataInit(Gimbal);
}

// 4pin Uart串口初始化
void Usart1Init(void)
{
    fifo_s_init(&usart1_fifo, usart1_fifo_buf, USART1_FIFO_BUF_LENGTH);
    usart1_init(usart1_buf[0], usart1_buf[1], USART_RX_BUF_LENGHT);

    Uart2SendDataInit();
}

// 4pin Uart口中断处理函数
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if (USART1->SR & UART_FLAG_IDLE) {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET) {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[0], this_time_rx_len);
            LastReceiveTime.Interrupt = HAL_GetTick();
        } else {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, USART_RX_BUF_LENGHT);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart1.hdmarx);
            fifo_s_puts(&usart1_fifo, (char *)usart1_buf[1], this_time_rx_len);
            LastReceiveTime.Interrupt = HAL_GetTick();
        }
    }
}

/*******************************************************************************/
/* Uart2 Send Data Renew                                                       */
/*     DataTestRenew                                                           */
/*     DataRcRenew                                                             */
/*     DataGimbalRenew                                                         */
/*******************************************************************************/

void DataTestRenew()
{
    Send_Data_Test.time_stamp = HAL_GetTick();

    Send_Data_Test.data.index++;
    Send_Data_Test.data.test_float = GenerateSinWave(10, 0, 4);

    append_CRC16_check_sum((uint8_t *)(&Send_Data_Test), sizeof(Data_Test_s));
}

void DataRcRenew()
{
    Send_Data_Rc.time_stamp = HAL_GetTick();

    memcpy(&Send_Data_Rc.data.rc_ctrl, get_remote_control_point(), sizeof(RC_ctrl_t));
    Send_Data_Rc.data.rc_offline = GetRcOffline();
    append_CRC16_check_sum((uint8_t *)(&Send_Data_Rc), sizeof(Data_Rc_s));
}

void DataGimbalRenew()
{
    Send_Data_Gimbal.time_stamp = HAL_GetTick();

    Send_Data_Gimbal.data.yaw_motor_offline = true;
    append_CRC16_check_sum((uint8_t *)(&Send_Data_Gimbal), sizeof(Data_Gimbal_s));
}

/*******************************************************************************/
/* Uart2 Receive Data Solve Functions                                          */
/*     Uart2DataSolve                                                          */
/*     DataUnpack                                                              */
/*******************************************************************************/

void Uart2DataSolve(uint8_t * frame)
{
    uint32_t time_stamp = 0;

    uint8_t index = 0;

    FrameHeader_t frame_header;

    memcpy(&frame_header, frame, sizeof(FrameHeader_t));
    index += sizeof(FrameHeader_t);

    memcpy(&time_stamp, frame + index, sizeof(uint32_t));
    index += sizeof(uint32_t);

    switch (frame_header.id) {
        case Uart2_Data_Test_ID: {
            memcpy(&Receive_Data_Test, frame, sizeof(Data_Test_s));
            LastReceiveTime.Data_Test = HAL_GetTick();
        } break;
        case Uart2_Data_Rc_ID: {
            memcpy(&Receive_Data_Rc, frame, sizeof(Data_Rc_s));
            LastReceiveTime.Data_Rc = HAL_GetTick();

#if __CONTROL_LINK_RC == CL_RC_UART2
            const RC_ctrl_t * rc_ctrl = get_remote_control_point();
            memcpy((RC_ctrl_t *)rc_ctrl, &Receive_Data_Rc.data.rc_ctrl, sizeof(RC_ctrl_t));
#endif
        } break;
        case Uart2_Data_Gimbal_ID: {
            memcpy(&Receive_Data_Gimbal, frame, sizeof(Data_Gimbal_s));
            LastReceiveTime.Data_Gimbal = HAL_GetTick();
        } break;
        default:
            break;
    }
}

/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void DataUnpack(void)
{
    uint8_t byte = 0;
    UnpackData_t * p_obj = &usart1_unpack_obj;

    while (fifo_s_used(&usart1_fifo)) {
        byte = fifo_s_get(&usart1_fifo);
        switch (p_obj->unpack_step) {
            case STEP_HEADER_SOF: {
                if (byte == UART2_COMMUNICATE_SOF) {
                    p_obj->unpack_step = STEP_LENGTH;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            } break;

            case STEP_LENGTH: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_ID;
            } break;

            case STEP_ID: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (UART2_FRAME_MAX_SIZE - UART2_HEADER_CRC_TIMESTAMP_LEN)) {
                    p_obj->unpack_step = STEP_TYPE;
                } else {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            } break;

            case STEP_TYPE: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            } break;

            case STEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == UART2_FRAME_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(p_obj->protocol_packet, UART2_FRAME_HEADER_SIZE)) {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            } break;

            case STEP_DATA_CRC16: {
                if (p_obj->index < UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len) {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;

                    if (verify_CRC16_check_sum(
                            p_obj->protocol_packet,
                            UART2_HEADER_CRC_TIMESTAMP_LEN + p_obj->data_len)) {
                        Uart2DataSolve(p_obj->protocol_packet);
                    }
                }
            } break;

            default: {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            } break;
        }
    }
}

/*******************************************************************************/
/* Uart2 Task Loop Function                                                    */
/*     Uart2TaskLoop                                                           */
/*******************************************************************************/

void Uart2TaskLoop(void)
{
    TASK_DURATION = HAL_GetTick() - LAST_TASK_TIME;
    LAST_TASK_TIME = HAL_GetTick();

#if __SELF_BOARD_ID == C_BOARD_BALANCE_CHASSIS
    Uart2CheckDurationAndSend(Rc);
#elif __SELF_BOARD_ID == C_BOARD_BALANCE_GIMBAL
    Uart2CheckDurationAndSend(Gimbal);
#elif __SELF_BOARD_ID == C_BOARD_DEFAULT
    Uart2CheckDurationAndSend(Test);
#endif

    DataUnpack();
}

/*******************************************************************************/
/* API                                                                         */
/*     GetUartOffline                                                          */
/*     GetUartRcToeError                                                       */
/*     GetUartGimbalYawMotorPos                                                */
/*     GetUartGimbalInitJudge                                                  */
/*******************************************************************************/

bool GetUartOffline(void)
{
    if (HAL_GetTick() - LastReceiveTime.Interrupt > UART2_OFFLINE_TIME) {
        return true;
    }
    return false;
}

bool GetUartRcOffline(void)
{
    if (GetUartOffline()) {
        return true;
    }
    return Receive_Data_Rc.data.rc_offline;
}

float GetUartGimbalYawMotorPos(void)
{
    if (GetUartOffline()) {
        return 0;
    }
    return Receive_Data_Gimbal.data.yaw_motor_pos;
}

bool GetUartGimbalInitJudge(void)
{
    if (GetUartOffline()) {
        return false;
    }
    return Receive_Data_Gimbal.data.init_judge;
}

uint32_t GetUartTimeStampForTest(void) { return Receive_Data_Test.time_stamp; }

/*------------------------------ End of File ------------------------------*/
