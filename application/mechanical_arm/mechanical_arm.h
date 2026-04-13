/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Apr-21-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef MECHANICAL_ARM_H
#define MECHANICAL_ARM_H

#include "robot_param.h"

#if MECHANICAL_ARM_TYPE != MECHANICAL_ARM_NONE

#include "struct_typedef.h"

// 任务相关宏定义
#define MECHANICAL_ARM_TASK_INIT_TIME 201  // 任务初始化 空闲一段时间
#define MECHANICAL_ARM_CONTROL_TIME 1      // 任务控制间隔 1ms

// clang-format off
typedef struct
{
    void     (*SetCali)(void);
    void     (*CmdCali)(void);
    void     (*GetStatus)(void);
    uint32_t (*GetDuration)(void);
} MechanicalArmApi_t;
// clang-format on

extern MechanicalArmApi_t mechanical_arm;

#endif  // MECHANICAL_ARM_TYPR
#endif  // MECHANICAL_ARM_H
/*------------------------------ End of File ------------------------------*/
