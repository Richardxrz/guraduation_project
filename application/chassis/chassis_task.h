/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"

extern void chassis_task(void const * pvParameters);

extern void set_cali_chassis_hook(const fp32 motor_middle[4]);

extern bool_t cmd_cali_chassis_hook(fp32 motor_middle[4]);

#endif
/*------------------------------ End of File ------------------------------*/
