/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_mecanum.c/h
  * @brief      麦轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.1.19       Harry_Wong        1.重新构建麦轮底盘代码，完成基础控制
  *  V1.1.0   2025.11.2       LYX               1.添加底盘的键鼠控制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H
#include "motor.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include  "user_lib.h"
#include "CAN_cmd_dji.h"



/*-------------------- Structural definition --------------------*/
typedef enum {
    CHASSIS_LOCK,      //底盘锁定，所有轮子速度设定为0
    CHASSIS_SINGLE,    //只有底盘的模式
    CHASSIS_FOLLOW,    //云台跟随模式
    CHASSIS_SPIN  ,    //云台小陀螺模式 
} ChassisMode_e;

/**
 * @brief  底盘轮子PID
 */
 typedef struct
{
    pid_type_def wheel_velocity[4];//麦轮速度解算PID

    pid_type_def follow; //云台跟随PID
} PID_t;

/**
 * @brief  底盘期望
 */
typedef struct
{
    float vx;
    float vy;
    float wz;
} Reference_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    ChassisMode_e mode;    // 底盘模式

    /*-------------------- Motors --------------------*/
    Motor_s wheel[4];  //底盘电机

    /*-------------------- Values --------------------*/
    Reference_t reference_rc;
    Reference_t reference; 

    fp32 feedback[4];
    fp32 set[4];
    
    fp32 yaw_delta;
    uint8_t sc_flag;
    uint32_t f_flag;
    bool shift_pressed;
    uint8_t spin_flag;
    LowPassFilter_t vx_flt;
    LowPassFilter_t vy_flt;
} Chassis_s;


extern void ChassisInit(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

extern inline int GetChassisSpinStateMEC(void);

#endif 
#endif 
