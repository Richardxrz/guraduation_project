/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_engineer.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-20-2024     Penguin         1. done
  *  V1.0.1     Jan-14-2025     Penguin         1. 实现机械臂的基本控制
  *
  @verbatim
  ==============================================================================
机械臂相关的一些方向定义
    - 机械臂水平向前时设置J0关节为0位置
    - 机械臂竖直向上时设置J1 J2 J3关节为0位置

    - 定义机械臂水平向前时的J0关节位置为0，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，从机械臂右侧看，逆时针为正方向（注：J1 J2关节的合位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0，从吸盘方向看，逆时针为正方向
    - 定义J4为末端机构右侧（上视，J3向前）电机，J5为末端机构左侧电机

    - 定义虚拟J4关节用来衡量末端机构的pitch, 虚拟J5关节用来衡量末端机构的roll
    - 定义J4正方向为：当J3归中时，从机械臂右侧看，逆时针为正方向
    - 定义J5正方向为：当J3归中时，从机械臂前方看，逆时针为正方向
    - vj4和j4 j5的关系：vj4 = (j4 - j5)/2

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "robot_param.h"

#ifndef MECHANICAL_ARM_ENGINEER_H
#define MECHANICAL_ARM_ENGINEER_H

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include "mechanical_arm.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define JOINT_NUM 7  // 关节数量

/*-------------------- Structural definition --------------------*/

typedef enum {
    MECHANICAL_ARM_SAFE,
    MECHANICAL_ARM_CALIBRATE,
    MECHANICAL_ARM_FOLLOW,
    MECHANICAL_ARM_DEBUG,
    MECHANICAL_ARM_CUSTOM,
    MECHANICAL_ARM_INIT,
    MECHANICAL_ARM_HOLD,  // 保持位姿模式，允许底盘移动
} MechanicalArmMode_e;

/**
 * @brief  机械臂数据结构体
 */
typedef struct
{
    const RC_ctrl_t * lookline;          // 遥控器指针
    MechanicalArmMode_e mode;      // 机械臂模式
    uint8_t error_code;            // 机械臂错误代码
    bool init_completed;           // 机械臂初始化完成标志
    bool custom_controller_ready;  // 自定义控制器准备就绪标志
    uint32_t reach_time;           // 达到目标的持续时间

    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
    /*-------------------- Motors --------------------*/
    Motor_s joint_motor[JOINT_NUM];
    /*-------------------- Values --------------------*/

    struct
    {
        struct
        {
            float angle;     // (rad)位置
            float velocity;  // (rad/s)速度
        } joint[JOINT_NUM];
    } ref;

    struct
    {
        struct
        {
            float angle;     // (rad)位置
            float velocity;  // (rad/s)速度
            float torque;    // (N*m)力矩
            int16_t round;   // 圈数(当前位于第几圈中，初始为0)
        } joint[JOINT_NUM];
    } fdb;

    struct
    {
        float dpos[7];
        uint8_t duration[7];
    } transform;

    struct
    {
        struct
        {
            float pos[7];
            float vj4_pos;
            float vj5_pos;
        } max;
        struct
        {
            float pos[7];
            float vj4_pos;
            float vj5_pos;
        } min;
    } limit;

    struct
    {
        bool pump_on;
    } cmd;

    struct
    {  // 0为角度环，1为速度环
        pid_type_def j0[2];
        pid_type_def j1[2];
        pid_type_def j2[2];
        pid_type_def j3[2];
        pid_type_def j4[2];
        pid_type_def j5[2];
        pid_type_def j6[2]; // 添加J6的PID
    } pid;

    struct
    {
        LowPassFilter_t j[7]; // 改为7
    } lpf;

    struct {
          float angle[2];  // 舵机0和1的目标角度（0-180°）
    } servo;

} MechanicalArm_s;

/*-------------------- Function Declaration --------------------*/

extern void MechanicalArmPublish(void);
extern void MechanicalArmInit(void);
extern void MechanicalArmHandleException(void);
extern void MechanicalArmSetMode(void);
extern void MechanicalArmObserver(void);
extern void MechanicalArmReference(void);
extern void MechanicalArmConsole(void);
extern void MechanicalArmSendCmd(void);
extern MechanicalArmMode_e GetMechanicalArmMode(void);
extern float GetMechanicalArmJ0Angle(void);

#endif
#endif  // MECHANICAL_ARM_ENGINEER_H
/*------------------------------ End of File ------------------------------*/
