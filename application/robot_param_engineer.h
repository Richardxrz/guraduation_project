/**
  * @file       robot_param_engineer.h
  * @brief      这里是工程机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "macro_typedef.h"
#include "motor.h"
#include "robot_typedef.h"

/*-------------------- Parameters Set --------------------*/
#define GIMBAL_TYPE GIMBAL_NONE // 云台类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_ENGINEER_ARM // 机械臂类型
#define CHASSIS_TYPE CHASSIS_MECANUM_WHEEL // 底盘类型
#define CONTROL_MODE CHASSIS_ONLY // 控制模式类型

// 机器人物理参数

/*-------------------- Chassis --------------------*/
//电机ID ---------------------
#define WHEEL_1_ID (1)
#define WHEEL_2_ID (2)
#define WHEEL_3_ID (3)
#define WHEEL_4_ID (4)

//电机CAN ---------------------
#define WHEEL_1_CAN (1)
#define WHEEL_2_CAN (1)
#define WHEEL_3_CAN (1)
#define WHEEL_4_CAN (1)

//电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (1)
#define WHEEL_4_DIRECTION (1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

//motor type
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//physical parameters ---------------------
#define WHEEL_RADIUS 0.08f  //(m)轮子直径
#define WHEEL_CENTER_DISTANCE 0.304138127f  //(m)轮子到车的距离（0.22 + 0.21）

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (2.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式
// clang-format off

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_MECANNUM_VEL (20.0f)
#define KI_MECANNUM_VEL (0.3f)
#define KD_MECANNUM_VEL (0.3f)
#define MAX_IOUT_MECANNUM_VEL (10000.0f)
#define MAX_OUT_MECANNUM_VEL (30000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (5.3f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define KD_CHASSIS_FOLLOW_GIMBAL (4.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (5.0f)


/*-------------------- Mechanical arm --------------------*/
#define ARM_DJI_CAN 1 // can1
#define ARM_DM_CAN 2 // can2
//rc parameters ---------------------

#define MECHANICAL_ARM_MODE_CHANNEL DT7_SW_LEFT  // 机械臂模式切换通道
// #define PUMP_CHANNEL DT7_SW_RIGHT  // 气泵切换通道

//motor parameters ---------------------
// TODO: 关节电机的类型不同需要修改参数
/**
 * @brief 关节电机ID
 * @note  
 */
// J0、J1、J2的ID有问题
#define JOINT_MOTOR_0_ID 5  // DJi
#define JOINT_MOTOR_1_ID 1  // DM
#define JOINT_MOTOR_2_ID 2  // DM
#define JOINT_MOTOR_3_ID 3  // DJi 相当于2006或者3508的7
#define JOINT_MOTOR_4_ID 5  // DJi
#define JOINT_MOTOR_5_ID 6 // DJi
#define JOINT_MOTOR_6_ID 8  // DJi

// can1
#define JOINT_MOTOR_0_CAN ARM_DJI_CAN
// can2
#define JOINT_MOTOR_1_CAN ARM_DM_CAN
#define JOINT_MOTOR_2_CAN ARM_DM_CAN
#define JOINT_MOTOR_3_CAN ARM_DM_CAN
#define JOINT_MOTOR_4_CAN ARM_DM_CAN
#define JOINT_MOTOR_5_CAN ARM_DM_CAN
#define JOINT_MOTOR_6_CAN ARM_DM_CAN

// Mptor type
#define JOINT_MOTOR_0_TYPE DJI_M6020
#define JOINT_MOTOR_1_TYPE DM_8009
#define JOINT_MOTOR_2_TYPE DM_8009
#define JOINT_MOTOR_3_TYPE DJI_M6020
#define JOINT_MOTOR_4_TYPE DJI_M3508
#define JOINT_MOTOR_5_TYPE DJI_M2006
#define JOINT_MOTOR_6_TYPE DJI_M2006

// 电机方向
#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION 1
#define JOINT_MOTOR_2_DIRECTION 1
#define JOINT_MOTOR_3_DIRECTION 1
#define JOINT_MOTOR_4_DIRECTION 1
#define JOINT_MOTOR_5_DIRECTION 1
#define JOINT_MOTOR_6_DIRECTION 1

// 减速比
#define JOINT_MOTOR_0_REDUCATION_RATIO 1
#define JOINT_MOTOR_1_REDUCATION_RATIO 1
#define JOINT_MOTOR_2_REDUCATION_RATIO 1
#define JOINT_MOTOR_3_REDUCATION_RATIO 1
#define JOINT_MOTOR_4_REDUCATION_RATIO 19
#define JOINT_MOTOR_5_REDUCATION_RATIO 36
#define JOINT_MOTOR_6_REDUCATION_RATIO 36

// 电机模式
#define JOINT_MOTOR_0_MODE 0
#define JOINT_MOTOR_1_MODE DM_MODE_MIT
#define JOINT_MOTOR_2_MODE DM_MODE_MIT
#define JOINT_MOTOR_3_MODE 0
#define JOINT_MOTOR_4_MODE 0
#define JOINT_MOTOR_5_MODE 0
#define JOINT_MOTOR_6_MODE 0

// 
#define J0_ANGLE_TRANSFORM 0.0f
#define J1_ANGLE_TRANSFORM 0.0f
#define J2_ANGLE_TRANSFORM 0.0f
#define J3_ANGLE_TRANSFORM 0.0f
#define J4_ANGLE_TRANSFORM 0.0f
#define J5_ANGLE_TRANSFORM 0.0f
#define J6_ANGLE_TRANSFORM 0.0f

// clang-format off
// TODO: 这里根据机械设计修改软限位参数
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION  M_PI * 3
#define MAX_JOINT_1_POSITION  2.50f
#define MAX_JOINT_2_POSITION  5.0f
#define MAX_JOINT_3_POSITION  M_PI * 4
#define MAX_JOINT_4_POSITION  M_PI * 5
#define MAX_JOINT_5_POSITION  M_PI * 3
#define MAX_JOINT_6_POSITION  M_PI
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION -MAX_JOINT_0_POSITION
#define MIN_JOINT_1_POSITION -5.0f
#define MIN_JOINT_2_POSITION -10.0f
#define MIN_JOINT_3_POSITION -MAX_JOINT_3_POSITION
#define MIN_JOINT_4_POSITION -M_PI
#define MIN_JOINT_5_POSITION -MAX_JOINT_5_POSITION
#define MIN_JOINT_6_POSITION -M_PI
//clang-format on

//LPF parameters ---------------------
#define J0_LPF_ALPHA 0.0f
#define J1_LPF_ALPHA 0.0f
#define J2_LPF_ALPHA 0.0f
#define J3_LPF_ALPHA 0.985f
#define J4_LPF_ALPHA 0.0f
#define J5_LPF_ALPHA 0.0f
#define J6_LPF_ALPHA 0.0f

//PID parameters ---------------------
// J0角度环PID参数
#define KP_JOINT_0_ANGLE 10.0f
#define KI_JOINT_0_ANGLE 0.1f
#define KD_JOINT_0_ANGLE 0.0f
#define MAX_IOUT_JOINT_0_ANGLE 0.3f
#define MAX_OUT_JOINT_0_ANGLE 10.0f
// J0速度环PID参数
#define KP_JOINT_0_VELOCITY 4000.0f
#define KI_JOINT_0_VELOCITY 0.0f
#define KD_JOINT_0_VELOCITY 10.0f
#define MAX_IOUT_JOINT_0_VELOCITY 1000.0f
#define MAX_OUT_JOINT_0_VELOCITY 30000.0f

// J1角度环PID参数
#define KP_JOINT_1_ANGLE 10.0f
#define KI_JOINT_1_ANGLE 0.0f
#define KD_JOINT_1_ANGLE 1.0f
#define MAX_IOUT_JOINT_1_ANGLE 0.3f
#define MAX_OUT_JOINT_1_ANGLE 5.0f
// J1速度环PID参数
#define KP_JOINT_1_VELOCITY 0.0f
#define KI_JOINT_1_VELOCITY 0.0f
#define KD_JOINT_1_VELOCITY 0.0f
#define MAX_IOUT_JOINT_1_VELOCITY 0.0f
#define MAX_OUT_JOINT_1_VELOCITY 0.0f

// J2角度环PID参数
#define KP_JOINT_2_ANGLE 10.0f
#define KI_JOINT_2_ANGLE 0.5f
#define KD_JOINT_2_ANGLE 1.0f
#define MAX_IOUT_JOINT_2_ANGLE 0.3f
#define MAX_OUT_JOINT_2_ANGLE 5.0f
// J2速度环PID参数
#define KP_JOINT_2_VELOCITY 0.0f
#define KI_JOINT_2_VELOCITY 0.0f
#define KD_JOINT_2_VELOCITY 0.0f
#define MAX_IOUT_JOINT_2_VELOCITY 0.0f
#define MAX_OUT_JOINT_2_VELOCITY 0.0f

// J3角度环PID参数
#define KP_JOINT_3_ANGLE 10.0f
#define KI_JOINT_3_ANGLE 0.1f
#define KD_JOINT_3_ANGLE 0.0f
#define MAX_IOUT_JOINT_3_ANGLE 0.3f
#define MAX_OUT_JOINT_3_ANGLE 10.0f
// J3速度环PID参数
#define KP_JOINT_3_VELOCITY 4000.0f
#define KI_JOINT_3_VELOCITY 0.5f
#define KD_JOINT_3_VELOCITY 10.0f
#define MAX_IOUT_JOINT_3_VELOCITY 1000.0f
#define MAX_OUT_JOINT_3_VELOCITY 30000.0f

// J4角度环PID参数
#define KP_JOINT_4_ANGLE 3.0f
#define KI_JOINT_4_ANGLE 0.0f
#define KD_JOINT_4_ANGLE 0.0f
#define MAX_IOUT_JOINT_4_ANGLE 0.0f
#define MAX_OUT_JOINT_4_ANGLE 10.0f
// J4速度环PID参数
#define KP_JOINT_4_VELOCITY 1000.0f
#define KI_JOINT_4_VELOCITY 0.0f
#define KD_JOINT_4_VELOCITY 5.0f
#define MAX_IOUT_JOINT_4_VELOCITY 0.0f
#define MAX_OUT_JOINT_4_VELOCITY 4200.0f //10000.0f

// J5角度环PID参数
#define KP_JOINT_5_ANGLE 3.0f
#define KI_JOINT_5_ANGLE 0.0f
#define KD_JOINT_5_ANGLE 0.0f
#define MAX_IOUT_JOINT_5_ANGLE 0.0f
#define MAX_OUT_JOINT_5_ANGLE 10.0f
// J5速度环PID参数
#define KP_JOINT_5_VELOCITY 20.0f
#define KI_JOINT_5_VELOCITY 0.0f
#define KD_JOINT_5_VELOCITY 2.0f
#define MAX_IOUT_JOINT_5_VELOCITY 0.0f
#define MAX_OUT_JOINT_5_VELOCITY 2000.0f //10000.0f

// J6角度环PID参数
#define KP_JOINT_6_ANGLE 3.0f
#define KI_JOINT_6_ANGLE 0.0f
#define KD_JOINT_6_ANGLE 0.0f
#define MAX_IOUT_JOINT_6_ANGLE 0.0f
#define MAX_OUT_JOINT_6_ANGLE 10.0f
// J6速度环PID参数
#define KP_JOINT_6_VELOCITY 20.0f
#define KI_JOINT_6_VELOCITY 0.0f
#define KD_JOINT_6_VELOCITY 2.0f
#define MAX_IOUT_JOINT_6_VELOCITY 0.0f
#define MAX_OUT_JOINT_6_VELOCITY 500.0f //10000.0f

// Init parameters ---------------------
// Other parameters ---------------------

#endif 
/* INCLUDED_ROBOT_PARAM_H */
