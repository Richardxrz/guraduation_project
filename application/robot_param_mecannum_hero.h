/**
  * @file       robot_param_mecanum_hero.h
  * @brief      这里是麦轮英雄机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_MODE_CHANNEL 0  // 选择底盘状态 开关通道号

#define CHASSIS_TYPE CHASSIS_MECANUM_WHEEL      // 选择底盘类型
#define GIMBAL_TYPE  GIMBAL_YAW_PITCH_DIRECT     // 选择云台类型
#define SHOOT_TYPE  SHOOT_FRIC_TRIGGER          // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS 0.08f                 //(m)轮子半径
#define WHEEL_CENTER_DISTANCE 0.31340867f  //(m)轮子到车的距离（0.22 + 0.21）

//motor parameters ---------------------
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
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

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

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_MECANNUM_VEL (115.0f)
#define KI_MECANNUM_VEL (0.2f)
#define KD_MECANNUM_VEL (0.0f)
#define MAX_IOUT_MECANNUM_VEL (1000.0f)
#define MAX_OUT_MECANNUM_VEL (16000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (5.3f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define KD_CHASSIS_FOLLOW_GIMBAL (4.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (0.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (5.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (2.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

/*-------------------- Gimbal --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (100000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_DIRECT_YAW_ID ((uint8_t)4)
#define GIMBAL_DIRECT_PITCH_ID ((uint8_t)2)

//电机can口
#define GIMBAL_DIRECT_YAW_CAN ((uint8_t)2)
#define GIMBAL_DIRECT_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_DIRECT_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_DIRECT_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_DIRECT_YAW_DIRECTION (1)
#define GIMBAL_DIRECT_PITCH_DIRECTION (-1)

//减速比
#define GIMBAL_DIRECT_YAW_REDUCTION_RATIO (1)
#define GIMBAL_DIRECT_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_DIRECT_YAW_MODE (0)
#define GIMBAL_DIRECT_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.200f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.760f)

//电机角度中值设置
#define GIMBAL_DIRECT_PITCH_MID (0.94f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_DIRECT_YAW_MID (-2.089f)   //云台初始化正对齐的时候使用的yaw轴正中心量
//PID parameters ---------------------
//YAW ANGLE
#define KP_GIMBAL_YAW_ANGLE (30.0f)//60
#define KI_GIMBAL_YAW_ANGLE (0.00f)//0.02
#define KD_GIMBAL_YAW_ANGLE (0.0f)
#define MAX_IOUT_GIMBAL_YAW_ANGLE (0.0f)//2
#define MAX_OUT_GIMBAL_YAW_ANGLE (7.0f)
//VELOCITY:角速度
#define KP_GIMBAL_YAW_VELOCITY (11000.0f)//23000
#define KI_GIMBAL_YAW_VELOCITY (7.0f)//8.0
#define KD_GIMBAL_YAW_VELOCITY (4.0f)//5.0
#define MAX_IOUT_GIMBAL_YAW_VELOCITY (6000.0f)//6000
#define MAX_OUT_GIMBAL_YAW_VELOCITY (30000.0f)//30000

//PITCH ANGLE
#define KP_GIMBAL_PITCH_ANGLE (45.0f)  //90
#define KI_GIMBAL_PITCH_ANGLE (0.0f)  //0.02
#define KD_GIMBAL_PITCH_ANGLE (0.0f)   //50
#define MAX_IOUT_GIMBAL_PITCH_ANGLE (0.0f)
#define MAX_OUT_GIMBAL_PITCH_ANGLE (8.0f)
//VELOCITY:角速度
#define KP_GIMBAL_PITCH_VELOCITY (8000.0f)
#define KI_GIMBAL_PITCH_VELOCITY (1.0f)
#define KD_GIMBAL_PITCH_VELOCITY (0.0f)
#define MAX_IOUT_GIMBAL_PITCH_VELOCITY (5000.0f)
#define MAX_OUT_GIMBAL_PITCH_VELOCITY (30000.0f)
/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.03f                 // (m)摩擦轮半径
#define BULLET_NUM 6                      // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                         // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 0.66666f  // 定义英雄电机到拨弹盘的齿轮减速比

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DM_4310)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 1
#define FRIC_MOTOR_R_ID 7
#define FRIC_MOTOR_L_ID 6

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 1
#define FRIC_MOTOR_L_CAN 1

//电机std_id
#define STD_ID 0x1FF

//单环拨弹速度
#define TRIGGER_SPEED (-7.0f)
//摩擦轮速度
#define FRIC_R_SPEED (800.0f)
#define FRIC_L_SPEED (-800.0f)
#define FRIC_SPEED_LIMIT (500.0f)

/*ECD parameters------------*/
//电机反馈码盘值范围(无)
#define HALF_ECD_RANGE (0.0f)
#define ECD_RANGE (0.0f)

//电机rpm 变化成 旋转速度的比例(无)
#define MOTOR_RPM_TO_SPEED (0.0f)
#define MOTOR_ECD_TO_ANGLE (0.0f)
#define FULL_COUNT (0.0f)

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 0.1f
#define BLOCK_TIME 1000
#define REVERSE_TIME 700
#define REVERSE_SPEED (2.0f)  // (rad/s)

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (2.0f)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环  (none)
#define TRIGGER_SPEED_PID_KP (0.0f)
#define TRIGGER_SPEED_PID_KI (0.0f)
#define TRIGGER_SPEED_PID_KD (0.0f)

#define TRIGGER_SPEED_PID_MAX_OUT (0.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (0.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (15.0f)
#define TRIGGER_ANGEL_PID_KI (0.01f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (10.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (2.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (800.0f)
#define FIRC_SPEED_PID_KI (0.1f)
#define FRIC_SPEED_PID_KD (233.3f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE 80  //80

#endif /* INCLUDED_ROBOT_PARAM_H */
