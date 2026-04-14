/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_mecanum.c/h
  * @brief      麦轮轮底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.1.19       Harry_Wong        1.重新构建麦克纳姆轮底盘，完成单底盘控制
  *  V1.1.0   2025.11.2       LYX               1.添加底盘的键鼠控制
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/


#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#include "chassis_mecanum.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "motor.h"
#include "math.h"
#include "robot_param_mecannum.h"
#include "mechanical_arm_engineer.h"


Chassis_s chassis;
PID_t chassis_pid;


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    //step1 获取所有所需变量指针
    chassis.lookline = get_remote_control_point();

    //step2 PID数据清零，设置PID参数
    const static fp32 wheel_vel[3]={KP_MECANNUM_VEL,KI_MECANNUM_VEL,KD_MECANNUM_VEL};
    for (int i=0;i<4;++i)
    {
        PID_init(&chassis_pid.wheel_velocity[i],PID_POSITION,wheel_vel,MAX_OUT_MECANNUM_VEL,MAX_IOUT_MECANNUM_VEL);
    }
    // 初始化低通滤波器
    const static fp32  key_alpha=0.005;
    LowPassFilterInit(&chassis.vx_flt,key_alpha);
    LowPassFilterInit(&chassis.vy_flt,key_alpha);
    //step3 初始化电机
    MotorInit(&chassis.wheel[0],WHEEL_1_ID,WHEEL_1_CAN,WHEEL_1_MOTOR_TYPE,WHEEL_1_DIRECTION,WHEEL_1_RATIO,WHEEL_1_MODE);
    MotorInit(&chassis.wheel[1],WHEEL_2_ID,WHEEL_2_CAN,WHEEL_2_MOTOR_TYPE,WHEEL_2_DIRECTION,WHEEL_2_RATIO,WHEEL_2_MODE);
    MotorInit(&chassis.wheel[2],WHEEL_3_ID,WHEEL_3_CAN,WHEEL_3_MOTOR_TYPE,WHEEL_3_DIRECTION,WHEEL_3_RATIO,WHEEL_3_MODE);
    MotorInit(&chassis.wheel[3],WHEEL_4_ID,WHEEL_4_CAN,WHEEL_4_MOTOR_TYPE,WHEEL_4_DIRECTION,WHEEL_4_RATIO,WHEEL_4_MODE);

    //step4 初始模式设置
    chassis.mode = CHASSIS_LOCK;
    chassis.sc_flag = 0;
    chassis.spin_flag = 0;
}


/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
    MechanicalArmMode_e arm_mode = GetMechanicalArmMode();
    // 修复：当机械臂处于DEBUG或FOLLOW模式时，底盘锁定
    if (arm_mode == MECHANICAL_ARM_DEBUG || arm_mode == MECHANICAL_ARM_FOLLOW)
    {
        chassis.mode = CHASSIS_LOCK;
        return;
    }
#endif

    if (switch_is_down(chassis.lookline->lookline.s[0]))
    {
        chassis.mode = CHASSIS_LOCK;
    }
    else if (switch_is_mid(chassis.lookline->lookline.s[0]))
    {
        chassis.mode = CHASSIS_SINGLE;
    }
    else if (switch_is_up(chassis.lookline->lookline.s[0]))
    {
        chassis.mode = CHASSIS_SINGLE;
    }
}


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) 
{
    for (int i=0;i<4;++i)
    {
        GetMotorMeasure(&chassis.wheel[i]);
    }

    for (int i=0;i<4;++i)
    {
        chassis.feedback[i] = chassis.wheel[i].fdb.vel;
    }
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    if (chassis.mode == CHASSIS_LOCK)
    {
        chassis.reference.vx=0;
        chassis.reference.vy=0;
        chassis.reference.wz=0;
    }
    else if (chassis.mode == CHASSIS_SINGLE)
    {
        chassis.reference.vx=fp32_deadline(chassis.lookline->lookline.ch[2],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
        chassis.reference.vy=fp32_deadline(-chassis.lookline->lookline.ch[3],-CHASSIS_RC_DEADLINE,CHASSIS_RC_DEADLINE)/CHASSIS_RC_MAX_RANGE*CHASSIS_RC_MAX_SPEED;
        chassis.reference.wz=0;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    chassis.set[0] = ( sqrt(2)/2*(chassis.reference.vx - chassis.reference.vy) - WHEEL_CENTER_DISTANCE*chassis.reference.wz )/WHEEL_RADIUS*chassis.wheel[0].reduction_ratio*chassis.wheel[0].direction;
    chassis.set[1] = ( sqrt(2)/2*(chassis.reference.vx + chassis.reference.vy) - WHEEL_CENTER_DISTANCE*chassis.reference.wz )/WHEEL_RADIUS*chassis.wheel[1].reduction_ratio*chassis.wheel[1].direction;
    chassis.set[2] = ( sqrt(2)/2*(chassis.reference.vx - chassis.reference.vy) + WHEEL_CENTER_DISTANCE*chassis.reference.wz )/WHEEL_RADIUS*chassis.wheel[2].reduction_ratio*chassis.wheel[2].direction;
    chassis.set[3] = ( sqrt(2)/2*(chassis.reference.vx + chassis.reference.vy) + WHEEL_CENTER_DISTANCE*chassis.reference.wz )/WHEEL_RADIUS*chassis.wheel[3].reduction_ratio*chassis.wheel[3].direction;

    for (int i=0;i<4;++i)
    {
        chassis.wheel[i].set.curr = PID_calc(&chassis_pid.wheel_velocity[i], chassis.feedback[i], chassis.set[i]);
    }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void){
    CanCmdDjiMotor(1,0x200,chassis.wheel[0].set.curr,chassis.wheel[1].set.curr,chassis.wheel[2].set.curr,chassis.wheel[3].set.curr);
}
inline int GetChassisSpinStateMEC(void)
{
    return chassis.spin_flag;
}
#endif
