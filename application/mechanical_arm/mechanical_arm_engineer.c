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

#include "mechanical_arm_engineer.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include <stdbool.h>

#include "CAN_communication.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "signal_generator.h"
#include "string.h"

/*------------------------------ Macro Definition ------------------------------*/

// clang-format off
#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define CUSTOM_CONTROLLER_DATA_ERROR_OFFSET   ((uint8_t)1 << 1)  // 自定义控制器数据异常偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
// clang-format on

#define MS_TO_S 0.001f  // ms转s

#define ANGLE_PID 0
#define VELOCITY_PID 1

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5
#define J6 6

#define DM_DELAY 500  // (us)dm电机发送延时

#define J0_KP_FOLLOW 0
#define J0_KD_FOLLOW 1.5

#define J1_KP_FOLLOW 0
#define J1_KD_FOLLOW 10

#define J2_KP_FOLLOW 0
#define J2_KD_FOLLOW 10

#define INIT_2006_SET_VALUE (-1000)  // 2006电机在进行初始化时的电流设置值
#define INIT_2006_MIN_VEL 1          // 2006电机初始化完成的速度阈值

// 气泵相关
// #define PUMP_ON_PWM 30000
// #define PUMP_OFF_PWM 0
// #define PUMP_PWM_CHANNEL 1

#define JointMotorInit(index)                                                                    \
    MotorInit(                                                                                   \
        &MECHANICAL_ARM.joint_motor[index], JOINT_MOTOR_##index##_ID, JOINT_MOTOR_##index##_CAN, \
        JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION,                             \
        JOINT_MOTOR_##index##_REDUCATION_RATIO, JOINT_MOTOR_##index##_MODE)

#define JointPidInit(index)                                                                \
    {                                                                                      \
        float j##index##_pid_angle[3] = {                                                  \
            KP_JOINT_##index##_ANGLE, KI_JOINT_##index##_ANGLE, KD_JOINT_##index##_ANGLE}; \
        float j##index##_pid_velocity[3] = {                                               \
            KP_JOINT_##index##_VELOCITY, KI_JOINT_##index##_VELOCITY,                      \
            KD_JOINT_##index##_VELOCITY};                                                  \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[0], PID_POSITION, j##index##_pid_angle,         \
            MAX_OUT_JOINT_##index##_ANGLE, MAX_IOUT_JOINT_##index##_ANGLE);                \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[1], PID_POSITION, j##index##_pid_velocity,      \
            MAX_OUT_JOINT_##index##_VELOCITY, MAX_IOUT_JOINT_##index##_VELOCITY);          \
    }

#define JointLowPassFilterInit(index) \
    LowPassFilterInit(&MECHANICAL_ARM.lpf.j[index], J##index##_LPF_ALPHA)

/*------------------------------ Variable Definition ------------------------------*/

MechanicalArm_s MECHANICAL_ARM;
#define MA MECHANICAL_ARM

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/******************************************************************/

void MechanicalArmPublish(void) {}

/******************************************************************/
/* Get Mode                                                       */
/******************************************************************/

MechanicalArmMode_e GetMechanicalArmMode(void)
{
    return MECHANICAL_ARM.mode;
}

float GetMechanicalArmJ0Angle(void)
{
    return MECHANICAL_ARM.fdb.joint[J0].angle;
}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      MechanicalArmInit                          */
/* auxiliary function: None                                       */
/******************************************************************/

void MechanicalArmInit(void)
{
    MECHANICAL_ARM.rc = get_remote_control_point();

    MECHANICAL_ARM.init_completed = false;
    MECHANICAL_ARM.reach_time = 0;
    // #Motor init ---------------------
    JointMotorInit(0);
    JointMotorInit(1);
    JointMotorInit(2);
    JointMotorInit(3);
    JointMotorInit(4);
    JointMotorInit(5);
    JointMotorInit(6);
    // #PID init ---------------------
    JointPidInit(0);
    JointPidInit(1);
    JointPidInit(2);
    JointPidInit(3);
    JointPidInit(4);
    JointPidInit(5);
    JointPidInit(6);
    // #LPF init ---------------------
    JointLowPassFilterInit(0);
    JointLowPassFilterInit(1);
    JointLowPassFilterInit(2);
    JointLowPassFilterInit(3);
    JointLowPassFilterInit(4);
    JointLowPassFilterInit(5);
    JointLowPassFilterInit(6);
    // #limit init ---------------------
    MECHANICAL_ARM.limit.max.pos[J0] = MAX_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.max.pos[J1] = MAX_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.max.pos[J2] = MAX_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.max.pos[J3] = MAX_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.max.pos[J4] = MAX_JOINT_4_POSITION;
    MECHANICAL_ARM.limit.max.pos[J5] = MAX_JOINT_5_POSITION;
    MECHANICAL_ARM.limit.max.pos[J6] = MAX_JOINT_6_POSITION;

    MECHANICAL_ARM.limit.min.pos[J0] = MIN_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.min.pos[J1] = MIN_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.min.pos[J2] = MIN_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.min.pos[J3] = MIN_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.min.pos[J4] = MIN_JOINT_4_POSITION;
    MECHANICAL_ARM.limit.min.pos[J5] = MIN_JOINT_5_POSITION;
    MECHANICAL_ARM.limit.min.pos[J6] = MIN_JOINT_6_POSITION;
    // #memset ---------------------
    memset(&MECHANICAL_ARM.fdb, 0, sizeof(MECHANICAL_ARM.fdb));
    memset(&MECHANICAL_ARM.ref, 0, sizeof(MECHANICAL_ARM.ref));
    // #ref init ---------------------
    MECHANICAL_ARM.ref.joint[J0].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.limit.max.pos[J1];
    MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.limit.min.pos[J2];
    MECHANICAL_ARM.ref.joint[J3].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J4].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J5].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J6].angle = 0.0f;

    // #Initial value setting ---------------------
    MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    MECHANICAL_ARM.error_code = 0;

    MECHANICAL_ARM.cmd.pump_on = false;

    MECHANICAL_ARM.transform.dpos[J0] = J0_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J1] = J1_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J2] = J2_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J3] = J3_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J4] = J4_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J5] = J5_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J6] = J6_ANGLE_TRANSFORM;

    MECHANICAL_ARM.transform.duration[J0] = 2; // TODO: 这里ai没有给出修改建议，但是我觉得应该修改，毕竟电机换了
    MECHANICAL_ARM.transform.duration[J1] = 4;
    MECHANICAL_ARM.transform.duration[J2] = 4;
    MECHANICAL_ARM.transform.duration[J3] = 2;
    MECHANICAL_ARM.transform.duration[J4] = 1;
    MECHANICAL_ARM.transform.duration[J5] = 1;
    MECHANICAL_ARM.transform.duration[J6] = 1;
}

/******************************************************************/
/* HandleException                                                */
/******************************************************************/
/**
 * @brief  机械臂异常处理函数
 * 
 * @description
 *          该函数用于处理机械臂的异常情况，主要包括:
 *          1. 初始化阶段检测：监测 J4、J5 关节电机速度，判断是否达到机械限位
 *          2. 虚拟关节限位设置：根据实际位置计算并设置虚拟 J4 关节的运动范围限制
 *          3. 状态同步：初始化完成后将参考位置同步到当前反馈位置
 *          4. 力矩异常检测：监测 J1、J2 关节的输出力矩，判断是否超出安全范围
 * 
 * @note
 *       - 初始化完成条件：J4 和 J5 电机反馈速度均小于 INIT_2006_MIN_VEL 阈值
 *       - 限位判定条件：低速状态持续超过 200ms
 *       - 虚拟 J4 关节位置计算：vj4 = (j4 - j5) / 2
 *       - 力矩异常阈值：10 (单位取决于电机力矩反馈标定)
 * 
 * @par 修改历史:
 *          - 初始版本：实现机械臂初始化检测和异常监控功能
 * 
 */
void MechanicalArmHandleException(void)
{
    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_INIT) {
        // 初始化时如果电机反馈的速度小于阈值，则认为电机初始化完成
        if (fabsf(MECHANICAL_ARM.joint_motor[J4].fdb.vel) < INIT_2006_MIN_VEL &&
            fabsf(MECHANICAL_ARM.joint_motor[J5].fdb.vel) < INIT_2006_MIN_VEL) {
            MECHANICAL_ARM.reach_time += MECHANICAL_ARM.duration;
        } else {
            MECHANICAL_ARM.reach_time = 0;
        }

        if (MECHANICAL_ARM.reach_time > 200 && !MECHANICAL_ARM.init_completed) {
            // 停止时间超过200ms则认为达到限位
            MECHANICAL_ARM.init_completed = true;
            float virtual_j4_pos =
                (MECHANICAL_ARM.fdb.joint[J4].angle - MECHANICAL_ARM.fdb.joint[J5].angle) / 2;
            float virtual_j5_pos =
                MECHANICAL_ARM.fdb.joint[J4].angle + MECHANICAL_ARM.fdb.joint[J5].angle;
            // 设置虚拟关节J4关节的位置限制
            MECHANICAL_ARM.limit.max.vj4_pos = virtual_j4_pos - 0.15f + M_PI;
            MECHANICAL_ARM.limit.min.vj4_pos = virtual_j4_pos + 0.15f;

            // 新增：同步ref到当前fdb值
            MECHANICAL_ARM.ref.joint[J0].angle = MECHANICAL_ARM.fdb.joint[J0].angle;
            MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.fdb.joint[J1].angle;
            MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.fdb.joint[J2].angle;
            MECHANICAL_ARM.ref.joint[J3].angle = MECHANICAL_ARM.fdb.joint[J3].angle;
            MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.fdb.joint[J4].angle;
            MECHANICAL_ARM.ref.joint[J5].angle = MECHANICAL_ARM.fdb.joint[J5].angle;
            MECHANICAL_ARM.ref.joint[J6].angle = MECHANICAL_ARM.fdb.joint[J6].angle;
        }
        // 新增：初始化过程中如果检测到高速转动超过一定时间，强制进入安全模式
        static uint32_t high_speed_duration = 0;
        if (fabsf(MECHANICAL_ARM.joint_motor[J4].fdb.vel) > 5.0f ||
            fabsf(MECHANICAL_ARM.joint_motor[J5].fdb.vel) > 5.0f) {
            high_speed_duration += MECHANICAL_ARM.duration;
            if (high_speed_duration > 500) {  // 高速转动超过 500ms
                MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
                high_speed_duration = 0;
            }
        } else {
            high_speed_duration = 0;
        }
    }    

    if (fabsf(MECHANICAL_ARM.joint_motor[J1].fdb.tor) > 10 ||
        fabsf(MECHANICAL_ARM.joint_motor[J2].fdb.tor) > 10) {
        MECHANICAL_ARM.error_code |= JOINT_TORQUE_MORE_OFFSET;
    }
}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSetMode                      */
/* auxiliary function:  None                                      */
/******************************************************************/

void MechanicalArmSetMode(void)
{
    // if (toe_is_error(DBUS_TOE)) {  // 安全，保命！！！！！！
    //     MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    //     return;
    // }

    if (MECHANICAL_ARM.error_code) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
        return;
    }

    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_CALIBRATE) {
        return;
    }

    if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        // MECHANICAL_ARM.mode = MECHANICAL_ARM_FOLLOW;
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_down(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        // 下档时进入HOLD模式，保持机械臂位姿，允许底盘移动
        MECHANICAL_ARM.mode = MECHANICAL_ARM_HOLD;
    }

    // 非安全模式下，初始化未完成时，进入初始化模式
    if ((MECHANICAL_ARM.mode != MECHANICAL_ARM_SAFE) && (!MECHANICAL_ARM.init_completed)) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_INIT;
    }
}

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmObserver                     */
/* auxiliary function:  UpdateMotorStatus                         */
/*                      JointStateObserve                         */
/******************************************************************/

static void UpdateMotorStatus(void);
static void JointStateObserve(void);

void MechanicalArmObserver(void)
{
    MA.duration = xTaskGetTickCount() - MA.last_time;
    MA.last_time = xTaskGetTickCount();

    /*获取自定义控制器状态，需要重新写一个函数*/
    // MA.custom_controller_ready = GetRefereeState();
    // MA.custom_controller_ready = !GetRefereeOffline();
    // MA.custom_controller_ready = GetCustomControllerDataPoint()->ready;

    UpdateMotorStatus();
    JointStateObserve();
}

/**
 * @brief  更新电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    uint8_t i;
    for (i = 0; i < 7; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
    }

    // 诊断代码：检查CAN2上哪些ID有数据更新
    static uint32_t debug_cnt = 0;
    if (++debug_cnt % 1000 == 0) {  // 每秒打印一次
        // 检查CAN2_DJI_MEASURE数组中哪些位置有数据
        extern const DjiMotorMeasure_t * GetDjiMotorMeasurePoint(uint8_t can, uint8_t i);

        for (i = 0; i < 11; i++) {
            const DjiMotorMeasure_t * measure = GetDjiMotorMeasurePoint(2, i);
            if (measure->ecd != 0 || measure->speed_rpm != 0) {
                // 这个索引位置有数据，对应硬件ID = i+1
                // 例如：索引2 → ID 3, 索引4 → ID 5, 索引6 → ID 7
            }
        }
    }
}

/**
 * @brief  关节状态观测
 *         处理 J0-J2 关节：直接计算角度、速度、扭矩反馈值
 *         处理 J3-J6 关节：增加多圈计数算法，解决编码器过零问题
 *         速度滤波：对 J3-J6 的速度进行低通滤波处理
 * @param  none
 */
static void JointStateObserve(void)
{
    static float last_angle[7], angle_fdb[7] = {0, 0, 0, 0, 0, 0, 0};
    float vel, dpos;
#define dangle MECHANICAL_ARM.transform.dpos

    /*-----处理J0 J1 J2关节的反馈信息（J4 J5作为差速机构要特殊处理）*/
    angle_fdb[J0] = theta_transform(
        MA.joint_motor[J0].fdb.pos, dangle[J0], MA.joint_motor[J0].direction,
        MA.transform.duration[J0]);

    dpos = angle_fdb[J0] - last_angle[J0];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J0].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J0] = angle_fdb[J0];
    MA.fdb.joint[J0].angle =
        (angle_fdb[J0] + M_PI * 2 * MA.fdb.joint[J0].round) / MA.joint_motor[J0].reduction_ratio;


    uint8_t i;
    for (i = 1; i < 3; i++) {
        MA.fdb.joint[i].angle = theta_transform(
                                    MA.joint_motor[i].fdb.pos, dangle[i],
                                    MA.joint_motor[i].direction, MA.transform.duration[i]) /
                                MA.joint_motor[i].reduction_ratio;
        MA.fdb.joint[i].velocity = MA.joint_motor[i].fdb.vel / MA.joint_motor[i].reduction_ratio *
                                   MA.joint_motor[i].direction;
        MA.fdb.joint[i].torque = MA.joint_motor[i].fdb.tor * MA.joint_motor[i].reduction_ratio *
                                 MA.joint_motor[i].direction;
    }

    /*-----处理 J3 关节的反馈信息 (涉及到多圈计数问题)-----*/
    angle_fdb[J3] = theta_transform(
                        MA.joint_motor[J3].fdb.pos, dangle[J3], MA.joint_motor[J3].direction,
                        MA.transform.duration[J3]) /
                    MA.joint_motor[J3].reduction_ratio;

    // 多圈处理
    dpos = angle_fdb[J3] - last_angle[J3];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J3].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J3] = angle_fdb[J3];
    MA.fdb.joint[J3].angle = angle_fdb[J3] + M_PI * 2 * MA.fdb.joint[J3].round;

    vel = MA.joint_motor[J3].fdb.vel / MA.joint_motor[J3].reduction_ratio *
          MA.joint_motor[J3].direction;

    MA.fdb.joint[J3].velocity = LowPassFilterCalc(&MA.lpf.j[J3], vel);

    /*-----处理 J4 J5 关节的反馈信息（差速机构特殊处理）-----*/
    // J4 电机原始反馈处理
    // angle_fdb[J4] = theta_transform(
    //     MA.joint_motor[J4].fdb.pos, dangle[J4], MA.joint_motor[J4].direction,
    //     MA.transform.duration[J4]);
    
    // MA.fdb.joint[J4].angle = angle_fdb[J4] / MA.joint_motor[J4].reduction_ratio;
    
    // vel = MA.joint_motor[J4].fdb.vel / MA.joint_motor[J4].reduction_ratio *
    //       MA.joint_motor[J4].direction;
    // MA.fdb.joint[J4].velocity = LowPassFilterCalc(&MA.lpf.j[J4], vel);

    // MA.fdb.joint[J4].torque = MA.joint_motor[J4].fdb.tor * MA.joint_motor[J4].reduction_ratio *
    //                           MA.joint_motor[J4].direction;

    angle_fdb[J4] = theta_transform(
        MA.joint_motor[J4].fdb.pos, dangle[J4], MA.joint_motor[J4].direction,
        MA.transform.duration[J4]);

    dpos = angle_fdb[J4] - last_angle[J4];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J4].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J4] = angle_fdb[J4];
    MA.fdb.joint[J4].angle =
        (angle_fdb[J4] + M_PI * 2 * MA.fdb.joint[J4].round) / MA.joint_motor[J4].reduction_ratio;

    // J5 电机原始反馈处理
    angle_fdb[J5] = theta_transform(
        MA.joint_motor[J5].fdb.pos, dangle[J5], MA.joint_motor[J5].direction,
        MA.transform.duration[J5]);

    dpos = angle_fdb[J5] - last_angle[J5];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J5].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J5] = angle_fdb[J5];
    MA.fdb.joint[J5].angle =
        (angle_fdb[J5] + M_PI * 2 * MA.fdb.joint[J5].round) / MA.joint_motor[J5].reduction_ratio;

    // angle_fdb[J5] = theta_transform(
    //     MA.joint_motor[J5].fdb.pos, dangle[J5], MA.joint_motor[J5].direction,
    //     MA.transform.duration[J5]);
    
    // MA.fdb.joint[J5].angle = angle_fdb[J5] / MA.joint_motor[J5].reduction_ratio;
    
    // vel = MA.joint_motor[J5].fdb.vel / MA.joint_motor[J5].reduction_ratio *
    //       MA.joint_motor[J5].direction;
    // MA.fdb.joint[J5].velocity = LowPassFilterCalc(&MA.lpf.j[J5], vel);
    
    // MA.fdb.joint[J5].torque = MA.joint_motor[J5].fdb.tor * MA.joint_motor[J5].reduction_ratio *
    //                           MA.joint_motor[J5].direction;

    /*-----处理 J6 关节的反馈信息-----*/
    angle_fdb[J6] = theta_transform(
        MA.joint_motor[J6].fdb.pos, dangle[J6], MA.joint_motor[J6].direction,
        MA.transform.duration[J6]);

    dpos = angle_fdb[J6] - last_angle[J6];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J6].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J6] = angle_fdb[J6];
    MA.fdb.joint[J6].angle =
        (angle_fdb[J6] + M_PI * 2 * MA.fdb.joint[J6].round) / MA.joint_motor[J6].reduction_ratio;

#undef dangle
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmReference                    */
/******************************************************************/

void MechanicalArmReference(void)
{
    uint8_t i;
    switch (MECHANICAL_ARM.mode) 
    {
        // case MECHANICAL_ARM_CUSTOM: {
            // MECHANICAL_ARM.ref.joint[J0].angle = MECHANICAL_ARM.rc->rc.ch[4] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.rc->rc.ch[0] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.rc->rc.ch[1] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J3].angle = 0;
            // MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.rc->rc.ch[2] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J5].angle = MECHANICAL_ARM.rc->rc.ch[3] * RC_TO_ONE * M_PI;
        // } break;
        // case MECHANICAL_ARM_DEBUG: {

        //         // j3
        //         MA.ref.joint[J3].angle += GetDt7RcCh(DT7_CH_LH) * 0.002f;
        //         MA.ref.joint[J3].angle = fp32_constrain(
        //             MA.ref.joint[J3].angle, MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

        //         MA.ref.joint[J4].angle = 0;
        //         MA.ref.joint[J5].angle = 0;
        //     } else if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        //         // j4
        //         MA.ref.joint[J4].angle += GetDt7RcCh(DT7_CH_RV) * 0.002f;
        //         // MA.ref.joint[J4].angle = GenerateSinWave(1, 0, 3);
        //         // MA.ref.joint[J4].angle =
        //         //     fp32_constrain(MA.ref.joint[J4].angle, MA.limit.min.pos[J4], MA.limit.max.pos[J4]);

        //         // j5
        //         MA.ref.joint[J5].angle += GetDt7RcCh(DT7_CH_LV) * 0.002f;
        //         // MA.ref.joint[J5].angle = GenerateSinWave(1, 0, 3);
        //         // MA.ref.joint[J5].angle =
        //         //     fp32_constrain(MA.ref.joint[J5].angle, MA.limit.min.pos[J5], MA.limit.max.pos[J5]);
        //     }
        //     float virtual_j4_pos = (MA.ref.joint[J4].angle - MA.ref.joint[J5].angle) / 2;
        //     float delta = 0;
        //     if (virtual_j4_pos > MA.limit.max.vj4_pos) {
        //         delta = virtual_j4_pos - MA.limit.max.vj4_pos;
        //     } else if (virtual_j4_pos < MA.limit.min.vj4_pos) {
        //         delta = virtual_j4_pos - MA.limit.min.vj4_pos;
        //     }
        //     MA.ref.joint[J4].angle -= delta;
        //     MA.ref.joint[J5].angle += delta;
        // } break;

        /**
         * @brief 新的夹爪控制映射（只控制J0，J1，J2，J3，J4, J6）
         * 
         * @note 左拨轮 -> J6
         *       左水平摇杆 -> J3
         *       左竖直摇杆 -> J1
         *       右水平摇杆 -> J4
         *       右竖直摇杆 -> J2
         *       J0,J5可以考虑锁住
         */
        case MECHANICAL_ARM_DEBUG: {

            // 【修复】检测模式切换，同步 ref 到 fdb 防止初始震动
            static MechanicalArmMode_e last_mode = MECHANICAL_ARM_SAFE;
        if (last_mode != MECHANICAL_ARM_DEBUG) 
        {
                MA.ref.joint[J0].angle = MA.fdb.joint[J0].angle;
                MA.ref.joint[J1].angle = MA.fdb.joint[J1].angle;
                MA.ref.joint[J2].angle = MA.fdb.joint[J2].angle;
                MA.ref.joint[J3].angle = MA.fdb.joint[J3].angle;
                MA.ref.joint[J4].angle = MA.fdb.joint[J4].angle;
                MA.ref.joint[J5].angle = MA.fdb.joint[J5].angle;
                MA.ref.joint[J6].angle = MA.fdb.joint[J6].angle;
        }
            last_mode = MECHANICAL_ARM_DEBUG;

            // 键鼠控制（无需模式切换，直接控制所有自由度）
        if (MA.rc->key.v != 0 || MA.rc->mouse.press_l || MA.rc->mouse.press_r || MA.rc->mouse.x != 0)
        {
                // J0: 鼠标左右移动控制 (机械臂左右旋转)
                MA.ref.joint[J0].angle += MA.rc->mouse.x * 0.0001f;
                MA.ref.joint[J0].angle = fp32_constrain(
                    MA.ref.joint[J0].angle, MA.limit.min.pos[J0], MA.limit.max.pos[J0]);

                // J1: R/F键 （机械臂肩关节旋转）
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_R)
                    MA.ref.joint[J1].angle += 0.002f;
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_F)
                    MA.ref.joint[J1].angle -= 0.002f;
                MA.ref.joint[J1].angle = fp32_constrain(
                    MA.ref.joint[J1].angle, MA.limit.min.pos[J1], MA.limit.max.pos[J1]);

                // J2: G/B键 （机械臂肘关节旋转）
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_G)
                    MA.ref.joint[J2].angle += 0.002f;
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_B)
                    MA.ref.joint[J2].angle -= 0.002f;
                MA.ref.joint[J2].angle = fp32_constrain(
                    MA.ref.joint[J2].angle, MA.limit.min.pos[J2], MA.limit.max.pos[J2]);

                // J3: Q/E键 （机械臂腕关节旋转）
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_Q)
                    MA.ref.joint[J3].angle += 0.002f;
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_E)
                    MA.ref.joint[J3].angle -= 0.002f;
                MA.ref.joint[J3].angle = fp32_constrain(
                    MA.ref.joint[J3].angle, MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

                // J4: Z/X键 （机械臂指关节旋转）
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_Z)
                    MA.ref.joint[J4].angle += 0.002f;
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_X)
                    MA.ref.joint[J4].angle -= 0.002f;
                MA.ref.joint[J4].angle = fp32_constrain(
                    MA.ref.joint[J4].angle, MA.limit.min.pos[J4], MA.limit.max.pos[J4]);

                // J5: C/V键 （机械臂指关节
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_C)
                    MA.ref.joint[J5].angle += 0.002f;
                if (MA.rc->key.v & KEY_PRESSED_OFFSET_V)
                    MA.ref.joint[J5].angle -= 0.002f;
                MA.ref.joint[J5].angle = fp32_constrain(
                    MA.ref.joint[J5].angle, MA.limit.min.pos[J5], MA.limit.max.pos[J5]);

                // J6: 鼠标左右键
                if (MA.rc->mouse.press_l)
                    MA.ref.joint[J6].angle += 0.002f;
                if (MA.rc->mouse.press_r)
                    MA.ref.joint[J6].angle -= 0.002f;
                MA.ref.joint[J6].angle = fp32_constrain(
                    MA.ref.joint[J6].angle, MA.limit.min.pos[J6], MA.limit.max.pos[J6]);
        }
            // 遥控器控制（保留模式切换逻辑）
        else {
        if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL]))
        {
            // j0 右水平
            MA.ref.joint[J0].angle -= GetDt7RcCh(DT7_CH_LH) * 0.002f;
            MA.ref.joint[J0].angle = fp32_constrain(
                MA.ref.joint[J0].angle, MA.limit.min.pos[J0], MA.limit.max.pos[J0]);

            // j1 左竖直摇杆
            MA.ref.joint[J1].angle += GetDt7RcCh(DT7_CH_RV) * 0.002f;
            MA.ref.joint[J1].angle = fp32_constrain(
                MA.ref.joint[J1].angle, MA.limit.min.pos[J1], MA.limit.max.pos[J1]);

            // j2 右竖直摇杆
            MA.ref.joint[J2].angle += GetDt7RcCh(DT7_CH_LV) * 0.002f;
            MA.ref.joint[J2].angle = fp32_constrain(
                    MA.ref.joint[J2].angle, MA.limit.min.pos[J2], MA.limit.max.pos[J2]);
            
            // j3
            // 左水平摇杆 -> J3 (roll)
            MA.ref.joint[J3].angle += GetDt7RcCh(DT7_CH_RH) * 0.002f;
            MA.ref.joint[J3].angle = fp32_constrain(
                MA.ref.joint[J3].angle, MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

            // 左滚轮 -> J6 (夹爪开合)
            MA.ref.joint[J6].angle += GetDt7RcCh(DT7_CH_ROLLER) * 0.002f;
            MA.ref.joint[J6].angle = fp32_constrain(
                MA.ref.joint[J6].angle, MA.limit.min.pos[J6], MA.limit.max.pos[J6]);

            // J0, J1, J2, J3 保持当前位置不变
        }
        if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) 
        {
            // 右摇杆水平 -> J4 (yaw)
            MA.ref.joint[J4].angle += GetDt7RcCh(DT7_CH_LH) * 0.002f;
            MA.ref.joint[J4].angle = fp32_constrain(
                MA.ref.joint[J4].angle, MA.limit.min.pos[J4], MA.limit.max.pos[J4]);

            // J5 左竖直摇杆
            MA.ref.joint[J5].angle += GetDt7RcCh(DT7_CH_RV) * 0.002f;
            MA.ref.joint[J5].angle = fp32_constrain(
                MA.ref.joint[J5].angle, MA.limit.min.pos[J5], MA.limit.max.pos[J5]);

            // 左滚轮 -> J6 (夹爪开合)
            MA.ref.joint[J6].angle += GetDt7RcCh(DT7_CH_ROLLER) * 0.002f;
            MA.ref.joint[J6].angle = fp32_constrain(
                MA.ref.joint[J6].angle, MA.limit.min.pos[J6], MA.limit.max.pos[J6]);
        }
        break;
        // 下面是切换为自定义控制器的代码
        // case MECHANICAL_ARM_FOLLOW: {
        //     if (MA.custom_controller_ready) {
        //         float pos[7] = {0};
        //         pos[J0] = GetCustomControllerPos(J0);
        //         pos[J1] = GetCustomControllerPos(J1);
        //         pos[J2] = GetCustomControllerPos(J2);
        //         pos[J3] = GetCustomControllerPos(J3);
        //         pos[J4] = GetCustomControllerPos(J4);
        //         pos[J5] = GetCustomControllerPos(J5);
        //         pos[J6] = GetCustomControllerPos(J6);

        //         if (fabsf(pos[J0] - MA.ref.joint[J0].angle) > 1.5f ||
        //             fabsf(pos[J1] - MA.ref.joint[J1].angle) > 1.5f ||
        //             fabsf(pos[J2] - MA.ref.joint[J2].angle) > 1.5f) {  // 位置突变
        //             MECHANICAL_ARM.error_code |= CUSTOM_CONTROLLER_DATA_ERROR_OFFSET;
        //             return;
        //         }

        //         MA.ref.joint[J0].angle =
        //             fp32_constrain(pos[J0], MA.limit.min.pos[J0], MA.limit.max.pos[J0]);
        //         MA.ref.joint[J1].angle =
        //             fp32_constrain(pos[J1], MA.limit.min.pos[J1], MA.limit.max.pos[J1]);
        //         MA.ref.joint[J2].angle =
        //             fp32_constrain(pos[J2], MA.limit.min.pos[J2], MA.limit.max.pos[J2]);
        //         MA.ref.joint[J3].angle =
        //             fp32_constrain(pos[J3], MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

        //         // 自定义控制器传过来的虚拟J4 J5位置以0为中心
        //         float vj4_pos = (pos[J4] - pos[J5]) / 2;
        //         float vj5_pos = (pos[J4] + pos[J5]) / 2;

        //         // 机械臂J4关节以((MA.limit.max.vj4_pos + MA.limit.min.vj4_pos) / 2)为中心
        //         // 机械臂J5关节以((MA.limit.max.vj5_pos + MA.limit.min.vj5_pos) / 2)为中心
        //         float vj4_pos_mid = (MA.limit.max.vj4_pos + MA.limit.min.vj4_pos) / 2;
        //         float vj5_pos_mid = (MA.limit.max.vj5_pos + MA.limit.min.vj5_pos) / 2;

        //         vj4_pos += vj4_pos_mid;
        //         vj5_pos += vj5_pos_mid;

        //         vj4_pos = fp32_constrain(vj4_pos, MA.limit.min.vj4_pos, MA.limit.max.vj4_pos);
        //         vj5_pos = fp32_constrain(vj5_pos, MA.limit.min.vj5_pos, MA.limit.max.vj5_pos);

        //         MA.ref.joint[J4].angle = vj4_pos + vj5_pos;
        //         MA.ref.joint[J5].angle = -(vj4_pos - vj5_pos);
        //     }
        // } break;
        case MECHANICAL_ARM_HOLD: {
            // 保持模式：不更新ref，保持当前目标位置
            break;
        }
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            for (i = 0; i < 7; i++) {
                MECHANICAL_ARM.ref.joint[i].velocity = 0;
                    }
                }
            }
        }
    }
}

/******************************************************************/
/* Console              用于控制机械臂的函数封装                    */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmConsole                      */
/******************************************************************/
fp32 PID_calc_value = 0;
void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_CUSTOM: {  // 自定义模式
            // 优先处理dm电机部分
            // 位置
            // MECHANICAL_ARM.joint_motor[J0].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J0].angle, -J0_ANGLE_TRANSFORM, 1, 1);
            MECHANICAL_ARM.joint_motor[J1].set.pos =
                theta_transform(MECHANICAL_ARM.ref.joint[J1].angle, -J1_ANGLE_TRANSFORM, 1, 1);
            MECHANICAL_ARM.joint_motor[J2].set.pos =
                theta_transform(MECHANICAL_ARM.ref.joint[J2].angle, -J2_ANGLE_TRANSFORM, 1, 1);
            // 速度
            MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J3].set.vel = 0;

            // 然后再处理dji电机部分，涉及到pid计算
            // J0
            PID_calc(
                &MECHANICAL_ARM.pid.j0[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J0].angle,
                MECHANICAL_ARM.ref.joint[J0].angle);
            MECHANICAL_ARM.joint_motor[J0].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j0[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J0].velocity,
                MECHANICAL_ARM.pid.j0[ANGLE_PID].out);
            // J3
            PID_calc(
                &MECHANICAL_ARM.pid.j3[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J3].angle,
                MECHANICAL_ARM.ref.joint[J3].angle);
            MECHANICAL_ARM.joint_motor[J3].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j3[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J3].velocity,
                MECHANICAL_ARM.pid.j3[ANGLE_PID].out);
            // J4
            PID_calc(
                &MECHANICAL_ARM.pid.j4[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J4].angle,
                MECHANICAL_ARM.ref.joint[J4].angle);
            MECHANICAL_ARM.joint_motor[J4].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j4[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J4].velocity,
                MECHANICAL_ARM.pid.j4[ANGLE_PID].out);
            // J5
            PID_calc(
                &MECHANICAL_ARM.pid.j5[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J5].angle,
                MECHANICAL_ARM.ref.joint[J5].angle);
            MECHANICAL_ARM.joint_motor[J5].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j5[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J5].velocity,
                MECHANICAL_ARM.pid.j5[ANGLE_PID].out);
            // J6
            PID_calc(
                &MECHANICAL_ARM.pid.j6[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J6].angle,
                MECHANICAL_ARM.ref.joint[J6].angle);
            MECHANICAL_ARM.joint_motor[J6].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j6[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J6].velocity,
                MECHANICAL_ARM.pid.j6[ANGLE_PID].out);
        } break;
        case MECHANICAL_ARM_FOLLOW: // TODO: 将云台模式换成机械臂跟随
        case MECHANICAL_ARM_DEBUG: {
            // 测试模式：在TEST_FIXED_CURRENT启用时，禁用PID计算
            #ifdef TEST_FIXED_CURRENT
            // 测试模式下禁用所有PID计算，set.value由ArmSendCmdDebug直接设置
            MA.joint_motor[J0].set.value = 0;
            MA.joint_motor[J1].set.vel = 0;
            MA.joint_motor[J1].set.tor = 0;
            MA.joint_motor[J2].set.vel = 0;
            MA.joint_motor[J2].set.tor = 0;
            MA.joint_motor[J3].set.value = 0;
            MA.joint_motor[J4].set.value = 0;
            MA.joint_motor[J5].set.value = 0;
            MA.joint_motor[J6].set.value = 0;
            #else
            /*机械臂J0 J1 J2基本不会出现过圈问题，不考虑过圈时的最优旋转方向问题。*/
            /**
             * @brief 1.首先计算位置环PID, 得到目标速度
             *        2.然后计算速度环PID，得到电机的控制量value
             */

            // J0
            MA.joint_motor[J0].set.vel =
                PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle) *
                MA.joint_motor[J0].direction * MA.joint_motor[J0].reduction_ratio;
            MA.joint_motor[J0].set.value =
                PID_calc(&MA.pid.j0[1], MA.fdb.joint[J0].velocity, MA.joint_motor[J0].set.vel);

            // J1
            // MA.joint_motor[J1].set.vel = 0;
            MA.joint_motor[J1].set.vel =
                PID_calc(&MA.pid.j1[0], MA.fdb.joint[J1].angle, MA.ref.joint[J1].angle) *
                MA.joint_motor[J1].direction * MA.joint_motor[J1].reduction_ratio;
            MA.joint_motor[J1].set.tor = 0;

            // J2
            // MA.joint_motor[J2].set.vel = 0;
            MA.joint_motor[J2].set.vel =
                PID_calc(&MA.pid.j2[0], MA.fdb.joint[J2].angle, MA.ref.joint[J2].angle) *
                MA.joint_motor[J2].direction * MA.joint_motor[J2].reduction_ratio;
            MA.joint_motor[J2].set.tor = 0;

            /*机械臂J3 J4 J5需要考虑过圈的处理（在Observer时已经记录圈数获得多圈反馈了）*/
            // J3 - 临时禁用PID测试
            // MA.joint_motor[J3].set.vel = 0;
            // MA.joint_motor[J3].set.value = 0;
            MA.joint_motor[J3].set.vel =
                PID_calc(&MA.pid.j3[0], MA.fdb.joint[J3].angle, MA.ref.joint[J3].angle) *
                MA.joint_motor[J3].direction * MA.joint_motor[J3].reduction_ratio;
            MA.joint_motor[J3].set.value =
                PID_calc(&MA.pid.j3[1], MA.fdb.joint[J3].velocity, MA.joint_motor[J3].set.vel);

            // J4
            MA.joint_motor[J4].set.vel =
                PID_calc(&MA.pid.j4[0], MA.fdb.joint[J4].angle, MA.ref.joint[J4].angle) *
                MA.joint_motor[J4].direction * MA.joint_motor[J4].reduction_ratio;
            //  MA.joint_motor[J4].set.vel = 0;
            MA.joint_motor[J4].set.value =
                PID_calc(&MA.pid.j4[1], MA.fdb.joint[J4].velocity, MA.joint_motor[J4].set.vel);
            // MA.joint_motor[J4].set.value = GenerateSinWave(1000, 0, 3);  // 测试代码已注释
            // MA.joint_motor[J4].set.value = 0;

            PID_calc_value = PID_calc(&MA.pid.j4[1], MA.fdb.joint[J4].velocity, MA.joint_motor[J4].set.vel);

            // J5
            MA.joint_motor[J5].set.vel =
                PID_calc(&MA.pid.j5[0], MA.fdb.joint[J5].angle, MA.ref.joint[J5].angle) *
                MA.joint_motor[J5].direction * MA.joint_motor[J5].reduction_ratio;
            MA.joint_motor[J5].set.value =
                PID_calc(&MA.pid.j5[1], MA.fdb.joint[J5].velocity, MA.joint_motor[J5].set.vel);
            // MA.joint_motor[J5].set.value = GenerateSinWave(2000, 0, 3);

            // J6
            MA.joint_motor[J6].set.vel =
                PID_calc(&MA.pid.j6[0], MA.fdb.joint[J6].angle, MA.ref.joint[J6].angle) *
                MA.joint_motor[J6].direction * MA.joint_motor[J6].reduction_ratio;
            MA.joint_motor[J6].set.value =
                PID_calc(&MA.pid.j6[1], MA.fdb.joint[J6].velocity, MA.joint_motor[J6].set.vel);

            #endif
            // 气泵
            // if (switch_is_up(GetDt7RcSw(PUMP_CHANNEL))) {
            //     MA.cmd.pump_on = true;
            // } else {
            //     MA.cmd.pump_on = false;
            // }
            // #endif  // TEST_FIXED_CURRENT (已注释,测试模式改在ArmSendCmdDebug中处理)
        } break;
        case MECHANICAL_ARM_HOLD: {
            // 保持模式：继续执行PID控制以保持当前位姿
            MA.joint_motor[J0].set.vel =
                PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle) *
                MA.joint_motor[J0].direction * MA.joint_motor[J0].reduction_ratio;
            MA.joint_motor[J0].set.value =
                PID_calc(&MA.pid.j0[1], MA.fdb.joint[J0].velocity, MA.joint_motor[J0].set.vel);

            MA.joint_motor[J1].set.vel =
                PID_calc(&MA.pid.j1[0], MA.fdb.joint[J1].angle, MA.ref.joint[J1].angle) *
                MA.joint_motor[J1].direction * MA.joint_motor[J1].reduction_ratio;
            MA.joint_motor[J1].set.tor = 0;

            MA.joint_motor[J2].set.vel =
                PID_calc(&MA.pid.j2[0], MA.fdb.joint[J2].angle, MA.ref.joint[J2].angle) *
                MA.joint_motor[J2].direction * MA.joint_motor[J2].reduction_ratio;
            MA.joint_motor[J2].set.tor = 0;

            MA.joint_motor[J3].set.vel =
                PID_calc(&MA.pid.j3[0], MA.fdb.joint[J3].angle, MA.ref.joint[J3].angle) *
                MA.joint_motor[J3].direction * MA.joint_motor[J3].reduction_ratio;
            MA.joint_motor[J3].set.value =
                PID_calc(&MA.pid.j3[1], MA.fdb.joint[J3].velocity, MA.joint_motor[J3].set.vel);

            MA.joint_motor[J4].set.vel =
                PID_calc(&MA.pid.j4[0], MA.fdb.joint[J4].angle, MA.ref.joint[J4].angle) *
                MA.joint_motor[J4].direction * MA.joint_motor[J4].reduction_ratio;
            MA.joint_motor[J4].set.value =
                PID_calc(&MA.pid.j4[1], MA.fdb.joint[J4].velocity, MA.joint_motor[J4].set.vel);

            MA.joint_motor[J5].set.vel =
                PID_calc(&MA.pid.j5[0], MA.fdb.joint[J5].angle, MA.ref.joint[J5].angle) *
                MA.joint_motor[J5].direction * MA.joint_motor[J5].reduction_ratio;
            MA.joint_motor[J5].set.value =
                PID_calc(&MA.pid.j5[1], MA.fdb.joint[J5].velocity, MA.joint_motor[J5].set.vel);

            MA.joint_motor[J6].set.vel =
                PID_calc(&MA.pid.j6[0], MA.fdb.joint[J6].angle, MA.ref.joint[J6].angle) *
                MA.joint_motor[J6].direction * MA.joint_motor[J6].reduction_ratio;
            MA.joint_motor[J6].set.value =
                PID_calc(&MA.pid.j6[1], MA.fdb.joint[J6].velocity, MA.joint_motor[J6].set.vel);
        } break;
        case MECHANICAL_ARM_INIT: {  // 设置初始化参数
            // MA.joint_motor[J0].set.value = 0;
            MA.joint_motor[J1].set.value = 0;
            MA.joint_motor[J2].set.value = 0;
            MA.joint_motor[J3].set.value = 0;
            MA.joint_motor[J4].set.value = 0;
            MA.joint_motor[J5].set.value = 0;  // J5初始化时不动
            MA.joint_motor[J6].set.value = 0;  // J6初始化时不动
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            // MECHANICAL_ARM.joint_motor[J0].set.value = 0;
            MECHANICAL_ARM.joint_motor[J1].set.value = 0;
            MECHANICAL_ARM.joint_motor[J2].set.value = 0;
            MECHANICAL_ARM.joint_motor[J3].set.value = 0;
            MECHANICAL_ARM.joint_motor[J4].set.value = 0;
            MECHANICAL_ARM.joint_motor[J5].set.value = 0;
            MECHANICAL_ARM.joint_motor[J6].set.value = 0;
        }
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSendCmd                      */
/* auxiliary function:  ArmSendCmdSafe                            */
/*                      ArmSendCmdFollow                          */
/******************************************************************/

void ArmSendCmdSafe(void);
void ArmSendCmdDebug(void);
void ArmSendCmdInit(void);

void MechanicalArmSendCmd(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 1; i < 3; i++) {
        if (cnt % 2 == 0) {
            delay_us(DM_DELAY);
        }
        // if (MECHANICAL_ARM.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
            // DmEnable(&MECHANICAL_ARM.joint_motor[i]);
            // cnt++;
        // }
    }

    delay_us(DM_DELAY);

    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_DEBUG:
        case MECHANICAL_ARM_HOLD: {
            ArmSendCmdDebug();
        } break;
        case MECHANICAL_ARM_INIT: {
            ArmSendCmdInit();
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_CUSTOM:
        case MECHANICAL_ARM_SAFE:
        default: {
            ArmSendCmdSafe();
        }
    }
}

void ArmSendCmdSafe(void)
{
    // 电机控制
    // DmMitStop(&MECHANICAL_ARM.joint_motor[J0]);
    // DM电机控制
    // delay_us(DM_DELAY);
    // DmMitStop(&MECHANICAL_ARM.joint_motor[J1]);
    // DmMitStop(&MECHANICAL_ARM.joint_motor[J2]);
    // delay_us(DM_DELAY);

    // DJI电机控制 - 修复：J3/J4/J5/J6都在CAN2上
    CanCmdDjiMotor(ARM_DM_CAN, 0x1FF, 0, 0, 0, 0);  // 修改为ARM_DM_CAN (CAN2)

    // 气泵控制
    // PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
}


// 完全替换整个函数
/**
 * @brief 发送控制量
 * 
 */
void ArmSendCmdDebug(void)
{
    // DM电机控制 (J1, J2)
    // DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J1], J1_KD_FOLLOW);
    // DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J2], J2_KD_FOLLOW);
    // delay_us(DM_DELAY);

    // 测试模式：给固定电流值验证硬件ID是否正确
    // 打开此段代码，所有电机会按固定电流值转动，方便观察哪个电机对应哪个ID
    // #define TEST_FIXED_CURRENT  // 取消注释以启用测试模式

    #ifdef TEST_FIXED_CURRENT
    // 测试方案：每个参数位置给3000mA电流，观察实际哪个电机在转
    // 注意：增大到3000以确保电机能明显转动
    CanCmdDjiMotor(
        ARM_DM_CAN, 0x1FF,
        0,  // 参数1 → 控制ID 5的电机 → 应该是J4
        0,     // 参数2 → 控制ID 6的电机 → 应该是J5
        0,     // 参数3 → 控制ID 7的电机 → 如果J3动了，说明J3实际ID是7
        0);    // 参数4 → 控制ID 8的电机 → 应该是J6
    #else
    // 正常发送控制指令
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x2FF,
        MA.joint_motor[J0].set.value,
        0, 0, 0);

    CanCmdDjiMotor(
        ARM_DM_CAN, 0x1FF,
        MA.joint_motor[J4].set.value,
        MA.joint_motor[J5].set.value,
        MA.joint_motor[J3].set.value,
        MA.joint_motor[J6].set.value);

    #endif

    // 气泵控制
    // if (MECHANICAL_ARM.cmd.pump_on) {
    //     PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_ON_PWM);
    // } else {
    //     PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
    // }
}

void ArmSendCmdInit(void)
{
    // DM电机控制 (J1, J2)
    // DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J1], J1_KD_FOLLOW);
    // DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J2], J2_KD_FOLLOW);
    // delay_us(DM_DELAY);

    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x2FF,
        MA.joint_motor[J0].set.value,
        0, 0, 0);

    CanCmdDjiMotor(
        ARM_DM_CAN, 0x1FF,
        // 0,
        MA.joint_motor[J4].set.value,
        // 1000, 0, 0);
        MA.joint_motor[J5].set.value,
        MA.joint_motor[J3].set.value,
        MA.joint_motor[J6].set.value);

    // 气泵控制
    // PwmCmdPump(PUMP_PWM_CHANNEL, PUMP_OFF_PWM);
}

#endif

#undef MA
/*------------------------------ End of File ------------------------------*/
