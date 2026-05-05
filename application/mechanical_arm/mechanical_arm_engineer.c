/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_engineer.c/h
  * @brief      机械臂功能。
  * @note
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
#include "PWM_cmd_servo.h"

/*------------------------------ Macro Definition ------------------------------*/
#define DEBUG 1

// clang-format off
#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define CUSTOM_CONTROLLER_DATA_ERROR_OFFSET   ((uint8_t)1 << 1)  // 自定义控制器数据异常偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
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

#define DM_DELAY 500  // (us)dm电机发送延时

#define J0_KP_FOLLOW 0
#define J0_KD_FOLLOW 1.5

#define J1_KP_FOLLOW 0
#define J1_KD_FOLLOW 10

#define J2_KP_FOLLOW 0
#define J2_KD_FOLLOW 10

#define J3_KP_FOLLOW 0
#define J3_KD_FOLLOW 10

#define J4_KP_FOLLOW 0
#define J4_KD_FOLLOW 10

#define INIT_2006_SET_VALUE (-1000)  // 2006电机在进行初始化时的电流设置值
#define INIT_2006_MIN_VEL 1          // 2006电机初始化完成的速度阈值

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

float angle[2];

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
    MECHANICAL_ARM.lookline = get_remote_control_point();

    MECHANICAL_ARM.init_completed = false;
    MECHANICAL_ARM.reach_time = 0;
    // #Motor init ---------------------
    JointMotorInit(0);
    JointMotorInit(1);
    JointMotorInit(2);
    JointMotorInit(3);
    JointMotorInit(4);
    // #PID init ---------------------
    JointPidInit(0);
    JointPidInit(1);
    JointPidInit(2);
    JointPidInit(3);
    JointPidInit(4);
    // #LPF init ---------------------
    JointLowPassFilterInit(0);
    JointLowPassFilterInit(1);
    JointLowPassFilterInit(2);
    JointLowPassFilterInit(3);
    JointLowPassFilterInit(4);
    // #limit init ---------------------
    MECHANICAL_ARM.limit.max.pos[J0] = MAX_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.max.pos[J1] = MAX_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.max.pos[J2] = MAX_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.max.pos[J3] = MAX_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.max.pos[J4] = MAX_JOINT_4_POSITION;

    MECHANICAL_ARM.limit.min.pos[J0] = MIN_JOINT_0_POSITION;
    MECHANICAL_ARM.limit.min.pos[J1] = MIN_JOINT_1_POSITION;
    MECHANICAL_ARM.limit.min.pos[J2] = MIN_JOINT_2_POSITION;
    MECHANICAL_ARM.limit.min.pos[J3] = MIN_JOINT_3_POSITION;
    MECHANICAL_ARM.limit.min.pos[J4] = MIN_JOINT_4_POSITION;
    // #memset ---------------------
    // 将一块内存区域的所有字节设置为指定的值
    memset(&MECHANICAL_ARM.fdb, 0, sizeof(MECHANICAL_ARM.fdb));
    memset(&MECHANICAL_ARM.ref, 0, sizeof(MECHANICAL_ARM.ref));
    // #ref init ---------------------
    MECHANICAL_ARM.ref.joint[J0].angle = 0.0f;
    MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.limit.max.pos[J1];
    MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.limit.min.pos[J2];
    MECHANICAL_ARM.ref.joint[J3].angle = MECHANICAL_ARM.limit.min.pos[J3];
    MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.limit.min.pos[J4];

    // #Initial value setting ---------------------
    MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    MECHANICAL_ARM.error_code = 0;

    MECHANICAL_ARM.cmd.pump_on = false;

    MECHANICAL_ARM.transform.dpos[J0] = J0_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J1] = J1_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J2] = J2_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J3] = J3_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J4] = J4_ANGLE_TRANSFORM;

    MECHANICAL_ARM.transform.duration[J0] = 2;
    MECHANICAL_ARM.transform.duration[J1] = 4;
    MECHANICAL_ARM.transform.duration[J2] = 4;
    MECHANICAL_ARM.transform.duration[J3] = 4;
    MECHANICAL_ARM.transform.duration[J4] = 4;

    // #Servo init ---------------------
    MA.servo.angle[0] = 75.0f;
    MA.servo.angle[1] = 75.0f;
}


/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSetMode                      */
/* auxiliary function:  None                                      */
/******************************************************************/

void MechanicalArmSetMode(void)
{
    if (MECHANICAL_ARM.error_code) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
        return;
    }

    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_CALIBRATE) {
        return;
    }

    if (switch_is_up(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_mid(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_down(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_HOLD;
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
    static float last_angle[5], angle_fdb[5] = {0, 0, 0, 0, 0};
    float vel,dpos;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
#define dangle MECHANICAL_ARM.transform.dpos


    // angle_fdb[J0] = theta_transform(
    //     MA.joint_motor[J0].fdb.pos, dangle[J0], MA.joint_motor[J0].direction,
    //     MA.transform.duration[J0]);

    // dpos = angle_fdb[J0] - last_angle[J0];
    // if (fabs(dpos) > M_PI) {
    //     MA.fdb.joint[J0].round += (dpos) < 0 ? 1 : -1;
    // }
    // last_angle[J0] = angle_fdb[J0];
    // MA.fdb.joint[J0].angle =
    //     (angle_fdb[J0] + M_PI * 2 * MA.fdb.joint[J0].round) / MA.joint_motor[J0].reduction_ratio;
    angle_fdb[J0] = theta_transform(
        MA.joint_motor[J0].fdb.pos, dangle[J0], MA.joint_motor[J0].direction,
        MA.transform.duration[J0]);
    
    MA.fdb.joint[J0].angle = angle_fdb[J0] / MA.joint_motor[J0].reduction_ratio;
    
    vel = MA.joint_motor[J0].fdb.vel / MA.joint_motor[J0].reduction_ratio *
          MA.joint_motor[J0].direction;
    MA.fdb.joint[J0].velocity = LowPassFilterCalc(&MA.lpf.j[J0], vel);
    
    MA.fdb.joint[J0].torque = MA.joint_motor[J0].fdb.tor * MA.joint_motor[J0].reduction_ratio *
                              MA.joint_motor[J0].direction;


    angle_fdb[J1] = theta_transform(
        MA.joint_motor[J1].fdb.pos, dangle[J1], MA.joint_motor[J1].direction,
        MA.transform.duration[J1]);

    dpos = angle_fdb[J1] - last_angle[J1];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J1].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J1] = angle_fdb[J1];
    MA.fdb.joint[J1].angle =
        (angle_fdb[J1] + M_PI * 2 * MA.fdb.joint[J1].round) / MA.joint_motor[J1].reduction_ratio;


    angle_fdb[J2] = theta_transform(
        MA.joint_motor[J2].fdb.pos, dangle[J2], MA.joint_motor[J2].direction,
        MA.transform.duration[J2]);

    dpos = angle_fdb[J2] - last_angle[J2];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J2].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J2] = angle_fdb[J2];
    MA.fdb.joint[J2].angle =
        (angle_fdb[J2] + M_PI * 2 * MA.fdb.joint[J2].round) / MA.joint_motor[J2].reduction_ratio;


    angle_fdb[J3] = theta_transform(
        MA.joint_motor[J3].fdb.pos, dangle[J3], MA.joint_motor[J3].direction,
        MA.transform.duration[J3]);

    dpos = angle_fdb[J3] - last_angle[J3];
    if (fabs(dpos) > M_PI) {
        MA.fdb.joint[J3].round += (dpos) < 0 ? 1 : -1;
    }
    last_angle[J3] = angle_fdb[J3];
    MA.fdb.joint[J3].angle =
        (angle_fdb[J3] + M_PI * 2 * MA.fdb.joint[J3].round) / MA.joint_motor[J3].reduction_ratio;


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
        case MECHANICAL_ARM_DEBUG: {

            static MechanicalArmMode_e last_mode = MECHANICAL_ARM_SAFE;
        if (last_mode != MECHANICAL_ARM_DEBUG) 
        {
                MA.ref.joint[J0].angle = MA.fdb.joint[J0].angle;
                MA.ref.joint[J1].angle = MA.fdb.joint[J1].angle;
                MA.ref.joint[J2].angle = MA.fdb.joint[J2].angle;
                MA.ref.joint[J3].angle = MA.fdb.joint[J3].angle;
                MA.ref.joint[J4].angle = MA.fdb.joint[J4].angle;

                 // 初始化舵机角度为中位
                angle[0] = 90.0f;
                angle[1] = 90.0f;
        }
            last_mode = MECHANICAL_ARM_DEBUG;

            // j0 右水平
            MA.ref.joint[J0].angle += GetDt7RcCh(DT7_CH_LH) * 0.002f;
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

            // j3 左水平摇杆
            MA.ref.joint[J3].angle += GetDt7RcCh(DT7_CH_RH) * 0.002f;
            MA.ref.joint[J3].angle = fp32_constrain(
                    MA.ref.joint[J3].angle, MA.limit.min.pos[J3], MA.limit.max.pos[J3]);

            // j4 模式开关
            if (switch_is_up(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
                MA.ref.joint[J4].angle += GetDt7RcCh(DT7_CH_RH) * 0.002f;
                MA.ref.joint[J4].angle = fp32_constrain(
                        MA.ref.joint[J4].angle, MA.limit.min.pos[J4], MA.limit.max.pos[J4]);
            }

            if (switch_is_mid(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
                MA.servo.angle[0] = 75.0f;
            } else if (switch_is_up(MECHANICAL_ARM.lookline->lookline.s[MECHANICAL_ARM_MODE_CHANNEL])) {
                MA.servo.angle[0] = 120.0f;
            }
            // ====================================================
        break;
        case MECHANICAL_ARM_HOLD: {
            // 保持模式：不更新ref，保持当前目标位置
            break;
        }
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            for (i = 0; i < 3; i++) {
                MECHANICAL_ARM.ref.joint[i].velocity = 0;
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
// fp32 PID_calc_value = 0;
void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_DEBUG: {
            // J0
            MA.joint_motor[J0].set.vel =
                PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle) *
                MA.joint_motor[J0].direction * MA.joint_motor[J0].reduction_ratio;
            MA.joint_motor[J0].set.value =
                PID_calc(&MA.pid.j0[1], MA.fdb.joint[J0].velocity, MA.joint_motor[J0].set.vel);

            // PID_calc_value = PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle);
            // J1
            MA.joint_motor[J1].set.vel =
                PID_calc(&MA.pid.j1[0], MA.fdb.joint[J1].angle, MA.ref.joint[J1].angle) *
                MA.joint_motor[J1].direction * MA.joint_motor[J1].reduction_ratio;
            MA.joint_motor[J1].set.value =
                PID_calc(&MA.pid.j1[1], MA.fdb.joint[J1].velocity, MA.joint_motor[J1].set.vel);

            // J2
            MA.joint_motor[J2].set.vel =
                PID_calc(&MA.pid.j2[0], MA.fdb.joint[J2].angle, MA.ref.joint[J2].angle) *
                MA.joint_motor[J2].direction * MA.joint_motor[J2].reduction_ratio;
            MA.joint_motor[J2].set.value =
                PID_calc(&MA.pid.j2[1], MA.fdb.joint[J2].velocity, MA.joint_motor[J2].set.vel);

            // J3
            MA.joint_motor[J3].set.vel =
                PID_calc(&MA.pid.j3[0], MA.fdb.joint[J3].angle, MA.ref.joint[J3].angle) *
                MA.joint_motor[J3].direction * MA.joint_motor[J3].reduction_ratio;
            MA.joint_motor[J3].set.value =
                PID_calc(&MA.pid.j3[1], MA.fdb.joint[J3].velocity, MA.joint_motor[J3].set.vel);
            
            // J4
            MA.joint_motor[J4].set.vel =
                PID_calc(&MA.pid.j4[0], MA.fdb.joint[J4].angle, MA.ref.joint[J4].angle) *
                MA.joint_motor[J4].direction * MA.joint_motor[J4].reduction_ratio;
            MA.joint_motor[J4].set.value =
                PID_calc(&MA.pid.j4[1], MA.fdb.joint[J4].velocity, MA.joint_motor[J4].set.vel);
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
            MA.joint_motor[J3].set.tor = 0;

            MA.joint_motor[J4].set.vel =
                PID_calc(&MA.pid.j4[0], MA.fdb.joint[J4].angle, MA.ref.joint[J4].angle) *
                MA.joint_motor[J4].direction * MA.joint_motor[J4].reduction_ratio;
            MA.joint_motor[J4].set.tor = 0;
        } break;
        case MECHANICAL_ARM_INIT: {  // 设置初始化参数
            MA.joint_motor[J0].set.value = 0;
            MA.joint_motor[J1].set.value = 0;
            MA.joint_motor[J2].set.value = 0;
            MA.joint_motor[J3].set.value = 0;
            MA.joint_motor[J4].set.value = 0;
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            MECHANICAL_ARM.joint_motor[J0].set.value = 0;
            MECHANICAL_ARM.joint_motor[J1].set.value = 0;
            MECHANICAL_ARM.joint_motor[J2].set.value = 0;
            MECHANICAL_ARM.joint_motor[J3].set.value = 0;
            MECHANICAL_ARM.joint_motor[J4].set.value = 0;
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
    CanCmdDjiMotor(ARM_DJI_CAN, 0x1FF, 0, 0, 0, 0);  // DJI CAN (CAN1)
    CanCmdDjiMotor(ARM_DM_CAN, 0x200, 0, 0, 0, 0);  // 修改为ARM_DM_CAN (CAN2)
}


/**
 * @brief 发送控制量
 * 
 */
void ArmSendCmdDebug(void)
{
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF,
        MA.joint_motor[J0].set.value,
        MA.joint_motor[J1].set.value,
        MA.joint_motor[J2].set.value,
        0);

    CanCmdDjiMotor(
        ARM_DM_CAN, 0x200,
        MA.joint_motor[J3].set.value,
        MA.joint_motor[J4].set.value,
        0,
        0);

    PwmCmdServo(0, MA.servo.angle[0]);
}

void ArmSendCmdInit(void)
{
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF,
        0,0,0,0);

    CanCmdDjiMotor(
        ARM_DM_CAN, 0x1FF,
        0,0,0,0);
}

#endif
#undef MA
/*------------------------------ End of File ------------------------------*/
