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
  *  V1.0.2     Jun-13-2024     Penguin         1. 添加默认的任务控制时间类宏定义
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "chassis_task.h"

#include "attribute_typedef.h"
#include "chassis_mecanum.h"
#include "cmsis_os.h"
#include "bsp_delay.h"  // 添加延时函数头文件

#ifndef CHASSIS_TASK_INIT_TIME
#define CHASSIS_TASK_INIT_TIME 357
#endif  // CHASSIS_TASK_INIT_TIME

#ifndef CHASSIS_CONTROL_TIME_MS
#define CHASSIS_CONTROL_TIME_MS 2
#endif  // CHASSIS_CONTROL_TIME_MS

// 添加性能测试相关变量
static uint32_t chassis_task_start_time = 0;
static uint32_t chassis_task_end_time = 0;
static float chassis_task_duration = 0;
static uint32_t loop_counter = 0;
static float avg_chassis_task_duration = 0;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

__weak void ChassisPublish(void);
__weak void ChassisInit(void);
__weak void ChassisHandleException(void);
__weak void ChassisSetMode(void);
__weak void ChassisObserver(void);
__weak void ChassisReference(void);
__weak void ChassisConsole(void);
__weak void ChassisSendCmd(void);

/**
 * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void chassis_task(void const * pvParameters)
{
    // 初始化延时函数（如果尚未初始化）
    delay_init();
    
    // 发布底盘数据结构
    ChassisPublish();
    // 空闲一段时间
    // 延时357ms，等待其他任务初始化
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    // 初始化底盘
    ChassisInit();

    while (1) {
        chassis_task_start_time = SysTick->VAL;  // 获取起始时间（使用SysTick计数器）

        // 更新状态量（读取传感器、电机反馈）
        ChassisObserver();
        // 处理异常（电机离线、通信丢失等）
        ChassisHandleException();
        // 设置底盘模式（跟随/小陀螺/不跟随等）
        ChassisSetMode();
        // 更新目标量（根据遥控器输入计算期望速度）
        ChassisReference();
        // 计算控制量（PID控制）
        ChassisConsole();
        // 发送控制量（通过CAN发送到电机）
        ChassisSendCmd();
        
        chassis_task_end_time = SysTick->VAL;  // 获取结束时间
        
        // 计算耗时（注意SysTick是向下计数的）
        uint32_t systick_reload = SysTick->LOAD;
        chassis_task_duration = (float)((systick_reload - chassis_task_end_time + chassis_task_start_time) 
                                      * 1000.0f / SystemCoreClock);  // 转换为毫秒
        
        // 计算平均耗时，每100次更新一次
        avg_chassis_task_duration = (avg_chassis_task_duration * loop_counter + chassis_task_duration) 
                                   / (loop_counter + 1);
        loop_counter++;
        if (loop_counter >= 100) {
            loop_counter = 0;
            
            // 通过调试接口输出性能数据
            // ModifyDebugDataPackage(1, avg_chassis_task_duration, "avg_chassis_dur");
            // ModifyDebugDataPackage(2, chassis_task_duration, "cur_chassis_dur");
        }
        
        // 系统延时⚠️ 关键：主动让出CPU，延时2ms
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
        
// 可选：监控栈使用情况
#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

__weak void ChassisPublish(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisInit(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisHandleException(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSetMode(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisObserver(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisReference(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisConsole(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}
__weak void ChassisSendCmd(void)
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/*------------------------------ Calibrate Function ------------------------------*/

/**
  * @brief          设置底盘校准值，将底盘的校准数据设置为传入的校准值
  * @param[in]      motor_middle:电机中值
  * @retval         返回空
  * @note           底盘任务内部调用的函数
  */
__weak void ChassisSetCaliData(const fp32 motor_middle[4])
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
}

/**
  * @brief          底盘校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     motor_middle:电机中值
  * @retval         返回空
  * @note           底盘任务内部调用的函数
  */
__weak bool_t ChassisCmdCali(fp32 motor_middle[4])
{
    /* 
     NOTE : 在其他文件中定义具体内容
    */
    static uint32_t cnt = 0;
    cnt++;
    if (cnt > 1000) {
        motor_middle[0] = 0.0f;
        motor_middle[1] = 0.0f;
        motor_middle[2] = 0.0f;
        motor_middle[3] = 0.0f;
        cnt = 0;
        return 1;
    } else {
        return 0;
    }
}

/**
  * @brief          底盘校准设置，将校准的底盘中值以及最小最大机械相对角度
  * @param[in]      motor_middle:电机中值
  * @retval         返回空
  * @note           提供给校准任务调用的钩子函数
  */
void set_cali_chassis_hook(const fp32 motor_middle[4]) { ChassisSetCaliData(motor_middle); }

/**
  * @brief          底盘校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     motor_middle:电机中值
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @note           提供给校准任务调用的钩子函数
  */
bool_t cmd_cali_chassis_hook(fp32 motor_middle[4]) { return ChassisCmdCali(motor_middle); }

/*------------------------------ End of File ------------------------------*/
