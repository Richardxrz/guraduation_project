#include "communication_task.h"

#include "bsp_uart.h"
#include "cmsis_os.h"
#include "communication.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t communication_high_water;
#endif

// 任务相关时间
#define COMMUNICATION_TASK_INIT_TIME 100
#define COMMUNICATION_TASK_TIME_MS 2

void communication_task(void const * pvParameters)
{
    Usart1Init();
    // 空闲一段时间
    vTaskDelay(COMMUNICATION_TASK_INIT_TIME);
    while (1) {
        Uart2TaskLoop();

        // 系统延时
        vTaskDelay(COMMUNICATION_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        communication_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
