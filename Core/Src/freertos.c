#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Framework task entry points. Robot-specific tasks are added by the project. */
void StartDefaultTask(void *argument);
void INS_task(void *argument);
void detect_task(void *argument);
void led_RGB_flow_task(void *argument);
void referee_usart_task(void *argument);
void robot_task(void *argument);

extern void MX_USB_DEVICE_Init(void);

static const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t INS_Task_attributes = {
    .name = "INS_Task",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
static const osThreadAttr_t detectTask_attributes = {
    .name = "detectTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t RGB_Task_attributes = {
    .name = "RGB_Task",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t refereeTask_attributes = {
    .name = "refereeTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
static const osThreadAttr_t robotTask_attributes = {
    .name = "robotTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

void MX_FREERTOS_Init(void)
{
    (void)osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
    (void)osThreadNew(INS_task, NULL, &INS_Task_attributes);
    (void)osThreadNew(detect_task, NULL, &detectTask_attributes);
    (void)osThreadNew(led_RGB_flow_task, NULL, &RGB_Task_attributes);
    (void)osThreadNew(referee_usart_task, NULL, &refereeTask_attributes);
    (void)osThreadNew(robot_task, NULL, &robotTask_attributes);
}

void StartDefaultTask(void *argument)
{
    (void)argument;
    MX_USB_DEVICE_Init();
    for (;;) {
        osDelay(1);
    }
}

void vApplicationTickHook(void)
{
}
