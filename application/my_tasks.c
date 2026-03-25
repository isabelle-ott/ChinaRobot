/**
 * @brief 所有任务放在这里，方便管理
 */
#include "my_tasks.h"
#include "bsp.h"
#include "chassis.h"
#include "imu.h"
#include "miniCMD.h"
#include "usart.h"
#include "motor_application.h"
#include "application.h"
#include "servo.h"
#include "openmv.h"
#include "miniCMD_makeCmd.h"
#include "QR_code.h"

static void init(void);
static void createTask(void);
StaticEventGroup_t initOK_buffer;
EventGroupHandle_t initOK; // 用于等初始化完成之后通知任务可以开始执行
#define taskWaitInit() xEventGroupWaitBits(initOK, 0x01, pdFALSE, pdFALSE, osWaitForever)

/* 由 CubeMX 创建的默认任务 */
void StartDefaultTask(void const *argument)
{
	initOK = xEventGroupCreateStatic(&initOK_buffer);
	createTask();
	init();
	osDelay(2000);
	xEventGroupSetBits(initOK, 0x01);
	/* 完成，挂起 */
	vTaskSuspend(NULL);
	for (;;)
	{
		beebee(-1);
		osDelay(1000);
	}
}

/************************************ 初始化与任务创建 ************************************/

TaskHandle_t MainTaskHandle; // 主任务
StackType_t MainTask_Buffer[512];
UBaseType_t MainTask_Priority = 3; //[0,6]
void MainTask(void const *argument);
StaticTask_t MainTask_TCB;

TaskHandle_t JT_TrackTaskHandle; // 阶梯平台巡线任务
StackType_t JT_TrackTask_Buffer[256];
UBaseType_t JT_TrackTask_Priority = 3; //[0,6]
void JT_TrackTask(void const *argument);
StaticTask_t JT_TrackTask_TCB;

TaskHandle_t ChassisTaskHandle; // 底盘控制任务
StackType_t ChassisTask_Buffer[256];
UBaseType_t ChassisTask_Priority = 6; //[0,6]
void ChassisTask(void const *argument);
StaticTask_t ChassisTask_TCB;

/* 创建任务 */
static void createTask(void)
{
	MainTaskHandle = xTaskCreateStatic((TaskFunction_t)MainTask, "MainTask",
									   sizeof(MainTask_Buffer) / 4, NULL,
									   MainTask_Priority, MainTask_Buffer, &MainTask_TCB);

	ChassisTaskHandle = xTaskCreateStatic((TaskFunction_t)ChassisTask, "ChassisTask",
										  sizeof(ChassisTask_Buffer) / 4, NULL,
										  ChassisTask_Priority, ChassisTask_Buffer, &ChassisTask_TCB);

	JT_TrackTaskHandle = xTaskCreateStatic((TaskFunction_t)JT_TrackTask, "JT_TrackTask",
										   sizeof(JT_TrackTask_Buffer) / 4, NULL,
										   JT_TrackTask_Priority, JT_TrackTask_Buffer, &JT_TrackTask_TCB);
}

/* 打印所有任务的历史最大栈深度，用于配置任务栈大小的参考 */
static void print_Taskstack(void)
{
	TaskHandle_t tasks[] = {MainTaskHandle, ChassisTaskHandle, JT_TrackTaskHandle};
	uint32_t tasks_len = sizeof(tasks) / sizeof(TaskHandle_t);
	print_f("----------- 共 %d 个任务 -----------\n", tasks_len);
	for (uint32_t i = 0; i < tasks_len; i++)
	{
		print_f("%s\t%d\n", pcTaskGetName(tasks[i]), uxTaskGetStackHighWaterMark2(tasks[i]));
	}
	log_info("----------------------------------\n");
}
MAKE_CMD(void, print_Taskstack, void);

/* 在所有任务开始运行之前的初始化 */
static void init(void)
{
	_logger_print_init(); // logger
	imu_init();
	servo_inti();
	openMV_init();
	My_QR_Handle._QR_Init();
}

/************************************ 主任务 ************************************/

void MainTask(void const *argument)
{
	taskWaitInit();
	LED(0, 0, 0);
	log_info("MainTask start");

	miniCMD_init();
	logger_restart_RX();
	char *cmd = NULL;
	while (1)
	{
		if (Button1_Read() || Button2_Read())
		{
			run_by_button();
		}

		if ((cmd = logger_get()) != NULL)
		{
			miniCMD_run(cmd);
			logger_restart_RX();
		}
	}
}
/************************************ 巡线任务 ************************************/

static volatile int32_t JT_Track_work_flag = 0; // 1:请求开始或工作中; -1:请求结束; 0:结束

void JT_Track_start(void)
{
	JT_Track_work_flag = 1;
	xTaskNotify(JT_TrackTaskHandle, 0x01, eSetValueWithOverwrite); // 对任务通知值置1
}

void JT_Track_stop(void)
{
	JT_Track_work_flag = -1;
	xTaskNotify(JT_TrackTaskHandle, 0x02, eSetValueWithOverwrite); // 对任务通知值置2
	while (JT_Track_work_flag != 0) taskYIELD();
}

void JT_TrackTask(void const *argument)
{
	taskWaitInit();
	log_info("JT_TrackTask start");
	uint32_t notify_value;
	while (1)
	{
		xTaskNotifyWait(0, 0xffffffff, &notify_value, osWaitForever); // 等待开始信号
		while (JT_Track_work_flag == 1)
		{
			// 执行任务
			JT_TrackTask_process();
			xTaskNotifyWait(0, 0xffffffff, &notify_value, 3); // 延时3ms，同时允许提前结束
		}
		JT_Track_work_flag = 0;
	}
}

/************************************ 底盘控制任务 ************************************/

void ChassisTask(void const *argument)
{
	taskWaitInit();
	log_info("ChassisTask start");
	chassis_init();

	while (1)
	{
		chassis_control();
	}
}
