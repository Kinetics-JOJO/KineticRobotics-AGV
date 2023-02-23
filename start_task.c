#include "Start_Task.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"
#include "Power.h"
#include "pwm.h"
#include "can1.h"
#include "Comm_umpire.h"
#include "RemoteControl.h"
#include "spi.h"
#include "IMU.h"
#include "PID.h"
#include "can2.h"
#include "laser.h"
#include "Configuration.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "User_Api.h"
#include "Higher_Class.h"
#include <math.h>
#include <stdlib.h>
#include "Ctrl_shoot.h"
#include "Usart_SendData.h"

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define FRT_USART_DATA_Ctrl_TASK_PRIO      2
//任务堆栈大小
#define FRT_USART_DATA_Ctrl_STK_SIZE       512
//任务句柄
TaskHandle_t  FRT_USART_DATA_CtrlTask_Handler;  

//任务优先级
#define FRT_Inverse_Kinematic_Ctrl_TASK_PRIO		3
//任务堆栈大小	
#define FRT_Inverse_Kinematic_Ctrl_STK_SIZE 		512  
//任务句柄
TaskHandle_t FRT_Inverse_Kinematic_CtrlTask_Handler;

//任务优先级
#define FRT_Gimbal_Ctrl_TASK_PRIO		4
//任务堆栈大小	
#define FRT_Gimbal_Ctrl_STK_SIZE 		512  
//任务句柄
TaskHandle_t FRT_Gimbal_CtrlTask_Handler;

//任务优先级
#define FRT_IMU_task_PRIO           5
//任务堆栈大小
#define FRT_IMU_task_STK_SIZE       512
//任务句柄
TaskHandle_t  FRT_IMU_task_Handler;   

void FRT_IMU(void *pvParameters);



void start_task(void *pvParameters)
{
	  taskENTER_CRITICAL();           //进入临界区
    xTaskCreate((TaskFunction_t )FRT_IMU,//IMU姿态解算任务      
                (const char*    )"FRT_IMU",   
                (uint16_t       )FRT_IMU_task_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )FRT_IMU_task_PRIO,
                (TaskHandle_t*  )&FRT_IMU_task_Handler);						
	  xTaskCreate((TaskFunction_t )FRT_Gimbal_Ctrl,//云台控制任务     
                (const char*    )"FRT_Gimbal_Ctrl",   
                (uint16_t       )FRT_Gimbal_Ctrl_STK_SIZE, 
                (void*          )NULL, 
                (UBaseType_t    )FRT_Gimbal_Ctrl_TASK_PRIO,
                (TaskHandle_t*  )&FRT_Gimbal_CtrlTask_Handler);
    xTaskCreate((TaskFunction_t )FRT_Inverse_Kinematic_Ctrl,//底盘控制任务     	
                (const char*    )"FRT_Inverse_Kinematic_Ctrl",   	
                (uint16_t       )FRT_Inverse_Kinematic_Ctrl_STK_SIZE, 
                (void*          )NULL,				 
                (UBaseType_t    )FRT_Inverse_Kinematic_Ctrl_TASK_PRIO,	
                (TaskHandle_t*  )&FRT_Inverse_Kinematic_CtrlTask_Handler);
    xTaskCreate((TaskFunction_t )FRT_USART_DATA_Ctrl,//通信传输任务     	
                (const char*    )"FRT_USART_DATA_Ctrl",   	
                (uint16_t       )FRT_USART_DATA_Ctrl_STK_SIZE, 
                (void*          )NULL,				 
                (UBaseType_t    )FRT_USART_DATA_Ctrl_TASK_PRIO,	
                (TaskHandle_t*  )&FRT_USART_DATA_CtrlTask_Handler);								
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
void startTast(void)
{
	    xTaskCreate((TaskFunction_t )start_task,          //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄 
}
