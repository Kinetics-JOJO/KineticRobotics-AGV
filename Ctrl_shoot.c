#include "Ctrl_shoot.h"
#include "Ctrl_gimbal.h"
#include "MPU6500.h"
#include "IMU.h"
#include "PID.h"
#include "can1.h"
#include "Configuration.h"
#include "User_Api.h"
#include <math.h>
#include <stdlib.h>
#include "RemoteControl.h"
#include "delay.h"
#include "pwm.h"
#include "Higher_Class.h"
#include "User_Api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "laser.h"
#include "can1.h"
#define Butten_Trig_Pin GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)

int32_t trigger_moto_position_ref2;//目标角度
int16_t trigger_moto_speed_ref2;//目标速度  //103行
int16_t trigger_moto_current;//传给电调的值

int32_t round_cnt = 0;//拨轮电机旋转圈数

int block_flag = 0;//堵转标志
int block_time = 0;//堵转时长
int blockruntime = 0;//堵转计数

int block_flag1 = 0;//堵转标志
int block_time1 = 0;//堵转时长
int blockruntime1 = 0;//堵转计数 

int time = 0;//连发计数
int time2 = 0;

int single_flag = 0;
int continue_flag = 0;

int32_t Magazine_motor_encode_target;
int32_t Magazine_motor_encode_target_buff = 0;
float degree_60_to_encode_ramp = 73728/128;//25941/1024;
float degree_60_to_encode = 0;//25941;//25941;
int degree_60_to_encode_cnt = 0;
int continue_Flag;
int continue_mouse_Flag;

int trigger_motor_key_time = 0;
int shoot_ready_flag = 0;
int Butten_Trig_Pin_flag = 0;


//发射机构初始配置
#define FireMotor_Direction 1 //发射拨盘的转动方向
#define T_Dect 380 //开始反转的时间  决定反转的频率  //同时转矩检测的区间也会 决定反转的频率
#define Rev_T  100//反转的时间  决定反转的幅度


void Encode_C(M_Data*ptr)
{
	if (abs(ptr->NowAngle - ptr->OldAngle) > 4095)
  {
    if(ptr->NowAngle < ptr->OldAngle)
		{
		   ptr->total_encode += 8191 - ptr->OldAngle + ptr->NowAngle;
		}
		else
		{
			ptr->total_encode -= 8191 - ptr->NowAngle + ptr->OldAngle;
		}
  }
  else 
	{
    ptr->total_encode += ptr->NowAngle - ptr->OldAngle;
	}		
}

float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

void shoot_ready_control(void)
{
	//block_flag = 0;
	block_time = 0;
	blockruntime = 0;
	//continue_flag = 0;
	RC_Ctl.rc.s1_last = RC_Ctl.rc.s1;
	RC_Ctl.mouse.press_l_last = RC_Ctl.mouse.press_l;
	if(Butten_Trig_Pin)////判断子弹到达微动开关处   //!Butten_Trig_Pin启动微动开关 
	{
		trigger_moto_current = PID_Control(&Fire_speed_pid, 0 ,motor_data[6].ActualSpeed);//	注意这里trigger_moto_current不能直接赋0不然会导致后面发射的时候拨盘动不了;
		trigger_motor_key_time = 0;
		Butten_Trig_Pin_flag = 1;
	}

	else if(!Butten_Trig_Pin && trigger_motor_key_time < 1000)//判断无子弹一段时间  //触碰开关与单发模式下的结合
	{  
		Butten_Trig_Pin_flag = 2;
		if(single_flag)//单发模式转到大约期望角度停转
		{
	//		 trigger_moto_speed_ref2 = PID_Control(&Fire_pid, trigger_moto_position_ref2, motor_data[6].total_ecd);
	//     trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2 ,motor_data[6].ActualSpeed);		
			if(trigger_motor_key_time > 20)//单发模式下强制计数停止//虽然不知道意义何在
			trigger_moto_current = PID_Control(&Fire_speed_pid, 0 ,motor_data[6].ActualSpeed);//PID速度环//	注意这里trigger_moto_current不能直接赋0
		}
		if(continue_flag)//连发模式直接停转//这里加这个处理可以做到单发连发切换效果，切换中间停顿一下
		trigger_moto_current = PID_Control(&Fire_speed_pid, 0 ,motor_data[6].ActualSpeed);//PID速度环	
		trigger_motor_key_time++;
	}

	else if(!Butten_Trig_Pin && trigger_motor_key_time == 1000)//微动开关一段时间没有子弹，进入拨弹
	{
		//Fire_PID_Init();
		Butten_Trig_Pin_flag = 2;
		if(!block_flag1)
		{
		  trigger_moto_speed_ref2 = 900*FireMotor_Direction;//微动开关没检测到子弹，波轮自动转
		  trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//微动开关任务中的卡弹反转处理
		//考虑到触碰开关任务期间拨盘转动较慢使用检测速度而不是检测转矩
		//if(!motor_data[6].ActualSpeed)//卡弹反转处理
		if(motor_data[6].ActualSpeed < 20)//卡弹反转处理	
		  blockruntime1++;
		else if(motor_data[6].ActualSpeed < 0)//debug测试一下
		  blockruntime1 = 0;	
		if(blockruntime1 > 100) //开始反转的时间 决定反转的频率
		{
		  trigger_moto_speed_ref2 = -FireMotor_Direction*5000;	//决定反转的力度
		  block_flag1 = 1;
		}
		if(block_flag1)
		{
			block_time1++;
			if(block_time1 > 40)//反转的时间 决定反转的幅度
			{
				block_flag1 = 0;
				block_time1 = 0;
				blockruntime1 = 0;
			}	
		}
	//微动开关任务中的卡弹反转处理
	//检测转矩
	//	if(motor_data[6].Intensity>3000)
	//	blockruntime1++;
	//	if(blockruntime1 > 30)
	//	  {
	//		 trigger_moto_speed_ref2 = -FireMotor_Direction*4000;	
	//	   block_flag1 = 1;
	//		}
	//		if(block_flag1)
	//		{
	//			block_time1++;	
	//			if(block_time1 > 50)
	//			{
	//				block_flag1 = 0;
	//				block_time1 = 0;
	//				blockruntime1 = 0;
	//			}
	//		}
			
		if(block_flag1)
		  trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	
	}
}


void shoot_task(void)//遥控器模式控制拨轮
{
	block_flag1 = 0;
  blockruntime1 = 0;
	trigger_motor_key_time = 0;
	Encode_C(&motor_data[6]);
	if(!block_flag)
	{
	if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s1_last != RC_SW_DOWN)
	{
	  time = xTaskGetTickCount();
		single_flag = 1;
    continue_flag = 0;			
		trigger_moto_position_ref2 = motor_data[6].total_encode + FireMotor_Direction*DEGREE_60_TO_ENCODER;
	}
		
	trigger_moto_speed_ref2 = PID_Control(&Fire_pid, trigger_moto_position_ref2, motor_data[6].total_encode);
	trigger_moto_speed_ref2 = -trigger_moto_speed_ref2;
	
  if(RC_Ctl.rc.s1 == RC_SW_DOWN && xTaskGetTickCount() - time > 500)//左拨杆一直朝下连发模式  清理子弹用
	{
    trigger_moto_speed_ref2 = FireMotor_Direction*5000;
		single_flag = 0;
    continue_flag = 1;		
	}

	PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	

	trigger_moto_current = Fire_speed_pid.Control_OutPut;
	
	RC_Ctl.rc.s1_last = RC_Ctl.rc.s1;
  }
  
	block_bullet_handle();
	//电流值强制清零
	if((RC_Ctl.rc.s1 != RC_SW_DOWN))
	{
	  trigger_moto_current = 0;
    blockruntime = 0;	
	}
	
}

//电脑模式控制拨轮
void shoot_task1()//电脑模式控制拨轮
{
	block_flag1 = 0;
  blockruntime1 = 0;
	trigger_motor_key_time = 0;
	Encode_C(&motor_data[6]);
	if(!block_flag)
	{
		if(RC_Ctl.mouse.press_l && RC_Ctl.mouse.press_l != RC_Ctl.mouse.press_l_last)
		{
			time = xTaskGetTickCount();
			single_flag = 1;
			continue_flag = 0;			
			trigger_moto_position_ref2 = motor_data[6].total_encode + FireMotor_Direction*DEGREE_60_TO_ENCODER;
		}
		
		trigger_moto_speed_ref2 = PID_Control(&Fire_pid, trigger_moto_position_ref2, motor_data[6].total_encode);
	  trigger_moto_speed_ref2 = -trigger_moto_speed_ref2;
		
		if(RC_Ctl.mouse.press_l && xTaskGetTickCount() - time > 500)//鼠标左键长按连发模式
		{
			single_flag = 0;
			continue_flag = 1;		
			if(Stronghold_flag)
			{
				trigger_moto_speed_ref2 = 5000;  
			}		
			else
			{
				//trigger_moto_speed_ref2 = TRIGGER_MOTOR_SPEED*Shootnumber;
				trigger_moto_speed_ref2=6000;
//				if(trigger_moto_speed_ref2 >= 5000) //500
//					 trigger_moto_speed_ref2 = 5000; //1000
			}

		}	
		
		PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	

		trigger_moto_current = Fire_speed_pid.Control_OutPut;

		RC_Ctl.mouse.press_l_last = RC_Ctl.mouse.press_l;//记录上次鼠标左键情况
  }
	
  block_bullet_handle();
	
  if(!RC_Ctl.mouse.press_l)//鼠标左键松开
	{
    blockruntime = 0;	
	}		
}



//卡弹自动反转处理
//此处反转与触碰开关处不冲突即触碰开关任务的反转与这里的反转任务是独立的
void block_bullet_handle()//卡弹自动反转60度
{
	if((motor_data[6].ActualSpeed < 20 && RC_Ctl.rc.s1 == RC_SW_DOWN) | (motor_data[6].ActualSpeed < 20 && RC_Ctl.mouse.press_l))//连发模式卡弹反转//400 400
	  blockruntime++;
	else if((motor_data[6].ActualSpeed < 0 && RC_Ctl.rc.s1 == RC_SW_DOWN)| (motor_data[6].ActualSpeed < 0 && RC_Ctl.mouse.press_l))
	  blockruntime = 0;	
	

	if(blockruntime > 45) //100
	{
	  trigger_moto_speed_ref2 = -7500;  //8500
	  block_flag = 1;
	}
	
	if(block_flag)
	{
		block_time++;
		if(block_time > 55)  //150
		{
			blockruntime = 0;
			block_flag  = 0;
			block_time = 0; 		
		}	
  }
	if(block_flag)
    trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
}

////Can通信摩擦轮
//void Can_Fri_Ctrl(int16_t Fri_LeftTarget,int16_t Fri_RightTarget)
//{
//	//纯速度环堕速处理
//	//调试前请调整PID数据
//	PID_Control(&Fir_Left_pid,Fri_LeftTarget,motor_data[7].ActualSpeed);
//	PID_Control(&Fir_Right_pid,Fri_RightTarget,motor_data[8].ActualSpeed);
//	
//}


