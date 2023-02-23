#include "Kalman.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "IMU.h"
#include "MPU6500.h"
#include "Configuration.h"
#include "led.h"
#include "buzzer.h"
#include "RemoteControl.h"
#include "Comm_umpire.h"
#include "Higher_Class.h"
#include "can2.h"
#include "PID.h"
#include "can1.h"
#include "pwm.h"
#include "laser.h"
#include "Ctrl_shoot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#define DITHERING_TIMNE 150 
#define Visual_TIME1 200
//状态机
u8 SpinTop_Flag =0;     //小陀螺标志
u8 Twist_Flag = 0;			//扭腰标志
u8 Vision_Flag = 0;			//视觉自瞄标志
u8 Shoot_Motor = 0;    //摩擦轮标志
u8 Shoot_Long = 0;     //吊射标志
u8 Steering_gear_test = 0;//舵机弹舱标志位
u8 Gimbal_180_flag = 0; //180度标志
u8 close_combatflag;   //近战标志
u8 Stronghold_flag = 0;//热量限制标志
u8 Inverse_flag = 0;   //拨轮反转
u8 chassis_power_level_1_Flag = 0;   //步兵底盘功率1级
u8 chassis_power_level_2_Flag = 0;   //步兵底盘功率2级
u8 chassis_power_level_3_Flag = 0;   //步兵底盘功率3级
u8 chassis_power_level_1_once_Flag = 0;   //步兵底盘功率1级第一次
u8 chassis_power_level_2_once_Flag = 0;   //步兵底盘功率2级第一次
u8 chassis_power_level_3_once_Flag = 0;   //步兵底盘功率3级第一次
u8 speed_17mm_level_Start_Flag = 0;  //开摩擦轮，默认速度
u8 speed_17mm_level_Low_Flag = 0;  //低速档
u8 speed_17mm_level_High_Flag = 0;  //高速档
u8 speed_17mm_level_Low_once_Flag = 0;  
u8 speed_17mm_level_High_once_Flag = 0;
u8 Usart_send_Flag = 0;//发送标识符
u8 TwistCnt,VisualCnt,ShootMotorCnt,speed_17mm_level_High_Cnt,speed_17mm_level_Start_Cnt,speed_17mm_level_Low_Cnt;
u8 chassis_power_level_3_Cnt,chassis_power_level_2_Cnt,chassis_power_level_1_Cnt,TriggerSingleMotorCnt,TriggerMotorCnt;
u8 ShootLongCnt,Steering_Gear_Cnt,Gimbal_180_Cnt,close_combatCnt,Stronghold_cnt,Inverse_cnt,SpinTop_cnt,Usart_send_Cnt;//计数用
u8 Cap_Safe_Flag =0;
float Gimbal_180_stop_flag_cnt = 0;
//////////////////////////////////////////////////
int32_t VerticalCnt=0,HorizontalCnt=0;
float Ramp_K = 1.0f;  //1.0f
//////////////////////////////////////////////////
int Heatmax = 0;//最大热量
int Shootnumber = 0;//可发射子弹数量
int SafeHeatflag = 1;//安全热量标志
int Shootspeed = 0;//射速
int Shootnumber_fired = 0;//已发射子弹数
int cycle_time = 0;//一个发射周期时间

ControlDATA_TypeDef Control_data;//控制数据
int slowly_increasing = 1800;//底盘逐渐加速
int PC_SPEED_X;//车体速度
float Wz_increasing = 0;

int Chassismode_flag = 0;//底盘模式标志位
int Gimbalmode_flag = 0;//云台模式标志位


float delta_speed =4.5f; //斜坡增益
float PC_Speed_Vx =0;
float PC_Speed_Vy =0;
int Wheel_position =1;
float speed_zoom = 1.0f ; //遥控器控制的速度
int16_t Target_chassis_power = 0;


//底盘功率限制变量包
//速度区间初始最大值（可修改）
int32_t Vertical_Initspeed=3000;
int32_t Horizontal_Initspeed=2000;
//速度区间输出最大值（变量）
int32_t Vertical_Maxspeed;
int32_t Horizontal_Maxspeed;


void chassis_control_acquisition(void)
{
		if (RC_Ctl.rc.s2 == RC_SW_MID)//遥控控制
		{
			Control_data.Vx = (RC_Ctl.rc.ch1-1024)*6.2;
			Control_data.Vy = (RC_Ctl.rc.ch0-1024)*6.2;
			Control_data.Chassis_Wz = (RC_Ctl.rc.ch2-1024)*0.035f;    //0.32f
			
			//放缩范围至-100—100 方便debug调整数值(100/660 = 0.15151515)
			Control_data.Vx_6020 = (RC_Ctl.rc.ch1-1024)*0.1515f;
			Control_data.Vy_6020 = (RC_Ctl.rc.ch0-1024)*0.1515f;
			Control_data.Chassis_Wz_6020 = (RC_Ctl.rc.ch2-1024)*0.1515f;    //0.32f
			
			
			Chassismode_flag = 0;
			offline_flag++;//裁判系统离线计数
	    if(offline_flag >= 2000)
		    offline_flag = 2000;
		}
		
		//按键按下的时候速度大小都增加
		else //电脑控制
		{  
			//Chassis_Power_Limit();  //功率闭环函数
			//输入值给定斜坡
			if(RC_Ctl.key.Vertical)
			{ 
			  VerticalCnt++;
				PC_Speed_Vx = Ramp_K*sqrt(5000*VerticalCnt);  //开方根函数加速度逐渐减小的加速斜坡
			}  
			else 
			{
				PC_Speed_Vx =0;
				VerticalCnt=0;
			}
			if (RC_Ctl.key.Horizontal)
			{   
				 HorizontalCnt++;
         PC_Speed_Vy = Ramp_K*sqrt(4000*HorizontalCnt);//斜坡启动有关  //3000
			}	 
			else 
		  {
				 PC_Speed_Vy=0;
				 HorizontalCnt=0;
			} 

		  Control_data.Vx =  RC_Ctl.key.Vertical*PC_Speed_Vx;
		  Control_data.Vy =  RC_Ctl.key.Horizontal*PC_Speed_Vy;
			 

			 //总输出速度限幅
			 VAL_LIMIT(Control_data.Vx ,-Vx_MAX,Vx_MAX);  
			 VAL_LIMIT(Control_data.Vy ,-Vy_MAX,Vy_MAX);
			 
			 //旋转斜坡
			 if(RC_Ctl.mouse.x != 0) 
			   Wz_increasing = 0.015f;    //0.053   //0.049      //Wz_increasing += 0.02f
       else
         Wz_increasing = 0;
			 
			 if(Wz_increasing >= 2.3f)   //2.3f    4.5f
			   Wz_increasing = 2.3f;        //2.3f    4.5f
			 Control_data.Chassis_Wz = 	Wz_increasing*RC_Ctl.mouse.x;

			if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP)//云台底盘分离和不分离模式切换
			{
				Chassismode_flag = 1;//云台底盘不分离模式	
			}
			else
			{
				yawcnt = 0;
				Chassismode_flag = 0;//云台底盘分离模式
			}
			 
	  }
}


void gimbal_control_acquisition(void)
{
	if (RC_Ctl.rc.s2 == RC_SW_MID)//遥控控制
	{
		Control_data.Gimbal_Wz = (RC_Ctl.rc.ch2-1024)*0.001f;  //0.006    0.003  0.005
		Control_data.Pitch_angle += (RC_Ctl.rc.ch3-1024)*0.0003f;//遥控器模式下底盘，云台控制量   0.0003
	
		Control_on_off_friction_wheel();//摩擦轮开关函数
		friction_wheel_ramp_function();//摩擦轮控制函数
	
		if(RC_Ctl.rc.ch4 !=1024)
		{
			SpinTop_Flag =1;
			Control_data.Gimbal_Wz = 	(RC_Ctl.rc.ch2-1024)*0.001f;
		}else SpinTop_Flag =0;		
		
		if(RC_Ctl.rc.s1 != RC_SW_DOWN)
			shoot_ready_control();
		else
			shoot_task();
		
	 if(RC_Ctl.rc.s1 == RC_SW_UP)
	 {
			PWM_Write(PWM2_CH3,0);
	 } 
	 else
	 {
		 PWM_Write(PWM2_CH3,82);
	 }
	
	 Gimbalmode_flag = 0;
	 
	}
		
		
	else//电脑控制
	{
		Control_data.Gimbal_Wz = 0.008f*RC_Ctl.mouse.x;  //0.012     //0.035   //0.018
		
		VAL_LIMIT(Control_data.Gimbal_Wz,-0.696f,0.696f);		//-0.396f  0.396f      //-0.496    0.496
		Control_data.Pitch_angle = Control_data.Pitch_angle -RC_Ctl.mouse.y*0.005f;  //0.003
			

	  if(RC_Ctl.mouse.press_r)//自瞄
		{
		  Vision_Flag = 1;
		  VisualCnt = 0;
		}	
		else
		{
		  Vision_Flag = 0;
		}
		
	
		//一键顺时针180度标志位
		if(Gimbal_180_Cnt > DITHERING_TIMNE)
		{
			if(RC_Ctl.key.Z && !RC_Ctl.key.Ctrl && !Gimbal_180_flag && SpinTop_Flag==0)
			{
				Gimbal_180_flag = 1;
				imu_last1 = imu.yaw;
				Gimbal_180_Cnt = 0;
			}

		}else Gimbal_180_Cnt++;	
	
	
		//小陀螺
		if(SpinTop_cnt>DITHERING_TIMNE)
		{
			if(RC_Ctl.key.G && Gimbal_180_flag==0)
			{
				SpinTop_Flag = !SpinTop_Flag;
				SpinTop_cnt = 0;
			}
		}
		else SpinTop_cnt++;
	
		//步兵底盘功率等级1~3级(初始化0级)
		if(chassis_power_level_1_Cnt > DITHERING_TIMNE)//1级
		{
			if(RC_Ctl.key.X && RC_Ctl.key.Ctrl && chassis_power_level_2_Flag == 0 && chassis_power_level_3_Flag == 0)
			{
				chassis_power_level_1_Flag = 1;
				chassis_power_level_1_once_Flag = 1;
				chassis_power_level_1_Cnt = 0;
			}
		}chassis_power_level_1_Cnt++;
		if(chassis_power_level_2_Cnt > DITHERING_TIMNE)//2级
		{
			if(RC_Ctl.key.C && RC_Ctl.key.Ctrl && chassis_power_level_1_Flag && chassis_power_level_3_Flag == 0)
			{
				chassis_power_level_1_Flag = 0;
				chassis_power_level_2_Flag = 1;
				chassis_power_level_2_once_Flag = 1;
				chassis_power_level_2_Cnt = 0;
			}
		}chassis_power_level_2_Cnt++; 
		if(chassis_power_level_3_Cnt > DITHERING_TIMNE)//3级
		{
			if(RC_Ctl.key.V && RC_Ctl.key.Ctrl && chassis_power_level_2_Flag)
			{
				chassis_power_level_2_Flag = 0;
				chassis_power_level_3_Flag = 1;
				chassis_power_level_3_once_Flag = 1;
				chassis_power_level_3_Cnt = 0;
			}
		}chassis_power_level_3_Cnt++;
	
	
		//17mm射速低速档高速档切换(开摩擦轮默认最低速度)
		if(speed_17mm_level_Start_Cnt > DITHERING_TIMNE)
		{
			if(RC_Ctl.key.E)//开摩擦轮，默认最低速度
			{
				speed_17mm_level_Start_Flag = 1;
				speed_17mm_level_Start_Cnt = 0;
				speed_17mm_level_Low_Flag = 1;
				speed_17mm_level_Low_once_Flag = 1;
				speed_17mm_level_High_Flag = 0;
				speed_17mm_level_High_once_Flag = 0;
			}
		}speed_17mm_level_Start_Cnt++;
		if(speed_17mm_level_Low_Cnt > DITHERING_TIMNE)//高速档
		{
	//		if(RC_Ctl.key.E && speed_42mm_level_Start_Flag && speed_42mm_level_High_Flag == 0 && speed_42mm_level_Low_once_Flag == 0)
			if(RC_Ctl.key.F && speed_17mm_level_Start_Flag && speed_17mm_level_Low_once_Flag)			
			{
				speed_17mm_level_High_Flag = 1;																					//////////////////			speed_17mm_level_Low_Flag = 1;
				speed_17mm_level_High_once_Flag = 1;                                    //////////////////			speed_17mm_level_Low_once_Flag = 1;
				speed_17mm_level_Low_Flag= 0;                                           //////////////////			speed_17mm_level_High_Flag = 0;
				speed_17mm_level_Low_once_Flag = 0;                                     //////////////////			speed_17mm_level_High_once_Flag = 0;
				speed_17mm_level_Low_Cnt = 0;                                          //////////////////			speed_17mm_level_Low_Cnt = 0;
			}
		}speed_17mm_level_Low_Cnt++;
		if(speed_17mm_level_High_Cnt > DITHERING_TIMNE)//关摩擦轮
		{
			if(RC_Ctl.key.B && speed_17mm_level_Start_Flag && (speed_17mm_level_High_once_Flag||speed_17mm_level_Low_once_Flag))
			{
				speed_17mm_level_Start_Flag = !speed_17mm_level_Start_Flag;  			////////////////////			speed_17mm_level_High_Cnt = 0;
				speed_17mm_level_High_Cnt = 0;
				speed_17mm_level_Low_Flag = 0;
				speed_17mm_level_Low_once_Flag = 0;
				speed_17mm_level_High_Flag = 0;
				speed_17mm_level_High_once_Flag = 0;
			}
		}speed_17mm_level_High_Cnt++;
	


	  if(Usart_send_Cnt > DITHERING_TIMNE)
	  {
		  if(RC_Ctl.key.Q && RC_Ctl.key.Ctrl &&  Usart_send_Flag == 0)
		  {
			  Usart_send_Cnt = 0;
			  Usart_send_Flag =! Usart_send_Flag;
		  }
	  }Usart_send_Cnt++;
	 	
	
	  if(chassis_power_level_1_Flag)
	  {
      Powerlimit_Ctrl(49);  //步兵底盘功率二级 50w
	  }
	  else if(chassis_power_level_2_Flag && chassis_power_level_1_once_Flag)
	  {
      Powerlimit_Ctrl(54);  //步兵底盘功率三级 55w
	  }
	  else if(chassis_power_level_3_Flag && chassis_power_level_2_once_Flag)
	  {
		  Powerlimit_Ctrl(54);  //步兵底盘功率三级 55w
	  }
	  //Powerlimit_Ctrl(Target_chassis_power);

	

		if(RC_Ctl.key.Shift)
		//if(RC_Ctl.key.Shift && PowerData[1]>15.0f)
		{
			PowerLimit_Flag = 1; //解除功率闭环消耗电容的电
		}else PowerLimit_Flag =0;
	

	

//		if(RC_Ctl.key.G == 1)//解除热量标志位
//		{
//			Stronghold_flag = 1;//裁判系统离线解除热量闭环不然发射不了子弹
//		}
//		else
//			Stronghold_flag = 0;
//		
		
//	if(Inverse_cnt > DITHERING_TIMNE)//拨轮手动反转
//	{
//		if(RC_Ctl.key.X == 1)
//		{
//			Inverse_flag = 1;
//			Inverse_cnt = 0;
//		}
//		else
//			Inverse_flag = 0;
//	}else Inverse_cnt++;

	
		offline_flag++;//裁判系统离线计数
		if(offline_flag >= 2000)
		{
			offline_flag = 2000;
			Stronghold_flag = 1; //裁判系统离线解除热量闭环不然发射不了子弹
		}
	
	
	  friction_wheel_ramp_function();//摩擦轮控制
	

		if(!RC_Ctl.mouse.press_l)//射击策略
		{
			cycle_time++;
			if(cycle_time >= 3000)
			{
				cycle_time = 3000;
				Shootnumber_fired = 0;
			}
			if(speed_17mm_level_Start_Flag)
				laser_on();
			if(!speed_17mm_level_Start_Flag)
				laser_off();
			shoot_ready_control();
		}
		else
		{
			cycle_time = 0;
			ShooterHeat_Ctrl();				
		}
		//shoot_task1();
	
		if(Inverse_flag)//拨轮手动反转
		{
			trigger_moto_speed_ref2 = 5000;
			trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
		}				

		
		if(Steering_gear_test)//开关弹舱
			PWM_Write(PWM2_CH3,0);
		else
			PWM_Write(PWM2_CH3,82);
		
		
		if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP && !Vision_Flag  && !Gimbal_180_flag)//云台底盘分离或不分离模式切换
			Gimbalmode_flag = 1; //云台底盘不分离
		else
			Gimbalmode_flag = 0;  //云台底盘分离
		
  }
		
}



//热量限制
static void ShooterHeat_Ctrl(void)
{		
	
	if(Game_robot_state.robot_level == 1)//机器人等级对应的热量上限
		Heatmax = 150;
	if(Game_robot_state.robot_level == 2)
		Heatmax = 280;
	if(Game_robot_state.robot_level == 3)
		Heatmax = 400;
	Shootnumber = (Heatmax - Umpire_PowerHeat.shooter_id1_17mm_cooling_heat)/10;//可发射子弹数量


	if( Shootnumber<=5)//已发射子弹数大于等于可发射子弹数时拨轮停转
	{  
	  SafeHeatflag = 0;
	  trigger_moto_current = 0;
	  Shootnumber_fired = 0;
	}
	else
	  SafeHeatflag	= 1;

	if(Stronghold_flag)//裁判系统离线解除热量闭环不然发射不了子弹
	  SafeHeatflag	= 1;

	 if(speed_17mm_level_Start_Flag && SafeHeatflag)//摩擦轮打开时拨轮才能启动
	//if(speed_17mm_level_Start_Flag)//摩擦轮打开时拨轮才能启动	 
	{
		 laser_on();//激光控制
		shoot_task1();//拨轮控制			
	}	 
	if(!speed_17mm_level_Start_Flag)
	{
		laser_off();//关闭激光
		trigger_moto_current = 0;
	}
}



