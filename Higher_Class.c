///*************************************************************************
// *高级动作实现
// *限制功率
// *扭腰模式
// *云台跟随模式
// *视觉自动瞄准模式
// *
// *************************************************************************/
#include "Higher_Class.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "IMU.h"
#include "MPU6500.h"
#include "Configuration.h"
#include "led.h"
#include "buzzer.h"
#include "RemoteControl.h"
#include "can1.h"
#include "can2.h"
#include "Ctrl_chassis.h"
#include "Comm_umpire.h"
#include <math.h>
#include <stdlib.h>
#include "PID.h"
#include "Kalman.h"
#include "Ctrl_shoot.h"
#include "pwm.h"
#include "laser.h"
#include "FreeRTOS.h"
#include "task.h"

#define Chassis_MAX_speed_output 10000
#define Chassis        t

float TotalAverage_speed;
float imu_yaw1 = 0;//用于底盘运动补偿
float K_limit ;
int yawcnt = 0;//用于底盘运动补偿

float imu_last1;//用于旋转180度的变量
float Twist_buff;     //扭腰存放变量
float SpinTop_buff;   //小陀螺存放变量
float SpinTop_timecount=0; //小陀螺启动粗暴计时防止超功率

//static float Vx_Lpf=0,Vy_Lpf=0,Wz_Lpf=0;	//限制加速度后的运动量
float speed_zoom_double = 0;
float forcast_data = 0;

int wait_time=0;//imu重置计数
int imu_reset_flag=3;	
int vision_yaw_time = 0;

//*************功率限制变量包*************
int16_t ACC_Offset = 0;//加速度偏置，用于功率控制
int16_t Can_Msg_Power;
float Actual_Power ;
float Powerpid_out=1;
float speed_zoom_init=0.8;//35w的时候给定初始速度系数为0.8
u8 PowerLimit_Flag =0;

//******************视觉变量******************
float vision_yaw_angle = 0;
float vision_yaw_angle_last = 0;
float vision_yaw_angle_forcast = 0;
float vision_yaw_angle_forcast_last = 0;
float vision_yaw_angle_target = 0;
float vision_yaw_angle_target_last = 0;
float vision_yaw_angle_result = 0;
float vision_yaw_angle_real = 0;
float vision_yaw_angle_real_last = 0;
float vision_yaw_speed_result = 0;
float vision_yaw_speed_real = 0;
float vision_pitch_angle_target = 0;
float vision_pitch_angle_target_last = 0;
float vision_pitch_angle_result = 0;
float vision_pitch_angle_real = 0;
float vision_pitch_speed_result = 0;
float vision_pitch_speed_real = 0;
float vision_run_time = 0;
float vision_run_fps = 0;

int vision_yaw_angle_jscope = 0;
int vision_yaw_angle_forcast_jscope = 0;
int vision_yaw_angle_real_jscope = 0;
int vision_yaw_angle_target_jscope = 0;
int vision_yaw_speed_real_jscope = 0;


/**
  * @brief  求取数值的绝对值
  * @param  value
  * @retval 绝对值
  */
float Func_Abs(float value)
{
	if(value >= 0)
		return value;
	else 
		return -value;	
}


void chassis_set_contorl(void)//底盘控制量设置
{  
	if(SpinTop_Flag )
	{	
		SpinTop_timecount+=0.0016f;  //开启小陀螺会损耗功率
		VAL_LIMIT(SpinTop_timecount,0,1); // 从零增到1  //变速小陀螺只需要给最大值赋变量即可
		SpinTop_buff =  SpinTop_SPEED * SpinTop_timecount * SpinTop_Direction ;  //尝试换一下小陀螺的方向 实在不行把小陀螺的优先级换成并行优先级
		Control_data.Chassis_Wz = SpinTop_buff ;
		speed_zoom_double = 2.0f;
	} 
	else 
	{ 
		 
		SpinTop_timecount =0;
    speed_zoom_double = 2.0f;
//	if(Twist_Flag)//扭腰
//  {
//	Encoder_angle_Handle();//码盘角度转换
//	if(Yaw_Encode_Angle>=10)	//云台在底盘右边
//	{
//		Twist_buff = -TWIST_SPEED;
//	}
//	else if(Yaw_Encode_Angle<=-10)	//云台在底盘左边
//	{
//		Twist_buff = TWIST_SPEED;
//	}
//	Control_data.Chassis_Wz = Twist_buff;
//	}
			

	 
   if(Gimbal_180_flag)//一键180
	 Control_data.Chassis_Wz = -8.3f;  //-8.3f

	if(Chassismode_flag)//底盘云台不分离模式底盘补偿
	{
	if(imu.yaw > 170 | imu.yaw <-170)
	{ 
		yawcnt = 0;
		return;
	}
		if(!(RC_Ctl.rc.ch2-1024) && !RC_Ctl.mouse.x)	
	{
		if(!yawcnt)
		{
			imu_yaw1 =  imu.yaw;
			yawcnt++;
		}		
		//	Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.8f + ( imu.yaw - imu_yaw1)*0.1f;  //0.2  0.1
	}
	else
	yawcnt = 0;
	
	}		
	
	else//云台底盘分离
	{
		if(RC_Ctl.rc.s2 == RC_SW_MID)//遥控器模式底盘控制区间
		{
			//Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.1f + (-Yaw_Encode_Angle) * 0.15f;  //0.2  0.1   0.27   0.1
		}
	  if(RC_Ctl.rc.s2 == RC_SW_UP)//键鼠模式底盘控制区间
		{
	   // Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.1f + (-Yaw_Encode_Angle) * 0.18f;   //后期需要调整乘以的系数，云台移动太快或者底盘补偿太慢会触发底盘归中（初期定论8192
		}                                                    //0.1    //0.1     //0.6  0.1    //0.27
	}
}
	VAL_LIMIT(Control_data.Chassis_Wz,-Wz_MAX,Wz_MAX);

} 


#define CAM_FOV 38.0f	//摄像头角度
void gimbal_set_contorl(void)
{
	vision_yaw_angle_real = T_yaw;
	vision_yaw_angle_real_last = T_yawlast;
	vision_yaw_speed_real = -gryoYaw;
	vision_yaw_angle_forcast_last = vision_yaw_angle_forcast;
	
	vision_pitch_angle_real = -imu.roll;
	vision_pitch_speed_real = -gryoPITCH;
	
	vision_yaw_angle = vision_yaw_angle_real + vision_yaw_angle_target;
	vision_yaw_angle_forcast = KalmanFilter(&Vision_kalman_x, vision_yaw_angle, -gryoYaw, vision_run_time, vision_run_fps);
	//vision_yaw_angle_forcast = KalmanFilter1(&Vision_kalman_x, vision_yaw_angle, vision_yaw_angle_last, vision_run_time, vision_run_fps);
	
	vision_yaw_angle_jscope = vision_yaw_angle;
	vision_yaw_angle_forcast_jscope = vision_yaw_angle_forcast;
	vision_yaw_angle_target_jscope = vision_yaw_angle_real;
	
	if (Vision_Flag)
	{
	  if (!Vision_Data.target_lose)  //yaw识别到
	  {
			vision_yaw_angle_result = PID_Control(&vision_yaw_angle_pid, vision_yaw_angle_forcast, vision_yaw_angle_real);
			vision_yaw_speed_result = PID_Control(&vision_yaw_speed_pid, vision_yaw_angle_result, vision_yaw_speed_real);				
			
			if ((vision_yaw_angle_forcast_last > 0 && vision_yaw_angle_target < 0) || (vision_yaw_angle_forcast_last < 0 && vision_yaw_angle_target > 0))
				vision_yaw_speed_result = vision_yaw_speed_result * fmax(vision_yaw_angle_forcast_last, vision_yaw_angle_target)/(fabs(vision_yaw_angle_forcast_last) + fabs(vision_yaw_angle_target));
			
			if (fabs(vision_yaw_angle_target) > 5)
				vision_yaw_speed_result = vision_yaw_speed_result * 5 / vision_yaw_angle_target;
			//vision_pitch_angle_result = PID_Control(&vision_pitch_angle_pid, (vision_pitch_angle_real + vision_pitch_angle_target), vision_pitch_angle_real);
			//vision_pitch_speed_result = PID_Control(&vision_pitch_speed_pid, vision_pitch_angle_result, vision_pitch_speed_real);	
			//pitch_moto_current_final = vision_pitch_speed_result;
		}	
	  else
	  {
      vision_yaw_angle = 0;
			vision_yaw_angle_forcast = 0;
			vision_yaw_angle_target = 0;
			vision_yaw_angle_result = 0;
			vision_yaw_speed_result = 0;
//      vision_pitch_angle = 0;
//			vision_yaw_angle_forcast = 0;
//			vision_yaw_angle_target = 0;
//			Kalman_Reset(&Vision_kalman_x);
//			Kalman_Reset(&Vision_kalman_y);
	  }	
	}

	if(Gimbal_180_flag)
	  Control_data.Gimbal_Wz = -0.45f;  //-0.25f

	else//底盘跟随云台
	{
		if(RC_Ctl.rc.s2 == RC_SW_MID)//遥控器模式云台控制区间
		{
			if((20 < Yaw_Encode_Angle && Control_data.Gimbal_Wz < 0)| (Yaw_Encode_Angle < -20 && Control_data.Gimbal_Wz >0))
				Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.6f;  //0.1f  0.6
			else 
				Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.6f;	   //2   0.7
		}			
	  else if(RC_Ctl.rc.s2 == RC_SW_UP&&SpinTop_Flag==0)//键鼠模式云台控制区间  
		{
			if( (50 < Yaw_Encode_Angle && Control_data.Gimbal_Wz < 0)| (Yaw_Encode_Angle < -50 && Control_data.Gimbal_Wz >0))
			  Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.7f;    //0.1f   //1.7    //1.2
			else 
			  Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.7f;       //2    //1.2
    }		
		else
		  Control_data.Gimbal_Wz =0.015f*RC_Ctl.mouse.x;    //小陀螺开启时的瞄准速度    //0.027
	}
	
	if(Control_data.Gimbal_Wz > 0.5f)//云台给定量限幅  //0.45f     0.6
		Control_data.Gimbal_Wz = 0.5f;                     //0.45f
	if(Control_data.Gimbal_Wz < -0.5f)                  //0.45f
		Control_data.Gimbal_Wz = -0.5f;                    //0.45f
}


void Power_off_function(void)//断电保护
{
			CAN1_SendCommand_chassis(0,0,0,0);
      CAN2_Send_Msg_chassis_turnover(0,0,0,0);
			CAN1_Send_Msg_gimbal(0,0,0);
			CAN2_Send_Msg_gimbal(0);
			fri_on = 0;
			speed_17mm_level_Start_Flag = 0;
			friction_wheel_ramp_function();
			Control_data.Vx = 0; 
			Control_data.Vy = 0; 
			Control_data.Chassis_Wz = 0;
			Control_data.Pitch_angle = 0;
			Control_data.Gimbal_Wz = 0;
			speed_17mm_level_Start_Flag = 0;//标志位清零
			//Steering_gear_test = 0;
			speed_17mm_level_High_Flag = 0;
			speed_17mm_level_Low_Flag = 0;
			Twist_Flag = 0;
			Vision_Flag = 0;
			Gimbal_180_flag = 0;
			SafeHeatflag = 1;
			block_flag = 0;
			Stronghold_flag = 0;
      offline_flag = 0;
      error = 0;
	    Yaw_AnglePid_i = 0;
	    T_yaw = 0;
      P1 = 0;
	    I1 = 0;
	    D1 = 0;
	    angle_output1 = 0;
			Yaw_Vision_Speed_Target = 0;
			Yaw_Vision_Speed_Target_Mouse = 0;
			Pitch_Vision_Speed_Target = 0;
	    Control_data.Gimbal_Wz = 0;
      E_yaw = imu.yaw;
      Chassismode_flag = 0;
      PWM_Write(PWM2_CH3,0);
      laser_off();		
    	
}





//该函数给各恒压功率的配置初始化速度系数，可以有效防止启动过猛导致前期电容电量耗得太快
//与此同时防止can通信中断导致底盘不能正常移动，电容板与MCU通信失败强制限制速度
void Powerlimit_Decision(void)
{
//	if(PowerData[3] == 44) //默认给定50w恒压
//	{		
//		if(SpinTop_Flag)
//			speed_zoom_init = 0.65f;
//		else
//			speed_zoom_init = 0.75f;
//	}
	if(PowerData[3] == 44)//升级加点45w恒压
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.45f;
		else
		  speed_zoom_init = 0.65f;
	}
	else if(PowerData[3] == 49)//升级加点50w恒压
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.55f;
		else
		  speed_zoom_init = 0.75f;
	}
	else if(PowerData[3] == 54)//55w后面统一初始系数1，后期需要更猛可以再加个判断
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.65f;
		else
		  speed_zoom_init = 0.85f;
	}
	else//can通信失败强制限初始速度,但还是可以通过shift加速键来解除闭环
		//speed_zoom_init =0.75f;
	  speed_zoom_init = 0.45f;
}
//功率闭环v2.0
//通过PID控制器算出的值和限制系数建立一个线性关系
//给每个轮子的PID输出值乘以限制系数来达到限制轮速得效果
//读取裁判系统实时数据
//通过比例系数来控制轮速
//
//

void Chassis_Power_Limit(void)
{
    Powerlimit_Decision(); //可修改Ramp_K来限制启动加速度，即斜坡斜率
	  PID_Control(&Power_Limit_pid, 20, PowerData[1]);  //以电容电压值作输出限制系数
    if(PowerLimit_Flag==1) //功率闭环手动解除键
		{  
			if(PowerData[1] <= 16)
				speed_zoom = 0.65f;
			else
				speed_zoom = 2.0f;//shift键加速，实际是给到速度限幅最大值
		}
		else 
		{
		  speed_zoom =speed_zoom_init-Power_Limit_pid.Control_OutPut*speed_zoom_double;  //输出限幅为5	 
   	  //VAL_LIMIT(speed_zoom,0.25f,speed_zoom_init);
			VAL_LIMIT(speed_zoom,0.5f,speed_zoom_init);
	  }
}


//配合超级电容板
//输入：设定恒压功率（35w——130w）
//不调用函数时电容板上电自启默认35w
void Powerlimit_Ctrl(int16_t Target_power)
{
	Can_Msg_Power = Target_power*100;
	CAN1_SendCommand_Powerlimit(Can_Msg_Power);
}

void imu_reset(void)//imu校准
{
	if(imu_reset_flag==0)
	{
		wait_time = xTaskGetTickCount();
		imu_reset_flag=1;
	}
	if(imu_reset_flag==1 && xTaskGetTickCount() - wait_time > 1500 && xTaskGetTickCount() - wait_time < 4500)
	{
		buzzer_on(95, 10000);
		imu.ready = 0;
		imu_reset_flag=2;
	}
	else if(xTaskGetTickCount() - wait_time >= 4500)
	{
		buzzer_off();
	}
}
