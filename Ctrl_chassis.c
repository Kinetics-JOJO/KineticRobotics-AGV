#include "Ctrl_chassis.h"
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "Configuration.h"
#include "PID.h"
#include "can1.h"
#include "can2.h"
#include <math.h>
#include <stdlib.h>
#include "Higher_Class.h"
#include "RemoteControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "User_Api.h"
#include "Comm_umpire.h"
#include "Ctrl_gimbal.h"
#include "laser.h"
#include "math.h"
#include "Usart_SendData.h"
#define CHASSIS_MAX_SPEED ((LIM_3510_SPEED/60)*2*PI*WHEEL_R/MOTOR_P)//底盘最大速度，mm/s
SpinTop_t SpinTop={0};
float SpinTop_Angle;
float W1_encode_angle,W2_encode_angle,W3_encode_angle,W4_encode_angle;
float Total_speed;
#define Chassis_DPI 0.0439453125f	//码值角度分辨率，360/8192
/*********************************************************************
 *实现函数：void virtual_encoders(int16_t aim[4])
 *功    能：计算虚拟码盘
 *输    入：控制器目标值
 *说    明：
 ********************************************************************/
int16_t Encoders[4];		//虚拟码盘值
void virtual_encoders(int16_t aim[4])
{
	uint8_t i;
	for(i=0; i<4; i++)
	{
		Encoders[i] += aim[i];
		if (Encoders[i]>8191) Encoders[i] -= 8192;
		else if (Encoders[i]<0) Encoders[i] += 8192;
	}
}
/*********************************************************************
 *实现函数：void Zero_handler(int16_t Angle_Val)
 *功    能：码盘值过0处理
 *输    入：换算过的角度值
 *说    明：
 ********************************************************************/
void Zero_handler(int16_t Angle_Val)
{
	if(Angle_Val>180)
	{
		Angle_Val-=360;
	}
	else if(Angle_Val<-180)
	{
		Angle_Val+=360;
	}
	
}
/*********************************************************************
 *实现函数：void Chassis_Encoder_angle_Handle(void)
 *功    能：云台电机旋转角度函数
 *输    入：NONE
 *说    明：
 ********************************************************************/
void Chassis_Encoder_angle_Handle(void)
{
	//can2各换向电机实时码盘值
	float W1_encode = motor2_data[1].NowAngle;
	float W2_encode = motor2_data[2].NowAngle;
	float W3_encode = motor2_data[3].NowAngle;
	float W4_encode = motor2_data[4].NowAngle;
	
	//	float W4_encode = motor_data[5].NowAngle;
	//码盘换算过来的角度值
  W1_encode_angle=((W1_encode-W1_Mid_encode)*Chassis_DPI)*Wheel_Direction;
	W2_encode_angle=((W2_encode-W2_Mid_encode)*Chassis_DPI)*Wheel_Direction;
	W3_encode_angle=((W3_encode-W3_Mid_encode)*Chassis_DPI)*Wheel_Direction;
	W4_encode_angle=((W4_encode-W4_Mid_encode)*Chassis_DPI)*Wheel_Direction;
	//过0处理
	Zero_handler(W1_encode_angle);
	Zero_handler(W2_encode_angle);
	Zero_handler(W3_encode_angle);
	Zero_handler(W4_encode_angle);

}


/*********************************************************************
 *实现函数：int encoders_err(int8_t i,int error)
 *功    能：计算实际目标码盘 与 目标虚拟码盘差 值
 *输    入：电机识别符，周期差值
 *返    回：实际目标码盘 与 目标虚拟码盘差 值
 *说    明：
 ********************************************************************/
int encoders_err(int8_t i,int error)
{
	int temp;
	
	temp = motor_data[i].NowAngle + error;
	temp = temp%8192;	
	if (temp<0) temp += 8192;						//实际目标码盘值
	temp = Encoders[i] - temp;
	if (temp<-3000) temp += 8192;
	else if (temp>3000) temp -= 8192;
	
	return temp;
}


/***************************
 *底盘电机速度控制
 *输入：4个电机的目标速度
 ***************************/
static void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	int16_t target_speed[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)
		PID_Control(&motor_speed_pid[i],target_speed[i],motor_data[i].ActualSpeed);
}

/***************************
 *舵轮换向电机角度控制（双环）
 *输入：4个电机的目标角度
 ***************************/
static void Chassis_6020Inverse_ctrl(int16_t M16020_angle,int16_t M26020_angle,int16_t M36020_angle,int16_t M46020_angle)
{
	
	int16_t target_angle[4];

	target_angle[0]=M16020_angle;	target_angle[1]=M26020_angle;
	target_angle[2]=M36020_angle;	target_angle[3]=M46020_angle;
	

	PID_Control(&chassis_angle_pid[0],target_angle[0],W1_encode_angle);
	PID_Control(&chassis_speed_pid[0],chassis_angle_pid[0].Control_OutPut,motor2_data[1].ActualSpeed);
	
	PID_Control(&chassis_angle_pid[1],target_angle[1],W2_encode_angle);
	PID_Control(&chassis_speed_pid[1],chassis_angle_pid[1].Control_OutPut,motor2_data[2].ActualSpeed);
	
	PID_Control(&chassis_angle_pid[2],target_angle[2],W3_encode_angle);
	PID_Control(&chassis_speed_pid[2],chassis_angle_pid[2].Control_OutPut,motor2_data[3].ActualSpeed);
	
	
	
	PID_Control(&chassis_angle_pid[3],target_angle[3],W4_encode_angle);
	PID_Control(&chassis_speed_pid[3],chassis_angle_pid[3].Control_OutPut,motor2_data[4].ActualSpeed);

//预留电机ID4 6020 分配给can1进行反馈控制
//	PID_Control(&chassis_angle_pid[3],target_angle[3],W4_encode_angle); 
//	PID_Control(&chassis_speed_pid[3],chassis_angle_pid[3].Control_OutPut,motor_data[5].ActualSpeed);
	

}

/***************************
 *底盘电机码值控制，位置闭环v2.0
 *输入：4个电机的每周期目标速度
 *计算码盘差值补偿到目标速度上，相较与用码盘差值做pid，这种方法响应性能更好更平滑
 ***************************/
static M_pid angle_pid[4];		//电机pid结构体
void motor_angle_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	float P,D;
	int error,temp;
	
	int16_t target_speed[4];
	int16_t target_angle[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)//转换为每周期目标码值
		target_angle[i] = (((float)target_speed[i]/ 60) * 8192)/PID_Hz;
	
	if(Encoders[0]==0)
	{
		if(Encoders[1]==0)
		{
			Encoders[0] = motor_data[0].NowAngle;
			Encoders[1] = motor_data[1].NowAngle;
			Encoders[2] = motor_data[2].NowAngle;
			Encoders[3] = motor_data[3].NowAngle;
		}
	}
	virtual_encoders(target_angle);	//计算虚拟码盘值
	
	for (i=0; i<4; i++)
	{
		P=0; D=0;									//中间值归零
		error = angle_pid[i].old_aim - motor_data[i].D_Angle;	//上一次未完成差值
		error += encoders_err(i,error);					//累加实际码盘值与虚拟码盘值的差
		angle_pid[i].old_aim = error + target_angle[i];					//更新旧目标值
		
		//**********P项**************
		P = PID_MOTOR_ANGLE_KP * error;
		//**********D项**************
		D = (error - angle_pid[i].old_err) * PID_MOTOR_ANGLE_KD;
		angle_pid[i].old_err = error;
		temp = P - D;
		if (temp > LIM_3508_SPEED) angle_pid[i].output = LIM_3508_SPEED;			//目标转速补偿输出限幅
		else if(temp < -LIM_3508_SPEED) angle_pid[i].output = -LIM_3508_SPEED;
		else angle_pid[i].output = temp;
		if ((angle_pid[i].output<70) && (angle_pid[i].output>-70)) angle_pid[i].output = 0;
		
		//输入到速度环
		temp = angle_pid[i].output + target_speed[i];	//补偿后的目标速度
		PID_Control(&motor_speed_pid[i],temp,motor_data[i].ActualSpeed);
	}
	    

}


/***************************
 *逆运动学控制
 *制底盘的水平速度、角速度
 *输入为底盘的运动目标，分别为Vx、Vy、Wv，单位mm/s,rad/s,前后为Vx
 *定义右前轮为w0，逆时针顺数
 ***************************/
#ifdef CHASSIS_POSITION_CRTL
//#define ANGLE_CTRL		//使用位置环
#endif

int16_t Chassis_mode=0;//底盘模式默认为0，即为码盘中点值

int debug_testflag=0;
float chassis_theta_rad=0;
float Real_angle=0;
int nan_flag=0;

//底盘状态机
#define RIGHT 2
#define LEFT  3
#define RIGHT_ROTATE 4
#define LEFT_ROTATE 5
#define RFLB_45 6
#define LFRB_45 7
#define SpinTop_On 1
#define STRAIGHT_BACK_WARD 0
#define STA_ZONE 40 //防误触区间范围+-40%遥感
//@Input:Control_data.Vx_6020,Control_data.Vy_6020,Control_data.Chassis_Wz_6020
/*********************************************************************
 *实现函数：void Chassis_6020position_judge(float Vx_judge,float Vy_judge,float Wx_judge)
 *功    能：判断底盘当前状态机，方便debug及后续对底盘各个状态下进行处理
 *输    入：Control_data.Vx_6020,Control_data.Vy_6020,Control_data.Chassis_Wz_6020
 *返    回：各状态机标识符
 *说    明：
 ********************************************************************/
void Chassis_6020position_judge(float Vx_judge,float Vy_judge,float Wx_judge)
{
	
	 if(SpinTop_Flag==1)  //怕小陀螺模式与后续的冲突决定单独提出来做中断处理
		Chassis_mode = SpinTop_On;
	else
	 {
	if((Real_angle>90)&&(Control_data.Chassis_Wz_6020==0))
	  Chassis_mode = RIGHT;
	else if ((Real_angle<-90)&&(Control_data.Chassis_Wz_6020==0))
		Chassis_mode = LEFT;
	else if((Control_data.Chassis_Wz_6020>0)&&(SpinTop_Flag==0))
	  Chassis_mode = RIGHT_ROTATE;
	else if ((Control_data.Chassis_Wz_6020<0)&&(SpinTop_Flag==0))
		Chassis_mode = LEFT_ROTATE;
	//右前和左后综合起来写，角度固定方向一致，只是3508电机速度环方向相反

	else if((Real_angle>0)&&(Real_angle<90)) 
		Chassis_mode = RFLB_45;
	else if ((Real_angle<0)&&(Real_angle>-90)) 
		Chassis_mode = LFRB_45;
	else
		Chassis_mode = STRAIGHT_BACK_WARD;
	}
}



/*********************************************************************
 *实现函数：void Chassis_position_Ctrl(void)
 *功    能：状态机下的电机位置环控制
 *输    入：NONE
 *返    回：chassis_theta_rad（弧度制单位）、Real_angle（相较于遥感转换的实际角度值）
 *说    明：遥感空闲状态返回值为nan，atan返回值域为-pi/2 ~ pi/2 
 ********************************************************************/
void Chassis_position_Ctrl(void)
{
	
	chassis_theta_rad=atan(Control_data.Vy_6020/Control_data.Vx_6020);    //atan返回-pi/2 ~ pi/2 
	Real_angle=chassis_theta_rad*180/PI;
//遥感空闲状态返回值为nan，避免电机误处理
	if(Real_angle!=Real_angle)
	{
		Real_angle=0;
		nan_flag=1;
	}else nan_flag=0;


	if((Chassis_mode==4)||(Chassis_mode==5)||(Chassis_mode==1))//LEFT_ROTATE mode=5 RIGHT 6   Spin_Top_ON 1
	Chassis_6020Inverse_ctrl(-45,45,-45,45);
	

	else if((Chassis_mode==6)||(Chassis_mode==7))//STRAIGHT_BACK_WARD mode=0
	Chassis_6020Inverse_ctrl(Real_angle,Real_angle,Real_angle,Real_angle);
	else//左右平移时
	Chassis_6020Inverse_ctrl(Real_angle,Real_angle,Real_angle,Real_angle);
}


//@Input:Control_data.Vx_6020,Control_data.Vy_6020,Control_data.Chassis_Wz_6020
static void Chassis_AngleInverse_Ctrl(float Angle_Vx,float Angle_Vy,float Angle_Wz)
{
	
	Chassis_Encoder_angle_Handle();//电机码盘角度换算处理
	Chassis_6020position_judge(Angle_Vx,Angle_Vy,Angle_Wz);
  Chassis_position_Ctrl();
  CAN2_Send_Msg_chassis_turnover(chassis_speed_pid[0].Control_OutPut,chassis_speed_pid[1].Control_OutPut,chassis_speed_pid[2].Control_OutPut,chassis_speed_pid[3].Control_OutPut);
	
	
	//CAN1_Send_Msg_chassis6020(chassis_speed_pid[3].Control_OutPut);
	//CAN2_Send_Msg_chassis_turnover(0,0,0,0);
}



static void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz)
{
	float w[4];		//四个轮子的实际转速rad/s
	int16_t n[4];	//转换为码盘速度的四个电机的转速
	uint8_t i=0;
	

	SpinTop.Angle  = (motor_data[4].NowAngle - Yaw_Mid_encode + 8192) % 8192 / 22.7555556f;
	if(SpinTop.Angle > 360)
		SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
	else
		SpinTop.Angle *= 0.0174532f;

  SpinTop_Angle = SpinTop.Angle * (180 / PI);
	
	SpinTop.Vx = Control_data.Vx * cos(SpinTop.Angle) + Control_data.Vy * sin(SpinTop.Angle);
	SpinTop.Vy = -Control_data.Vx * sin(SpinTop.Angle) + Control_data.Vy * cos(SpinTop.Angle);
		
	
	
	Total_speed=Vy + Vx;
	
//舵轮换向3508速度环

//	w[0] = (-SpinTop.Vy - SpinTop.Vx +CHASSIS_K*Wz)/WHEEL_R*speed_zoom;    //speed_zoom 功率限制系数  右前轮
//	w[1] = (SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;   //左前
//	w[2] = (SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //左后
//	w[3] = (-SpinTop.Vy - SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //右后
//		
 if(Chassis_mode==2)
	 
 {
	 
	w[0] = (Vy + Vx -CHASSIS_K*Wz)/WHEEL_R;
	w[1] = (-Vy - Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[2] = (-Vy - Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[3] = (Vy + Vx - CHASSIS_K*Wz)/WHEEL_R;

	 
	 
	 
 }else if(Chassis_mode==6)
 
 {
	 
	w[0] = (   Vx -CHASSIS_K*Wz)/WHEEL_R;
	w[1] = (   -Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[2] = (   -Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[3] = (   Vx - CHASSIS_K*Wz)/WHEEL_R;
	 
 }
 
 else
 
{
	w[0] = (-Vy + Vx -CHASSIS_K*Wz)/WHEEL_R;
	w[1] = (Vy - Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[2] = (Vy - Vx - CHASSIS_K*Wz)/WHEEL_R;
	w[3] = (-Vy + Vx - CHASSIS_K*Wz)/WHEEL_R;

}
  //}


	for(i=0;i<4;i++)
	 n[i] = ((float)w[i]*MOTOR_P / (2*PI)) * 60;	//转换为电机码盘速度

	//限制斜着走的速度
	for(i=0;i<4;i++)
	{
		if(n[i] > LIM_3508_SPEED)
		{
			uint8_t j=0;
			float temp = (float)n[i] / LIM_3508_SPEED;	//比例
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//等比例缩小
		}
		else if(n[i] < -LIM_3508_SPEED) 
		{
			uint8_t j=0;
			float temp = -(float)n[i] / LIM_3508_SPEED;	//比例
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//等比例缩小
		}
	}
//	#ifdef ANGLE_CTRL	//位置闭环
//	motor_angle_ctrl(n[0],n[1],n[2],n[3]);
//	#else	//速度闭环
	motor_speed_ctrl(n[0],n[1],n[2],n[3]);
//	#endif
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
														motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);

}


void FRT_Inverse_Kinematic_Ctrl(void *pvParameters)//底盘控制任务
{
	vTaskDelay(357);
	while(1)
	{ 
		if ( RC_Ctl.rc.s2 == RC_SW_DOWN)
		{
	    Power_off_function();		
		  //laser_off();		
		}
		else
		{
		  //number_t(2);	
		  //laser_on();
		  //Chassis_power_level();
		  Chassis_Power_Limit(); //通过修改斜坡启动斜率，以及限制轮组系数来达到效果
	    chassis_control_acquisition();//底盘控制量获取，状态机设置
		  chassis_set_contorl();//底盘控制量设置
				
			Chassis_AngleInverse_Ctrl(Control_data.Vx_6020,Control_data.Vy_6020,Control_data.Chassis_Wz_6020);//舵轮步兵底盘角度总控制
			Inverse_Kinematic_Ctrl(Control_data.Vx,Control_data.Vy,Control_data.Chassis_Wz);//底盘控制接口
		}
	  vTaskDelay(2);
	}
}

