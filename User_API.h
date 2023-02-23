#ifndef __USERAPI_H
#define __USERAPI_H
#include "sys.h"


typedef struct
{
	float Vx;			//前后速度
	float Vy;			//左右速度
	float Chassis_Wz;			//旋转速度
	float Vx_6020;
	float Vy_6020;
	float Chassis_Wz_6020;
	float Gimbal_Wz;
	float Gimbal_Wzlast;
	float Pitch_angle;	//pitch轴角度
	
}ControlDATA_TypeDef;

//射击方案
typedef struct
{
	u16 Frequency;	//射频
	float Speed;		//射速
	
}ShootProject_TypeDef;

extern ControlDATA_TypeDef Control_data;//控制数据
extern u8 Twist_Flag;				//扭腰
extern u8 Vision_Flag;			//视觉自瞄
extern u8 Shoot_Flag;				//射击
extern u8 Shoot_Motor;			//摩擦轮
extern u8 chassis_power_level_1_Flag;   //步兵底盘功率1级
extern u8 chassis_power_level_2_Flag;   //步兵底盘功率2级
extern u8 chassis_power_level_3_Flag;   //步兵底盘功率3级
extern u8 chassis_power_level_1_once_Flag;   //步兵底盘功率1级第一次
extern u8 chassis_power_level_2_once_Flag;   //步兵底盘功率2级第一次
extern u8 chassis_power_level_3_once_Flag;   //步兵底盘功率3级第一次
extern u8 speed_17mm_level_Start_Flag;
extern u8 speed_17mm_level_Low_Flag;
extern u8 speed_17mm_level_High_Flag;
extern u8 speed_17mm_level_Low_once_Flag;
extern u8 speed_17mm_level_High_once_Flag;
extern u8 Gimbal_180_flag;
extern u8 close_combatflag;
extern u8 Steering_gear_test;
extern u8 Shoot_Long;
extern u8 SpinTop_Flag;     //小陀螺标志
extern int SafeHeatflag;
extern u8 Stronghold_flag;
extern int Chassismode_flag;
extern int Gimbalmode_flag;
extern int Shootnumber;
extern int Shootspeed; 
extern int Shootnumber_fired;
extern int Wheel_position;
extern float speed_zoom;
extern u8 Cap_Safe_Flag;
extern float Ramp_K;
extern int32_t VerticalCnt;
extern int32_t HorizontalCnt;
extern u8 Usart_send_Flag;

	//底盘功率限制参数包
extern int32_t Horizontal_Initspeed;
extern int32_t Vertical_Initspeed;
extern int32_t Horizontal_Maxspeed;
extern int32_t Vertical_Maxspeed;
extern float speed_zoom;
void User_Api(void);
void computer_ctrl(void);
void computer_handle(void);
void FRT_computer_ctrl(void *pvParameters);
static void ShooterHeat_Ctrl(void);
void chassis_control_acquisition(void);
void gimbal_control_acquisition(void);
/* 极值限制函数宏定义 */

#define VAL_LIMIT(val,  min, max)\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\




#endif


