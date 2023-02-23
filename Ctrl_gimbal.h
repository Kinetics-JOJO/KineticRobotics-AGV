#ifndef GIMBAL_H
#define GIMBAL_H

#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "can1.h"

extern int32_t Pitch_PID_target;
extern int32_t Pitch_PID_real;
extern int32_t Pitch_PID_current;
extern float Yaw_Encode_Angle,Pitch_Encode_Angle;
extern float speed_17mm_level;
extern float speed_17mm_result;
extern float gryoYaw;
extern float gryoPITCH;
extern int fri_on;
extern float E_yaw;
extern float E_yaw1;
extern float P1;
extern float I1;
extern float D1;
extern float angleYaw;
extern float T_yaw;		//目标角度
extern float T_yawlast;
extern float Yaw_AnglePid_i;//角度环积分项
extern float angle_output1;
extern float Yaw_Vision_Speed_Target;
extern float Yaw_Vision_Speed_Target_Mouse;
extern float Yaw_Vision_Speed_Target1;
extern float Pitch_Vision_Speed_Target;
extern float error;
static void Pitch_pid(float Target_angle);
static void Yaw_angle_pid(float Targrt_d_angle);
extern int imu_reset_flag;


void Encoder_angle_Handle(void);
static void ramp_calc(void);
void friction_wheel_ramp_function(void);
static void ramp_calc1(void);
void Control_on_off_friction_wheel(void);
void Power_off_function(void);
void Gimbal_Ctrl(float pitch,float yaw_rate);
void FRT_Gimbal_Ctrl(void *pvParameters);

#endif
