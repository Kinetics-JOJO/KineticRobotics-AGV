#ifndef __H_CLASS_H
#define __H_CLASS_H
#include "sys.h"

extern float vision_yaw_angle;
extern float vision_yaw_angle_last; 
extern float vision_yaw_angle_forcast;
extern float vision_yaw_angle_target;
extern float vision_yaw_angle_target_last;
extern float vision_yaw_angle_result;
extern float vision_yaw_angle_real;
extern float vision_yaw_angle_real_last;
extern float vision_yaw_speed_result;
extern float vision_yaw_speed_real;
extern float vision_pitch_angle_target;
extern float vision_pitch_angle_target_last;
extern float vision_pitch_angle_result;
extern float vision_pitch_angle_real;
extern float vision_pitch_speed_result;
extern float vision_pitch_speed_real;
extern float vision_run_time;
extern float vision_run_fps;
extern int yawcnt;
extern float imu_yaw1;
extern float imu_last1;
extern float SpinTop_timecount;
extern float SpinTop_Decreasecount;
extern float Actual_Power;
extern u8 PowerLimit_Flag;
extern float speed_zoom_double;
void Twist(void);//扭腰模式
void Chassis_follow_Gimbal(float Vx, float Vy, float Wz, float pitch_angle,float Gimbal_Wz);//底盘跟随云台回中模式
void Vision_Gimbal(float Vx,float Vy,float Chassis_Wz,float pitch_angle,float Gimbal_Wz);
void Vision_Gimbal_Reset(void);//自瞄复位

void Chassis_power_level(void);
void Power_off_function(void);
void Chassis_compensation (float Wz);
void chassis_set_contorl(void);
void gimbal_set_contorl(void);

void Astrict_Acc(float Vx,float Vy,float Wz);
void Chassis_Power_Limit(void);
void Powerlimit_Ctrl(int16_t Target_power);
void Powerlimit_Decision(void);
void imu_reset(void);
#endif

