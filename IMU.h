#ifndef IMUSO3_H
#define IMUSO3_H

#include "sys.h"
#include "MPU6500.h"


extern uint8_t bFilterInit;	//开启快速收敛标志位
extern float imu_last;
extern float pitch,roll,yaw; 		//欧拉角
extern short aacx,aacy,aacz;		//加速度传感器原始数据
extern short gyrox,gyroy,gyroz;	//陀螺仪原始数据
//校准时间
#define ACC_CALC_TIME  3000//ms
#define GYRO_CALC_TIME   3000000l	//us

/* Function prototypes */
static float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

static void IMUSO3Thread(void);		//imu读取、姿态解算总接口
static void IMU_Temp_Ctrl(void);		//imu温度闭环
void IMU_Calibration(void);	//imu校准
static void IMUSO3Threadout(void);
void Attitude_Calculation(float gx,float gy,float gz,float ax,float ay,float az);

#endif

