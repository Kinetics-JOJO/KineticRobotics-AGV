#ifndef __KALMAN_H
#define __KALMAN_H
#include "stdint.h"
#include "sys.h"
#include "Configuration.h"

typedef struct Kalman_data
{
		float A;
		float Q;
		float R;
		float x_last;
		float p_last;
		float x_now;	
		float p_now;
		float x_next;
		float p_next;
		float K;
}Kalman1;

extern Kalman1 Vision_kalman_x;
extern Kalman1 Vision_kalman_y;
extern Kalman1 vision_kalman_x;

void kalmanCreate(void);
float KalmanFilter(Kalman1 *kalman1, float real, float speed, float run_time_clock, float fps);
float KalmanFilter1(Kalman1 *kalman1, float real, float real_last, float run_time_clock, float fps);
void Kalman_Reset(Kalman1 *p);












#endif
