#include "Kalman.h"

Kalman1 Vision_kalman_x;
Kalman1 Vision_kalman_y;
Kalman1 vision_kalman_x;

void kalmanCreate(void)
{
	  Vision_kalman_y.A = 1;
    Vision_kalman_y.Q = 3;
    Vision_kalman_y.R = 1;

    Vision_kalman_x.A = 1;
    Vision_kalman_x.Q = 1;
    Vision_kalman_x.R = 40;
	
    vision_kalman_x.A = 1;
    vision_kalman_x.Q = 1;
    vision_kalman_x.R = 40;	
}

float KalmanFilter(Kalman1 *kalman1, float real, float speed, float run_time_clock, float fps)
{
    kalman1->x_now = kalman1->A * real + speed * (run_time_clock + fps * 0.001f) * 1.0f;  
	  //kalman1->x_now = kalman1->A * real + speed * run_time_clock * 1.0f;  
    kalman1->p_now = kalman1->A * kalman1->p_last + kalman1->Q;               
    kalman1->K = kalman1->p_now / (kalman1->p_now + kalman1->R);    //¿¨¶ûÂüÔöÒæ            
    kalman1->x_next = kalman1->x_now + kalman1->K*(real - kalman1->x_now);    //Ô¤¹À 
    kalman1->p_next = (1 - kalman1->K) * kalman1->p_now;     
   	kalman1->x_last = kalman1->x_next;  
    kalman1->p_last = kalman1->p_next;
	  return kalman1->x_next;
}

float KalmanFilter1(Kalman1 *kalman1, float real, float real_last, float run_time_clock, float fps)
{
    kalman1->x_now = kalman1->A * real + (((real - real_last) * run_time_clock * 1000) / fps) * 1.0f;  
    kalman1->p_now = kalman1->A * kalman1->p_last + kalman1->Q;               
    kalman1->K = kalman1->p_now / (kalman1->p_now + kalman1->R);    //¿¨¶ûÂüÔöÒæ            
    kalman1->x_next = kalman1->x_now + kalman1->K*(real - kalman1->x_now);    //Ô¤¹À 
    kalman1->p_next = (1 - kalman1->K) * kalman1->p_now;     
   	kalman1->x_last = kalman1->x_next;  
    kalman1->p_last = kalman1->p_next;
	  return kalman1->x_next;
}

void Kalman_Reset(Kalman1 *kalman1)
{
		kalman1->x_now = 1;
		kalman1->p_now = 1;
		kalman1->x_next = 1;
		kalman1->p_next = 1;
		kalman1->x_last = 1;
		kalman1->p_last = 1;
}


