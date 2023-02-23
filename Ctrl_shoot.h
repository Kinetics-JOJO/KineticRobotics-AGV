#ifndef SHOOT_H
#define SHOOT_H
#include "sys.h"
#include "can1.h"

extern int shoot_ready_flag;
extern int block_flag;
extern int single_flag;
extern int16_t trigger_moto_speed_ref2;
void Encode_C(M_Data*ptr);
static void Fire_PID_Init(void);
void shoot_task(void);
void FRT_shoot_task(void *pvParameters);
void block_bullet_handle(void);
void shoot_task1(void);
void shoot_ready_control(void);
#endif
