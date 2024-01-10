#ifndef __DRIVER_FRICMOTOR
#define __DRIVER_FRICMOTOR

#include "stm32f4xx_hal.h"
#include "shoot_task.h"

#define Init_PWM	 900

typedef struct
{
 		float sum_x2;//xƽ���ĺ�
    float sum_y ;//y�ĺ�
    float sum_x ;//x�ĺ�
    float sum_xy ;//x����y�ĺ�
		float a;//y=ax+b�������б��
    float b;//y=ax+b������ĳ���
		float bullet_speed;
		float last_bullet_speed;
		float x[30];
    float Shoot_speed[30];
		float Speed_pre;
		uint16_t adapting_flag;
		uint16_t shoot_num;
		uint32_t adapting_num;
} FricMotor_Pid_Control;



void FricMotor_init(void);
void FricMotor_Control(void);
void FricGunControl_PID(uint16_t pwm);
#endif
