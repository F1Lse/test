#ifndef __MATH_PROCESS_H
#define __MATH_PROCESS_H


#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "cmsis_os.h"


#define pi 3.14159265358979323846f

/*����޷�*/
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

/*����ֵ*/
#define ABS(x) ((x>0)? (x): (-x))

/**
  * @Brief   ��ֹ���޵ļӷ��������+��
  * @Param  �����ӱ�����������ֵ�����������ֵ
  */
#define ADD_LIMIT(var,delta,limit) \
        ( (limit)-(var)>=(delta)? (var)+=(delta): ((var)=(limit)) )

/**
  * @Brief   ��ֹ���޵ļ����������-��
  * @Param  ����������������ֵ����������Сֵ
  */
#define MIN_LIMIT(var,delta,limit) \
        ( (limit)-(var)<=(-delta)? (var)-=(delta): ((var)=(limit)) )


typedef struct
{
    float change_scale;  //б��
    float real_target;   //��ǰֵ����
    float limit_target;  //Ŀ����
    uint32_t ticks;
    uint32_t last_ticks;
} Slope_Struct;

typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

void    abs_limit(float *a, float ABS_MAX,float offset);
void    Slope_On(Slope_Struct *V);
void    slope_init(Slope_Struct *V, float change_scale,
                   float real_target, float limit_target);
void    ramp_calc(ramp_function_source_t *ramp_source_type,
                  float frame_period, float input, float max, float min);

float   circle_error(float set,float get,float circle_para);
bool    task_delay(uint16_t t[2], uint16_t task_T, uint16_t delay_ms);

#endif

