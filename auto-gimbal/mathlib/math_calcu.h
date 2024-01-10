#ifndef _MATH_CALCU_H_
#define _MATH_CALCU_H_

#include "math.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "arm_math.h"

#define N2 100
#define e  2.718282f
#define SIGMOID_PERIOD 0.133333f
#define SIGMOID_MAX    10

#define pi 3.14159265358979323846f

/*����޷�*/
#define Output_Limit(output,max,min) \
        ((output)<=(max) && (output)>=(min)? output: ((output)>(max)? (output = max):(output = min)))

/*����ֵ*/
#define ABS(x)		((x>0)? (x): (-x))

typedef struct
{
    float change_scale;
    uint16_t real_target;
    uint16_t limit_target;
    TickType_t ticks;
    TickType_t last_ticks;
} Slope_Struct;

typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

void Slope_On(Slope_Struct *V);
typedef struct
{
    float input;        //��������
    float out;          //�������
    float min_value;    //�޷���Сֵ
    float max_value;    //�޷����ֵ
    float frame_period; //ʱ����
} ramp_function_source_t;

extern ramp_function_source_t chassis_x_ramp;
extern ramp_function_source_t chassis_y_ramp;
extern ramp_function_source_t chassis_w_ramp;
extern ramp_function_source_t chassis_super_x_ramp;
extern ramp_function_source_t chassis_super_y_ramp;

void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min);
void Bubble_Sort(float *a,uint8_t n);
float GildeAverageValueFilter(float NewValue,float *Data);
float Sigmoid_function(float x);
float circle_error(float set,float get,float circle_para);
float data_limit(float data, float max, float min);
float lowpassfilter(float x);
#endif
