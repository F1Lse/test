#include "math_process.h"


/**
  * @brief  б���ṹ�壨�٣���ʼ��
  * @note   ������Ҫ�ı�Ŀ��ֵʱʹ��
  */
void slope_init(Slope_Struct *V, float change_scale,
                float real_target, float limit_target)
{
    /*��ֹ�ظ���ʼ��*/
    if(limit_target == V->limit_target)
        return;

    V->change_scale = change_scale;
    V->real_target = real_target;
    V->limit_target = limit_target;
}

/**
  * @brief          б����������
  * @param[in]      б���ṹ��
  * @note           ������ϵͳ�����е��ã���
  *                 ���ڱ��������������У���Ϊб��ȷ����������������ȡ����ִ��ʱ������
  * @note           ʹ��׼������Ҫ����һ���ṹ���������ʼ����ʼֵ��Ŀ��ֵ�Լ�б��
  *                 ʹ�÷��������ô˺������ٵ��ýṹ������� real_target
  */
void Slope_On(Slope_Struct *V)
{
    V->last_ticks = V->ticks;
    V->ticks = osKernelSysTick();
    if(V->real_target !=V->limit_target)  //�Ƿ��Ѿ�����Ŀ��ֵ
    {
        if(V->real_target < V->limit_target)  //�Ӳ���
        {
            V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target > V->limit_target)  //�޷�
            {
                V->real_target =  V->limit_target;
            }
        }
        else if(V->real_target > V->limit_target)  //������
        {
            V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
            if(V->real_target < V->limit_target)  //�޷�
            {
                V->real_target =  (short)V->limit_target;
            }
        }
    }
}

/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->frame_period = frame_period;

    ramp_source_type->input = input;

    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

/**
  * @brief          ð��������
  * @param[in]      �������飬�����С
  * @retval         void
  */
void Bubble_Sort(int *a,uint8_t n)
{
    float buf;
    for(uint8_t i=0; i<n; ++i)
    {
        for(uint8_t j=0; j<n-1-i; ++j)
        {
            if(a[j]<a[j+1])
            {
                buf=a[j];
                a[j] = a[j+1];
                a[j+1] = buf;
            }
        }
    }
}

/**
  *@Brief     ��������ƫ�����
  *@Param    set �趨ֵ get����ֵ circle_paraһȦ��ֵ
  *@Note     ���������£�ֱ�Ӽ����PID�е�ƫ��ֵ ���������� error �ķ����Ǵ�getָ��set���Ż�
  */
float circle_error(float set,float get,float circle_para)
{
    float error;
    if(set > get)
    {
        if(set - get> circle_para/2)
            error = set - get - circle_para;
        else
            error = set - get;
    }
    else if(set < get)
    {
        if(set - get<-1*circle_para/2)
            error = set - get +circle_para;
        else
            error = set - get;
    }
    else	error = 0;

    return error;
}

/**
  * @Brief   ���ݰ���ƫ�������޷�
  * @Param  ���ݱ���ָ�룬���ֵ��ƫ��������ʼֵ��
  */
void abs_limit(float *a, float ABS_MAX,float offset)
{
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;
}

/**
  * @Brief   ����������ʱ
  * @Param  ��¼��ǰʱ���������ʱʱ������飨��ʼ��Ϊ{0��0}�����������ڣ���ʱʱ��
  */
bool task_delay(uint16_t t[2], uint16_t task_T, uint16_t delay_ms)
{
    if( t[0] == 0 )
        t[1] = delay_ms/task_T;  //��Ҫ����ִ�еĴ���
    t[0] += 1;
    if( t[0] >= t[1] )
    {
        t[0] = 0;
        return true;
    }
    else
        return false;
}

