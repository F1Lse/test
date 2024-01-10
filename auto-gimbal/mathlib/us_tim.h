/**
 * @file    us_tim.h
 * @author  ZZJ (WX:z1429881129)
 * @brief   ΢�뼶��ʱ/��ʱģ��
 * @version 1.0
 * @atten   Ŀǰģ����������ʱ��������漰�� ��1 �����⻹δ�����ϸ�Ĳ��ԣ�
 *          us��ʱ���� ust_delay(UST_TYPE us) ��δ���ԣ�
 * @date    2022-02-27
 * @copyright Copyright (c) Taurus 2022
 *
 * @note    ģ��ʹ�õĶ�ʱ������Ҫͨ������Ԥ��Ƶϵ����ʹ����ÿ1us�ı�һ��
 *          ʹ�÷�����
 *              0. ���ú�1us����һ�εĶ�ʱ�����޸�.h�ļ��е� UST_PRECISION��UST_HTIM��UST_UST_TIM_CNT_SIZE
 *              1. ������ж��з������´��룬�޸ĳ���Ӧ�Ķ�ʱ��
 *                  if (htim->Instance == TIM5)
 *                  {
 *                      prv_ust.overflow_cnt++;
 *                  }
 *              2. �� main() �е��� ust_tim_start(); ʹ�ܶ�ʱ�������ж�
 *              3. ����Ҫ��ʱ/��ʱ���ļ��������������磺 ust_t ust_user;
 *              4. ��ʱ
 *                  �����ʱ������ͳ��ĳ�δ����ִ��ʱ��
 *                      ust_interval_test_start(&ust_user);
 *                          ......
 *                      ust_interval_test_end(&ust_user);
 *                  ���ڼ�ʱ������ͳ��ĳ�������ִ������
 *                      ust_period_test(&ust_user);
 *                          ......
 *              5. �鿴��ʱ��� ust_user.dt
 *              6. ��ʹ���궨ʱ��������ʹ�� ust_tim_end(); ʧ�ܶ�ʱ��
 */
#ifndef __US_CNT_H__
#define __US_CNT_H__

#include "stdint.h"
#include "stm32f4xx_hal.h"

/**
 * @brief ģ���������
 */
#define UST_PRECISION 1         //��ʱ�����ȣ���λ��us
#define UST_HTIM htim5          //ʹ�õĶ�ʱ�����
#define UST_UST_TIM_CNT_SIZE 32 //��ʱ��ARR�Ĵ���λ��,��ͨ����оƬ�ֲ�ʱ�������

#if (UST_UST_TIM_CNT_SIZE == 32)
    #define UST_PERIOD 0xFFFFFFFF   //��ʱ�����������ֵ
    #define UST_TYPE uint32_t
#elif (UST_UST_TIM_CNT_SIZE == 16)
    #define UST_PERIOD 0xFFFF   //��ʱ�����������ֵ
    #define UST_TYPE uint16_t
#endif

/**
 * @brief �û�ʹ��ģ�����ݽṹ������
 */
typedef struct
{
    UST_TYPE last_tim;
    UST_TYPE last_cnt;
    
    UST_TYPE now_tim;
    UST_TYPE now_cnt;
    
    float dt; //��λ��ms
    uint8_t interval_start_flag; //�����ʱ������־
}ust_t;

/**
 * @brief ģ���ڲ���������
 */
typedef struct
{
    __IO UST_TYPE overflow_cnt; //��ǰ��ʱ���������
    __IO UST_TYPE predict_overflow_cnt; //Ԥ����ʱ����ʱ��ʱ���������
    __IO UST_TYPE predict_tim; //Ԥ����ʱ����ʱ��ʱ���ļ���ֵ
}prv_ust_t;

/**
 * @brief ģ���ڲ������ⲿ����
 */
extern prv_ust_t prv_ust;

/**
 * @brief ģ���ڲ�ʹ�õĶ�ʱ������ⲿ����
 */
extern TIM_HandleTypeDef UST_HTIM;

/**
 * @brief ������ʱ����������ж�
 */
void ust_tim_start(void);

/**
 * @brief �رն�ʱ����������ж�
 */
void ust_tim_end(void);

/**
 * @brief �����ִ�����ڲ���
 * @note  ����Ҫ���ԵĴ��봦����ʹ��
 * @param ust 
 * @return float �����ʱ���
 */
float ust_period_test(ust_t* ust);

/**
 * @brief �����ִ�������ʱ��ʼ
 *
 * @param ust 
 */
void ust_interval_test_start(ust_t* ust);

/**
 * @brief ���ִ�������ʱ����
 * @note  ��ʹ��ǰδ��ʼ�����ʱ����Ƶ�ʱ��dtΪ-1
 * @param ust 
 * @return float �����ʱ���
 */
float ust_interval_test_end(ust_t* ust);

/**
 * @brief us����ʱ����
 * @param us ��ʱʱ�䣨us��
 */
void ust_delay(UST_TYPE us);

#endif
