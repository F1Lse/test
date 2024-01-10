/**
 * @file ubf.h
 * @brief ʵʱ���»��λ�����
 * @note ʹ�÷����ο� .c�ļ��·�
 * 
 * @author ZZJ
 * @version 1.0 2022-02-25 �״���ɻ������ܣ��������Ĵ�����ѹջ��������ɾ����
 *          1.1 2022-03-04 ������ջ������Ĺ��ܡ�
 *          1.2 2022-03-14 ���ӽ�������������ת�浽���������еĹ��ܡ�
 *          1.3 2022-03-24 ����ʵʱ�����µľ�ֵ�˲������Լ�Ȩ�������ݲ�ֺ����Լ�ȨԤ��Ĺ��ܣ�
 *                         ���Լ�ȨԤ�⹦����δ���ƣ�
 *                         ����˻�����ɾ�������ڴ�й¶����չ���δ��ʼ��дָ��������BUG��
 * @copyright Copyright (c) Taurus 2022
 * 
 */
#ifndef __UPDATE_BUFFER_H__
#define __UPDATE_BUFFER_H__

#include "stdint.h"

/**
 * @brief ������ָ���������
 */
typedef struct _ubf_t* ubf_t;

/**
 * @brief ����������
 * 
 * @param num ���ݵ�Ԫ����
 * @param size ���ݵ�Ԫռ�ֽ��� 
 * @return ubf_t ������ָ��
 */
ubf_t ubf_create(uint32_t num, uint32_t size);

/**
 * @brief ɾ�����������ͷ��ڴ�
 * 
 * @param ubf ��ɾ��������ָ���ָ��
 */
void ubf_delete(ubf_t* ubf);

/**
 * @brief Ͷι����
 * 
 * @param ubf ������ָ��
 * @param pdata ����ָ��
 * @return uint8_t 
 */
void ubf_push(ubf_t ubf, const void* pdata);

/**
 * @brief ��ȡ k ʱ�̵�����
 * 
 * @param ubf ������ָ��
 * @param k ʱ��������k��[0, num-1]
 * @return void* ��������ָ��
 */
void* ubf_pop(ubf_t ubf, uint32_t k);

/**
 * @brief ��ȡ�������е�ǰ��������
 * 
 * @param ubf ������ָ��
 * @return uint32_t �������е�ǰ��������
 */
uint32_t ubf_get_stock(ubf_t ubf);

/**
 * @brief ��ջ�����
 * 
 * @param ubf ������ָ��
 */
void ubf_clear(ubf_t ubf);

/**
 * @brief   ȡ���������������ݣ��������ݴ��µ������г����鷵��
 * @notice  ע�⴫�������������ָ��һ��Ҫ��ǰ�����ã�
 *
 * @param   ubf ������ָ��
 * @param   pdata ���������׵�ַ���û�������
 * @param   k ʱ��������k��[0, num-1]
 * @param   num �������ݸ���
 * @return  uint32_t ʵ�ʷ������ݸ���
 */
uint32_t ubf_pop_into_array_new2old(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

/**
 * @brief ȡ���������������ݣ��������ݴӾɵ������г����鷵��
 * @notice  ע�⴫�������������ָ��һ��Ҫ��ǰ�����ã�
 *
 * @param ubf ������ָ��
 * @param pdata ���������׵�ַ���û�������
 * @param k ʱ��������k��[0, num-1]
 * @param num �������ݸ���
 * @return uint32_t ʵ�ʷ������ݸ���
 */
uint32_t ubf_pop_into_array_old2new(ubf_t ubf, void* pdata, uint32_t k, uint32_t num);

/**
 * @brief ʵʱ���ݸ��²�����ƽ���˲�
 * 
 * @param ubf   ������ָ��
 * @param pdata ��������ָ��
 * @param num   ƽ����ĸ
 * @return float 
 */
float ubf_avg_filter(ubf_t ubf, float* pdata, uint32_t num);

/**
 * @brief ���Լ�Ȩ�˲������߽׵�ͨ�˲�����
 * 
 * @param ubf      ������ָ��
 * @param pdata    ��������ָ��
 * @param pweight  Ȩ�������׵�ַ
 * @param num      ������
 * @return float 
 */
float ubf_lin_wei_filter(ubf_t ubf, float* pdata, float* pweight, uint32_t num);

/**
 * @brief          ���Լ�Ȩ��
 * 
 * @param ubf      ������ָ��
 * @param pdata    ��������ָ��
 * @param pweight  Ȩ�������׵�ַ 
 * @param num      ������
 * @return float 
 */
float ubf_lin_wei_gradual_deduction(ubf_t ubf, float* pdata, float* pweight, uint32_t num);

/**
 * @brief          �������Լ�ȨԤ��
 * 
 * @param ubf      ������ָ��
 * @param pdata    ��������ָ��
 * @param pweight  Ȩ�������׵�ַ 
 * @param num      ������
 * @param k        ��ǰ��kʱ�̿�ʼ��֡
 * @param flag     �Ƿ���Ԥ��
 *                      0���ر�Ԥ�⣬���뵱ǰ���ݣ�
 *                      1����Ԥ�⣬�޸ĵ�ǰ����ΪԤ��ֵ
 * @return float   ��ǰ���ݻ�Ԥ����
 */
float ubf_lin_wei_predict(ubf_t ubf, float* pdata, float* pweight, uint32_t num, uint8_t k, uint8_t flag);

#endif
