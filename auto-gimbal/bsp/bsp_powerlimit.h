#ifndef __BSP_POWERLIMIT_H
#define __BSP_POWERLIMIT_H

#include "stdint.h"
#include "pid.h"

#define POWERLIMIT_CNT			    50

#define SUPERCAP_DISCHAGER_VOLAGE	12.5f		//�����������ŵ��ѹ С�ڸõ�ѹ ���������������

typedef struct
{
    float       judge_power;			//����ϵͳ���������ĵ���ʵʱ����
    uint16_t    judge_power_buffer;     //����ϵͳ���������ĵ��̻�������

    float       chassis_power;	        //ʵ�ʵ��̹��� 1kHz
    float       supply_power;		    //��Դ�������
    float       max_power;			    //���޹���
    float		power_buffer;		    //��ǰ��������ʣ��ֵ
    float       limit_kp;				//���Ʊ���
    float       limit_temp;		        //���Ʊ���
    uint8_t     status;			        //���Ʊ�־λ��1ʱ���Ƶ�����0ʱ������
    uint8_t     cnt;					//������Ϣ�Լ�λ���ж���Ϣ��������
} powercontrol_t;

typedef struct
{
    uint8_t mode;				//�������ݳ��ģʽλ 0Ϊ���� ����粻�ŵ� 1Ϊֻ�䲻�� 2Ϊ�߳�߷�  3Ϊֻ�ŵ�
    float volage;				//�������ݵ�ѹ ��ӳ������������
    float power;
    uint8_t volume_percent;		//�������������ٷֱ� ����UI��ʾ
    float charge_power_fdb;		//��ǰ�������ݳ�繦�ʵķ���ֵ �ɵ�Դ����-���̹��ʵó� ������ȫ��ȷ
    float charge_power_ref;		//�������ݳ�繦��Ŀ��ֵ
    float charge_current_set;	//�������ݳ������������������ݰ�,���������
} supercap_t;

extern powercontrol_t powercontrol;
extern supercap_t supercap;
extern uint8_t supercap_status_flag;

void Power_Control(int16_t * current);
void PowerControl_Init(void);
void PowerParam_Update(void);
void SuperCap_Control(void);	//��������״̬���º���
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data);
void can1_send_supercap(void);
void SuperCap_Mode_Update(void);


#endif
