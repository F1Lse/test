#ifndef __VISION_PREDICT_H__
#define __VISION_PREDICT_H__

#include "stm32f4xx_hal.h"
#include "pid.h"
/* �ͻ��������� */
typedef struct {
    float max;
    float min;
    float now;
    float last;
    int8_t out;  //��������ķ���
} delay_loop_t;
typedef enum {
    vMODE_AUTO        = 0,    /* ���� */
    vMODE_bENERGY     = 1,    /* ���������� */
    vMODE_sENERGY     = 2,    /* С�������� */
    vMODE_ANTISPIN    = 4,    /* ��Զ���뷴С����ģʽ */
} vision_mode_e;
/* �Ӿ�״̬ */
typedef enum {
    vUNAIMING   = 0,    /* û��ʶ��Ŀ�� */
    vAIMING     = 1,    /* ʶ��Ŀ�� */
    vFIRST_LOST = 2     /* �״ζ�ʧĿ�� */
} vsn_output_status_e;

/* �Ӿ����ݰ���Ԫ */
typedef struct {
    float now;
    float kal;
} vsn_data_pkg2_t;

/* �Ӿ����ݰ���Ԫ */
typedef struct {
    float now;
    float last;
    float kal;
} vsn_data_pkg3_t;

/* YAW���Ӿ��㷨�м���� */
typedef struct {
    vsn_data_pkg3_t vsn_agl_err;
    vsn_data_pkg2_t vsn_angular_speed;
    vsn_data_pkg3_t imu_angular_speed;
    vsn_data_pkg2_t angular_speed;
    vsn_data_pkg2_t tangent_speed;
    vsn_data_pkg2_t radial_speed;
    vsn_data_pkg2_t real_speed;
    vsn_data_pkg2_t real_speed_angle;
    vsn_data_pkg2_t predict_distance;
    vsn_data_pkg2_t opposite_line;
    vsn_data_pkg3_t predict_angle;
} vsn_perdict_data_t;
typedef struct {
    vsn_perdict_data_t    yaw;
    vsn_perdict_data_t    pit;
    vsn_data_pkg3_t   pos;      /* װ�װ���̬�� */
    vsn_data_pkg3_t   dis;
    vsn_data_pkg3_t   tof;      /* ����ʱ�� */

    float       period;         /* �Ӿ����� */
    float       test_period;    /* ʵ���Ӿ����� */
    uint8_t     valid_cnt;      /* �����յ���Ч���ݼ���λ */
} vsn_data_t;
/* �Ӿ�ģ��ӿ� */
typedef struct {
    vision_mode_e mode;  /* �Ӿ�ģʽ */
    int rotation_direction; //��ת����
    volatile vsn_output_status_e status;    /* �Ӿ����״̬ */
    volatile uint8_t new_frame_flag;        /* ��һ֡���ݱ�־ */
    volatile uint8_t shoot_enable;          /* ���������־ */
    
    float gimbal_pit_ecd; /* ʱ����ͬ��ʱ��̨pit�Ƕ� */
    float pit_angle_error;
    float pit_predict_angle;
    
    float gimbal_yaw_angle; /* ʱ����ͬ��ʱ��̨yaw�Ƕ� */
    float yaw_angle_error;
    float yaw_predict_angle;
} vsn_output_t;
/* �Ӿ�����״̬ö������ */
typedef enum {
    AIMING = 0,
    UNAIMING = 1,
    TAIL_ERROR = 2,    
    REPEAT_ERROR = 3
} vsn_rx_status_e;

/* �Ӿ������ṹ������ */
typedef struct {
    uint32_t repeat_cnt;    /* �յ��ظ�֡���� */
    vsn_rx_status_e status; /* �Ӿ�����״̬ */
    union {
        uint8_t buff[23];/* �Ӿ������� */
        __packed struct {           /* �Ӿ����ݴ������� */    
            float yaw;              /* �Ӿ����yaw�Ƕ� */
            float pit;              /* �Ӿ����pit�Ƕ� */
            float dis;              /* �Ӿ���Ծ��� */
            float tof;              /* �������ʱ�� */
            float pos;              /* װ�װ���̬�� */
            uint8_t shoot_flag;     /*Ӣ�ۻ���λ*/ 
            uint8_t cnt     :6;     /* �Ӿ��෢���Լ�λ */
            uint8_t ist_flag:1;     /* �Ӿ��࿪ʼ��֡ */
            uint8_t aim_flag:1;     /* �Ӿ�ʶ��Ŀ�� */
            uint8_t eof;            /* �Ӿ�����֡֡β */
        } data;                     /* �Ӿ����� */
    } rx[2];
} vsn_drive_t;
extern vsn_output_t  vision; /* �Ӿ�ģ�����ӿ� */
extern vsn_data_t    vd;     /* �Ӿ�ԭʼ����������м����(����) */
static void vsn_deinit(void);
static void vsn_auto_aiming_calc(void);
static void vsn_predict_angle_calc(void);
static void vsn_shoot_enable(void) ;
static float vsn_gradual_deduction(float array[6]) ;
void vsn_init(void);
int rotation_direction_judge(void);
void vsn_gimbal_ref_calc(void);
void vsn_calc(void);
#endif
