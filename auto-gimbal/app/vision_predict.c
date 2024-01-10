#include "gimbal_task.h"
#include "chassis_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "func_generator.h"
#include "usb_task.h"
#include "protocol_camp.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_can.h"
#include "bsp_JY901.h"
#include "bsp_T_imu.h"
#include "ubf.h"
#include "us_tim.h"
#include "vision_predict.h"
#include "shoot_task.h"
/* Բ���� */
#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ����ֵ */
#define ABSv(x) ( (x>0)? (x): (-(x)) )

/* ���� */
#ifndef SIGN
#define SIGN(x) ( (x)>0? (1): ( (x)<0? (-1): 0 ) )
#endif

/* ����޷� */
#ifndef OUTPUT_LIMIT
#define OUTPUT_LIMIT(output,max,min) \
    ( (output) <= (max) && (output)>=(min)? output: \
    ( (output) > (max)? (output = max):(output = min)) )
#endif
extern vision_ctrl_info_t  vision_ctrl;//�Զ���������
vsn_drive_t vsn_drive;
vsn_output_t  vision;
void vsn_calc(void) {
    /* ��������ͳһ������ͳһ */
    vd.pit.vsn_agl_err.now   = vision_ctrl.pit;            /* �Ӿ�PIT��ԽǶȣ��㣩 */
//    vd.yaw.vsn_agl_err.now   = vision_ctrl.yaw;            /* �Ӿ�YAW��ԽǶȣ��㣩 */
    vd.yaw.vsn_agl_err.now   =  gimbal.yaw_imu_offset;            /* �Ӿ�YAW��ԽǶȣ��㣩 �����޸ģ�Ϊ�˴򵼺�ͷ����*/
    vd.dis.now               = vision_ctrl.dis * 1.0e-3f;  /* ���루m�� */
    vd.tof.now               = vision_ctrl.tof;    /* ȥ���Ӿ���ӵ�40ms��ͨ����ʱ(s) */
//    vd.pos.now               = imu_data.yaw
//                             + vision_ctrl.yaw;            /* �ӵ������ */
	    vd.pos.now               = imu_data.yaw
                             + gimbal.yaw_imu_offset;            /* �ӵ������ �����޸ģ�Ϊ�˴򵼺�ͷ����*/
	
    
	    if (vision_ctrl.aiming_flag) {
          vsn_drive.status = AIMING;
      } else {
           vsn_drive.status = UNAIMING;
        }
    /* ��ȡ�Ӿ��������� */
    vd.period = 7.5f * 1.0e-3f;  /* �̶��Ӿ����ڣ�s�� */
    /* �����Ӿ�ģʽ�޸Ĳ��� */
   // vsn_mode_switch_init();
    
    /* �Ӿ�Ԥ������ */
    if (vd.valid_cnt && vsn_drive.status != AIMING) {  /* �״ζ�֡���쳣������׹�޽� */
        vd.valid_cnt = 0;
        vsn_deinit();                 /* �Ӿ���ʷ������� */
        vision.status = vFIRST_LOST;  /* ���Ϊ�״ζ�֡״̬ */
    } else if (vsn_drive.status == AIMING) {  /* ʶ��Ŀ�� */
        ++vd.valid_cnt;  /* ��Ч���ݼ��� */
        vsn_auto_aiming_calc();  /*������������*/
        vsn_predict_angle_calc();  /*Ԥ��Ƕȼ���*/
        vsn_shoot_enable();  /*��ǹʱ����Ƶ����*/
        vision.new_frame_flag = 1;  /* ��עΪ����Ч����*/
        vision.status = vAIMING;  /* ���Ϊʶ��Ŀ��״̬ */
//        if (ABS(vd.pit.vsn_agl_err.now - 45.0f) < 1e-5f) {  /* ������׹�޽� */
//            vision.status = vUNAIMING;  /* ��ʱҲ����δʶ�� */
//        }
    }
		else {  /* δʶ��Ŀ�� */
        vision.status = vUNAIMING;  /* ���Ϊ��ʧĿ��ģʽ */
			vsn_auto_aiming_calc();  /*������������*/
			vision.shoot_enable=0;
    }
		    /* ���ݷ������ */
//    float* pvsn_data = NULL;
//    switch (vsn_std_switch) {
//        /* �Ӿ�ԭʼ���� */
//        case 0: pvsn_data = &vd.pit.vsn_agl_err.now; break;
//        case 1: pvsn_data = &vd.yaw.vsn_agl_err.now; break;
//        case 2: pvsn_data = &vd.dis.now; break;
//        case 3: pvsn_data = &vd.tof.now; break;
//        case 4: pvsn_data = &vd.pos.now; break;
//        /* �м����� */
//        case 5: pvsn_data = &vd.yaw.angular_speed.now; break;
//        case 6: pvsn_data = &vd.yaw.radial_speed.kal; break;
//        default: break;
//    }
//    ubf_push(ubf_var, pvsn_data);
//    float vsn_data_array[100] = {0}, var;
//    uint32_t real_num = ubf_pop_into_array_new2old(ubf_var, vsn_data_array, 0, 100);
//    arm_var_f32(vsn_data_array, real_num, &var);  /* �������ݷ��� */
//    std = sqrtf(var);

    /* ֱ�ӽ����Ӿ����� */
//    vision.new_frame_flag   = 1;                                         /* ��עΪ����Ч���� */
//    vision.pit_angle_error  = vd.pit.now;                                /* PIT�Ƕ�ƫ�� */
//    vision.yaw_angle_error  = vd.yaw.vsn_agl_err.now;                    /* YAW��Ƕ�ƫ�� */
//    vision.yaw_predict_angle= 0;                                         /* YAW��Ƕ�ƫ��Ԥ�� */
//    vision.gimbal_pit_ecd = ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->pit;
//    vision.gimbal_yaw_angle = ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->yaw;
}



/* ���� */
/*�Ӿ��˲�*/
/* ���������˲�����λƥ�� (63 - 30)(ms) */
float   kal_R_dis_tof       = 4.00f;    /* Ŀ����뿨���� R */

/* �Ӿ��˲�������ʱ�� */
uint8_t stable_time_flt     = 7;        /* �Ӿ��˲�������ʱ�䣨ms�� */

/* Ԥ������˲� */
float   kal_R_predict_angle = 1.00f;    /* Ԥ��ǿ����� R */

/* �������ϸ�� */
float   w_start_deadzone    = 0.00f;    /* ���Խ��ٶ����� */
float predict_agl_limit     = 15.0f;    /* Ԥ��Ƕ��޷� */
/* ================================== TEST PARAM ================================== */
float test_vision_predict_kp1 = 1.0f;//0.8f
float test_vision_predict_kp2 = 0.2f;
float test_vision_predict_error_kp = 1.7f;
float test_vision_angle_error_pit_kp = 1.3f;
float test_vision_angle_error_yaw_kp = 1.5f;
static ubf_t ubf_vd_pit;
static ubf_t ubf_vd_yaw;
static ubf_t ubf_vd_dis;
static ubf_t ubf_vd_tof;
/* ----------------------------------------------------------------- */
/*                        �Ӿ��������˲���                           */
/* ----------------------------------------------------------------- */
static Kalman1_param_t kal_pit_agl_err; /* �Ӿ�PIT��ԽǶ� */
static Kalman1_param_t kal_yaw_agl_err; /* �Ӿ�YAW��ԽǶ� */
static Kalman1_param_t kal_yaw_imu_spd; /* �����Ǽ��ٶ� */
static Kalman1_param_t kal_yaw_vsn_spd; /* װ�װ���Խ��ٶ� */
static Kalman1_param_t kal_pit_imu_spd; /* �����Ǽ��ٶ� */
static Kalman1_param_t kal_pit_vsn_spd; /* װ�װ���Խ��ٶ� */
static Kalman1_param_t kal_tof;         /* �Ӿ�������ӵ�����ʱ�� */
static Kalman1_param_t kal_dis;         /* �Ӿ���Ծ��� */
static Kalman1_param_t kal_yaw_predict_agl; /* YAWԤ��Ƕ� */
static Kalman1_param_t kal_pit_predict_agl; /* PITԤ��Ƕ� */

/* ----------------------------------------------------------------- */
/*                           �Ӿ�����                                */
/* ----------------------------------------------------------------- */
/* �Ƕ���λƥ�䣨�Ӿ������Լ�ͨ����ʱ����ֵƥ�����Ӿ�����ɣ� */
uint32_t delay_auto_pit_ecd   = 35;     /* ����Ƕ���λƥ��ʱ�� */ 
uint32_t delay_auto_yaw_imu   = 30;     /* ����Ƕ���λƥ��ʱ�� */ //24
uint32_t delay_energy_pit_ecd = 80;     /* �������ؽǶ���λƥ��ʱ�� */
uint32_t delay_energy_yaw_imu = 80;     /* �������ؽǶ���λƥ��ʱ�� */

/* ���ٶȷ���ƥ�� */
float   kal_R_yaw_agl       = 2.00f;    /* �Ӿ��Ƕȿ����� R */
float   kal_R_yaw_aim_spd   = 2.00f;    /* �Ӿ�YAW��Խ��ٶȿ����� R */
float   kal_R_yaw_imu_spd   = 3.20f;    /* IMU YAW���ٶȿ����� R */
uint8_t delay_imu_wz        = 30;       /* ���ٶ���λƥ��ʱ�� */

float   kal_R_pit_agl       = 3.00f;    /* �Ӿ��Ƕȿ����� R */
float   kal_R_pit_aim_spd   = 10.00f;   /* �Ӿ�PIT��Խ��ٶȿ����� R */
float   kal_R_pit_imu_spd   = 10.00f;   /* IMU PIT���ٶȿ����� R */
uint8_t delay_imu_wy        = 41;       /* ���ٶ���λƥ��ʱ�� */
 vsn_data_t   vd;        /* �Ӿ����ݽṹ�� */
 vsn_output_t vision_predict;    /* �Ӿ�����������ṹ�� */
 /* ��ز�Ԥ������ϵͳ */
float kp1 = 1.6f;
float kp2 = 0.0f;
float kp3 = 0.7f;//0 0 1.8
float shoot_enable_test = 0.5f;

delay_loop_t dlp = {
    .max = 1.8f,
    .min = 1.3f
};
 
 /* ----------------------------------------------------------------- */
/*                           �ֲ�ȫ�ֱ���                            */
/* ----------------------------------------------------------------- */
/* Ԥ����ϵ�� */
float   predict_kp_yaw;     /* 1.8f  0.7f */
float   predict_kp_pit;
/* ����Ƕ�ƥ��ʱ�� */
uint8_t match_time_pit=35;   
uint8_t match_time_yaw=30;
/* ================================== TEST PARAM ================================== */
void vsn_init(void) {
 vision_mode_e last_vision_mode = vision.mode;
    memset(&vision,     0, sizeof(vsn_output_t));
    memset(&vsn_drive,  0, sizeof(vsn_drive_t));
    memset(&vd,         0, sizeof(vsn_data_t));
    vision.mode = last_vision_mode;
    
    /* һ�׿������˲�����ʼ�� */
    Kalman1FilterCreate(&kal_pit_agl_err,   1, kal_R_pit_agl);
    Kalman1FilterCreate(&kal_yaw_agl_err,   1, kal_R_pit_agl);
    
    Kalman1FilterCreate(&kal_yaw_imu_spd,   1, kal_R_yaw_imu_spd);
    Kalman1FilterCreate(&kal_yaw_vsn_spd,   1, kal_R_yaw_aim_spd);

    Kalman1FilterCreate(&kal_pit_imu_spd,   1, kal_R_pit_imu_spd);
    Kalman1FilterCreate(&kal_pit_vsn_spd,   1, kal_R_pit_aim_spd);
    
    Kalman1FilterCreate(&kal_tof,           1, kal_R_dis_tof);
    Kalman1FilterCreate(&kal_dis,           1, kal_R_dis_tof);
    
    Kalman1FilterCreate(&kal_yaw_predict_agl,   1, kal_R_predict_angle);
    Kalman1FilterCreate(&kal_pit_predict_agl,   1, kal_R_predict_angle);

    /* ���ݻ����� */
    gim_msg_ubf= ubf_create(100, sizeof(gim_msg_t));  /* ��̨��ʷ��̬��Ϣ */
    ubf_vd_pit = ubf_create(10,  sizeof(float));    /* �Ӿ��˲�����ʷ���� */
    ubf_vd_yaw = ubf_create(10,  sizeof(float));    /* �Ӿ��˲�����ʷ���� */
    ubf_vd_dis = ubf_create(10,  sizeof(float));    /* �Ӿ��˲�����ʷ���� */
    ubf_vd_tof = ubf_create(10,  sizeof(float));    /* �Ӿ��˲�����ʷ���� */
}
static void vsn_deinit(void) {
		
	    /* ������� */
    vision_mode_e last_vision_mode = vision.mode;
    //memset(&vision_ctrl, 0, sizeof(vision_ctrl_info_t));
    memset(&vd,          0, sizeof(vsn_data_t));
    memset(&vision,      0, sizeof(vsn_output_t));
    vision.mode = last_vision_mode;
    /* ����Ӿ���ʷ���ݻ��� */
    ubf_clear(ubf_vd_pit);
    ubf_clear(ubf_vd_yaw);
		ubf_clear(ubf_vd_dis);
    ubf_clear(ubf_vd_tof);
	    /* ��ʼ���������˲��� */
    Kalman1FilterDeinit(&kal_pit_agl_err);
    Kalman1FilterDeinit(&kal_yaw_agl_err);
    
    Kalman1FilterDeinit(&kal_yaw_imu_spd);
    Kalman1FilterDeinit(&kal_yaw_vsn_spd);
    
    Kalman1FilterDeinit(&kal_pit_imu_spd);
    Kalman1FilterDeinit(&kal_pit_vsn_spd);
    
    Kalman1FilterDeinit(&kal_tof);
    Kalman1FilterDeinit(&kal_dis);
    
    Kalman1FilterDeinit(&kal_yaw_predict_agl);
    Kalman1FilterDeinit(&kal_pit_predict_agl);
}
static void vsn_kal_reinit(void) {//��̬��������������
    Kalman1FilterReinit(&kal_pit_agl_err, 1, kal_R_pit_agl);
    Kalman1FilterReinit(&kal_yaw_agl_err, 1, kal_R_yaw_agl);
	
		Kalman1FilterReinit(&kal_yaw_imu_spd, 1, kal_R_yaw_imu_spd);
    Kalman1FilterReinit(&kal_yaw_vsn_spd, 1, kal_R_yaw_aim_spd);
    
    Kalman1FilterReinit(&kal_pit_imu_spd, 1, kal_R_pit_imu_spd);
    Kalman1FilterReinit(&kal_pit_vsn_spd, 1, kal_R_pit_aim_spd);

    Kalman1FilterReinit(&kal_tof,         1, kal_R_dis_tof);
    Kalman1FilterReinit(&kal_dis,         1, kal_R_dis_tof);
    
    Kalman1FilterReinit(&kal_yaw_predict_agl, 1, kal_R_predict_angle);
    Kalman1FilterReinit(&kal_pit_predict_agl, 1, kal_R_predict_angle);
}
static void vsn_auto_aiming_calc(void) {
    /* �������Ԥ����������� */
    vision.pit_angle_error  = vd.pit.vsn_agl_err.now;  /* PIT�Ӿ���ԽǶ�ƫ�� */
    vision.yaw_angle_error  = vd.yaw.vsn_agl_err.now;  /* YAW�Ӿ���ԽǶ�ƫ�� */
    vision.gimbal_pit_ecd   = ((gim_msg_t*)ubf_pop(gim_msg_ubf, match_time_pit))->pit;  /* ��λƥ������̨PIT�Ƕ� */
    vision.gimbal_yaw_angle = ((gim_msg_t*)ubf_pop(gim_msg_ubf, match_time_yaw))->yaw;  /* ��λƥ������̨YAW�Ƕ� */
}
static void vsn_predict_angle_calc(void) {
 /* -------------------------- �Ӿ�ԭʼ�����˲��뱣�� -------------------------- */
    vsn_kal_reinit();  /*<! ������������R�������� */

    vd.pit.vsn_agl_err.kal = Kalman1FilterCalc(&kal_pit_agl_err, vd.pit.vsn_agl_err.now);/* �Ӿ�PIT��Ƕȣ��㣩�仯��ֵ Ҫ��IMU��YAWƥ�� */
    vd.yaw.vsn_agl_err.kal = Kalman1FilterCalc(&kal_yaw_agl_err, vd.yaw.vsn_agl_err.now);/* �Ӿ�YAW��Ƕȣ��㣩�仯��ֵ Ҫ��IMU��YAWƥ�� */
    vd.dis.kal = Kalman1FilterCalc(&kal_dis, vd.dis.now);  /* �Ӿ�ֱ�߾��루m�� */
    vd.tof.kal = Kalman1FilterCalc(&kal_tof, vd.tof.now);  /* �������ʱ�䣨s�� */
    
    ubf_push(ubf_vd_pit, &vd.pit.vsn_agl_err.kal);
    ubf_push(ubf_vd_yaw, &vd.yaw.vsn_agl_err.kal);
    ubf_push(ubf_vd_dis, &vd.dis.kal);
    ubf_push(ubf_vd_tof, &vd.tof.kal);
    
    /* -------------------------- ����뿪ʼԤ�� -------------------------- */
    if (vd.valid_cnt > stable_time_flt) {  /* �ȴ��˲�������*/ 
        /* �Ӿ���Խ��ٶȣ���/s�� */
        float vsn_yaw2diff[6] = {0};
        ubf_pop_into_array_new2old(ubf_vd_yaw, vsn_yaw2diff, 0, 6);
        vd.yaw.vsn_angular_speed.now = vsn_gradual_deduction(vsn_yaw2diff);
        vd.yaw.vsn_angular_speed.kal = Kalman1FilterCalc(&kal_yaw_vsn_spd, vd.yaw.vsn_angular_speed.now);
        
        float vsn_pit2diff[6] = {0};
        ubf_pop_into_array_new2old(ubf_vd_pit, vsn_pit2diff, 0, 6);
        vd.pit.vsn_angular_speed.now = vsn_gradual_deduction(vsn_pit2diff);
        vd.pit.vsn_angular_speed.kal = Kalman1FilterCalc(&kal_pit_vsn_spd, vd.pit.vsn_angular_speed.now);
        
        /* ��������Խ��ٶ� LSB->(��/s) ���Ӿ���Խ��ٶ���λƥ��*/ //0
        vd.yaw.imu_angular_speed.now = ((gim_msg_t*)ubf_pop(gim_msg_ubf, 0))->wz/16.3835f;  /* �������ݵ����� */
        vd.yaw.imu_angular_speed.last= ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wz))->wz/16.3835f;  /* ���ٶ���λƥ�� */
        vd.yaw.imu_angular_speed.kal = Kalman1FilterCalc(&kal_yaw_imu_spd, vd.yaw.imu_angular_speed.last);
        
        vd.pit.imu_angular_speed.now = ((gim_msg_t*)ubf_pop(gim_msg_ubf, 0))->wy/16.3835f; /* �������ݵ����� */
        vd.pit.imu_angular_speed.last= ((gim_msg_t*)ubf_pop(gim_msg_ubf, delay_imu_wy))->wy/16.3835f;  /* ���ٶ���λƥ�� */
        vd.pit.imu_angular_speed.kal = Kalman1FilterCalc(&kal_pit_imu_spd, vd.pit.imu_angular_speed.last);
        
        /* ���Խ��ٶȣ���/s�������˲�  */
        vd.yaw.angular_speed.now = vd.yaw.vsn_angular_speed.kal + vd.yaw.imu_angular_speed.kal;    
        vd.pit.angular_speed.now = vd.pit.vsn_angular_speed.kal + vd.pit.imu_angular_speed.kal;
        
        /* ���Խ��ٶ��������� */
        if (ABS(vd.yaw.angular_speed.now) < w_start_deadzone) {
            vd.yaw.angular_speed.now = 0;
        }
        
        /* �Ӿ�Ԥ����� */
        vd.pit.predict_angle.now = vd.pit.angular_speed.now * vd.tof.kal * predict_kp_pit;
        vd.yaw.predict_angle.now = vd.yaw.angular_speed.now * vd.tof.kal * predict_kp_yaw;
                vd.yaw.predict_angle.now = vd.yaw.angular_speed.now * vd.tof.kal * predict_kp_yaw;
        vd.yaw.predict_angle.kal = Kalman1FilterCalc(&kal_yaw_predict_agl, vd.yaw.predict_angle.now);
        vd.pit.predict_angle.kal = Kalman1FilterCalc(&kal_pit_predict_agl, vd.pit.predict_angle.now);
        
        /* ���Ԥ��Ƕ����� */
        OUTPUT_LIMIT(vd.yaw.predict_angle.kal, predict_agl_limit, -predict_agl_limit);
        OUTPUT_LIMIT(vd.pit.predict_angle.kal, predict_agl_limit, -predict_agl_limit);
        
        vision.yaw_predict_angle = vd.yaw.predict_angle.kal;
        vision.pit_predict_angle = vd.pit.predict_angle.kal;
        
        /* Ŀ���˶������ж� */
        vision.rotation_direction = rotation_direction_judge();  
    }
}

/** 
 * @brief �Ӿ���̨�趨ֵ���㺯��
 */
void vsn_gimbal_ref_calc(void) {
    switch (vision.status) {
        case vAIMING: {  /* ʶ��Ŀ�� */
            if (vision.new_frame_flag) { /* ���յ��µ����� */
                /* �����־����ֹ�ظ�����ͬһ֡ */
                vision.new_frame_flag = 0;
                /* �޸���̨�趨ֵ */
//                 gimbal.pid.pit_ecd_ref = vision.pit_angle_error+gimbal.pid.pit_ecd_fdb;
//                gimbal.pid.yaw_angle_ref = vision.gimbal_yaw_angle+vision.yaw_angle_error+vision.yaw_predict_angle;
                gimbal.pid.yaw_angle_ref= vision.yaw_angle_error;//�Ӿ����ԽǶ�
                gimbal.pid.pit_ecd_ref  = vision.pit_angle_error;
                
//                gimbal.pid.pit_ecd_ref = vision.gimbal_pit_ecd + vision.pit_angle_error * 8191.0f / 360;
//                gimbal.pid.yaw_angle_ref = vision.gimbal_yaw_angle + vision.yaw_angle_error;
                
							// gimbal.pid.pit_ecd_ref  = vision.gimbal_pit_ecd + ((vision.pit_angle_error + vision.pit_predict_angle) * 8191.0f / 360.0f);
            }
            break;
        }
        case vFIRST_LOST: {  /* �״ζ�ʧĿ�� */
            vision.status = vUNAIMING;
          //  gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
             gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb;
            gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
            break;
        }
        case vUNAIMING: {  /* δʶ��Ŀ�� */
            if (ctrl_mode == REMOTER_MODE) {
               // gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
                  gimbal.pid.pit_ecd_ref  +=rc.ch2 * 0.0008f;
                gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
            } else {
                gimbal.pid.yaw_angle_ref= vision.yaw_angle_error;//�ɵ�������
                gimbal.pid.pit_ecd_ref  = vision.pit_angle_error;
            }
            break;
        }
        default: {break;}
    }
}

int cha_pitch;
int cha_yaw;
/* ��ǹʱ������Ƶ���� */
static void vsn_shoot_enable(void) {
		switch (vision.mode) {
        case vMODE_AUTO:
        case vMODE_ANTISPIN: {
            if (ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb) <10.0f && 
                ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb) < 5.0f) {
                vision.shoot_enable = 1;
            } else {
                vision.shoot_enable = 0;
            }
						 cha_pitch=ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb);
					 	cha_yaw=ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb);
					//vision.shoot_enable = 1;
//						if(vd.dis.now>6)
//						{
//							shoot.trigger_period=300;//����6m����һ��300msһ��
//						}
//						else
//						{
//							shoot.trigger_period=125;
//						}
						
            break;
        }
        case vMODE_bENERGY:
        case vMODE_sENERGY: {
            if (ABSv(gimbal.pid.yaw_angle_ref - gimbal.pid.yaw_angle_fdb) < 1.0f && 
                ABSv(gimbal.pid.pit_ecd_ref - gimbal.pid.pit_ecd_fdb) < 1.0f) {
                vision.shoot_enable = 1;
            } else {
                vision.shoot_enable = 0;
            }
            break;
        }
    }
}

/** 
 * @brief �Ӿ���Ȩ��
 */
static float vsn_gradual_deduction(float array[6]) {
    /* �����ȶ���ϸ�����������õ��� */
    float diff_k1 = 0.90f;    /* ��ϵ��1 */ 
    float diff_k2 = 0.06f;    /* ��ϵ��2 */
    float diff_k3 = 0.04f;    /* ��ϵ��3 */
    float error[3];
    for(int i=0; i < 3; i++) {
        error[i] = array[i] - array[3+i];
        if(error[i] > 2.0f ) return 0;
    }  
    return (
        ( diff_k1 * error[0]
        + diff_k2 * error[1]
        + diff_k3 * error[2] ) / (3 * vd.period)
    );
}
float test_threshold = 1.0f;
uint32_t test_cnt = 20;
int rotation_direction_judge(void) {
    static uint8_t valid_cnt, invalid_cnt;
    static int8_t dir;
    if(ABS(vd.yaw.angular_speed.now) > test_threshold) {  /* Ŀ�����ƶ� */
        invalid_cnt = 0;
        valid_cnt = 1;
        if (vd.yaw.angular_speed.now > 0) {
            dir = 1;
        } else {
            dir = -1;
        }
    } else {
        if ((valid_cnt && invalid_cnt++ > test_cnt) || valid_cnt == 0) {
            valid_cnt = 0;
            dir = 0;
        }
    }
    return dir;
}
