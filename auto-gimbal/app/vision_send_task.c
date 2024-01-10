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
#include "vision_send_task.h"
extern chassis_odom_info_t chassis_odom;

void vision_send_task(void const *argu) {
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;) {
        taskENTER_CRITICAL();
        
			/*��������������*/
		     gim_msg_t temp_gim_msg = {
            .pit = moto_pit.ecd, //��ز�PIT�Ƕ���λƥ��Ҫ�ñ���ֵ
            .yaw = imu_data.yaw,
            .wy  = imu_data.wy,
            .wz  = imu_data.wz,
        };
        ubf_push(gim_msg_ubf, &temp_gim_msg);//��λƥ������
				/*�Ӿ���̨��Ϣ*/
				chassis_odom.gimbal_pitch_imu    = imu_data.pitch;           //��λ���� �Ӿ���PIT�Ƕ���������Ҫ��IMU
        chassis_odom.gimbal_yaw_imu     = imu_data.yaw;             //��λ����
        chassis_odom.gimbal_pitch_rate_imu  = -imu_data.wy / 16.3835f;  //��λ����/s
        chassis_odom.gimbal_yaw_rate_imu = imu_data.wz / 16.3835f;   //��λ����/s
        taskEXIT_CRITICAL();
        
        /* ����1ms�������� */
        osDelayUntil(&thread_wake_time, 1); 
    }
}
