/**
	* @file bsp_T_imu.c
	* @version 1.0
	* @date 2020.1.4
  *
  * @brief  Taurus�������ڲ��
  *
  *	@author YY
  *
  */

#include "bsp_T_imu.h"
#include "bsp_can.h"
#include "gimbal_task.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "status_task.h"

float 	palstance_buffer[2];
float 	angle_buffer[2];

Taurus_imu_data_t   imu_data;
void T_imu_calcu(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    switch(can_id)
    {
    case TIMU_PALSTANCE_ID:	//���ٶ�
    {
        memcpy(palstance_buffer,CAN_Rx_data,8);
        imu_data.wy = palstance_buffer[0];  //��������ˮƽ��ת180��ʱ��Ҫ��-��
        imu_data.wz = palstance_buffer[1];
    }
    break;
    case TIMU_ANGLE_ID:			//�Ƕ�
    {
        memcpy(angle_buffer,CAN_Rx_data,8);
        imu_data.pitch = angle_buffer[0];  //��������ˮƽ��ת180��ʱ��Ҫ��-��
        imu_data.yaw   = angle_buffer[1];
    }
    break;
    }
}
