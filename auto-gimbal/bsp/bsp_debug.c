//#include "bsp_debug.h"
//#include "stdint.h"
//#include "modeswitch_task.h"
//#include "remote_msg.h"
//#include "chassis_task.h"
//#include "status_task.h"
//#include "cmsis_os.h"

//debug_t debug;

//extern osThreadId can_msg_send_task_t;
//extern osThreadId chassis_task_t;
//extern osThreadId gimbal_task_t;
//extern osThreadId mode_sw_task_t;
//extern osThreadId shoot_task_t;
//extern osThreadId status_task_t;
//extern osThreadId judge_send_task_t;
//extern osThreadId usart_msg_send_task_t;
//extern osThreadId debug_task_t;

//static void debug_mode_switch(void);
//static void debug_mode_to_shoot(void);
//static void debug_mode_to_fly(void);
//static void debug_mode_to_vision(void);
//static void debug_mode_to_one_shoot(void);

//static void debug_chassis_control(void);
//static void debug_gimbal_control(void);
//static void debug_shoot_control(void);

//void debug_init(void)
//{
//    debug.mode = eDEBUG_STOP;
//    status.debug_status = 0;
//}

//void debug_task(void const *argu)
//{
//    uint32_t period = osKernelSysTick();
//for(;;)
//{
//    /* ���� */
//    if( !lock_flag )    unlock_init();  //��������
//
//    /* ɾ�������������� */
//
////    osThreadTerminate(chassis_task_t);
////    osThreadTerminate(gimbal_task_t);
////    osThreadTerminate(shoot_task_t);
//
//    /* ��Ч���� */
//    status.debug_status = 1;
//
//    /* ��Ƶ */
//    static uint8_t cnt = 0;
//    if(cnt++>=10)     cnt = 0;
//
//    /* ����ģʽ�л� */
//    debug_mode_switch();
//
//    /* ���̿��� */
//    if( cnt % 2 == 0 )
//        debug_chassis_control();
//    /* ��̨���� */
//    if( cnt % 1 == 0 )
//        debug_gimbal_control();
//    /* ���������� */
//    if( cnt % 1 == 0 )
//        debug_shoot_control();
//
//    osDelayUntil(&period, 1);
//}
//}
//static void debug_mode_switch(void)
//{
//    debug_mode_to_vision();
//
//    static uint8_t ch_enable = 1;
//    if( rc.ch5 == 0 )   ch_enable = 1;
//    if( ctrl_mode == PROTECT_MODE && ch_enable && rc.ch5 == 660 )  //����ģʽ���л�
//    {
//
//    }
//    else if( ctrl_mode == PROTECT_MODE && \
//             ch_enable && rc.ch5 == 660 )
//    {
//        ch_enable = 0;
//        ctrl_mode = PROTECT_MODE;
//        debug.mode = eDEBUG_STOP;
//    }
//}

//static void debug_chassis_control(void)
//{

//}

//static void debug_gimbal_control(void)
//{

//}

//static void debug_shoot_control(void)
//{

//}


//static void debug_mode_to_shoot(void)
//{
//    debug.mode = eDEBUG_SHOOT;
//}


//static void debug_mode_to_fly(void)
//{
//    debug.mode = eDEBUG_FLY;
//    scale.ch3 = RC_CH3_SCALE_DEBUG;
//    scale.ch4 = RC_CH4_SCALE_DEBUG;
//    chassis.wheel_max = 8500;  /* �ص����ʿ���ʱ��Ҫ��ʼ�� */
//}

//static void debug_mode_to_vision(void)
//{
//    debug.mode = eDEBUG_VISION;
//}
