#define __SHOOT_TASK_GLOBALS

#include "shoot_task.h"
#include "control_def.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "remote_msg.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "protocol_camp.h"
extern vision_ctrl_info_t  vision_ctrl;
static void ShootParam_Update(void);
static void shoot_mode_sw(void);
static void house_init(void);
static void house_control(void);
fric_t fric;
void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
		shoot_init();
    for(;;)
    {
			taskENTER_CRITICAL();
			/* �����ʼ�� */
				  static uint8_t last_fric_enable, fric_enable;
        fric_enable = !!Game_Robot_Status.mains_power_shooter_output;
        if (fric_enable && !last_fric_enable) {
            fric.init_cnt = 0;
            fric.init_flag = 0;
						fric.acc_flag =0;
        }
				last_fric_enable = fric_enable;
				 FricMotor_init();   
        shoot_mode_sw();  /* ������ģʽ�л� */
        FricMotor_Control();	/* Ħ���ֵ������ */
        TriggerMotor_control();	/* ����������� */
        house_control();         /* ���ոǿ��� */
				taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

int flag=0;
int cnt=0;
void shoot_init(void)
{
    /* �������ײ��ʼ�� */
    FricMotor_init();  //Ħ���ֳ�ʼ�� �������п��Ƴ�ʼ��
    TriggerMotor_init();//���̳�ʼ��
		house_init();       ////���ճ�ʼ��
	
    shoot.firc_mode     = FIRC_MODE_STOP;
    shoot.stir_mode     = STIR_MODE_PROTECT;
    shoot.house_mode    = HOUSE_MODE_PROTECT;  
	/* ǹ�ܲ�����ʼ�� */
    shoot.trigger_period = TRIGGER_PERIOD;
	 shoot.barrel.cooling_rate   = 25;
    shoot.barrel.heat_max       = 240;
    shoot.shoot_speed           = 30;
}

static void shoot_mode_sw(void)
{
    /* ϵͳ��ʷ״̬�� */
    static ctrl_mode_e last_ctrl_mode = PROTECT_MODE;
    
   /* ���²���ϵͳ���� */
	 
    ShootParam_Update();
	//�õ���������Ƶ
		if(vision_ctrl.shoot_cmd!=0&&vision_ctrl.shoot_cmd>0)
		{			
				if(vision_ctrl.shoot_cmd>25)//���25hz
				{
					shoot.trigger_period=40;
				}
				else
				{
				shoot.trigger_period=(1/vision_ctrl.shoot_cmd)*1000;//�Ӿ�ֱ�ӷ�hz
				}
		}
		else 
		{
				shoot.trigger_period=TRIGGER_PERIOD;	
		}


    /* ģʽ�л� */
    switch( ctrl_mode )
    {
        case PROTECT_MODE:
        {
            shoot.firc_mode = FIRC_MODE_STOP;
            shoot.stir_mode = STIR_MODE_PROTECT;
            shoot.house_mode= HOUSE_MODE_PROTECT;
        }
        break;
        case REMOTER_MODE:
				case AUTO_MODE:
        {
           /* Ħ���ֺͲ���ģʽ�л� */
            switch( rc.sw2 )
            {
                case RC_UP:
                {
                    //LASER_UP;
                    LASER_DOWN;
                    shoot.firc_mode = FIRC_MODE_STOP;
                    shoot.stir_mode = STIR_MODE_STOP;
                }
                break;
                case RC_MI:
                {
                    LASER_UP;
									   if (fric.init_flag) {
                        shoot.firc_mode= FIRC_MODE_RUN;  //����Ħ����
                    }
                    shoot.stir_mode = STIR_MODE_STOP;
										//shoot.stir_mode = STIR_MODE_SINGLE;  //�������ⵯ���ã�
                }
                break;
                case RC_DN:
                {
                    LASER_UP;
										if (fric.init_flag) {
                        shoot.firc_mode = FIRC_MODE_RUN;  // //����Ħ����
                    }
										 shoot.stir_mode = STIR_MODE_SERIES; //����
										  if (!fric.acc_flag) {
                        shoot.stir_mode = STIR_MODE_STOP;//δ��ɼ��ٲ�������
                    }
                }
                break;
                default: break;
            }
             /* ���ո�ģʽ�л� */
            static uint8_t house_switch_enable = 1;
            if( last_ctrl_mode != REMOTER_MODE )
                shoot.house_mode = HOUSE_MODE_CLOSE;
            if( rc.ch5 == 0 )   house_switch_enable = 1;
            if( house_switch_enable && rc.ch5 == -660 )  //切换弹舱开关状态标志位
            {
                house_switch_enable = 0;
                shoot.house_mode = (shoot_house_mode_e) (!(uint8_t)shoot.house_mode);  //开关弹舱盖
            }
        }
        break;
        default: break;
    }
   /* ��ʷ״̬���� */
    last_ctrl_mode = ctrl_mode;
}

/* ����������ϵͳ���ݸ��� */
static void ShootParam_Update(void)
{
   /* ���²���ϵͳ���� */
    if( Game_Robot_Status.shooter_id1_17mm_speed_limit != 0 )
    {
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;  //
			shoot.barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;  //
        shoot.barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate;  //
    }
     /* ���� ģ�����ϵͳ ���� */
    shoot.barrel.heat -= shoot.barrel.cooling_rate * SHOOT_PERIOD * 0.001f;  //
    if( shoot.barrel.heat < 0 )	  shoot.barrel.heat = 0;
    shoot.barrel.heat_remain = shoot.barrel.heat_max - shoot.barrel.heat;  //
//		shoot.barrel.heat_remain = shoot.barrel.heat_max ;  //��������������
}

static void house_init(void)
{
    //HAL_TIM_PWM_Start(Magazine_Time_CH);
	Magazine_PWM = COVER_PWM_CLOSE;
}

//uint16_t test_pwm_open = 600;
static void house_control(void)
{
    switch( shoot.house_mode )
    {
        case HOUSE_MODE_OPEN: //�򿪵��ո�       
					{
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_OPEN;
        }
        break;
        case HOUSE_MODE_CLOSE:  //ʹ��pwm
					{
            HAL_TIM_PWM_Start(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
        }            
        break;
        case HOUSE_MODE_PROTECT:  //���ո�����
					{
            HAL_TIM_PWM_Stop(Magazine_Time_CH);
            Magazine_PWM = COVER_PWM_CLOSE;
        }
        default: break;
    }
}
