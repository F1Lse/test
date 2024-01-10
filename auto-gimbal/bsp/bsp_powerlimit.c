/**
  ******************************************************************************
  * FileName    : bsp_powerlimit.c
  * Version     : v1.0
  * Author      : BZW
  * Date        : 2021-04-24
  * modification：
  * Functions   :
  * Description :
  *         功率控制逻辑：
  *         目标：比赛全程，底盘实际功率接近当前最大功率，当前剩余缓存能量一直无法恢复（即满功率状态）
  *         分析：
  *               1. 缓存能量可以设置为30J，比较安全，哪怕轮速限制放开，电流限制比例降低，也有比较充足的空间限制住功率
  *               2. 电流限制比例，在能实现功能的前提下，越小越好。实际效果：限功率时行驶比较顺滑
  *               3. 轮速限制：在能实现功能的前提下，越大越好。基本设置为启动和刹车时的最大轮速即可。
  *                            即使限制放开，能达到的速度也是有限的，只要缓存能量足够，加上电流限制，总能限制回来。
  *
  *
  ******************************************************************************
  */
#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_can.h"
#include "can.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_judge.h"
#include "control_def.h"

/* ----------------------------------------------------------------------------------------------------------------------- */
/* 功率控制要调的参数：最小缓冲能量   最大功率 */
#define 	BUFFER_MIN	   30.0f	//预测缓冲能量小于该值则进行电流分配  30
uint8_t   MAX_POWER_JUDGE = 60;		//裁判系统给的机器人最大功率 该变量用于给初值  接收到裁判系统时会按裁判系统信息更新

/* ----------------------------------------------------------------------------------------------------------------------- */
extern CAN_TxHeaderTypeDef Tx1Message;
extern CAN_RxHeaderTypeDef Rx1Message;

powercontrol_t powercontrol = {0};
supercap_t supercap;
uint8_t supercap_status_flag = 0;

void PowerControl_Init(void)
{
    powercontrol.max_power = MAX_POWER_JUDGE;
    powercontrol.limit_kp = 0.25f;  //过大会犬吠 0.4
    chassis.keyboard_input = 40.0f;
    powercontrol.power_buffer = 60.0f;
}

//float test_power = 5800.0f;  //5600 80W
//float test_input = 42.0f;
//float test_kp    = 0.2;
void PowerParam_Update(void)
{
    /* 裁判系统反馈赋值 */
    powercontrol.judge_power = Power_Heat_Data.chassis_power;  //更新数据但不用
    powercontrol.judge_power_buffer = Power_Heat_Data.chassis_power_buffer;  //更新裁判系统回传的缓冲能量
    powercontrol.max_power = Game_Robot_Status.chassis_power_limit;

    powercontrol.power_buffer = powercontrol.judge_power_buffer;  //直接使用裁判系统回传的缓冲能量
    /* 缓冲能量限幅 */
    if( powercontrol.power_buffer >= MAX_POWER_JUDGE )	 
        powercontrol.power_buffer = MAX_POWER_JUDGE;

    /* 底盘最大轮速刷新 */
    if( rc.kb.bit.SHIFT && supercap.volage > 14.0f )  //超级电容电压比电调最低工作电压高，且有余量供加速放电时
    {
        chassis.wheel_max = SPEED_SUPERCAP;
        chassis.keyboard_input = 50.0f;
    }
    else if( chassis.mode == CHASSIS_MODE_KEYBOARD_SUPPLY )  //补给模式：左右反向，速度上限和灵敏度降低
    {
        chassis.wheel_max = SPEED_SUPPLY;
        chassis.keyboard_input = 10.0f;
    }
//    else
//    {
//        powercontrol.limit_kp = test_kp;
//        chassis.wheel_max = test_power;
//        chassis.keyboard_input = test_input;
//    }
    else if(powercontrol.max_power == 45 )
    {
        chassis.wheel_max = SPEED_45W;
        chassis.keyboard_input = 35.0f;
    }
    else if( powercontrol.max_power == 50 )
    {
        chassis.wheel_max = SPEED_50W;
        chassis.keyboard_input = 40.0f;
    }
    else if(powercontrol.max_power == 55 )
    {
        chassis.wheel_max = SPEED_55W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 60 )
    {
        chassis.wheel_max = SPEED_60W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 80 )
    {
        chassis.wheel_max = SPEED_80W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 100 )
    {
        chassis.wheel_max = SPEED_100W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 120 || powercontrol.max_power >120 )
    {
        chassis.wheel_max = SPEED_120W;
        chassis.keyboard_input = 45.0f;
    }
    else  //步兵功率规则外
    {
        chassis.wheel_max = SPEED_50W;
        chassis.keyboard_input = 40.0f;
    }
}

/**
  * @name     Power_Control
  * @brief    功率控制
  * @param    current:底盘电机电流数组首地址
  * @retval
  * @date     2021-04-24
  * @note
  */
void Power_Control(int16_t * current)
{
    static uint8_t powerlimit_cnt=0;  //限制计数位	每次缓冲能量低于5后限制100ms 即限制50次功率
    if ( supercap.mode!=2 || supercap.volage<14.0f )  //超级电容非放电模式下才进行功率控制
    {
        if ( powercontrol.power_buffer < BUFFER_MIN )
        {
            powercontrol.status=1;  //缓冲能量小时限制状态位 置1
            powerlimit_cnt = 0;
            powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//计算限制比例
        }

        if ( powercontrol.status )
        {
            powerlimit_cnt++;
            for(uint8_t i=0; i<4; i++)
            {
                current[i] /= ( ABS(powercontrol.limit_temp) + 1.0f ) ;
            }
        }

        if ( powerlimit_cnt >= POWERLIMIT_CNT )
        {
            powercontrol.status = 0;  //限制状态位清零
            powerlimit_cnt = 0;  //限制计数位清零
        }
    }
}


/**
  * @name     SuperCap_Mode_Update
  * @brief    超级电容模式切换
  * @param    None
  * @retval
  * @date     2021-04-24
  * @note     0：保护模式
  *           1：只充电模式
  *           2：充放电模式（电源管理模块全部给电容充电，电容给底盘放电，
  *                         电容电压降低时：电流闭环，力矩下降）
  */
void SuperCap_Mode_Update(void)
{
    if( !lock_flag || ctrl_mode == PROTECT_MODE )
    {
        supercap.mode = 0; //机器人未解锁，超级电容进入保护模式
    }
    else if( supercap.volage >= 23.7f && !rc.kb.bit.SHIFT )
    {
        supercap.mode = 0;  //电压到达最高上限，且没有使用SHIFT
    }   
    else if( rc.kb.bit.SHIFT && supercap_status_flag )
    {
        if( supercap.volage >= SUPERCAP_DISCHAGER_VOLAGE )
            supercap.mode = 2;  //容组压降大于10V，可进行充放电操作
        else if( powercontrol.power_buffer <= BUFFER_MIN )
            supercap.mode = 0;  //底盘要进行功率限制，停止充电
        else
            supercap.mode = 1;  //放电过程中最低电压小于12V，且底盘用的功率不大时充电
    }
    else if( !rc.kb.bit.SHIFT )
    {
        if( powercontrol.power_buffer <= BUFFER_MIN )
            supercap.mode = 0;  //底盘要进行功率限制，停止充电
        else
            supercap.mode = 1;  //放电过程中最低电压小于12V，且底盘用的功率不大时充电
    }
}

/**
  * @name     SuperCap_Control
  * @brief    超级电容控制
  * @param    None
  * @retval
  * @note     模式更新，与充放电功率及电流控制
  */
void SuperCap_Control(void)
{
    SuperCap_Mode_Update();
    if(supercap.mode==0)  //超级电容保护模式
    {
        supercap.charge_power_ref=0;
        supercap.charge_current_set=0;
    }
    else if(supercap.mode==1)  //超级电容只充电模式
    {
        /* 反馈充电功率 = 电源功率-底盘功率 */
        supercap.charge_power_fdb=powercontrol.supply_power-powercontrol.chassis_power-5.0f;  //超级电容板空载5W功率
        if(supercap.charge_power_fdb<0)		supercap.charge_power_fdb=0;
        /* 目标充电功率 = 最大功率-底盘功率  就是可利用的充电功率 */
        supercap.charge_power_ref=powercontrol.max_power-powercontrol.chassis_power-10.0f;  //充电功率要略低于理想充电功率
        if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;  //当已经超功率时，会消耗缓冲能量，即底盘功率大于最大可用功率，设下限
        /* 目标充电电流电流=空闲功率/超级电容电压 */
        supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
        if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
        else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
    }
    else if(supercap.mode==2)  //超级电容边充电边放电模式
    {

        /* 超级电容放电下 空闲功率 = 裁判系统最大功率 */
        supercap.charge_power_ref=powercontrol.max_power-10;  //充电功率要略低于理想充电功率
        if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;
        supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
        if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
        else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
    }
}


/**
  * @name     can1_send_supercap
  * @brief
  * @param    None
  * @retval
  * @note
  */
void can1_send_supercap(void)
{
    uint8_t supercap_send_buff[8]= {0};
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = 0x010;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    supercap_send_buff[0] = supercap.mode;  //超级电容模式
    memcpy(supercap_send_buff+1,&supercap.charge_current_set,sizeof(supercap.charge_current_set));

    /* CAN发送电容控制模式 */
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  //查询发送邮箱是否为空
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_send_buff,(uint32_t*)CAN_TX_MAILBOX1);
}


uint32_t test_run_period = 0;
/**
  * @name   Power_data_handler
  * @brief
  * @param  can_id: 数据来源CAN
  *			CAN_Rx_data: CAN接收数据缓存
  * @retval
  * @note   电容控制板测得电容电压与电源管理模块的chassis输出功率
  *         功率控制板测得底盘实际功率
  */
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    uint16_t supercap_voltage_buf = 0;
    uint16_t source_power_buf = 0;
    uint16_t chassis_power_buf = 0;
    
    supercap_status_flag = 1;  //超级电容通信状态正常标志
    static uint32_t run_time,last_run_time = 0;
    run_time = HAL_GetTick();
    test_run_period = run_time - last_run_time;
    
    memcpy(&supercap_voltage_buf, CAN_Rx_data, 2);
    memcpy(&source_power_buf, (CAN_Rx_data+2), 2);
    memcpy(&chassis_power_buf, (CAN_Rx_data+4), 2);

    /* 电容电压信息 */
    supercap.volage = supercap_voltage_buf / 100.0f;
    supercap.volume_percent = (supercap.volage - SUPERCAP_DISCHAGER_VOLAGE) /
                              (SUPERCAP_MAX_VOLAGE - SUPERCAP_DISCHAGER_VOLAGE) * 100;
    /* 电源功率 */
    powercontrol.supply_power = source_power_buf /100.0f;

    /* 底盘实时功率信息 */
    powercontrol.chassis_power = chassis_power_buf / 100.0f;

}


