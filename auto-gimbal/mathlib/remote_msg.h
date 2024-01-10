#ifndef __REMOTE_MSG_H__
#define __REMOTE_MSG_H__

#include "stm32f4xx_hal.h"

#define DBUS_MAX_LEN  50
#define DBUS_BUFLEN   18

/**
  * @brief ң����״̬
  */
/* ң����ದ�� */
#define RC_LEFT_LU  ( 1<<0 ) // ������
#define RC_LEFT_RU  ( 1<<1 ) // ������
#define RC_LEFT_RD  ( 1<<2 ) // ������
#define RC_LEFT_LD  ( 1<<3 ) // ������

/* ң���Ҳದ�� */
#define RC_RIGHT_LU ( 1<<4 )   // ������
#define RC_RIGHT_RU ( 1<<5 )   // ������
#define RC_RIGHT_RD ( 1<<6 )   // ������
#define RC_RIGHT_LD ( 1<<7 )   // ������

/* channel sensitivity */
typedef struct
{
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch5;
} scale_t;

/* remote control information structure */
typedef __packed struct
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t ch5;
    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;

    /* mouse movement and button information */
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t l;
        uint8_t r;
    } mouse;
    /* keyboard key information */
    __packed union
    {
        uint16_t key_code;
        __packed struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } bit;
    } kb;
    uint8_t init_status;
} rc_info_t;

/* PC_Key_Definition */
typedef enum
{
    KB_Q = 0,
    KB_E = 1,
    KB_R = 2,
    KB_F = 3,
    KB_G = 4,
    KB_Z = 5,
    KB_X = 6,
    KB_C = 7,
    KB_V = 8,
    KB_B = 9,
    KB_CTRL = 10,
} key_index_e;

typedef enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
} rc_sw_mode_e;

typedef enum
{
    KEY_RUN = 1,
    KEY_END = 0
} rc_key_status_e;

extern rc_info_t rc;
extern scale_t scale;
extern uint8_t rc_normal_flag;
extern uint8_t kb_status[11];

void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
void keyboard_scan(key_index_e key_index);
void key_status_clear(key_index_e key_index);
uint8_t key_scan_clear(key_index_e key_index);
uint8_t rc_FSM(uint8_t trig_flag);
uint8_t rc_FSM_check(uint8_t target_status);

#endif
