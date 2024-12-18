#ifndef _REMOTE_COMM_H_
#define _REMOTE_COMM_H_

#include "stdint.h"

#define DBUS_MAX_LEN  50
#define DBUS_BUFLEN   18

/**
  * @brief ң����״̬
  */
/* ң����ದ��?*/
#define RC_LEFT_LU  ( 1<<0 ) // ������
#define RC_LEFT_RU  ( 1<<1 ) // ������
#define RC_LEFT_RD  ( 1<<2 ) // ������
#define RC_LEFT_LD  ( 1<<3 ) // ������

/* ң���Ҳ�?
��? */
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

    
    uint8_t init_status; /* ң�����ϵ粦��״̬�� */
} rc_info_t;



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

extern rc_info_t    rc;

extern scale_t      scale;

extern void         rc_callback_handler(rc_info_t *rc, uint8_t *buff);
extern void         rc_FSM_init(uint8_t trig_flag);
extern uint8_t      rc_FSM_check(uint8_t target_status);

#endif
