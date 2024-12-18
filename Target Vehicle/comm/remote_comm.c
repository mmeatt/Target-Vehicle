#include "remote_comm.h"
#include "string.h"

#ifndef ABS
    #define ABS(x)  ( (x)>0? (x): (-(x)) )
#endif

/**
  * @brief ң����״̬����
  */
/* ң����ದ�� */
#define RC_LEFT_LU_CH_VALUE  ( rc.ch3 < -500 && rc.ch4 >  500 )   // ������
#define RC_LEFT_RU_CH_VALUE  ( rc.ch3 >  500 && rc.ch4 >  500 )   // ������
#define RC_LEFT_RD_CH_VALUE  ( rc.ch3 >  500 && rc.ch4 < -500 )   // ������
#define RC_LEFT_LD_CH_VALUE  ( rc.ch3 < -500 && rc.ch4 < -500 )   // ������

/* ң���Ҳದ�� */
#define RC_RIGHT_LU_CH_VALUE ( rc.ch2 >  500 && rc.ch1 < -500 )   // ������
#define RC_RIGHT_RU_CH_VALUE ( rc.ch2 >  500 && rc.ch1 >  500 )   // ������
#define RC_RIGHT_RD_CH_VALUE ( rc.ch2 < -500 && rc.ch1 >  500 )   // ������
#define RC_RIGHT_LD_CH_VALUE ( rc.ch2 < -500 && rc.ch1 < -500 )   // ������

/* ң�������� */
rc_info_t rc;


/* ���������� */
scale_t scale;


/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff) {
    /* ���ݽ��� */
    rc->ch1 = (buff[0]      | buff[1]  << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2]  << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3]  << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5]  << 7) & 0x07FF;
    rc->ch4 -= 1024;
    rc->ch5=  (buff[16]     | buff[17] << 8) & 0x07FF;
    rc->ch5 -= 1024;
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    if ((ABS(rc->ch1) > 660) || \
        (ABS(rc->ch2) > 660) || \
        (ABS(rc->ch3) > 660) || \
        (ABS(rc->ch4) > 660)    ) {
        memset(rc, 0, sizeof(rc_info_t));
    }

		
		if(ABS(rc->ch2) < 30 )
		rc->ch2 =0;
        
    if(ABS(rc->ch1) < 30)
        rc->ch1 = 0;
    if(ABS(rc->ch2) < 30)
        rc->ch2 = 0;
    if(ABS(rc->ch3) < 30)
        rc->ch3 = 0;
    if(ABS(rc->ch4) < 30)
        rc->ch4 = 0;
    
}

/**
  * @brief       ң����������������״̬��
  * @param[out]  state: ң�����ϵ�ʱ��״̬
  * @param[in]   trig_flag: ң��������״̬��־
  * @author      ZZJ
  */
void rc_FSM_init(uint8_t trig_flag) 
{
    static uint8_t last_trig_flag;
    static uint8_t state;
    if (trig_flag == 1 && last_trig_flag == 0) {  //��⵽�����ź�������
        if (RC_LEFT_LU_CH_VALUE)   state |= RC_LEFT_LU;
        if (RC_LEFT_RU_CH_VALUE)   state |= RC_LEFT_RU;
        if (RC_LEFT_RD_CH_VALUE)   state |= RC_LEFT_RD;
        if (RC_LEFT_LD_CH_VALUE)   state |= RC_LEFT_LD;
        if (RC_RIGHT_LU_CH_VALUE)  state |= RC_RIGHT_LU;
        if (RC_RIGHT_RU_CH_VALUE)  state |= RC_RIGHT_RU;
        if (RC_RIGHT_RD_CH_VALUE)  state |= RC_RIGHT_RD;
        if (RC_RIGHT_LD_CH_VALUE)  state |= RC_RIGHT_LD;
    } else if (trig_flag == 0 && last_trig_flag == 1){  //ң�����Ͽ�����
        state = 0x00;
    }
    last_trig_flag = trig_flag;
    rc.init_status = state;
    
    if (!trig_flag && last_trig_flag) {  /* ң����ʧ����״̬����λ */
        rc.init_status = 0;
    }
}

/**
  * @brief       ң��������״̬����飬�鿴��ǰң�����Ƿ���Ŀ��״̬
  * @param[out]  boolres:   0��1
  * @param[in]   trig_flag: ң��������״̬��־
  * @author      ZZJ
  */
uint8_t rc_FSM_check(uint8_t target_status) 
{
    uint8_t res = 0;
    if (rc.init_status & target_status) {
        res = 1;
    }
    return res;
//    return 0;
}
