#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "stdint.h"
#include "can_comm.h"
#include "bsp_motor.h"

#define RIGHT_WHEEL 0
#define LEFT_WHEEL  1 

typedef enum
{
    CHASSIS_PROTECT_MODE,       //���̱���ģʽ
    CHASSIS_CTRL_MODE,          //�����ֶ�����ģʽ
    CHASSIS_AUTO_MODE           //�����Զ��ƶ�ģʽ
}chassis_mode_e;

typedef struct
{
    /**********����pid************/
    /******λ�û�*******/
    float right_agl_ref;
    float right_agl_fdb;
    float right_agl_err;
    /******�ٶȻ�*******/
    float right_spd_ref;
    float right_spd_fdb;
    float right_spd_err;
    
    /**********����pid************/
    /******λ�û�*******/
    float left_agl_ref;
    float left_agl_fdb;
    float left_agl_err;
    /******�ٶȻ�*******/
    float left_spd_ref;
    float left_spd_fdb;
    float left_spd_err;
    
    float vehicle_pos_ref;
    float vehicle_pos_fdb;
    float vehicle_pos_err;
    
    float vehicle_speed_ref;
    float vehicle_speed_fdb;
    float vehicle_speed_err;
    
}chassis_pid_t;

typedef enum
{
    CHASSIS_UNINIT = 0,     //������������δ��ʼ��
    CHASSIS_INIT            //�������������ѳ�ʼ��
}chassis_init_e;

typedef enum
{
    CALIBRATION_UNINIT = 0, //����λ��δУ׼
    CALIBRATION_INIT,       //����λ����У׼
    CALIBRATION_NEEDINIT    //����λ����ҪУ׼����Ϊ������Ϊ������ԭ����
}calibration_status_e;

typedef enum
{
    CLOSE_START_ZONE,  //���������Ĳ��ɵ�������
    FAR_START_ZONE,    //�������Զ�Ĳ��ɵ�������
    CLOSE_BREAK_ZONE,  //����������ɲ������
    FAR_BREAK_ZONE,    //�������Զ��ɲ������
    MIDDLE_ZONE        //�м��������������
}vehicle_position_e;

typedef enum
{
    MOVE_FAR_DIR,      //����Զ��У׼�㷽���˶�
    MOVE_CLOSE_DIR     //��������У׼�㷽���˶�
}vehicle_direction_e;

typedef struct
{
    float move_speed;
    float wheel_speed[2];
    int16_t wheel_current[2];
    chassis_pid_t   chassis_pid;
    chassis_mode_e  chassis_mode;
    chassis_init_e  chassis_init;
    vehicle_direction_e vehicle_direction;
    vehicle_position_e vehicle_pos_status;
    calibration_status_e calibration_status;
    
}chassis_t;

void chassis_task(void const *argu);
void chassis_pid_calcu(void);
extern chassis_t chassis;
extern motor_measure_t right_wheel_motor;
extern motor_measure_t left_wheel_motor;
#endif
