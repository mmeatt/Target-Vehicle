#ifndef _GIMABL_TASK_H_
#define _GIMABL_TASK_H_

#include "stdint.h"
#include "can_comm.h"
#include "bsp_motor.h"
#include "pid.h"

typedef enum
{
    GIMBAL_PROTECT_MODE,       //云台保护模式
    GIMBAL_CTRL_MODE,          //云台手动控制模式
    GIMBAL_AUTO_MODE           //云台自动模式
}gimbal_mode_e;

typedef struct
{
    float gimbal_agl_ref;
    float gimbal_agl_fdb;
    float gimbal_agl_err;
    
    float gimbal_spd_ref;
    float gimbal_spd_fdb;
    float gimbal_spd_err;
}gimbal_pid_t;

typedef enum
{
    GIMBAL_UNINIT = 0,
    GIMBAL_INIT
}gimbal_init_e;

typedef struct
{
    float spin_T;
    float spin_time;
    float spin_speed;
    int16_t yaw_current;
    gimbal_mode_e gimbal_mode;
    gimbal_init_e gimbal_init;
    gimbal_pid_t gimbal_pid;
}gimbal_t;

void gimbal_task(void const *argu);
void gimbal_pid_calcu(void);
extern gimbal_t gimbal;
extern motor_measure_t yaw_motor;
extern pid_t yaw_agl;
extern pid_t yaw_spd;

#endif
