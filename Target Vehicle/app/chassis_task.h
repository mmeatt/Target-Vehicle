#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "stdint.h"
#include "can_comm.h"
#include "bsp_motor.h"

#define RIGHT_WHEEL 0
#define LEFT_WHEEL  1 

typedef enum
{
    PROTECT_MODE,
    CTRL_MODE,
    AUTO_MODE
}chassis_mode_e;

typedef struct
{
    /**********右轮pid************/
    /******位置环*******/
    float right_agl_ref;
    float right_agl_fdb;
    float right_agl_err;
    /******速度环*******/
    float right_spd_ref;
    float right_spd_fdb;
    float right_spd_err;
    
    /**********左轮pid************/
    /******位置环*******/
    float left_agl_ref;
    float left_agl_fdb;
    float left_agl_err;
    /******速度环*******/
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
    CHASSIS_UNINIT = 0,
    CHASSIS_INIT
}chassis_init_e;

typedef enum
{
    CALIBRATION_UNINIT = 0,
    CALIBRATION_INIT,
    CALIBRATION_NEEDINIT
}calibration_status_e;

typedef enum
{
    CLOSE_START_ZONE,
    FAR_START_ZONE,
    CLOSE_BREAK_ZONE,
    FAR_BREAK_ZONE,
    MIDDLE_ZONE
}vehicle_position_e;

typedef enum
{
    MOVE_FAR_DIR,
    MOVE_CLOSE_DIR
}vehicle_direction_e;

typedef struct
{
    calibration_status_e calibration_status;
    vehicle_direction_e vehicle_direction;
    float move_speed;
    float wheel_speed[2];
    chassis_mode_e  chassis_mode;
    chassis_pid_t   chassis_pid;
    chassis_init_e  chassis_init;
    vehicle_position_e vehicle_pos_status;
}chassis_t;


extern motor_measure_t right_wheel_motor;
extern motor_measure_t left_wheel_motor;
#endif
