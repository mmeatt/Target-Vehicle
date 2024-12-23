#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "stdint.h"
#include "can_comm.h"
#include "bsp_motor.h"

#define RIGHT_WHEEL 0
#define LEFT_WHEEL  1 

typedef enum
{
    CHASSIS_PROTECT_MODE,       //底盘保护模式
    CHASSIS_CTRL_MODE,          //底盘手动控制模式
    CHASSIS_AUTO_MODE           //底盘自动移动模式
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
    CHASSIS_UNINIT = 0,     //车辆底盘数据未初始化
    CHASSIS_INIT            //车辆底盘数据已初始化
}chassis_init_e;

typedef enum
{
    CALIBRATION_UNINIT = 0, //车辆位置未校准
    CALIBRATION_INIT,       //车辆位置已校准
    CALIBRATION_NEEDINIT    //车辆位置需要校准（因为两轮因为非运行原因差动）
}calibration_status_e;

typedef enum
{
    CLOSE_START_ZONE,  //离出发点近的不可到达区段
    FAR_START_ZONE,    //离出发点远的不可到达区段
    CLOSE_BREAK_ZONE,  //离出发点近的刹车区段
    FAR_BREAK_ZONE,    //离出发点远的刹车区段
    MIDDLE_ZONE        //中间的正常运行区段
}vehicle_position_e;

typedef enum
{
    MOVE_FAR_DIR,      //车往远离校准点方向运动
    MOVE_CLOSE_DIR     //车往靠近校准点方向运动
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
