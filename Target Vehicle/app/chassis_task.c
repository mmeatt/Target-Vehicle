#include "chassis_task.h"
#include "bsp_motor.h"
#include "math_calcu.h"
#include "pid.h"
#include "remote_comm.h"
#include "control_def.h"
#include "string.h"
#include "cmsis_os.h"
#include "judge_comm.h"

#ifndef ABS
#define ABS(x) ((x>0)? (x): (-(x)))
#endif

motor_measure_t right_wheel_motor;
motor_measure_t left_wheel_motor;
chassis_t chassis;
pid_t right_wheel_agl;
pid_t right_wheel_spd;
pid_t left_wheel_agl;
pid_t left_wheel_spd;
pid_t vehicle_spd;
pid_t vehicle_agl;

/*Ð±ÆÂ±äÁ¿*/
static ramp_function_source_t chassis_x_ramp;

static void Manual_control(void);
static void vehicle_calibration(void);


static chassis_init_e chassis_param_init(void)
{
    PID_struct_init(&right_wheel_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&right_wheel_spd, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&left_wheel_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&left_wheel_spd, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&vehicle_spd, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&vehicle_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    
    if((right_wheel_motor.msg_cnt!=0) && (left_wheel_motor.msg_cnt!=0)) 
        return CHASSIS_INIT;
    else
        return CHASSIS_UNINIT;
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
}

static void chassis_control(void)
{
    switch(chassis.chassis_mode)
    {
        case PROTECT_MODE:{
            memset((void *)&chassis.move_speed,0,sizeof(chassis.move_speed));
            break;
        }
        case CTRL_MODE:{
            Manual_control();
            
        }
        case AUTO_MODE:{
        }
    }
}

static void Manual_control(void)
{
    chassis.move_speed = rc.ch3 * RC_CH3_SCALE;
    if(ABS(chassis.move_speed) > MOVE_SPEED_LIMIT)
    {
        if(chassis.move_speed > 0)
            chassis.move_speed = MOVE_SPEED_LIMIT;
        else
            chassis.move_speed = -MOVE_SPEED_LIMIT;
    }
    
    chassis.wheel_speed[RIGHT_WHEEL] = chassis.move_speed;
    chassis.wheel_speed[LEFT_WHEEL] = -chassis.move_speed;
}

static void Auto_control(void)
{
    float auto_move_speed;
    uint16_t get_chassis_power;
    get_chassis_power = Power_Heat_Data.chassis_power;
    if(chassis.calibration_status == CALIBRATION_INIT) 
    {
        switch(get_chassis_power)
        {
            case 50:{
                auto_move_speed = MOVE_SPEED_LEVEL1;
                break;
            }
            case 60:{
                auto_move_speed = MOVE_SPEED_LEVEL2;
                break;
            }
            case 70:{
                auto_move_speed = MOVE_SPEED_LEVEL3;
                break;
            }
            case 80:{
                auto_move_speed = MOVE_SPEED_LEVEL4;
                break;
            }
            case 100:{
                auto_move_speed = MOVE_SPEED_LEVEL5;
                break;
            }
            case 120:{
                auto_move_speed = MOVE_SPEED_LEVEL6;
                break;
            }
            default:break;
        }
        switch (chassis.vehicle_pos_status)
        {
            case CLOSE_BREAK_ZONE:{}
            case FAR_BREAK_ZONE:{}
            case MIDDLE_ZONE:{}
            case CLOSE_START_ZONE:{}
            case FAR_START_ZONE:{}
            default:break;
        }
    }
    else
    {
        
    }
    
}

static void get_vehicle_position(void)
{
    if((right_wheel_motor.total_ecd < HIT_PROTECT_DISTANCE_CLOSE) && (ABS(left_wheel_motor.total_ecd) < HIT_PROTECT_DISTANCE_CLOSE))
    {
        chassis.vehicle_pos_status = CLOSE_BREAK_ZONE;
    }
    else if((right_wheel_motor.total_ecd > HIT_PROTECT_DISTANCE_FAR) && (ABS(left_wheel_motor.total_ecd) > HIT_PROTECT_DISTANCE_FAR))
    {
        chassis.vehicle_pos_status = FAR_BREAK_ZONE;
    }
    else if((right_wheel_motor.total_ecd > HIT_PROTECT_DISTANCE_CLOSE) && (right_wheel_motor.total_ecd < HIT_PROTECT_DISTANCE_FAR) &&
        (ABS(left_wheel_motor.total_ecd) > HIT_PROTECT_DISTANCE_CLOSE) && (ABS(left_wheel_motor.total_ecd) < HIT_PROTECT_DISTANCE_FAR))
    {
        chassis.vehicle_pos_status = MIDDLE_ZONE;
    }
    else if((right_wheel_motor.total_ecd < HIT_PROTECT_DISTANCE_CLOSE) && (ABS(left_wheel_motor.total_ecd) < HIT_PROTECT_DISTANCE_CLOSE))
    {
        chassis.vehicle_pos_status = CLOSE_START_ZONE;
    }
    else if((right_wheel_motor.total_ecd > HIT_PROTECT_DISTANCE_FAR) && (ABS(left_wheel_motor.total_ecd) > HIT_PROTECT_DISTANCE_FAR))
    {
        chassis.vehicle_pos_status = FAR_START_ZONE;
    }
}

static void vehicle_calibration(void)
{
    chassis.move_speed = CALIBRATION_MOVE_SPEED;
    
}

