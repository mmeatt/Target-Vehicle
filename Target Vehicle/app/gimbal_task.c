#include "gimbal_task.h"
#include "bsp_motor.h"
#include "math_calcu.h"
#include "pid.h"
#include "remote_comm.h"
#include "control_def.h"
#include "string.h"
#include "cmsis_os.h"
#include "judge_comm.h"
#include "freertos.h"
#include "bsp_dwt.h"
#include <math.h>
#include "wfly_comm.h"

motor_measure_t yaw_motor;
gimbal_t gimbal;
pid_t yaw_agl;
pid_t yaw_spd;

float text_spin;
float gimbal_dt;
extern TaskHandle_t can_comm_task_t;

static gimbal_init_e gimbal_param_init(void);
static uint32_t dwt_count;
static void gimbal_control(void);
static void Manual_control(void);
static void Auto_control(void);
void gimbal_pid_calcu(void);

void gimbal_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    gimbal_param_init();
    for (;;)
    {
        taskENTER_CRITICAL();
        gimbal_dt = DWT_GetDeltaT(&dwt_count);
        gimbal_control();
        osSignalSet(can_comm_task_t, GIMBAL_MOTOR_MSG_SEND);
        //gimbal_pid_calcu();
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
    }
}

/*云台结构体初始化*/
static gimbal_init_e gimbal_param_init(void)
{
    PID_struct_init(&yaw_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&yaw_spd, 28000, 10000, 180.0f, 1.0f, 0, 0, 0, 0, Integral_Limit );
    scale.ch1 = RC_CH1_SCALE;
    scale.ch2 = RC_CH2_SCALE;
    if(yaw_motor.msg_cnt!=0) 
        return GIMBAL_INIT;
    else
        return GIMBAL_UNINIT;
    
}

/*云台结构体初始化*/
static void gimbal_control(void)
{
    switch(gimbal.gimbal_mode)
    {
        case GIMBAL_PROTECT_MODE:{
            memset((void *)&gimbal.spin_speed,0,sizeof(gimbal.spin_speed));
            break;
        }
        case GIMBAL_CTRL_MODE:{
            Manual_control();
            break;
            
        }
        case GIMBAL_AUTO_MODE:{
            Auto_control();
            break;

        }
    }
    gimbal_pid_calcu();
}

/*云台控制函数，共三个模式保护手动和自动*/
static void Manual_control(void)
{
    gimbal.spin_speed = (float)rc.ch1 * RC_CH1_SCALE + SBUS.Ch2 * RC_CH1_SCALE;
}

/*云台手动控制函数*/
static void Auto_control(void)
{
    float auto_spin_spd;
    if(rc.sw2 == RC_UP)
    {
        switch(Game_Robot_Status.shooter_barrel_heat_limit)
        {
            case 120:{
                auto_spin_spd = SPIN_SPEED_LEVEL1;
                break;
            }
            case 180:{
                auto_spin_spd = SPIN_SPEED_LEVEL2;
                break;
            }
            case 200:{
                auto_spin_spd = SPIN_SPEED_LEVEL3;
                break;
            }
            case 240:{
                auto_spin_spd = SPIN_SPEED_LEVEL4;
                break;
            }
            case 300:{
                auto_spin_spd = SPIN_SPEED_LEVEL5;
                break;
            }     
            default:break;
        }
        gimbal.spin_speed = auto_spin_spd;
    }
    else if(rc.sw2 == RC_MI)
    {
        gimbal.spin_time += gimbal_dt;
        switch(Game_Robot_Status.shooter_barrel_heat_limit)
        {
            case 120:{
                gimbal.spin_T = SPIN_T_LEVEL1;
                break;
            }
            case 180:{
                gimbal.spin_T = SPIN_T_LEVEL2;
                break;
            }
            case 200:{
                gimbal.spin_T = SPIN_T_LEVEL3;
                break;
            }
            case 240:{
                gimbal.spin_T = SPIN_T_LEVEL4;
                break;
            }
            case 300:{
                gimbal.spin_T = SPIN_T_LEVEL5;
                break;
            }     
            default:break;
        }
        gimbal.spin_speed = 50 * sin(2 * PI / gimbal.spin_T * gimbal.spin_time) + 100;
        
    }
    if(SBUS.sw2 == SBUS_UP)
    {
        switch(Game_Robot_Status.shooter_barrel_heat_limit)
        {
            case 120:{
                auto_spin_spd = SPIN_SPEED_LEVEL1;
                break;
            }
            case 180:{
                auto_spin_spd = SPIN_SPEED_LEVEL2;
                break;
            }
            case 200:{
                auto_spin_spd = SPIN_SPEED_LEVEL3;
                break;
            }
            case 240:{
                auto_spin_spd = SPIN_SPEED_LEVEL4;
                break;
            }
            case 300:{
                auto_spin_spd = SPIN_SPEED_LEVEL5;
                break;
            }     
            default:break;
        }
        gimbal.spin_speed = auto_spin_spd;
    }
    else if(SBUS.sw2 == SBUS_MI)
    {
        gimbal.spin_time += gimbal_dt;
        switch(Game_Robot_Status.shooter_barrel_heat_limit)
        {
            case 120:{
                gimbal.spin_T = SPIN_T_LEVEL1;
                break;
            }
            case 180:{
                gimbal.spin_T = SPIN_T_LEVEL2;
                break;
            }
            case 200:{
                gimbal.spin_T = SPIN_T_LEVEL3;
                break;
            }
            case 240:{
                gimbal.spin_T = SPIN_T_LEVEL4;
                break;
            }
            case 300:{
                gimbal.spin_T = SPIN_T_LEVEL5;
                break;
            }     
            default:break;
        }
        gimbal.spin_speed = 50 * sin(2 * PI / gimbal.spin_T * gimbal.spin_time) + 100;
        
    }
    
    
}

/*云台自动控制函数*/
void gimbal_pid_calcu(void)
{
    gimbal.gimbal_pid.gimbal_spd_ref = gimbal.spin_speed;
    gimbal.gimbal_pid.gimbal_spd_fdb = yaw_motor.speed_rpm;
    gimbal.yaw_current = (int16_t)pid_calc(&yaw_spd,gimbal.gimbal_pid.gimbal_spd_fdb,gimbal.gimbal_pid.gimbal_spd_ref);
}

