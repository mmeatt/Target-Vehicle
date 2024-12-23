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

motor_measure_t yaw_motor;
gimbal_t gimbal;
pid_t yaw_agl;
pid_t yaw_spd;

static gimbal_init_e gimbal_param_init(void);
static void gimbal_control(void);
static void Manual_control(void);
static void Auto_control(void);

void gimbal_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    gimbal_param_init();
    for (;;)
    {
        taskENTER_CRITICAL();
        gimbal_control();
        gimbal_pid_calcu();
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 1);
    }
}

/*云台结构体初始化*/
static gimbal_init_e gimbal_param_init(void)
{
    PID_struct_init(&yaw_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&yaw_spd, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
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
            
        }
        case GIMBAL_AUTO_MODE:{
            Auto_control();

        }
    }
}

/*云台控制函数，共三个模式保护手动和自动*/
static void Manual_control(void)
{
    gimbal.spin_speed = rc.ch1 * RC_CH1_SCALE;
}

/*云台手动控制函数*/
static void Auto_control(void)
{
    float auto_spin_spd;
    switch(Game_Robot_Status.robot_level)
    {
        case 1:{
            auto_spin_spd = SPIN_SPEED_LEVEL1;
            break;
        }
        case 2:{
            auto_spin_spd = SPIN_SPEED_LEVEL2;
            break;
        }
        case 3:{
            auto_spin_spd = SPIN_SPEED_LEVEL3;
            break;
        }
        case 4:{
            auto_spin_spd = SPIN_SPEED_LEVEL4;
            break;
        }
        case 5:{
            auto_spin_spd = SPIN_SPEED_LEVEL5;
            break;
        }
        case 6:{
            auto_spin_spd = SPIN_SPEED_LEVEL6;
            break;
        }
        case 7:{
            auto_spin_spd = SPIN_SPEED_LEVEL7;
            break;
        }
        case 8:{
            auto_spin_spd = SPIN_SPEED_LEVEL8;
            break;
        }   
        case 9:{
            auto_spin_spd = SPIN_SPEED_LEVEL9;
            break;
        }
        case 10:{
            auto_spin_spd = SPIN_SPEED_LEVEL10;
            break;
        }
        default:break;
    }
    gimbal.spin_speed = auto_spin_spd;
}

/*云台自动控制函数*/
void gimbal_pid_calcu(void)
{
    gimbal.gimbal_pid.gimbal_spd_ref = gimbal.spin_speed;
    gimbal.gimbal_pid.gimbal_spd_fdb = yaw_motor.speed_rpm;
    gimbal.yaw_current = (int16_t)pid_calc(&yaw_spd,gimbal.gimbal_pid.gimbal_spd_fdb,gimbal.gimbal_pid.gimbal_spd_ref);
}

