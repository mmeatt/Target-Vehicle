#include "debug_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "data_scope.h"
#include "math.h"
#include "gimbal_task.h"
#include "chassis_task.h"
#include "modeswitch_task.h"

#ifndef ABS
#define ABS(x) ((x>0)? (x): (-(x)))
#endif

static uint8_t debug_wave = 4;

static void VehicleDataWavePkg(uint8_t wave_index)
{
    switch (debug_wave)
    {
        case 1:{//底盘pid数据打印
            DataScope_Get_Channel_Data(chassis.chassis_pid.left_spd_ref);
            DataScope_Get_Channel_Data(chassis.chassis_pid.left_spd_fdb);
            DataScope_Get_Channel_Data(chassis.chassis_pid.left_spd_err);
            DataScope_Get_Channel_Data(chassis.chassis_pid.right_spd_ref);
            DataScope_Get_Channel_Data(chassis.chassis_pid.right_spd_fdb);
            DataScope_Get_Channel_Data(chassis.chassis_pid.right_spd_err);
            break;
        }
        case 2:{//云台pid数据打印
            DataScope_Get_Channel_Data(gimbal.gimbal_pid.gimbal_spd_ref);
            DataScope_Get_Channel_Data(gimbal.gimbal_pid.gimbal_spd_fdb);
            DataScope_Get_Channel_Data(gimbal.gimbal_pid.gimbal_spd_err);
            break;
        }
        case 3:{
            DataScope_Get_Channel_Data(chassis.move_speed);
            DataScope_Get_Channel_Data(chassis_x_ramp.out);
            DataScope_Get_Channel_Data(chassis.vehicle_direction);
            break;
        
        }
        case 4:{
            DataScope_Get_Channel_Data(right_wheel_motor.total_ecd);
            DataScope_Get_Channel_Data(right_wheel_motor.round_cnt);
            DataScope_Get_Channel_Data(left_wheel_motor.total_ecd);
            DataScope_Get_Channel_Data(left_wheel_motor.round_cnt);
            break;
        
        }
        default:break;
    }
}

void DataWavePkg(void)
{
    VehicleDataWavePkg(debug_wave);
}

void debug_task(void const *argu)
{
    uint32_t thread_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        DataWave(&huart3);
        taskEXIT_CRITICAL();
        osDelayUntil(&thread_wake_time, 2);
    }
}
