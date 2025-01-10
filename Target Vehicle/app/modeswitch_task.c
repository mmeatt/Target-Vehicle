#include "modeswitch_task.h"
#include "cmsis_os.h"
#include "remote_comm.h"
#include "gimbal_task.h"

ctrl_mode_e ctrl_mode;
uint8_t lock_flag;
uint8_t motor_flag = 1;

void mode_switch_task(void const *argu)
{
    ctrl_mode = PROTECT_MODE;
    lock_flag = 0;
    for (;;)
    {
        if (!lock_flag)
        {
            unlock_init(); // 解锁操作
        }
        else
        { 
            sw1_mode_handler(); 
        }
        osDelay(10);
    }
}

static void sw1_mode_handler(void)
{ // 由拨杆1决定系统模式切换，主要是云台、底盘和发射器
    switch (rc.sw1)
    {
    case RC_UP:
        ctrl_mode = REMOTER_MODE;
        gimbal.gimbal_mode = GIMBAL_CTRL_MODE;
        break;
    case RC_MI:
        ctrl_mode = PROTECT_MODE;
        gimbal.gimbal_mode = GIMBAL_PROTECT_MODE;
        break;
    case RC_DN:
        ctrl_mode= AUTO_MODE;
        gimbal.gimbal_mode = GIMBAL_AUTO_MODE;
        break;
    default:
        break;
    }
}

/* 解锁函数 */
static void unlock_init(void)
{
    if (rc.sw1 == RC_MI && rc.sw2 == RC_UP)
    { // 左拨杆居中，右拨杆置上
        if (rc.ch4 < -600 && rc.ch3 > 600)
        {
            lock_flag = 1; // 左控制杆拨至右下
        }
    }
}
