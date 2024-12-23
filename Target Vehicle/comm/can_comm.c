#include "can_comm.h"
#include "string.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "judge_comm.h"
#include "remote_comm.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "modeswitch_task.h"

motor_current_t motor_cur;

static void User_can1_callback(uint32_t ID, uint8_t *CAN_RxData);
static void User_can2_callback(uint32_t ID, uint8_t *CAN_RxData);

void can_device_init(void)
{
    uint32_t can_ID1[] = {LEFT_WHEEL_MOTOR_ID, RIGHT_WHEEL_MOTOR_ID, CAN_YAW_MOTOR_ID, 0, 0xFFF};
    uint32_t can_ID2[] = {0x001, 0x002, 0, 0, 0, 0, 0xFFF};
    canx_init(&hcan1, can_ID1, User_can1_callback);
    canx_init(&hcan2, can_ID2, User_can2_callback);
}

/**
 * @brief can_msg_send_task
 * @param
 * @attention
 * @note
 */
void can_comm_task(const void* argu)
{
	uint32_t thread_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
        if(ctrl_mode == PROTECT_MODE)
        {
            memset(&chassis.wheel_current,0,2*sizeof(chassis.wheel_current));
            memset(&gimbal.yaw_current,0,sizeof(gimbal.yaw_current));
        }
		can1_send_message(0x200,chassis.wheel_current[LEFT_WHEEL],chassis.wheel_current[RIGHT_WHEEL],gimbal.yaw_current,0);
		taskEXIT_CRITICAL();
		osDelayUntil(&thread_wake_time,1);
	}
}


static void User_can1_callback(uint32_t ID, uint8_t *CAN_RxData)
{
    switch (ID)
    {
    case LEFT_WHEEL_MOTOR_ID:
    {
        left_wheel_motor.msg_cnt++;
        encoder_data_handler(&left_wheel_motor, CAN_RxData);

        break;
    }

    case RIGHT_WHEEL_MOTOR_ID:
    {
        right_wheel_motor.msg_cnt++;
        encoder_data_handler(&right_wheel_motor, CAN_RxData);
        break;
    }
    case CAN_YAW_MOTOR_ID:
    {
        yaw_motor.msg_cnt++;
        encoder_data_handler(&yaw_motor, CAN_RxData);
    }
    
    default:
        break;
    }
}

static void User_can2_callback(uint32_t ID, uint8_t *CAN_RxData)
{

}
