#ifndef __CAN_COMM_H__
#define __CAN_COMM_H__

#include "can.h"
#include "chassis_task.h"
#define GIMBAL_MOTOR_MSG_SEND     ( 1 << 6 )
#define CHASSIS_MOTOR_MSG_SEND    ( 1 << 7 )

/* motor current parameter structure */
typedef struct
{
    int16_t gimbal_cur;
    int16_t wheel_cur[2];
} motor_current_t;

typedef enum
{
    /* can1 */
    LEFT_WHEEL_MOTOR_ID = 0x201,
    RIGHT_WHEEL_MOTOR_ID = 0x202,
    CAN_YAW_MOTOR_ID = 0x205,
    /* can2 */

} can_msg_id_e;

extern motor_current_t motor_cur;

void can_device_init(void);
void can_msg_send_task(void const *argu); /* can信息发送任务 */
void can_msg_send_fail_task(void const *argu);

#endif
