#ifndef _BSP_MOTOR_H_
#define _BSP_MOTOR_H_

#include "stdint.h"

typedef struct
{
    uint16_t ecd;
    uint16_t last_ecd;

    int16_t  speed_rpm;
    int16_t  given_current;

    int32_t  round_cnt;
    int32_t  total_ecd;
    int32_t  total_ecd_limit;

    uint16_t offset_ecd;
    uint32_t msg_cnt;
} motor_measure_t;

extern motor_measure_t motor_trigger;
extern motor_measure_t motor_pit;
extern motor_measure_t motor_yaw;
void get_moto_offset(motor_measure_t* ptr, uint8_t* CAN_Rx_data);
void encoder_data_handler(motor_measure_t* ptr, uint8_t* CAN_Rx_data);


#endif
