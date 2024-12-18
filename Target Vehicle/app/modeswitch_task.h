#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE = 0,   //����ģʽ
    REMOTER_MODE,   //ң��ģʽ
    AUTO_MODE

} ctrl_mode_e;

extern uint8_t lock_flag;
extern ctrl_mode_e ctrl_mode;

void mode_switch_task(void const *argu);
static void sw1_mode_handler(void);
static void unlock_init(void);

#endif
