#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stdint.h"

#define HIT_PROTECT_DISTANCE_CLOSE  150000
#define HIT_PROTECT_DISTANCE_FAR    750000
#define MOVE_LIMIT_CLOSE            200000
#define MOVE_LIMIT_FAR              670000

#define MOVE_SPEED_LIMIT           10
#define CALIBRATION_MOVE_SPEED     -2

#define VEHICLE_START_INPUT        0.004
#define VEHICLE_STOP_INPUT        -0.004

#define MOVE_SPEED_LEVEL1          0.0f
#define MOVE_SPEED_LEVEL2          2.0f
#define MOVE_SPEED_LEVEL3          6.0f
#define MOVE_SPEED_LEVEL4          8.0f
#define MOVE_SPEED_LEVEL5          12.0f
#define MOVE_SPEED_LEVEL6          14.0f

#define SPIN_SPEED_LEVEL1          25
#define SPIN_SPEED_LEVEL2          50
#define SPIN_SPEED_LEVEL3          80
#define SPIN_SPEED_LEVEL4          120
#define SPIN_SPEED_LEVEL5          160

#define SPIN_T_LEVEL1              1.5f
#define SPIN_T_LEVEL2              2.0f
#define SPIN_T_LEVEL3              2.5f
#define SPIN_T_LEVEL4              3.0f
#define SPIN_T_LEVEL5              3.5f

#define RC_CH2_SCALE 0.003f
#define RC_CH1_SCALE (-0.15f)
#define RC_CH4_SCALE 15
#define RC_CH3_SCALE 0.015

#endif
