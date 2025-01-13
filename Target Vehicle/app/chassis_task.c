#include "chassis_task.h"
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
#include "wfly_comm.h"

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
pid_t vehicle_pos;
float chassis_dt;
float chassis_time;

/*б�±���*/
ramp_function_source_t chassis_x_ramp;

/*���̺�������*/
static chassis_init_e chassis_param_init(void);
static void chassis_control(void);
static void Manual_control(void);
static void Auto_control(void);
static void vehicle_calibration(void);
static void get_vehicle_position(void);
static void vehicle_calibration(void);
static int duo_wheel_reinit(void);
static uint32_t dwt_count;

/*����������*/
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    chassis_param_init();
    for(;;)
    {
        taskENTER_CRITICAL();
        chassis_dt = DWT_GetDeltaT(&dwt_count);
        get_vehicle_position();
        chassis_control();
        //chassis_pid_calcu();
        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, 1);
    }
}


/*���̽ṹ���ʼ��*/
static chassis_init_e chassis_param_init(void)
{
    PID_struct_init(&right_wheel_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&right_wheel_spd, 10000, 2000, 10, 1, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&left_wheel_agl, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&left_wheel_spd, 10000, 2000, 10, 1, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&vehicle_spd, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    PID_struct_init(&vehicle_pos, 0, 0, 0, 0, 0, 0, 0, 0, Integral_Limit );
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
    if((right_wheel_motor.msg_cnt!=0) && (left_wheel_motor.msg_cnt!=0)) 
        return CHASSIS_INIT;
    else
        return CHASSIS_UNINIT;
    
}

/*���̿��ƺ�����������ģʽ�����ֶ����Զ�*/
static void chassis_control(void)
{
    switch(chassis.chassis_mode)
    {
        case CHASSIS_PROTECT_MODE:{
            memset((void *)&chassis.move_speed,0,sizeof(chassis.move_speed));
            break;
        }
        case CHASSIS_CTRL_MODE:{
            Manual_control();
            break;
        }
        case CHASSIS_AUTO_MODE:{
            Auto_control();
            break;
            
        }
    }
    chassis_pid_calcu();
}

/*�����ֶ����ƺ���*/
static void Manual_control(void)
{
    chassis.move_speed = rc.ch3 * RC_CH3_SCALE + SBUS.Ch4 * RC_CH3_SCALE;
    if(ABS(chassis.move_speed) > MOVE_SPEED_LIMIT)
    {
        if(chassis.move_speed > 0)
            chassis.move_speed = MOVE_SPEED_LIMIT;
        else
            chassis.move_speed = -MOVE_SPEED_LIMIT;
    }
    
    chassis.wheel_speed[RIGHT_WHEEL] = chassis.move_speed / 0.06f * 19.0f;
    chassis.wheel_speed[LEFT_WHEEL] = -chassis.move_speed / 0.06f * 19.0f;
}

/*�����Զ����ƺ���*/
static void Auto_control(void)
{
    float auto_move_speed;
    uint16_t get_chassis_power;
    get_chassis_power = Game_Robot_Status.chassis_power_limit;
    if(chassis.calibration_status == CALIBRATION_INIT) 
    {
        chassis_time += chassis_dt;
        switch(get_chassis_power)//���ݲ���ϵͳ�趨�ĵ��̹��ʵ�����Ӧ�ĵ�����ֵ
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
        switch(chassis.vehicle_direction)//����ĵ�����ֵΪ����ֵ�������˶�����Ӹ���
        {
            case MOVE_CLOSE_DIR:{
                auto_move_speed = -1 * auto_move_speed;
                break;
            }
            case MOVE_FAR_DIR:{
                break;
            }
        }
        switch (chassis.vehicle_pos_status)//���ݵ�ǰ����λ���жϳ����ƶ���ʽ
        {
            case CLOSE_BREAK_ZONE:{//�ڿ���У׼���ɲ����
                if(chassis.vehicle_direction == MOVE_CLOSE_DIR)//��������ǿ�����ʹ��б�¼��٣����ٵ��������
                {
                    ramp_calc(&chassis_x_ramp,2.0f,-VEHICLE_STOP_INPUT,auto_move_speed,0.0f);
                    chassis.move_speed = chassis_x_ramp.out;
                    if(chassis.move_speed == 0.0f)
                    {
                        chassis.vehicle_direction = MOVE_FAR_DIR;
                    }
                }
                else//���������Զ�룬ʹ��б�¼���
                {
                    ramp_calc(&chassis_x_ramp,2.0f,VEHICLE_START_INPUT,auto_move_speed,0);
                    chassis.move_speed = chassis_x_ramp.out;
                }
                break;
            }
            case FAR_BREAK_ZONE:{//��Զ��У׼���ɲ����
                if(chassis.vehicle_direction == MOVE_FAR_DIR)//���������Զ�룬ʹ��б�¼��٣����ٵ��������
                {
                    ramp_calc(&chassis_x_ramp,2.0f,VEHICLE_STOP_INPUT,auto_move_speed,0.0f);
                    chassis.move_speed = chassis_x_ramp.out;
                    if(chassis.move_speed == 0.0f)
                    {
                        chassis.vehicle_direction = MOVE_CLOSE_DIR;
                    }
                }
                else//��������ǿ�����ʹ��б�¼���
                {
                    ramp_calc(&chassis_x_ramp,2.0f,-VEHICLE_START_INPUT,0.0f,auto_move_speed);
                    chassis.move_speed = chassis_x_ramp.out;
                }
                break;
            }
            case MIDDLE_ZONE:{//���м������������
                switch(chassis.vehicle_direction)
                {
                    case MOVE_FAR_DIR:{
                        if(chassis.move_speed < auto_move_speed)//����ٶ��ڼ�������û�дﵽ�趨���������٣�����ͨ���������䳤�ȼ�������жϣ�
                        {
                             ramp_calc(&chassis_x_ramp,2.0f,VEHICLE_START_INPUT,auto_move_speed,0);
                             chassis.move_speed = chassis_x_ramp.out;
                        }
                        else
                        {
                            chassis.move_speed = auto_move_speed;
                        }
                        break;
                    }
                    case MOVE_CLOSE_DIR:{
                        if(chassis.move_speed > -auto_move_speed)
                        {
                             ramp_calc(&chassis_x_ramp,2.0f,-VEHICLE_START_INPUT,0.0f,auto_move_speed);
                             chassis.move_speed = chassis_x_ramp.out;
                        }
                        else
                        {
                            chassis.move_speed = auto_move_speed;
                        }
                        break;
                    }
                }
                break;
            }
            case CLOSE_START_ZONE:{//�ڿ���У׼��Ĳ��ɵ�����
                if(chassis.vehicle_direction == MOVE_CLOSE_DIR)
                {
                    chassis.move_speed = 0;
                    chassis.vehicle_direction = MOVE_FAR_DIR;
                }
                else if(chassis.vehicle_direction == MOVE_FAR_DIR)
                {
                    ramp_calc(&chassis_x_ramp,2.0f,VEHICLE_START_INPUT,auto_move_speed,0);
                    chassis.move_speed = chassis_x_ramp.out;
                }
                break;
            }
            case FAR_START_ZONE:{//��Զ��У׼��Ĳ��ɵ�����
                if(chassis.vehicle_direction == MOVE_FAR_DIR)
                {
                    chassis.move_speed = 0;
                    chassis.vehicle_direction = MOVE_CLOSE_DIR;
                }
                else if(chassis.vehicle_direction == MOVE_CLOSE_DIR)
                {
                    float speed1 = -auto_move_speed;
                    ramp_calc(&chassis_x_ramp,2.0f,-VEHICLE_START_INPUT,0.0f,auto_move_speed);
                    chassis.move_speed = chassis_x_ramp.out;
                }
                break;
            }
            default:break;
        }
        if(chassis_time >5)
        {
            if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_RESET)
            {
                if(duo_wheel_reinit())
                {
                    chassis.vehicle_direction = MOVE_FAR_DIR;
                }
               
            }
        }
    }
    else
    {
        vehicle_calibration();
    }
    chassis.wheel_speed[RIGHT_WHEEL] = chassis.move_speed / 0.06f * 19.0f;
    chassis.wheel_speed[LEFT_WHEEL] = -chassis.move_speed / 0.06f * 19.0f;
    
}

/*�õ�������ǰ����λ�ú������������ָ���ת������Ȧ���ж�����*/
static void get_vehicle_position(void)
{
    if((right_wheel_motor.total_ecd < HIT_PROTECT_DISTANCE_CLOSE) && (ABS(left_wheel_motor.total_ecd) < HIT_PROTECT_DISTANCE_CLOSE))
    {
        chassis.vehicle_pos_status = CLOSE_START_ZONE;
    }
    else if(((right_wheel_motor.total_ecd > MOVE_LIMIT_FAR) && (right_wheel_motor.total_ecd < HIT_PROTECT_DISTANCE_FAR)) &&
        ((ABS(left_wheel_motor.total_ecd) > MOVE_LIMIT_FAR) && (ABS(left_wheel_motor.total_ecd) < HIT_PROTECT_DISTANCE_FAR)))
    {
        chassis.vehicle_pos_status = FAR_BREAK_ZONE;
    }
    else if((right_wheel_motor.total_ecd > MOVE_LIMIT_CLOSE) && (right_wheel_motor.total_ecd < MOVE_LIMIT_FAR) &&
        (ABS(left_wheel_motor.total_ecd) > MOVE_LIMIT_CLOSE) && (ABS(left_wheel_motor.total_ecd) < MOVE_LIMIT_FAR))
    {
        chassis.vehicle_pos_status = MIDDLE_ZONE;
    }
    else if(((right_wheel_motor.total_ecd < MOVE_LIMIT_CLOSE) && (right_wheel_motor.total_ecd > HIT_PROTECT_DISTANCE_CLOSE)) &&
        ((ABS(left_wheel_motor.total_ecd) < MOVE_LIMIT_CLOSE) && (ABS(left_wheel_motor.total_ecd) > HIT_PROTECT_DISTANCE_CLOSE)))
    {
        chassis.vehicle_pos_status = CLOSE_BREAK_ZONE;
    }
    else if((right_wheel_motor.total_ecd > HIT_PROTECT_DISTANCE_FAR) && (ABS(left_wheel_motor.total_ecd) > HIT_PROTECT_DISTANCE_FAR))
    {
        chassis.vehicle_pos_status = FAR_START_ZONE;
    }
}

/*����λ��У׼������ʹ��һ���ϵ͵��ٶ���У׼�㷽���ƶ�����΢������������ͣ��*/
static void vehicle_calibration(void)
{
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == GPIO_PIN_RESET)
    {
        chassis.move_speed = 0;
        if(duo_wheel_reinit())
        {
             chassis.calibration_status = CALIBRATION_INIT;
            chassis.vehicle_direction = MOVE_FAR_DIR;
        }
       
    }
    else
    {
        chassis.move_speed = CALIBRATION_MOVE_SPEED;
    }
    
}

/*���̵�����ݸ�У׼����*/
static int duo_wheel_reinit(void)
{
    memset(&right_wheel_motor.ecd,0,sizeof(right_wheel_motor.ecd));
    memset(&right_wheel_motor.total_ecd,0,sizeof(right_wheel_motor.total_ecd));
    memset(&right_wheel_motor.round_cnt,0,sizeof(right_wheel_motor.round_cnt));
    memset(&left_wheel_motor.ecd,0,sizeof(right_wheel_motor.ecd));
    memset(&left_wheel_motor.total_ecd,0,sizeof(right_wheel_motor.total_ecd));
    memset(&left_wheel_motor.round_cnt,0,sizeof(right_wheel_motor.round_cnt));
    return 1;
}


void chassis_pid_calcu(void)
{
    chassis.chassis_pid.left_spd_ref = chassis.wheel_speed[LEFT_WHEEL];
    chassis.chassis_pid.left_spd_fdb = left_wheel_motor.speed_rpm;
    chassis.chassis_pid.right_spd_ref = chassis.wheel_speed[RIGHT_WHEEL];
    chassis.chassis_pid.right_spd_fdb = right_wheel_motor.speed_rpm;
    chassis.wheel_current[LEFT_WHEEL] = (int16_t)pid_calc(&left_wheel_spd,chassis.chassis_pid.left_spd_fdb,chassis.chassis_pid.left_spd_ref);
    chassis.wheel_current[RIGHT_WHEEL] = (int16_t)pid_calc(&right_wheel_spd,chassis.chassis_pid.right_spd_fdb,chassis.chassis_pid.right_spd_ref);
}
