/******************************************************************************
 *@brief
 *@copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
 *@license
 *******************************************************************************/
#ifndef __pid_H__
#define __pid_H__

#include "stdint.h"
#include "stm32f4xx_hal.h"

typedef enum
{
    POSITION_PID,
    DELTA_PID,
} PID_mode_e;

typedef enum pid_Improvement_e
{
    NONE_IMPROVE = 0X00,                 //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegrationRate = 0x20,     //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef struct __pid_t
{
    float p;
    float i;
    float d;

    float set[3]; // Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float get[3]; // ����ֵ
    float err[3]; // ���

    float d_error;
    float pout; // p���
    float iout; // i���
    float dout; // d���
    float ITerm;

    float pos_out;      // ����λ��ʽ���
    float last_pos_out; // �ϴ����
    float delta_u;      // ��������ֵ
    float delta_out;    // ��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;

    float CoefA; // For Changing Integral
    float CoefB; // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B

    float max_err;
    float deadband; // err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;     // ����޷�
    uint32_t IntegralLimit; // �����޷�

    uint8_t Improve;

    uint32_t DWT_CNT;
    float dt;

    float derivative_lpf_rc;

    void (*f_param_init)(struct __pid_t *pid, // PID������ʼ��
                         uint32_t maxout,
                         uint32_t intergral_limit,
                         float kp,
                         float ki,
                         float kd,
                         float deadband,
                         float A,
                         float B,
                         uint8_t improve);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d); // pid���������޸�

} pid_t;

void PID_struct_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd,
    float deadband,
    float A,
    float B,
    uint8_t improve);

typedef struct
{
    float K1;
    float K2;
    float Last_DeltIn;
    float Now_DeltIn;
    float Out;
    float OutMax;
} FeedForward_Typedef;

typedef struct
{
    float k;
    float Last_ref;
    float Now_ref;
    float Out;
    float OutMax;
    float slope;
} D_AGL_FeedForward_Typedef;

float pid_calc(pid_t *pid, float fdb, float ref);
void abs_limit(float *a, float ABS_MAX, float offset);
void D_AGL_FeedForward_Calc(D_AGL_FeedForward_Typedef *AGL_FF, float now_Ref, float period);
void FeedForward_Calc(FeedForward_Typedef *FF, float Now_DeltIn);
// PID�Ż����ں�������
static void f_Trapezoid_Intergral(pid_t *pid);
static void f_Integral_Limit(pid_t *pid);
static void f_Derivative_On_Measurement(pid_t *pid);
static void f_Changing_Integration_Rate(pid_t *pid);
static void f_Derivative_Filter(pid_t *pid);
#endif
