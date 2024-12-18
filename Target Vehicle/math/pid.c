#include "pid.h"
#include "string.h"
enum
{
    LLAST = 0,
    LAST = 1,
    NOW = 2,
};

pid_t pid_chassis_spd[4];
#ifndef ABS
#define ABS(x) ((x > 0) ? (x) : (-x))
#endif

#define LIMIT(x, limit) (x) = (((x) <= (-limit)) ? (-limit) : (((x) >= (limit)) ? (limit) : (x)))

/*������ʼ��--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid,
    uint32_t maxout,
    uint32_t intergral_limit,
    float kp,
    float ki,
    float kd,
    float deadband,
    float A,
    float B,
    uint8_t improve)
{

    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->deadband = deadband;

    pid->p = kp;
    pid->i = ki;
    pid->d = kd;

    pid->CoefA = A;
    pid->CoefB = B;

    pid->Improve = improve;
}

/*��;���Ĳ����趨(����)------------------------------------------------------------*/
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
 * ������PID��λ��PID
 *
 * ���������ڸ���Ŀ��ֵ�Ͳ���ֵ����PID���������������֧�ֶ��ָĽ��㷨���������λ��֡����ٻ��֡�΢�����С�΢���˲����ͻ����޷���
 *
 * @param[in] pid ָ��pid_t�ṹ���ָ�룬����PID���ƵĲ�����״̬��
 * @param[in] get ��ǰ����ֵ��
 * @param[in] set Ŀ��ֵ��
 * @return ����õ���PID���ֵ������PID�Ĺ���ģʽ�����ص�λ��PID�������PID�����
 */
float pid_calc(pid_t *pid, float get, float set)
{
    // ���µ�ǰ�Ĳ���ֵ��Ŀ��ֵ�����
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get; // �������

    // �������Ƿ񳬹����������ƣ����Ƿ���������Χ��
    if (pid->max_err != 0 && ABS(pid->err[NOW]) > pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    // ����PID���������
    pid->pout  = pid->p * pid->err[NOW];                    // ������
    pid->ITerm = pid->i * pid->err[NOW];                   // �������ʱֵ��
    pid->dout  = pid->d * (pid->err[NOW] - pid->err[LAST]); // ΢����

    // Ӧ�ø��ָĽ��㷨
    // ���λ���
    if (pid->Improve & Trapezoid_Intergral)
        f_Trapezoid_Intergral(pid);
    // ���ٻ���
    if (pid->Improve & ChangingIntegrationRate)
        f_Changing_Integration_Rate(pid);
    // ΢������
    if (pid->Improve & Derivative_On_Measurement)
        f_Derivative_On_Measurement(pid);
    // ΢���˲���
    if (pid->Improve & DerivativeFilter)
        f_Derivative_Filter(pid);
    // �����޷�
    if (pid->Improve & Integral_Limit)
        f_Integral_Limit(pid);

    // �����ۼ�
    pid->iout += pid->ITerm;

    // �Ի�����������޷�
    abs_limit(&(pid->iout), pid->IntegralLimit, 0);

    // �������յ�PID���
    pid->pos_out = pid->pout + pid->iout + pid->dout; // ����λ��PID���

    // ��PID����������ֵ����
    abs_limit(&(pid->pos_out), pid->MaxOutput, 0);

    // �����ϴ����ֵ
    pid->last_pos_out = pid->pos_out;

    // �������Ͳ���ֵ����ʷ��¼
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];

    // ����PIDģʽ������Ӧ�����ֵ
    return pid->pos_out;
}

/*pid�����ʼ��-----------------------------------------------------------------*/
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
    uint8_t improve)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
    /*init pid param */
    pid->f_param_init(pid, maxout, intergral_limit, kp, ki, kd, deadband, A, B, improve);
}

void abs_limit(float *a, float ABS_MAX, float offset)
{
    if (*a > ABS_MAX + offset)
        *a = ABS_MAX + offset;
    if (*a < -ABS_MAX + offset)
        *a = -ABS_MAX + offset;
}

/**********************************************************************************************************
 *�� �� ��: FeedForward_Calc
 *����˵��: ǰ���㷨
 *��    ��: PID_Struct *P  PID�����ṹ��
 *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
 *�� �� ֵ: PID�����������ֵ
 **********************************************************************************************************/
void FeedForward_Calc(FeedForward_Typedef *FF, float Now_DeltIn)
{
    FF->Now_DeltIn = Now_DeltIn;
    FF->Out = FF->Now_DeltIn * FF->K1 + (FF->Now_DeltIn - FF->Last_DeltIn) * FF->K2;
    FF->Last_DeltIn = FF->Now_DeltIn;
    abs_limit(&(FF->Out), FF->OutMax, 0);
}

/**********************************************************************************************************
 *�� �� ��: D_AGL_FeedForward_Calc
 *����˵��: ǰ���㷨(�����⻷����΢��)
 *��    ��: PID_Struct *P  PID�����ṹ��
 *        ActualValue    PID���㷴��������ǰ��ʵ���ֵ��
 *�� �� ֵ: PID�����������ֵ
 **********************************************************************************************************/
void D_AGL_FeedForward_Calc(D_AGL_FeedForward_Typedef *AGL_FF, float now_Ref, float period)
{
    AGL_FF->Now_ref = now_Ref;
    AGL_FF->slope = (AGL_FF->Now_ref - AGL_FF->Last_ref);
    AGL_FF->Out = AGL_FF->k * ((AGL_FF->Now_ref - AGL_FF->Last_ref) / period);
    AGL_FF->Last_ref = AGL_FF->Now_ref;
}

/*
 * �������ƣ�f_Trapezoid_Intergral
 * �������������㲢����PID�������Ļ����
 * �����б�
 *  - pid: ָ�����PID����״̬�Ľṹ���ָ�롣
 * ����ֵ���ޡ�
 */

static void f_Trapezoid_Intergral(pid_t *pid)
{
    // ���������������η�����л���
    pid->ITerm = pid->i * ((pid->err[NOW] + pid->err[LAST]) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(pid_t *pid)
{
    if (pid->err[NOW] * pid->iout > 0)
    {
        // ���ֳ��ۻ�����
        // Integral still increasing
        if (ABS(pid->err[NOW]) <= pid->CoefB)
            return;                                          // Full integral
        if (ABS(pid->err[NOW]) <= (pid->CoefA + pid->CoefB)) //       B < err < A+B
            pid->ITerm *= (pid->CoefA - ABS(pid->err[NOW]) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0; // cancel integral
    }
}

static void f_Integral_Limit(pid_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->iout + pid->ITerm;
    temp_Output = pid->pout + pid->iout + pid->dout;
    if (ABS(temp_Output) > pid->MaxOutput)
    {
        if (pid->err[NOW] * pid->iout > 0)
        {
            // ���ֳ��ۻ�����
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (ABS(temp_Iout) > ABS(pid->IntegralLimit))

        pid->ITerm = 0;

    LIMIT(temp_Iout, pid->IntegralLimit);
}

static void f_Derivative_On_Measurement(pid_t *pid)
{

    // if (pid->OLS_Order > 2)
    //     pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
    // else
    //     pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

static void f_Derivative_Filter(pid_t *pid)
{
    pid->dout = pid->dout * pid->dt / (pid->derivative_lpf_rc + pid->dt) +
                pid->last_delta_out * pid->derivative_lpf_rc / (pid->derivative_lpf_rc + pid->dt);
}
