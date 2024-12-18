/**
  * @file     math_calcu.c
  * @version  v2.0
  * @date     July,6th 2019
	*
  * @brief    ��ѧ���㺯��
	*
  *	@author   Fatmouse
  *
  */
#include "math_calcu.h"
#include "string.h"
#include "math.h"
#include "cmsis_os.h"

/**
  * @brief          б���������㣬���������ֵ���е��ӣ����뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float frame_period, float input, float max, float min)
{
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->frame_period = frame_period;

    ramp_source_type->input = input;

    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;

    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

void slope_calc(slope_t *V)
{
    V->last_ticks = V->ticks;
    V->ticks = osKernelSysTick();
	if((V->ticks-V->last_ticks)<10)		//С�۷��ʼ����ֹ����У׼
	{
		if(V->real_target !=V->limit_target)
		{
			if(V->real_target < V->limit_target)//�Ӳ���
			{
				V->real_target = V->real_target + V->change_scale* (V->ticks-V->last_ticks);
				if(V->real_target > V->limit_target)//�޷�
				{
					V->real_target =  V->limit_target;
				}
			}
			else if(V->real_target > V->limit_target)//������
			{
				V->real_target =  V->real_target - V->change_scale* (V->ticks-V->last_ticks);
				if(V->real_target < V->limit_target)//�޷�
				{
					V->real_target =  (short)V->limit_target;
				}
			}
		}
	}
}

/**
  * @brief          ð��������
  * @param[in]      ����Ӵ�С����
  * @retval         void
  */
void Bubble_Sort(float *a,uint8_t n)
{
    float buf;
    for(uint8_t i=0; i<n; ++i)
    {
        for(uint8_t j=0; j<n-1-i; ++j)
        {
            if(a[j]<a[j+1])
            {
                buf=a[j];
                a[j] = a[j+1];
                a[j+1] = buf;
            }
        }
    }
}

/**
  * @brief          �����˲�����   ע��N2Ϊ�������ڴ�С
  * @author
  * @param[in]      �˲�ǰ��ֵ���˲���������
  * @retval         �˲����ֵ
  */
float GildeAverageValueFilter(float NewValue,float *Data)
{
    float max,min;
    float sum;
    uint16_t i;
    Data[0]=NewValue;
    max=Data[0];
    min=Data[0];
    sum=Data[0];
    for(i=N2-1; i!=0; i--)
    {
        if(Data[i]>max) max=Data[i];
        else if(Data[i]<min) min=Data[i];
        sum+=Data[i];
        Data[i]=Data[i-1];
    }
    i=N2-2;
    sum=sum-max-min;
    sum=sum/i;
    return(sum);
}

/**
  * @brief          Sigmoid����
  * @author
  * @param[in]      �Ա���
  * @retval         ӳ���ĺ���ֵ
  */
float Sigmoid_function(float x)
{
    float y;
    float temp_x=ABS(x) - SIGMOID_MAX;			//��sigmoid��������ƽ���������ֵ
    y= 1 / (1 + powf(e,-temp_x));
    return y;
}


/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		�������ݼ���ƫ��ֵ
	*@param[in] set �趨ֵ get����ֵ circle_para һȦ��ֵ
	*@note	���������£�ֱ�Ӽ����PID�е�ƫ��ֵ
*/
float circle_error(float set,float get,float circle_para)
{
    float error;
    if(set > get)
    {
        if(set - get> circle_para/2)
            error = set - get - circle_para;
        else
            error = set - get;
    }
    else if(set < get)
    {
        if(set - get<-1*circle_para/2)
            error = set - get +circle_para;
        else
            error = set - get;
    }
    else	error = 0;

    return error;
}

float data_limit(float data, float max, float min)
{
    if(data >= max)					return max;
    else if(data <= min)		return min;
    else 										return data;
}

/**
 * @brief ��С���˷�ֱ����� y=ax+b������ϵ��a��b
 * 
 * @param x[] ��ϵ������  
 * @param y[] ��ϵ�������
 * @param num ��ϵ���
 * @param a   ϵ��a����ָ��
 * @param b   ϵ��b����ָ��
 * @attention num �����������Ԫ�ظ�����x[]��y[]��Ԫ�ظ����������
 */
void least_square_linear_fit(float x[], float y[], const int num, float* a, float* b)
{
    float sum_x2 = 0.0f;
    float sum_y  = 0.0f;
    float sum_x  = 0.0f;
    float sum_xy = 0.0f;

    for (int i = 0; i < num; ++i)
    {
        sum_x2 += x[i]*x[i];
        sum_y  += y[i];
        sum_x  += x[i];
        sum_xy += x[i]*y[i];
    }

    *a = (num*sum_xy - sum_x*sum_y)/(num*sum_x2 - sum_x*sum_x);
    *b = (sum_x2*sum_y - sum_x*sum_xy)/(num*sum_x2-sum_x*sum_x);
}

/**
 * @brief �������Ǽ���
 * 
 * @param x     ����x����
 * @param y     ����y����
 * @param x_th  x���긡���������߽���������
 * @param y_th  y���긡���������߽���������
 * @return float ��������(-PI,PI] (rad)������������Ĭ��Ϊ0
 */
float vector_arg(float x, float y, float x_th, float y_th) {
    float arg = 0;
    x_th = x_th > 0? x_th: -x_th;
    y_th = y_th > 0? y_th: -y_th;

    if (x > x_th) {
        if (y > y_th || y < -y_th) {
            arg = atanf(y/x);           /* ��һ�������� */
        } else {
            arg = 0;                    /* x������ */
        }
    } else if (x >= -x_th && x <= x_th) {
        if (y > y_th) {
            arg = PI / 2;               /* y������ */
        } else if (y < -y_th) {
            arg = -PI / 2;              /* y������ */
        } else {
            arg = 0;                    /* ԭ������ */
        }
    } else if (x < -x_th) {
        if (y > y_th) {
            arg = atanf(y/x) + PI;      /* �ڶ����� */
        } else if (y < -y_th) {
            arg = atanf(y/x) - PI;      /* �������� */
        } else {
            arg = PI;                   /* x������ */
        }
    }
    return arg;
}

/* �ͻ��������� */
void delay_loop_init(delay_loop_t* dlp, float max, float min) {
    memset(dlp, 0, sizeof(delay_loop_t));
    if (max < min) {
        float temp = max;
        max = min;
        min = temp;
        dlp->max = max;
        dlp->min = min;
    }
}

int8_t delay_loop_calc(delay_loop_t* dlp, float now_data) {
    dlp->now = now_data;
    if (dlp->now > dlp->max && dlp->last < dlp->max) {  /* ������ */
       dlp->out = 1;
    } else if (dlp->now < dlp->min && dlp->last > dlp->min) {  /* ������ */
        dlp->out = -1;
    }
    dlp->last = dlp->now;
    return dlp->out;
}





