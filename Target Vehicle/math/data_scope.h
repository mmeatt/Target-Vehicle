#ifndef __DATA_PRTOCOL_H__
#define __DATA_PRTOCOL_H__

#include "usart.h"

//#define MINIBALANCE       //��Ҫʹ��VOFA+������λ������ע�ʹ˺�
#define MAX_SEND_NUM 10     //MINIBALANCE���10��ͨ����VOFA+���Գ���10��ͨ��

void DataWavePkg(void);                         //��д��������
void DataScope_Get_Channel_Data(float Data);    //��˳��ע�����ӡ����
void DataWave(UART_HandleTypeDef* huart);       //һ���԰�˳��ע�����ݴ�ӡ����λ��

#endif
