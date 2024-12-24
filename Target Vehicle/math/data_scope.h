#ifndef __DATA_PRTOCOL_H__
#define __DATA_PRTOCOL_H__

#include "usart.h"

//#define MINIBALANCE       //若要使用VOFA+串口上位机，则注释此宏
#define MAX_SEND_NUM 10     //MINIBALANCE最多10个通道，VOFA+可以超过10个通道

void DataWavePkg(void);                         //改写次弱函数
void DataScope_Get_Channel_Data(float Data);    //按顺序注册待打印数据
void DataWave(UART_HandleTypeDef* huart);       //一次性按顺序将注册数据打印到上位机

#endif
