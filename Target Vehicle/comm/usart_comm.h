#ifndef _USART_COMM_H_
#define _USART_COMM_H_

#include "usart.h"

#define     DBUS_HUART      huart1
#define     JUDGE_HUART     huart2

#define  DMA_DBUS_LEN       18
#define  DMA_JUDGE_LEN		100

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif
