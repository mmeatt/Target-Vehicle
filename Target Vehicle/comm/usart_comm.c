#include "usart_comm.h"
#include "remote_comm.h"
#include "judge_comm.h"
#include "cmsis_os.h"
#include "math_calcu.h"
#include "string.h"
#include "usart.h"
#include "wfly_comm.h"

uint8_t	dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_sbus_buf[25];

void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) //DBUS����
    {
//        rc_callback_handler(&rc,dma_dbus_buf);
//        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
//        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
        sbus_callback_handler(&SBUS,dma_sbus_buf);
        memset(dma_sbus_buf, 0, 25);
        HAL_UART_Receive_DMA(huart, dma_sbus_buf, 25);
    }
    else if (huart->Instance == USART2)	//JUDGE����
    {
        judge_data_handler(dma_judge_buf);
        memset(dma_judge_buf, 0, DMA_JUDGE_LEN);
        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
    }
}

/**
  * @brief ���ڿ����ж�   ע������it.c��ÿ�����ڵ��ж��е��øú���
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //�ж��Ƿ��ǿ����ж�
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
//        HAL_UART_DMAStop(huart);							//ֹͣ����DMA����
		HAL_UART_AbortReceive(huart);						//���رս����ж�
        USER_UART_IDLECallback(huart);                     //���ô��ڹ��ܻص�����
    }
}

/**
* @brief  ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void USER_UART_Init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);

    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
}



