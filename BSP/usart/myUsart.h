#ifndef __MYUSART_H__
#define __MYUSART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include "foc.h"
#include "current_sense.h"
#include <stdio.h>
#include <string.h>


// DMA接收缓冲区大小
#define USART2_RX_BUFFER_SIZE 128
// 发送缓冲区大小
#define USART2_TX_BUFFER_SIZE 256

// 外部变量声明
extern uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;

// 函数声明
void MyUsart_SendAllCurrentsFromGlobal(void);
void MyUsart_Init_DMA_Receive(void);
void MyUsart_IDLE_Callback(void);

#ifdef __cplusplus
}
#endif

#endif // !1