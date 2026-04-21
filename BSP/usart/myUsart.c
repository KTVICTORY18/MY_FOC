#include "myUsart.h"
#include "MT6825GT.h"
#include "user_protocol.h"
#include "openmv.h"


// DMA接收缓冲区
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];

/**
 * @brief 初始化串口2的DMA不定长接收
 * 
 * 使用UART空闲中断 + DMA接收实现不定长数据接收
 * 调用时机：在main函数初始化完成后调用
 */
void MyUsart_Init_DMA_Receive(void)
{

    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF);
    // 清空接收缓冲区
    memset(usart2_rx_buffer, 0, USART2_RX_BUFFER_SIZE);
    
    // 使能UART空闲中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    
    // 启动DMA接收（循环模式）
    HAL_UART_Receive_DMA(&huart2, usart2_rx_buffer, USART2_RX_BUFFER_SIZE);
}

/**
 * @brief UART空闲中断回调函数
 * 
 * 当检测到UART空闲时（一帧数据接收完成），触发此函数
 * 该函数需要在stm32g4xx_it.c的USART2_IRQHandler中调用
 */
void MyUsart_IDLE_Callback(void)
{
    // 检查是否是空闲中断
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)
    {
        // 清除空闲中断标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        
        // 停止DMA接收
        HAL_UART_DMAStop(&huart2);
        
        // 计算接收到的数据长度
        uint16_t recv_length = USART2_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        
        // 如果接收到数据，进行解析
        if (recv_length > 0)
        {
            if (OpenMV_ParsePacket(&openmv, usart2_rx_buffer, recv_length)) {//openmv的数据包由0xBB开头
                // OpenMV 数据包解析成功，无需其他处理
            } else {
                // 不是 OpenMV 数据包，调用原有的协议解析函数（上位机命令）
                Protocol_Parse(usart2_rx_buffer, recv_length);
            }
        }
        
        // 清空接收缓冲区
        memset(usart2_rx_buffer, 0, USART2_RX_BUFFER_SIZE);
        
        // 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart2, usart2_rx_buffer, USART2_RX_BUFFER_SIZE);
    }
}


void MyUsart_SendAll(float iq, float id, float iu, float iv, float iw, float ud, float uq,float iq_ref,float id_ref, float target_speed,float current_speed,float va,float vb,float vc,float alpha,float beta,float target_angle,float current_angle)
{
    static char tx_buffer[USART2_TX_BUFFER_SIZE];
    int len;
    
    // VOFA+ FireWater协议：数据用英文逗号分隔，以\n结尾
    len = snprintf(tx_buffer, USART2_TX_BUFFER_SIZE, 
                   "%.3f, %.3f, %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", 
                   iq, id, iu, iv, iw, ud, uq, iq_ref, id_ref, target_speed, current_speed, va, vb, vc, alpha, beta, target_angle, current_angle);
    
    if (len > 0 && len < USART2_TX_BUFFER_SIZE)
    {
        // 等待上一次DMA传输完成
        while (huart2.gState != HAL_UART_STATE_READY)
        {
            // 如果DMA正在传输，等待完成
            if (HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
            {
                // 可以添加超时处理，这里简单等待
            }
        }
        
        // 通过DMA发送数据
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)tx_buffer, len);
    }
}


/**
 * @brief 通过串口2的DMA同时输出FOC控制结构体和电流采样结构体中的所有电流值（从全局变量读取）
 */
void MyUsart_SendAllCurrentsFromGlobal(void)
{
    MyUsart_SendAll(
        foc_ctrl.iq_actual,
        foc_ctrl.id_actual,
        current_sense.currents.Iu,
        current_sense.currents.Iv,
        current_sense.currents.Iw,
        foc_ctrl.ud,
        foc_ctrl.uq,
        foc_ctrl.iq_ref,
        foc_ctrl.id_ref,
        foc_ctrl.target_speed,
        foc_ctrl.current_speed,
        three_phase.va,
        three_phase.vb,
        three_phase.vc,
        alpha_beta.alpha,
        alpha_beta.beta,
        foc_ctrl.target_angle,
        angle_data.angle_rad
    );
}