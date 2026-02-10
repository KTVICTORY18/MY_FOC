/**
 ******************************************************************************
 * @file           : can_comm.c
 * @brief          : FDCAN通信封装实现
 * @description    : 封装FDCAN的初始化、发送和接收功能
 ******************************************************************************
 */

#include "can_comm.h"

// 外部FDCAN句柄
extern FDCAN_HandleTypeDef hfdcan1;

/**
 * @brief 数据长度转换为FDCAN DLC编码
 * @param length: 数据长度（字节）
 * @return FDCAN DLC编码
 * 
 * @note CAN FD只支持特定长度：0-8, 12, 16, 20, 24, 32, 48, 64
 *       如果输入长度不是这些值，会自动向上取整到最接近的支持长度
 *       例如：14字节 -> 使用16字节DLC
 */
static uint32_t CAN_Comm_LengthToDLC(uint8_t length)
{
    if (length <= 8)
        return (uint32_t)length << 16;
    else if (length <= 12)
        return FDCAN_DLC_BYTES_12;
    else if (length <= 16)
        return FDCAN_DLC_BYTES_16;
    else if (length <= 20)
        return FDCAN_DLC_BYTES_20;
    else if (length <= 24)
        return FDCAN_DLC_BYTES_24;
    else if (length <= 32)
        return FDCAN_DLC_BYTES_32;
    else if (length <= 48)
        return FDCAN_DLC_BYTES_48;
    else
        return FDCAN_DLC_BYTES_64;
}

/**
 * @brief FDCAN DLC编码转换为数据长度
 * @param dlc: FDCAN DLC编码
 * @return 数据长度（字节）
 */
static uint8_t CAN_Comm_DLCToLength(uint32_t dlc)
{
    uint8_t length = (dlc >> 16) & 0x0F;
    
    if (length <= 8)
        return length;
    
    // FD格式的特殊长度
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 8;
    }
}

/**
 * @brief 初始化FDCAN通信
 * @note 配置过滤器、启用中断、启动FDCAN
 * @return HAL_StatusTypeDef
 * 
 * 使用说明：
 * 1. 在main.c中调用MX_FDCAN1_Init()之后调用此函数
 * 2. 此函数会配置接收ID=1的报文
 * 3. 接收到的数据会通过CAN_Comm_RxCallback回调函数返回
 */
HAL_StatusTypeDef CAN_Comm_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    
    // 配置标准ID过滤器 - 只接收ID=1的报文
    sFilterConfig.IdType = FDCAN_STANDARD_ID;           // 标准ID（11位）
    sFilterConfig.FilterIndex = 0;                       // 使用过滤器0
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;        // 掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 接收到FIFO0
    sFilterConfig.FilterID1 = CAN_COMM_TARGET_ID;        // 过滤器ID = 1
    sFilterConfig.FilterID2 = 0x7FF;                     // 掩码（0x7FF表示精确匹配）
    
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    // 配置全局过滤器（拒绝不匹配的帧）
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 
                                      FDCAN_REJECT,      // 拒绝不匹配的标准ID
                                      FDCAN_REJECT,      // 拒绝不匹配的扩展ID
                                      FDCAN_FILTER_REMOTE, 
                                      FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    // 启用FIFO0新消息中断
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, 
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 
                                        0) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    // 启动FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief 发送数据到默认ID（ID=1）
 * @param data: 数据指针
 * @param length: 数据长度（1-64字节）
 * @return HAL_StatusTypeDef
 * 
 * 使用示例：
 * uint8_t tx_data[] = {0xAA, 0x01, 0x02, 0x03, 0xFF};
 * CAN_Comm_Transmit(tx_data, 5);
 */
HAL_StatusTypeDef CAN_Comm_Transmit(uint8_t *data, uint8_t length)
{
    return CAN_Comm_Transmit_To_ID(CAN_COMM_TARGET_ID, data, length);
}

/**
 * @brief 发送数据到指定CAN ID
 * @param can_id: CAN ID（标准ID: 0-0x7FF）
 * @param data: 数据指针
 * @param length: 数据长度（1-64字节）
 * @return HAL_StatusTypeDef
 * 
 * 使用示例：
 * uint8_t tx_data[] = {0xAA, 0x01, 0x02, 0x03, 0xFF};
 * CAN_Comm_Transmit_To_ID(0x123, tx_data, 5);
 */
HAL_StatusTypeDef CAN_Comm_Transmit_To_ID(uint32_t can_id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    
    // 检查参数
    if (data == NULL || length == 0 || length > CAN_COMM_MAX_DATA_LEN)
    {
        return HAL_ERROR;
    }
    
    // 配置发送头
    TxHeader.Identifier = can_id;                    // CAN ID
    TxHeader.IdType = FDCAN_STANDARD_ID;             // 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;         // 数据帧
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;          // 不使用位速率切换
    TxHeader.FDFormat = FDCAN_FD_CAN;                // FD格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    
    // 设置数据长度（转换为DLC编码）
    TxHeader.DataLength = CAN_Comm_LengthToDLC(length);
    
    // 发送数据
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
}

/**
 * @brief FDCAN接收FIFO0回调函数
 * @note 此函数会被HAL库自动调用，不要手动调用
 * @param hfdcan: FDCAN句柄
 * @param RxFifo0ITs: 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[CAN_COMM_MAX_DATA_LEN];
        
        // 从FIFO0读取消息
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 转换数据长度
            uint8_t data_length = CAN_Comm_DLCToLength(RxHeader.DataLength);
            
            // 调用用户回调函数
            CAN_Comm_RxCallback(RxHeader.Identifier, RxData, data_length);
        }
    }
}

void CAN_Comm_RxCallback(uint32_t can_id, uint8_t *data, uint8_t length)
{
    
}
