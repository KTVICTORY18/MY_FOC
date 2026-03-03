/**
 ******************************************************************************
 * @file           : can_comm.h
 * @brief          : FDCAN通信封装头文件
 * @description    : 封装FDCAN的初始化、发送和接收功能
 ******************************************************************************
 */

#ifndef __CAN_COMM_H__
#define __CAN_COMM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include "user_protocol.h"

// CAN通信配置
#define CAN_COMM_TARGET_ID      0x001   // 目标设备ID（接收和发送都使用ID=1）
#define CAN_COMM_MAX_DATA_LEN   64      // FDCAN最大数据长度

// 函数声明
HAL_StatusTypeDef CAN_Comm_Init(void);
HAL_StatusTypeDef CAN_Comm_Transmit(uint8_t *data, uint8_t length);
HAL_StatusTypeDef CAN_Comm_Transmit_To_ID(uint32_t can_id, uint8_t *data, uint8_t length);

// 接收回调函数（用户需要实现）
void CAN_Comm_RxCallback(uint32_t can_id, uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_COMM_H__ */
