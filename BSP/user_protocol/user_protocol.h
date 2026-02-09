#ifndef __USER_PROTOCOL_H__
#define __USER_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "foc.h"

// 协议帧格式定义
#define PROTOCOL_HEADER         0xAA    // 帧头
#define PROTOCOL_TAIL           0xFF    // 帧尾
#define PROTOCOL_MIN_LENGTH     8       // 最小帧长度: AA + 功能码 + Mode + 4字节数据 + CRC + FF
#define PROTOCOL_MAX_LENGTH     12      // 最大帧长度: AA + 功能码 + Mode + 8字节数据 + CRC + FF

// 功能码定义
#define FUNC_CODE_SET_PARAMETER     0x01    // 设置模式参数
#define FUNC_CODE_CALIBRATE         0x02    // FOC校准并保存到Flash
#define FUNC_CODE_READ_CONFIG       0x03    // 读取配置信息（预留）
#define FUNC_CODE_ERASE_CONFIG      0x04    // 擦除配置信息（预留）

// 函数声明
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length);
void Protocol_Parse(const uint8_t *data, uint16_t length);

extern uint8_t calibrated_flag;//主循环调用的校准标志位

#ifdef __cplusplus
}
#endif

#endif /* __USER_PROTOCOL_H__ */
