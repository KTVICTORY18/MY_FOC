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
// 功能码定义
#define FUNC_CODE_SET_PARAMETER     0x01    // 设置模式参数（速度环、电流环、低速环、步进角度环）
#define FUNC_CODE_CALIBRATE         0x02    // FOC校准并保存到Flash
#define FUNC_CODE_MODIFY_ID         0x03    // 修改电机id
#define FUNC_CODE_ABS_ANGLE_SPEED   0x04    // 设置绝对角度+速度（以指定速度运动到绝对角度）
#define FUNC_CODE_REL_ANGLE_SPEED   0x05    // 设置相对角度+速度（以指定速度相对运动指定角度）


// 函数声明
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length);
void Protocol_Parse(const uint8_t *data, uint16_t length);

extern uint8_t calibrated_flag;     // 主循环调用的校准标志位
extern uint8_t save_config_flag;    // 主循环调用的保存配置标志位

#ifdef __cplusplus
}
#endif

#endif /* __USER_PROTOCOL_H__ */
