/**
 * @file MT6825GT.h
 * @brief MT6825GT-STD磁编码器驱动头文件
 * 
 * MT6825GT-STD是18位绝对位置磁编码器，通过SPI接口通信
 * - 分辨率：18位（262144步，约0.00137度/步）
 * - SPI模式：Mode 3 (CPOL=1, CPHA=1)
 * - CS引脚：PA4
 * - 需要读取3个字节（24位），取低18位作为角度值
 */

#ifndef __MT6825GT_H__
#define __MT6825GT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"

//径向磁铁离磁编码器最好为2.5mm,3.0mm数据会有翻转的跳变比如从220度到40度

// MT6825GT配置
#define MT6825_CS_PORT           GPIOA
#define MT6825_CS_PIN            GPIO_PIN_4
#define MT6825_ANGLE_MAX         262144.0f  // 18位最大值 (2^18)，使用浮点数提高精度
#define MT6825_ANGLE_MASK        0x3FFFF    // 18位掩码
// 定义常量，避免重复计算
#define MT6825_RAW_TO_RAD_CONST  (2.0f * 3.1415926535f / 262144.0f)  // 原始值转为弧度一圈2π
#define MT6825_RAW_TO_DEG_CONST  (360.0f / 262144.0f)  // 原始值转为角度一圈360度

// 已移除未使用的寄存器地址和状态位定义，直接使用数值提高效率

// MT6825GT角度数据结构体
typedef struct {
    uint32_t angle_raw;      // 原始角度值（18位，0-262143）
    float angle_deg;        // 角度（度，0.0-360.0）
    float angle_rad;        // 角度（弧度，0.0-2π）
    uint8_t data_valid;     // 数据有效性标志（1-有效，0-无效）
    // 状态位
    uint8_t no_mag_warning; // 无磁警告（寄存器0x04 bit[1]）
    uint8_t over_speed;     // 超速警告（寄存器0x05 bit[3]）
    uint8_t pc1;            // 奇偶校验位PC1（寄存器0x04 bit[0]）
    uint8_t pc2;            // 奇偶校验位PC2（寄存器0x05 bit[2]）
} MT6825_AngleData_t;

extern MT6825_AngleData_t angle_data;

// 函数声明
void MT6825_Init(void);
void MT6825_ReadAngleData(MT6825_AngleData_t *data);  // 一次性读取所有角度数据到结构体


#ifdef __cplusplus
}
#endif

#endif /* __MT6825GT_H__ */

