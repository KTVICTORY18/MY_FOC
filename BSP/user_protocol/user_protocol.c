#include "user_protocol.h"
#include "storage.h"
#include <string.h>

/**
 * @brief CRC8校验计算（多项式0x07）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC8校验值
 */
uint8_t CRC8_Calculate(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;
    
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief 协议数据解析函数
 * 
 * 协议格式：
 * - AA + 功能码 + FOC_Mode_t + 4/8字节数据 + CRC8 + FF
 * - 功能码0x01: 设置模式参数
 *   - FOC_MODE_SPEED_ANGLE_LOOP: 8字节数据（两个float：角度+速度）
 *   - 其他模式: 4字节数据（一个float）
 * - 功能码0x02: FOC校准并保存到Flash
 * 
 */
void Protocol_Parse(const uint8_t *data, uint16_t length)
{
    if(data[0] != PROTOCOL_HEADER||data[length-1] != PROTOCOL_TAIL)
    {
        return;
    }
    // 4. 提取功能码
    uint8_t func_code = data[1];
    uint8_t data_length = 0;
    switch (func_code)
    {
        //设置模式参数功能码 0x01
        case FUNC_CODE_SET_PARAMETER:
        {
            FOC_Mode_t mode = (FOC_Mode_t)data[2];
            
            if (mode == FOC_MODE_SPEED_ANGLE_LOOP)
            {
                data_length = 8;  // 两个float
            }
            else
            {
                data_length = 4;  // 一个float
            }
            
            // CRC校验（校验范围：功能码 + Mode + 数据）
            uint8_t crc_received = data[length - 2];  // CRC在倒数第二个字节
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 2 + data_length);  // 功能码 + Mode + 数据
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            
            // 解析数据并调用对应函数
            float value1 = 0.0f;
            float value2 = 0.0f;
            
            // 提取第一个float（小端序）
            memcpy(&value1, &data[3], sizeof(float));
            
            if (mode == FOC_MODE_SPEED_ANGLE_LOOP)
            {
                // 提取第二个float
                memcpy(&value2, &data[7], sizeof(float));
                
                // 调用速度+角度模式函数
                FOC_Set_Speed_Angle(value1, value2);  // value1=角度, value2=速度
            }
            else
            {   
                // 调用单参数模式函数
                FOC_Set_Parameter(mode, value1);
            }
            
            break;
        }
        
        //FOC校准并保存到Flash功能码 0x02
        //AA 02 0E FF
        case FUNC_CODE_CALIBRATE:  
        {
            
            // CRC校验（校验范围：功能码 + Mode字节）
            uint8_t crc_received = data[length - 2];
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 1);  // 功能码 + Mode字节
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            // 执行FOC校准
            FOC_Calibrate();
            // 保存配置到Flash
            Storage_WriteConfig(&config_info);
            break;
        }
        //读取配置信息功能码 0x03
        case FUNC_CODE_READ_CONFIG:  
            break;
        //擦除配置信息功能码 0x04
        case FUNC_CODE_ERASE_CONFIG:  
            break;
        default:
            break;
    }
}
