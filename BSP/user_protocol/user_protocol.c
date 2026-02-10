#include "user_protocol.h"
#include "storage.h"
#include <string.h>


uint8_t calibrated_flag = 0;
uint8_t save_config_flag = 0;

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
 * - AA + ID + 功能码 + FOC_Mode_t + 4/8字节数据 + CRC8 + FF
 * - 功能码0x01: 设置模式参数
 *   - FOC_MODE_SPEED_ANGLE_LOOP: 8字节数据（两个float：角度+速度）
 *   - 其他模式: 4字节数据（一个float）
 * - 功能码0x02: FOC校准并保存到Flash
 * - 功能码0x03: 修改电机ID
 * 
 * 数据例子：
 * 1. 功能码0x01 - 速度+角度模式（以20r/min的速度运动到3.14rad）：
 *    AA 01 01 05 C3 F5 48 40 00 00 A0 41 E7 FF
 *    解析：AA(帧头) 01(ID=1) 01(功能码) 05(模式) C3F54840(角度3.14) 0000A041(速度20) E7(CRC) FF(帧尾)
 * 
 * 2. 功能码0x01 - 速度+角度模式（以-20r/min的速度运动到3.14rad）：
 *    AA 01 01 05 C3 F5 48 40 00 00 A0 C1 6E FF
 *    解析：AA(帧头) 01(ID=1) 01(功能码) 05(模式) C3F54840(角度3.14) 0000A0C1(速度-20) 6E(CRC) FF(帧尾)
 * 
 * 3. 功能码0x01 - 速度环模式（设置速度为50r/min）：
 *    AA 01 01 03 00 00 48 42 8E FF
 *    解析：AA(帧头) 01(ID=1) 01(功能码) 03(速度环模式) 00004842(速度50) 8E(CRC) FF(帧尾)
 * 
 * 4. 功能码0x01 - 角度环模式（设置角度为1.57rad）：
 *    AA 01 01 02 C3 F5 C8 3F 4B FF
 *    解析：AA(帧头) 01(ID=1) 01(功能码) 02(角度环模式) C3F5C83F(角度1.57) 4B(CRC) FF(帧尾)
 * 
 * 5. 功能码0x02 - FOC校准：
 *    AA 01 02 5B FF
 *    解析：AA(帧头) 01(ID=1) 02(功能码) 5B(CRC) FF(帧尾)
 * 
 * 6. 功能码0x03 - 修改电机ID（修改为ID=5）：
 *    AA 01 03 05 5F FF
 *    解析：AA(帧头) 01(ID=1) 03(功能码) 05(新ID=5) 5F(CRC) FF(帧尾)
 */
void Protocol_Parse(const uint8_t *data, uint16_t length)
{
    // 1. 检查帧头和帧尾
    if(data[0] != PROTOCOL_HEADER || data[length-1] != PROTOCOL_TAIL)
    {
        return;
    }
    
    // 2. 检查电机ID是否匹配
    uint8_t motor_id = data[1];
    if(motor_id != config_info.motor_id)
    {
        return;  // ID不匹配，忽略该帧
    }
    
    // 3. 提取功能码
    uint8_t func_code = data[2];
    uint8_t data_length = 0;
    
    switch (func_code)
    {
        //设置模式参数功能码 0x01
        //AA ID 功能码 Mode 数据 CRC FF
        case FUNC_CODE_SET_PARAMETER:
        {
            FOC_Mode_t mode = (FOC_Mode_t)data[3];
            
            if (mode == FOC_MODE_SPEED_ANGLE_LOOP)
            {
                data_length = 8;  // 两个float
            }
            else
            {
                data_length = 4;  // 一个float
            }
            
            // CRC校验（校验范围：ID + 功能码 + Mode + 数据）
            uint8_t crc_received = data[length - 2];  // CRC在倒数第二个字节
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 2 + 1 + data_length);  // ID + 功能码 + Mode + 数据
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            
            // 解析数据并调用对应函数
            float value1 = 0.0f;
            float value2 = 0.0f;
            
            // 提取第一个float（小端序）
            memcpy(&value1, &data[4], sizeof(float));
            
            if (mode == FOC_MODE_SPEED_ANGLE_LOOP)
            {
                // 提取第二个float
                memcpy(&value2, &data[8], sizeof(float));
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
        //AA ID 功能码 CRC FF
        case FUNC_CODE_CALIBRATE:  
        {
            config_info.calibrated = 0;
            // CRC校验（校验范围：ID + 功能码）
            uint8_t crc_received = data[length - 2];
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 2);  // ID + 功能码
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            calibrated_flag = 1;//在主循环中进行校准
            break;
        }
        
        //修改电机ID功能码 0x03
        //AA ID 功能码 new_id CRC FF
        case FUNC_CODE_MODIFY_ID:  
        {
            // CRC校验（校验范围：ID + 功能码 + 新ID）
            uint8_t crc_received = data[length - 2];
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 3);  // ID + 功能码 + 新ID
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            
            // 提取新的电机ID
            uint8_t new_motor_id = data[3];
            
            // 更新配置信息中的电机ID
            config_info.motor_id = new_motor_id;
            
            // 设置标志位，在主循环中保存到Flash
            save_config_flag = 1;
            
            break;
        }
        
        default:
            break;
    }
}
