#include "user_protocol.h"
#include "storage.h"
#include <string.h>

//逆时针是正转,顺时针是反转


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
 * - AA + ID + 功能码 + 数据 + CRC8 + FF
 * crc校验是从id到数据的最后一位
 * 功能码0x01: 设置模式参数（单参数模式）
 *   - 支持模式：FOC_MODE_LOW_SPEED_LOOP(0), FOC_MODE_STEP_ANGLE_LOOP(1), 
 *               FOC_MODE_SPEED_LOOP(3), FOC_MODE_CURRENT_LOOP(4)
 *   - 数据格式：AA ID 01 Mode(1字节) Value(4字节float) CRC FF
 *   - 示例：设置速度环50rpm
 *     AA 01 01 03 00 00 48 42 8E FF
 *     设置低速环30rpm
 *     AA 01 01 00 00 00 F0 41 22 FF
 * 
 * 功能码0x02: FOC校准并保存到Flash
 *   - 数据格式：AA ID 02 CRC FF
 *   - 示例：AA 01 02 1B FF
 * 
 * 功能码0x03: 修改电机ID
 *   - 数据格式：AA ID 03 NewID(1字节) CRC FF
 *   - 示例：修改为ID=1
 *     AA 00 03 01 38 FF
 *     修改电机id为2
 *     AA 00 03 01 00 FF
 * 
 * 功能码0x04: 设置绝对角度+速度（以指定速度运动到绝对角度）
 *   - 数据格式：AA ID 04 Angle(4字节float) Speed(4字节float) CRC FF
 *   - 参数说明：
 *     - Angle: 目标绝对角度（弧度，0~2π）
 *     - Speed: 运动速度（rpm，正值=正转，负值=反转）
 *   - 示例1：以20rpm正转到3.14rad
 *     AA 01 04 C3 F5 48 40 00 00 A0 41 E3 FF
 *   - 示例2：以-20rpm反转到3.14rad
 *     AA 01 04 C3 F5 48 40 00 00 A0 C1 6A FF
 * 
 * 功能码0x05: 设置相对角度+速度（以指定速度相对运动指定角度）
 *   - 数据格式：AA ID 05 StepAngle(4字节float) Speed(4字节float) CRC FF
 *   - 参数说明：
 *     - StepAngle: 相对角度（弧度，正值=正转，负值=反转）
 *     - Speed: 运动速度（rpm，正值=正转，负值=反转）
 *   - 示例1：以30rpm正转90度(1.57rad)
 *     AA 01 05 C3 F5 C8 3F 00 00 F0 41 D4 FF
 *   - 示例2：以30rpm反转90度(-1.57rad)
 *     AA 01 05 C3 F5 C8 BF 00 00 F0 41 43 FF
 *   - 示例3：以50rpm正转180度(3.14rad)
 *     AA 01 05 C3 F5 48 40 00 00 48 42 [CRC] FF
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
        //设置模式参数功能码 0x01（仅支持单参数模式）
        //AA ID 01 Mode(1字节) Value(4字节float) CRC FF
        case FUNC_CODE_SET_PARAMETER:
        {
            FOC_Mode_t mode = (FOC_Mode_t)data[3];
            
            // 只支持单参数模式
            if (mode == FOC_MODE_LOW_SPEED_LOOP || 
                mode == FOC_MODE_STEP_ANGLE_LOOP || 
                mode == FOC_MODE_SPEED_LOOP || 
                mode == FOC_MODE_CURRENT_LOOP)
            {
                data_length = 4;  // 一个float
                
                // CRC校验（校验范围：ID + 功能码 + Mode + 数据）
                uint8_t crc_received = data[length - 2];
                uint8_t crc_calculated = CRC8_Calculate(&data[1], 2 + 1 + data_length);
                
                if (crc_received != crc_calculated)
                {
                    return;
                }
                
                // 解析数据
                float value = 0.0f;
                memcpy(&value, &data[4], sizeof(float));
                
                // 调用单参数模式函数
                FOC_Set_Parameter(mode, value);
            }
            
            break;
        }
        
        //FOC校准并保存到Flash功能码 0x02
        //AA ID 02 CRC FF
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
        //AA ID 03 NewID(1字节) CRC FF
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
        
        //设置绝对角度+速度功能码 0x04
        //AA ID 04 Angle(4字节float) Speed(4字节float) CRC FF
        case FUNC_CODE_ABS_ANGLE_SPEED:
        {
            data_length = 8;  // 两个float
            
            // CRC校验（校验范围：ID + 功能码 + 数据）
            uint8_t crc_received = data[length - 2];
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 2 + data_length);
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            
            // 解析数据
            float target_angle = 0.0f;
            float target_speed = 0.0f;
            memcpy(&target_angle, &data[3], sizeof(float));
            memcpy(&target_speed, &data[7], sizeof(float));
            
            // 调用绝对角度+速度模式
            FOC_Set_Speed_Angle(target_angle, target_speed);
            
            break;
        }
        
        //设置相对角度+速度功能码 0x05
        //AA ID 05 StepAngle(4字节float) Speed(4字节float) CRC FF
        //这个协议的通讯接口速度是标量没有方向，只有角度有方向
        case FUNC_CODE_REL_ANGLE_SPEED:
        {
            data_length = 8;  // 两个float
            
            // CRC校验（校验范围：ID + 功能码 + 数据）
            uint8_t crc_received = data[length - 2];
            uint8_t crc_calculated = CRC8_Calculate(&data[1], 2 + data_length);
            
            if (crc_received != crc_calculated)
            {
                return;
            }
            
            // 解析数据
            float step_angle = 0.0f;
            float target_speed = 0.0f;
            memcpy(&step_angle, &data[3], sizeof(float));
            memcpy(&target_speed, &data[7], sizeof(float));
            
            // 调用相对角度+速度模式
            FOC_Set_Step_Speed_Angle(step_angle, target_speed);
            
            break;
        }
        
        default:
            break;
    }
}
