#include "openmv.h"
#include "can_comm.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "user_protocol.h"

OpenMV_Handle_t openmv;
/**
 * @brief 初始化 OpenMV 处理模块
 */
void OpenMV_Init(OpenMV_Handle_t *handle)
{
    memset(handle, 0, sizeof(OpenMV_Handle_t));
     
    // 初始化 Yaw 轴（底部电机）PID 参数
    // 根据实际情况调整这些参数
    OpenMV_PID_Init(&handle->pid_yaw, 1.0f, 0.0f, 0.0f);
    OpenMV_PID_Init(&handle->pid_pitch, 1.0f, 0.0f, 0.0f);
    
    handle->pid_yaw.target = 0.0f;  // 目标是中心点，偏差为0
    handle->pid_yaw.integral_limit = 100.0f;
    handle->pid_yaw.output_limit = 60.0f;  // 限制最大速度输出
    handle->pid_pitch.target = 0.0f;  // 目标是中心点，偏差为0
    handle->pid_pitch.integral_limit = 100.0f;
    handle->pid_pitch.output_limit = 100.0f;  // 限制最大速度输出60rpm
}

/**
 * @brief 初始化 PID 控制器
 */
void OpenMV_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = 100.0f;
    pid->output_limit = 10.0f;
}

/**
 * @brief 位置式 PID 计算
 * @param pid PID 控制器指针
 * @param current_value 当前值（这里是偏差值 dx）
 * @return 输出速度
 */
float OpenMV_PID_Calculate(PID_Controller_t *pid, float current_value)
{
    // 计算误差（目标是0，即中心点）
    pid->error = pid->target - current_value;
    if(pid->error<5.0f && pid->error>-5.0f){//误差较小时不进行积分，防止积分风up
        pid->error = 0.0f;
    }
    
    // 积分项累积
    pid->integral += pid->error;
    
    // 积分限幅，防止积分饱和
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // 微分项
    float derivative = pid->error - pid->last_error;
    
    // PID 输出计算
    pid->output = pid->Kp * pid->error + 
                  pid->Ki * pid->integral + 
                  pid->Kd * derivative;
    
    // 输出限幅
    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }
    
    // 保存当前误差供下次使用
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief 解析完整的 OpenMV 数据包（适配串口空闲中断）
 * @param handle OpenMV 处理句柄
 * @param data 接收到的数据缓冲区
 * @param length 数据长度
 * @return 1: 解析成功, 0: 解析失败
 */
uint8_t OpenMV_ParsePacket(OpenMV_Handle_t *handle, uint8_t *data, uint16_t length)
{
    // 检查数据包长度是否正确
    if (length != 7) {
        return 0;
    }
    
    // 验证帧头
    if (data[0] != 0xBB) {
        return 0;
    }
    
    // 验证帧尾
    if (data[6] != 0xFF) {
        return 0;
    }
    
    // 解析数据包
    handle->packet.header = data[0];
    handle->packet.found = data[1];
    
    // 小端序解析 16 位有符号整数
    handle->packet.dx = (int16_t)(data[2] | (data[3] << 8));
    handle->packet.dy = (int16_t)(data[4] | (data[5] << 8));
    
    handle->packet.tail = data[6];
    
    // 标记数据就绪
    handle->data_ready = 1;
    handle->last_update_time = HAL_GetTick();
    
    return 1;
}

// 生成直接控制底部电机移动的帧
void OpenMV_Control_Yaw(float target_velocity_yaw){
    // 生成 CAN 数据帧，假设电机 ID 是 0x02，功能码是 0x01，数据是速度值（4 字节 float）
    uint8_t can_data[10];
    can_data[0] = 0xAA; // 帧头
    can_data[1] = 0x01; // 电机 ID
    can_data[2] = 0x01; // 功能码：设置速度
    can_data[3] = 0x03;//foc模式 low_speed
    memcpy(&can_data[4], &target_velocity_yaw, sizeof(float)); // 将 float 数据复制到 CAN 数据中
    uint8_t crc = CRC8_Calculate(&can_data[1], 7); // 计算 CRC（从 ID 开始到数据结束）
    can_data[8] = crc; // CRC 校验
    can_data[9] = 0xFF; // 帧尾
    CAN_Comm_Transmit_To_ID(0x001, can_data, 10); // 发送 CAN 数据帧
}

// 封装函数：将OpenMV的数据通过CAN发给ID为1的电机
void OpenMV_Send_Data_To_ID1(OpenMV_Handle_t *handle)
{
    // 只在包含有效数据时发送
    uint8_t can_data[7];
    can_data[0] = handle->packet.header; // 0xBB
    can_data[1] = handle->packet.found;  
    can_data[2] = (handle->packet.dx >> 8) & 0xFF; // 高八位
    can_data[3] = handle->packet.dx & 0xFF;        // 低八位
    can_data[4] = (handle->packet.dy >> 8) & 0xFF; // 高八位
    can_data[5] = handle->packet.dy & 0xFF;        // 低八位
    can_data[6] = handle->packet.tail;   // 0xFF
    
    CAN_Comm_Transmit_To_ID(0x001, can_data, 7);
}

/**
 * @brief 处理 OpenMV 数据并计算 PID 输出
 * @param handle OpenMV 处理句柄
 */
void OpenMV_Process(OpenMV_Handle_t *handle)
{
    if (handle->data_ready) {
        handle->data_ready = 0;
        
        if (handle->packet.found == 0x01) {
            // 找到目标，进行 PID 控制
            float target_velocity_yaw = OpenMV_PID_Calculate(&handle->pid_yaw, (float)handle->packet.dx);//通过pid计算得到目标速度
            float target_velocity_pitch = OpenMV_PID_Calculate(&handle->pid_pitch, (float)handle->packet.dy);//通过pid计算得到目标速度
            
            OpenMV_Control_Yaw(target_velocity_yaw);//通过can控制底部电机移动
            FOC_Set_Parameter(FOC_MODE_SPEED_LOOP, target_velocity_pitch);
            OpenMV_Send_Data_To_ID1(handle);
        } else {
            // 未找到目标，停止或保持当前位置
            handle->pid_yaw.integral = 0.0f;  // 清除积分
            handle->pid_yaw.last_error = 0.0f;
            handle->pid_pitch.integral = 0.0f;  // 清除积分
            handle->pid_pitch.last_error = 0.0f;
            
            OpenMV_Control_Yaw(0.0f);//通过can控制底部电机移动
            // 可以设置速度为0
            FOC_Set_Parameter(FOC_MODE_SPEED_LOOP, 0.0f);//如果没有找到目标设置速度为0
            OpenMV_Send_Data_To_ID1(handle);
        }
    }
    
    // 超时检测（可选）
    if (HAL_GetTick() - handle->last_update_time > 1000) {//长时间没有接收到数据包也进行停止操作
        // 超过1秒未收到数据，可以执行安全停止
        OpenMV_Control_Yaw(0.0f);//通过can控制底部电机移动
        FOC_Set_Parameter(FOC_MODE_SPEED_LOOP, 0.0f);
    }
}

/**
 * @brief 通过串口2的DMA打印 OpenMV 的目标偏移数据（x/y 轴偏差、是否检测到目标）
 * @param handle OpenMV 处理句柄
 * @note  使用 HAL_UART_Transmit_DMA 通过 huart2 发送，
 *        发送前会等待上一次 DMA 传输完成，避免缓冲区冲突。
 *        可在主循环或定时任务中周期性调用，例如每 100ms 调用一次。
 */
void OpenMV_Print_Info(OpenMV_Handle_t *handle)
{
    if (handle == NULL) {
        return;
    }

    // 静态缓冲区，DMA 需要在传输期间保持数据稳定
    static char tx_buffer[64];
    int len;

    if (handle->packet.found == 0x01) {//找到了
        len = snprintf(tx_buffer, sizeof(tx_buffer),
                       "%d,%6d,%6d\r\n",
                       handle->packet.found, handle->packet.dx, handle->packet.dy);
    } else {
        len = snprintf(tx_buffer, sizeof(tx_buffer),
                       "%d,%6d,%6d\r\n",
                       handle->packet.found, handle->packet.dx, handle->packet.dy);
    }

    if (len <= 0 || len >= (int)sizeof(tx_buffer)) {
        return;
    }

    // 等待上一次 DMA 发送完成，避免覆盖缓冲区
    uint32_t wait = 0;
    while (huart2.gState != HAL_UART_STATE_READY && wait++ < 100000) {
        ;
    }

    // 通过 DMA 发送
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buffer, (uint16_t)len);
}


