#include "openmv.h"
#include "can_comm.h"
#include <string.h>
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

//生成直接控制底部电机移动的帧
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
        } else {
            // 未找到目标，停止或保持当前位置
            handle->pid_yaw.integral = 0.0f;  // 清除积分
            handle->pid_yaw.last_error = 0.0f;
            handle->pid_pitch.integral = 0.0f;  // 清除积分
            handle->pid_pitch.last_error = 0.0f;
            
            OpenMV_Control_Yaw(0.0f);//通过can控制底部电机移动
            // 可以设置速度为0
            FOC_Set_Parameter(FOC_MODE_SPEED_LOOP, 0.0f);//如果没有找到目标设置速度为0
        }
    }
    
    // 超时检测（可选）
    if (HAL_GetTick() - handle->last_update_time > 1000) {//长时间没有接收到数据包也进行停止操作
        // 超过1秒未收到数据，可以执行安全停止
        OpenMV_Control_Yaw(0.0f);//通过can控制底部电机移动
        FOC_Set_Parameter(FOC_MODE_SPEED_LOOP, 0.0f);
    }
}
