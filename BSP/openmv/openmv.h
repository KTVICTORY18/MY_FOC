#ifndef __OPENMV_H
#define __OPENMV_H

#include "main.h"
#include "usart.h"
#include "foc.h"

// OpenMV 数据包结构
typedef struct {
    uint8_t header;      // 0xBB
    uint8_t found;       // 0x01找到, 0x00未找到
    int16_t dx;          // X轴偏差
    int16_t dy;          // Y轴偏差
    uint8_t tail;        // 0xFF
} OpenMV_Packet_t;

// PID 控制器结构
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;        // 目标位置（通常为0，即中心）
    float error;         // 当前误差
    float last_error;    // 上次误差
    float integral;      // 积分累积
    float output;        // 输出值
    float integral_limit; // 积分限幅
    float output_limit;   // 输出限幅
} PID_Controller_t;

// OpenMV 控制结构
typedef struct {
    OpenMV_Packet_t packet;
    PID_Controller_t pid_yaw;   // 底部电机（Yaw轴）PID
    PID_Controller_t pid_pitch; // 顶部电机（Pitch轴）PID
    uint8_t data_ready;
    uint32_t last_update_time;
} OpenMV_Handle_t;

// 函数声明
void OpenMV_Init(OpenMV_Handle_t *handle);
void OpenMV_PID_Init(PID_Controller_t *pid, float kp, float ki, float kd);
uint8_t OpenMV_ParsePacket(OpenMV_Handle_t *handle, uint8_t *data, uint16_t length);
float OpenMV_PID_Calculate(PID_Controller_t *pid, float current_value);
void OpenMV_Process(OpenMV_Handle_t *handle);
void OpenMV_Send_Data_To_ID1(OpenMV_Handle_t *handle);
void OpenMV_Print_Info(OpenMV_Handle_t *handle);

extern OpenMV_Handle_t openmv;

#endif // __OPENMV_H
