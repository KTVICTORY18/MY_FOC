/**
 * @file pid.h
 * @brief 通用 PID 控制器（可用于位置/速度/电流等环）
 *
 * 设计目标：
 * - 通用型：位置环、速度环、电流环都可复用
 * - 抗积分饱和：输出限幅 + 积分回推
 * - 可选微分：默认可设为0，避免噪声放大
 *
 * PID风格说明（QDrive兼容）：
 * - 本实现采用离散域参数（不乘/除Ts）
 * - Ki和Kd参数应该是已经离散化的值
 * - Ts参数保留用于兼容性，但不影响计算
 * - 可与QDrive项目共享相同的PID参数
 *
 * 使用示例：
 *   PID_t speed_pid;
 *   PID_Init(&speed_pid, Ts);  // Ts仅用于兼容性
 *   PID_SetGains(&speed_pid, kp, ki, kd);
 *   PID_SetOutputLimit(&speed_pid, -out_max, out_max);
 *   float out = PID_Update(&speed_pid, setpoint, measurement);
 *
 * 调参建议：
 * - 先关积分（ki=0），调 kp 到不过冲/不振荡的最快响应
 * - 再逐步增加 ki 消除静差；振荡则减小 kp 或 ki
 * - kd 如需使用，请先用很小值（或保持 0）
 */

#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float Ts;          // 采样时间 (s) - 仅用于兼容性，不影响计算
    float integral;    // 积分项
    float prev_err;    // 前一拍误差

    float out_min;     // 输出下限
    float out_max;     // 输出上限

    float integral_max; // 积分上限
    float integral_min; // 积分下限

    uint8_t is_integral_enable; // 是否积分限幅
} PID_t;

void PID_Init(PID_t *pid, float sample_time_s);
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);
void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float setpoint, float measurement);

// 增量式 PID（输出为累计值，内部保存上一拍输出）
typedef struct {
    float kp;
    float ki;
    float kd;

    float Ts;           // 采样时间 (s) - 仅用于兼容性，不影响计算
    float prev_err;     // e(k-1)
    float prev_prev_err;// e(k-2)

    float out;          // 当前输出 u(k)
    float out_min;      // 输出下限
    float out_max;      // 输出上限
} PID_Inc_t;

void PID_Inc_Init(PID_Inc_t *pid, float sample_time_s);
void PID_Inc_SetGains(PID_Inc_t *pid, float kp, float ki, float kd);
void PID_Inc_SetOutputLimit(PID_Inc_t *pid, float out_min, float out_max);
void PID_Inc_Reset(PID_Inc_t *pid);
float PID_Inc_Update(PID_Inc_t *pid, float setpoint, float measurement);

//电流环pid
extern PID_Inc_t PID_IQ;
extern PID_Inc_t PID_ID;
extern PID_t PID_Speed;
extern PID_t PID_Angle;

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */


