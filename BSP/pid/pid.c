/**
 * @file pid.c
 * @brief 通用 PID 控制器实现
 */

#include "pid.h"
#include "foc.h"

//速度环pid
 /***位置式PID公式:u=Kpe(t)+Ki*e(t)的积分+Kd[e(t)-e(t-1)]***/
PID_t PID_Speed;//速度环使用位置式pid

//位置环pid
PID_t PID_Angle;//位置环使用位置式pid

//电流环pid
PID_Inc_t PID_IQ;
PID_Inc_t PID_ID;

void PID_Init(PID_t *pid, float sample_time_s)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->Ts = sample_time_s;
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->out_min = 0.0f;
    pid->out_max = 0.0f;
    pid->integral_max = 0.0f;
    pid->integral_min = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimit(PID_t *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement)
{
    // 误差
    float err = setpoint - measurement;
     // 2. 误差归一化：将误差限制在 [-PI, PI] 之间
    // 这样 PID 永远会走最短的那段弧
    if(pid->is_integral_enable == 0){//角度环不使用积分限幅并且抑制突变
        while (err > FOC_PI)  err -= FOC_2PI;
        while (err < -FOC_PI) err += FOC_2PI;
    }
    

    // 积分
    pid->integral += err;
    //积分限幅
    if(pid->is_integral_enable == 1){
        if(pid->integral>=pid->integral_max) pid->integral = pid->integral_max;
        if(pid->integral<=pid->integral_min) pid->integral = pid->integral_min;
    }
    // 计算输出
    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * (err - pid->prev_err);

    //更新历史误差
    pid->prev_err = err;
    //输出限幅
    if (out > pid->out_max) out = pid->out_max;
    else if (out < pid->out_min) out = pid->out_min;

    return out;
}

// ========== 增量式 PID 实现 ==========

void PID_Inc_Init(PID_Inc_t *pid, float sample_time_s)
{
    pid->kp = 0.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->Ts = sample_time_s;
    pid->prev_err = 0.0f;
    pid->prev_prev_err = 0.0f;
    pid->out = 0.0f;
    pid->out_min = -1.0f;
    pid->out_max = 1.0f;
}

void PID_Inc_SetGains(PID_Inc_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Inc_SetOutputLimit(PID_Inc_t *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void PID_Inc_Reset(PID_Inc_t *pid)
{
    pid->prev_err = 0.0f;
    pid->prev_prev_err = 0.0f;
    pid->out = 0.0f;
}

float PID_Inc_Update(PID_Inc_t *pid, float setpoint, float measurement)
{
    // 误差
    float err = setpoint - measurement;      // e(k)
    float err_1 = pid->prev_err;            // e(k-1)
    float err_2 = pid->prev_prev_err;       // e(k-2)

    // 增量式PID离散公式（QDrive风格：离散域参数，不含Ts）
    // du(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    // 注意：Ki和Kd参数应该是已经离散化的值，不需要再乘/除Ts
    float de_p = err - err_1;
    float de_d = err - 2.0f * err_1 + err_2;

    float du = pid->kp * de_p
             + pid->ki * err
             + pid->kd * de_d;

    // 抗积分饱和（增量式PID版本）
    // 如果当前输出已饱和，且du会加剧饱和，则限制du
    float out_temp = pid->out + du;
    if (out_temp > pid->out_max) {
        // 输出会超过上限，限制增量
        if (du > 0.0f) {
            du = pid->out_max - pid->out;  // 限制du使输出刚好等于上限
            if (du < 0.0f) du = 0.0f;     // 如果已经超过，du=0
        }
    } else if (out_temp < pid->out_min) {
        // 输出会低于下限，限制增量
        if (du < 0.0f) {
            du = pid->out_min - pid->out;  // 限制du使输出刚好等于下限
            if (du > 0.0f) du = 0.0f;     // 如果已经超过，du=0
        }
    }

    // 累加得到新输出
    pid->out += du;

    // 输出限幅（双重保护）
    if (pid->out > pid->out_max) {
        pid->out = pid->out_max;
    } else if (pid->out < pid->out_min) {
        pid->out = pid->out_min;
    }

    // 更新历史误差
    pid->prev_prev_err = pid->prev_err;
    pid->prev_err = err;

    return pid->out;
}


