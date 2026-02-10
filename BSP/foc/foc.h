/**
 * @file foc.h
 * @brief 开环FOC（磁场定向控制）驱动库
 * 
 * 使用说明：
 * 1. 在main.c中调用 FOC_Init() 初始化FOC
 * 2. 使用 FOC_SetSpeedRPM(速度) 设置目标转速（RPM）
 *   或使用 FOC_SetSpeed(电角度速度) 设置电角度速度
 * 3. 在主循环或定时器中断中定期调用 FOC_Update()
 * 
 * 注意事项：
 * - 请根据实际GB4310电机参数调整 FOC_POLE_PAIRS（极对数）
 * - 可以根据实际效果调整 FOC_VOLTAGE_LIMIT 和 iq_ref 的值
 * - 建议在定时器中断中调用 FOC_Update() 以获得更精确的控制
 */

#ifndef __FOC_H__
#define __FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include "adc.h"           // ADC驱动，用于读取电池电压
#include "current_sense.h" // 三相电流采样
#include "pid.h"           // 通用PID，用于电流环
#include "MT6825GT.h"      // MT6825编码器
#include "cordic.h"        // CORDIC硬件加速器
#include <math.h>

// FOC配置参数
//20KHZ
#define FOC_PWM_PERIOD          4250        // PWM周期值（对应TIM1的Period）
#define FOC_UPDATE_FREQ         20000      // FOC更新频率（Hz），与PWM频率一致，方便在每个周期更新
#define FOC_VOLTAGE_LIMIT       0.3f      // 电压限制系数（0-1），GB4310需要更小的电压以减少抖动
#define FOC_PI                  3.14159265358979323846f
#define FOC_2PI                 (2.0f * FOC_PI)
#define FOC_SQRT3               1.7320508075688772f
#define INV_SQRT3        0.577350269f
#define FOC_UPDATE_FREQ_SPEED_ANGLE 1000.0f //速度环和角度环的更新频率
#define FOC_POLE_PAIRS 14 //极对数
// ADC电池电压读取配置
#define FOC_ADC_VREF           3.3f        // ADC参考电压（V），通常是VDDA
#define FOC_ADC_RESOLUTION     4096.0f     // 12位ADC分辨率（2^12 = 4096）
#define FOC_VBAT_DIVIDER_RATIO    8.5f    // 电池电压分压比75k+10k/10k
#define FOC_MAX_CURRENT          1.65f   // 最大电流,单位A (根据硬件设计调整)
#define FOC_MAX_SPEED            1000.0f   // 最大速度,单位rpm

//这个参数能够达到1ms内达到目标位置
//需要堵转 12.4v kp0.5,ki0.05
//云台相电阻太大了，反电动势太大了，所以需要大电压
#define KP_IQ 6.2f   // 减弱kp能够有效缓解电机静止时的沙沙声音
#define KI_IQ 0.62f   // 
#define KP_ID 6.2f   // 
#define KI_ID 0.62f   // 

#define KP_SPEED 3e-3f   // 速度环kp
#define KI_SPEED 3.9e-4f   // 速度环ki
#define KD_SPEED 0.0f   // 速度环kd

#define KP_ANGLE            1200.0f
#define KI_ANGLE            0.0f
#define KD_ANGLE            0.0f

//FOC三个模式
//FOC_MODE_CURRENT_LOOP：电流环
//FOC_MODE_ANGLE_LOOP：角度环
//FOC_MODE_SPEED_LOOP：速度环
typedef enum{
    FOC_MODE_LOW_SPEED_LOOP = 0,    //低速
    FOC_MODE_STEP_ANGLE_LOOP = 1, //步进
    FOC_MODE_ANGLE_LOOP = 2,      //角度环
    FOC_MODE_SPEED_LOOP = 3,      //速度环
    FOC_MODE_CURRENT_LOOP = 4,    //电流环
    FOC_MODE_SPEED_ANGLE_LOOP = 5,//速度+角度模式（以指定速度运行到目标角度）
}FOC_Mode_t;

// FOC控制结构体
typedef struct {
    float target_speed;         // 目标速度，每分钟多少圈
    float current_speed;         // 当前速度 rpm,每分钟多少圈
    float target_angle;         // 目标角度（弧度）
    float electrical_angle;     // 当前电角度（弧度）
    float id_ref;               // d轴电流参考值（开环时设为0）
    float iq_ref;               // q轴电流参考值（控制转矩）
    float id_actual;            // 实际d轴电流
    float iq_actual;            // 实际q轴电流
    float ud;                   // d轴电压
    float uq;                   // q轴电压
    float voltage_limit;        // 电压限制
    float vbat_voltage;         // 测量出来的vbat的供电电压
    FOC_Mode_t mode;           // 当前模式
} FOC_Control_t;

// 三相电压结构体
typedef struct {
    float va;
    float vb;
    float vc;
} ThreePhase_t;

// αβ坐标系结构体
typedef struct {
    float alpha;
    float beta;
} AlphaBeta_t;


//配置信息结构体
typedef struct { //
    float phase_resistance;     // 相位电阻
    float iu_offset;            // U相电流偏置
    float iv_offset;            // V相电流偏置
    float iw_offset;            // W相电流偏置
    float zero_electric_angle;  // 电角度零点
    uint8_t encoder_direction;  // 编码器方向 (1: 正向, 0: 反向)
    uint8_t calibrated;         // 校准完成标志
    uint8_t motor_id;           // 电机id
    uint8_t _data[105];         // 占位数据，使结构体大小为128字节
} ConfigInfo_t;

extern ThreePhase_t three_phase;//三相电压
extern FOC_Control_t foc_ctrl;//FOC控制结构体
extern ConfigInfo_t config_info;//配置信息
extern AlphaBeta_t alpha_beta;//αβ坐标系


// FOC函数接口
void FOC_Init(void);                        // 初始化FOC
void FOC_Angle_And_Speed_Update(void);      // 角度环和速度环的更新 1khz
void FOC_Current_Loop_Update(void);         // 电流环更新函数 20khz
void FOC_Update_Duty(float uq, float ud, float electrical_angle);  // 更新三相PWM占空比
void FOC_Apply_PWM(void);                   // 应用PWM占空比
void FOC_Read_Vbat_Voltage(void);           // 读取电池电压（会临时停止注入转换，不要频繁调用）
void FOC_Calibrate(void);                   // FOC校准
void FOC_SetPhaseVoltage(float ud, float uq, const float electrical_angle);
// CORDIC硬件加速的三角函数（快速版本）
void FOC_FastSinCos(float angle, float *sin_val, float *cos_val);
void FOC_Set_Parameter(FOC_Mode_t mode,float value);
void FOC_Update_PID_Parameter(void);
void FOC_Set_Speed_Angle(float target_angle, float target_speed);
void FOC_Set_Step_Speed_Angle(float step_angle, float target_speed);
#ifdef __cplusplus
}
#endif

#endif