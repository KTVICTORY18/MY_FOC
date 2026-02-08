/**
 * @file current_sense.h
 * @brief 电流采样模块 - 使用INA240A1PWR电流检测放大器
 * 
 * 功能说明：
 * 1. 使用ADC1的注入转换模式采样U相和W相电流
 * 2. ADC注入通道1 (PA0) -> U相电流 (Rank 1)
 * 3. ADC注入通道2 (PA1) -> W相电流 (Rank 2)
 * 4. V相电流通过计算得到：Iv = -(Iu + Iw)
 * 5. 注入转换由TIM1的TRGO触发（在PWM中点），确保采样时机准确
 * 
 * INA240A1PWR参数：
 * - 增益：20 V/V (INA240A1)
 * - 输出范围：0 ~ VCC (通常3.3V)
 * - 零点：VCC/2 = 1.65V (当电流为0时)
 * 
 * 注入转换模式优势：
 * - 在PWM中点精确采样，避免开关噪声
 * - 不需要DMA，直接从注入数据寄存器读取
 * - 采样时机由硬件自动控制，更可靠
 * 
 * 使用说明：
 * 1. 在main.c中调用 CurrentSense_Init() 初始化
 * 2. 调用 CurrentSense_Start() 启动注入转换
 * 3. 在TIM1中断中调用 CurrentSense_Update() 更新电流值（注入转换完成后）
 * 4. 使用 CurrentSense_GetPhaseCurrents() 获取三相电流
 */

#ifndef __CURRENT_SENSE_H__
#define __CURRENT_SENSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"
#include <stdint.h>

// ========== 配置参数 ==========
// INA240A1PWR参数
#define CURRENT_SENSE_ADC_RESOLUTION      4096.0f    // 12位ADC分辨率 (2^12)
#define CURRENT_SENSE_ADC_VREF           3.3f       // ADC参考电压 (V)
#define CURRENT_SENSE_GAIN               20.0f      // INA240A1增益 (V/V)
#define CURRENT_SENSE_SHUNT_RESISTANCE   0.05f      // 分流电阻 (Ω)，50毫欧姆

// 电流计算系数
// 电流(A) = (ADC值 - 零点ADC值) * Vref / ADC分辨率 / Gain / R_shunt
#define CURRENT_SENSE_SCALE_FACTOR       (CURRENT_SENSE_ADC_VREF / CURRENT_SENSE_ADC_RESOLUTION / CURRENT_SENSE_GAIN / CURRENT_SENSE_SHUNT_RESISTANCE)


// 三相电流结构体
typedef struct {
    float Iu;      // U相电流 (A)
    float Iv;      // V相电流 (A) - 计算得到
    float Iw;      // W相电流 (A)
} PhaseCurrents_t;

// 电流采样状态结构体
typedef struct {
    uint16_t adc_raw_u;      // U相ADC原始值（最新）
    uint16_t adc_raw_w;      // W相ADC原始值（最新）
    PhaseCurrents_t currents;     // 三相电流（默认对外输出）
    float iu_offset;         // U相电流偏置（A），用于FOC校准
    float iv_offset;         // V相电流偏置（A），用于FOC校准
    float iw_offset;         // W相电流偏置（A），用于FOC校准
} CurrentSense_t;

extern CurrentSense_t current_sense;

// 函数声明
void CurrentSense_Init(void);
void CurrentSense_Start(void);
void CurrentSense_Stop(void);
void CurrentSense_Update(void);


#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_SENSE_H__ */

