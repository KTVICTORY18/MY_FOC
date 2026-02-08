/**
 * @file current_sense.c
 * @brief 电流采样模块实现
 */

#include "current_sense.h"
#include "adc.h"
#include "tim.h"  // 用于检查TIM1状态
#include <string.h>
#include <math.h>

// 外部变量声明
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

// 电流采样实例
CurrentSense_t current_sense;

/**
 * @brief 电流采样初始化
 */
void CurrentSense_Init(void)
{
    // 清零结构体
    memset(&current_sense, 0, sizeof(CurrentSense_t));
    
    // 初始化零点偏移（默认值，实际应该通过校准获得）
    // 零点电压 = VCC/2 = 1.65V，对应ADC值 = 1.65 / 3.3 * 4096 = 2048
    
    // 初始化电流偏置（用于FOC校准）
    current_sense.iu_offset = 0.0f;
    current_sense.iv_offset = 0.0f;
    current_sense.iw_offset = 0.0f;
    
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);//启用adc校准
    //tim1的trgo触发是4通道的pwm信号
}

/**
 * @brief 启动ADC注入转换采样（中断模式）
 * 
 * 启动ADC注入转换，由TIM1的TRGO触发
 * 使用中断模式，转换完成后触发中断回调
 */
void CurrentSense_Start(void)
{
    // 启动ADC注入转换（中断模式）
    // 注入转换由TIM1的TRGO触发，自动执行
    // 开启tim1的第四个通道进行pwm输出，然后通过trgo触发adc注入转换
    // 转换完成后触发HAL_ADCEx_InjectedConvCpltCallback()回调
    
    if (HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK)//开启adc注入中断并启动注入转换
    {
        // 启动失败，可以在这里添加错误处理
        Error_Handler();
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // 必须开启，否则 ADC 不会被触发
}

/**
 * @brief 停止ADC注入转换采样
 */
void CurrentSense_Stop(void)
{
    HAL_ADCEx_InjectedStop(&hadc1);
}




/**
 * @brief 极致优化的电流更新 (建议耗时: < 1us)
 * 
 * 注入转换模式：直接从注入数据寄存器读取最新转换结果
 * 注意：单次采样噪声较大，建议使用更强的滤波（降低FILTER_ALPHA）
 */
void CurrentSense_Update(void)
{
    // 1. 直接从注入数据寄存器读取最新的ADC值（寄存器级操作，避免HAL库开销）
    // Rank 1 = U相 (ADC_CHANNEL_1)
    // Rank 2 = W相 (ADC_CHANNEL_2)
    register uint32_t adc_u = hadc1.Instance->JDR1; // PA0采集的还是u相电流
    register uint32_t adc_w = hadc1.Instance->JDR2;
    
    // 存储原始ADC值（用于调试）
    current_sense.adc_raw_u = (uint16_t)adc_u;
    current_sense.adc_raw_w = (uint16_t)adc_w;
    
    // 2. 减去零点偏移，然后乘以缩放系数得到实际电流（关键修复！）
    current_sense.currents.Iu = ((float)adc_u) * CURRENT_SENSE_SCALE_FACTOR;
    current_sense.currents.Iw = ((float)adc_w) * CURRENT_SENSE_SCALE_FACTOR;
    
    // 3. 利用基尔霍夫电流定律计算第3相
    current_sense.currents.Iv = -(current_sense.currents.Iu + current_sense.currents.Iw);
}




