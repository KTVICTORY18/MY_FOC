/**
 * @file filter.h
 * @brief 低通滤波器实现
 * @author Based on QDrive-Software FOC library
 * @date 2026-01-19
 * @version V1.0.0
 */

#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 二阶低通滤波器结构体
 */
typedef struct {
    float Ts;       // 采样周期, 单位s
    float Fc;       // 截止频率, 单位Hz
    float b0, b1, b2;  // 分子系数
    float a0, a1, a2;  // 分母系数
    float xin[3];   // 输入历史值
    float yout[3];  // 输出历史值
} LowPassFilter_2_Order_t;

/**
 * @brief 初始化二阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param Ts 采样周期, 单位s
 * @param Fc 截止频率, 单位Hz
 * @param dampingRatio 阻尼比, 默认0.707
 */
void LowPassFilter_2_Order_Init(LowPassFilter_2_Order_t *filter, float Ts, float Fc, float dampingRatio);

/**
 * @brief 执行二阶低通滤波
 * @param filter 滤波器结构体指针
 * @param x 输入值
 * @return 滤波后的输出值
 */
float LowPassFilter_2_Order_Update(LowPassFilter_2_Order_t *filter, float x);

#ifdef __cplusplus
}
#endif

#endif // __FILTER_H__
