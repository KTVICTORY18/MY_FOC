/**
 * @file filter.c
 * @brief 低通滤波器实现
 * @author Based on QDrive-Software FOC library
 * @date 2026-01-19
 * @version V1.0.0
 */

#include "filter.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化二阶低通滤波器
 * @param filter 滤波器结构体指针
 * @param Ts 采样周期, 单位s
 * @param Fc 截止频率, 单位Hz
 * @param dampingRatio 阻尼比, 默认0.707
 */
void LowPassFilter_2_Order_Init(LowPassFilter_2_Order_t *filter, float Ts, float Fc, float dampingRatio)
{
    if (filter == NULL) return;
    
    // 保存参数
    filter->Ts = Ts;
    filter->Fc = Fc;
    
    // 计算滤波器系数
    float Wc = 2.0f / Ts * tanf(M_PI * Fc * Ts);
    
    filter->b0 = Wc * Wc * Ts * Ts;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;
    
    filter->a0 = 4.0f + 4.0f * dampingRatio * Wc * Ts + filter->b0;
    filter->a1 = -8.0f + 2.0f * filter->b0;
    filter->a2 = 4.0f - 4.0f * dampingRatio * Wc * Ts + filter->b0;
    
    // 初始化历史值为0
    memset(filter->xin, 0, sizeof(filter->xin));
    memset(filter->yout, 0, sizeof(filter->yout));
}

/**
 * @brief 执行二阶低通滤波
 * @param filter 滤波器结构体指针
 * @param x 输入值
 * @return 滤波后的输出值
 */
float LowPassFilter_2_Order_Update(LowPassFilter_2_Order_t *filter, float x)
{
    if (filter == NULL) return x;
    
    filter->xin[2] = x;
    filter->yout[2] = (filter->b0 * filter->xin[2] + 
                       filter->b1 * filter->xin[1] + 
                       filter->b2 * filter->xin[0] - 
                       filter->a1 * filter->yout[1] - 
                       filter->a2 * filter->yout[0]) / filter->a0;
    
    // 更新历史值
    filter->xin[0] = filter->xin[1];
    filter->xin[1] = filter->xin[2];
    filter->yout[0] = filter->yout[1];
    filter->yout[1] = filter->yout[2];
    
    return filter->yout[2];
}
