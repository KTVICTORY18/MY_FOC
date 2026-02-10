#include "foc.h"
#include "MT6825GT.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"

//逆时针是正转,顺时针是反转

// 外部变量声明
extern ADC_HandleTypeDef hadc2;
extern CORDIC_HandleTypeDef hcordic;
extern MT6825_AngleData_t angle_data;
ThreePhase_t three_phase;//三相电压
FOC_Control_t foc_ctrl;//FOC控制结构体
ConfigInfo_t config_info = {0};//配置信息
AlphaBeta_t alpha_beta;//αβ坐标系
static float last_angle = 0.0f;//之前的机械角度

static float previous_angle = 0.0f;//控制速度环和位置环时的上一个角度

static float low_speed_angle = 0.0f;//低速模式的起始角度
static float low_speed = 0.0f;
static float move_target_angle = 0.0f;//位置速度模式，以一定速度运动到目标位置
static float move_error = 0.0f;//位置速度模式认定达到目标位置的误差


/**
 * @brief 使用CORDIC硬件加速计算sin和cos（同时计算，性能最优）
 * @param angle 输入角度（弧度）
 * @param sin_val 输出sin值指针
 * @param cos_val 输出cos值指针
 * 性能：~14个时钟周期（@170MHz约82ns），比软件sin/cos快50-100倍
 */
void FOC_FastSinCos(float angle, float *sin_val, float *cos_val)
{
    // CORDIC硬件要求输入角度范围：[-π, π]
    // 先归一化到[-π, π]
    while (angle > FOC_PI) angle -= FOC_2PI;
    while (angle < -FOC_PI) angle += FOC_2PI;
    
    // CORDIC输入格式：Q1.31（定点数）
    // 角度范围：[-1, 1] 对应 [-π, π]
    int32_t angle_q31 = (int32_t)(angle * 2147483648.0f / FOC_PI);
    
    int32_t results[2];
    
    // 使用CORDIC计算（轮询模式）
    // CORDIC配置为COSINE模式时，会同时输出cos和sin
    // 输出：results[0] = cos, results[1] = sin
    HAL_CORDIC_Calculate(&hcordic, &angle_q31, results, 1, HAL_MAX_DELAY);
    
    // Q1.31转换为浮点数
    *cos_val = (float)results[0] / 2147483648.0f;
    *sin_val = (float)results[1] / 2147483648.0f;
}

//设置三相电压的占空比
//通过占空比设置三相电压
//参数为比例系数
//新的驱动板u,v,w的定义是反过来的
void FOC_SetDuty(float va,float vb,float vc)
{   
    // 强制限制在 0.0 到 1.0 之间
    if(va > 1.0f) va = 1.0f; else if(va < 0.0f) va = 0.0f;
    if(vb > 1.0f) vb = 1.0f; else if(vb < 0.0f) vb = 0.0f;
    if(vc > 1.0f) vc = 1.0f; else if(vc < 0.0f) vc = 0.0f;
    htim1.Instance->CCR3 = (uint32_t)(va * FOC_PWM_PERIOD); //3通道输出的才是u相位
    htim1.Instance->CCR2 = (uint32_t)(vb * FOC_PWM_PERIOD);
    htim1.Instance->CCR1 = (uint32_t)(vc * FOC_PWM_PERIOD);
}


// /**
//  * @brief 设置三相电压（优化版：接收已计算的sin/cos，避免重复计算）
//  * @param ud D轴电压
//  * @param uq Q轴电压
//  * @param cos_angle 电角度的余弦值（已计算）
//  * @param sin_angle 电角度的正弦值（已计算）
//  */
// static inline void FOC_SetPhaseVoltage_Fast(float ud, float uq, float cos_angle, float sin_angle) 
// {
//     ud *= 0.99f;
//     uq *= 0.99f;
    
//     // 帕克逆变换 dq坐标系到alpha-beta坐标系
//     // 优化：直接使用传入的sin/cos，避免重复计算
//     float Ualpha = (-uq * sin_angle + ud * cos_angle) * 0.5f;// 除以2将Ua范围限制在[-0.5,0.5],使后续Uu,Uv,Uw范围在[0,1]
//     float Ubeta = (uq * cos_angle + ud * sin_angle) * 0.5f;

//     // 克拉克逆变换 alpha-beta坐标系到三相坐标系
//     three_phase.va = Ualpha + 0.5f;//加0.5使得Uu均值为0.5,在[0,1]之间变化
//     three_phase.vb = (FOC_SQRT3 * Ubeta - Ualpha) * 0.5f + 0.5f;
//     three_phase.vc = (-FOC_SQRT3 * Ubeta - Ualpha) * 0.5f + 0.5f;
    
//     FOC_SetDuty(three_phase.va, three_phase.vb, three_phase.vc);
// }

//使用svpwm
static inline void FOC_SetPhaseVoltage_Fast(float ud, float uq, float cos_angle, float sin_angle) 
{
    // 1. 逆 Park 变换 (不要乘 0.5)
    float Ualpha = -uq * sin_angle + ud * cos_angle;
    float Ubeta  =  uq * cos_angle + ud * sin_angle;

    // 2. 逆 Clarke 变换
    float va = Ualpha;
    float vb = (FOC_SQRT3 * Ubeta - Ualpha) * 0.5f;
    float vc = (-FOC_SQRT3 * Ubeta - Ualpha) * 0.5f;

    // 3. 注入中性点偏移 (简易 SVPWM)，这一步是把利用率从 86.6% 提升到 100% 的关键
    float v_min = va;
    if (vb < v_min) v_min = vb;
    if (vc < v_min) v_min = vc;
    
    float v_max = va;
    if (vb > v_max) v_max = vb;
    if (vc > v_max) v_max = vc;

    float v_offset = (v_max + v_min) * 0.5f;
    
    // 4. 映射到 [0, 1] 范围
    //减去v_offset是为了使va, vb, vc的范围在-0.5到0.5之间
    //加上0.5是为了使va, vb, vc的范围在0到1之间
    // 此时 va, vb, vc 的差值最大可以达到 1.0 倍母线电压
    three_phase.va = (va - v_offset) * 1.0f + 0.5f; 
    three_phase.vb = (vb - v_offset) * 1.0f + 0.5f;
    three_phase.vc = (vc - v_offset) * 1.0f + 0.5f;
    
    FOC_SetDuty(three_phase.va, three_phase.vb, three_phase.vc);
}



// 保留旧版本兼容性（用于校准等场景）
void FOC_SetPhaseVoltage(float ud, float uq, const float electrical_angle) 
{
    float cos_angle, sin_angle;
    FOC_FastSinCos(electrical_angle, &sin_angle, &cos_angle);
    FOC_SetPhaseVoltage_Fast(ud, uq, cos_angle, sin_angle);
}

//更新id和iq的pid参数如果电压变化的话
void FOC_Update_PID_Parameter(void){
    float factor = 1.0f / foc_ctrl.vbat_voltage;
    PID_Inc_SetGains(&PID_IQ, PID_IQ.kp*factor, PID_IQ.ki*factor, PID_IQ.kd*factor);
    PID_Inc_SetGains(&PID_ID, PID_ID.kp*factor, PID_ID.ki*factor, PID_ID.kd*factor);
}

// 初始化FOC
void FOC_Init(void)
{
    // 初始化控制参数
    foc_ctrl.target_speed = 0.0f;
    foc_ctrl.current_speed = 0.0f;
    foc_ctrl.target_angle = 0.0f;        // 初始化目标角度
    foc_ctrl.electrical_angle = 0.0f;
    foc_ctrl.id_ref = 0.0f;              // d轴电流参考值为0（最大转矩控制）
    foc_ctrl.iq_ref = 0.0f;              // q轴电流参考值（控制转矩），可根据需要调整
    foc_ctrl.voltage_limit = FOC_VOLTAGE_LIMIT;
    foc_ctrl.vbat_voltage = 0.0f;
    foc_ctrl.id_actual = 0.0f;
    foc_ctrl.iq_actual = 0.0f;
    foc_ctrl.ud = 0.0f;
    foc_ctrl.uq = 0.0f;
    foc_ctrl.current_speed = 0.0f;
    foc_ctrl.mode = FOC_MODE_CURRENT_LOOP;//初始为电流环
    
    // 初始化配置信息
    config_info.phase_resistance = 0.0f;
    config_info.iu_offset = 0.0f;
    config_info.iv_offset = 0.0f;
    config_info.iw_offset = 0.0f;
    config_info.zero_electric_angle = 0.0f;
    config_info.encoder_direction = 0;
    config_info.calibrated = 0;

    // 配置CORDIC硬件加速器（用于快速sin/cos计算）
    CORDIC_ConfigTypeDef sCordicConfig = {0};
    sCordicConfig.Function = CORDIC_FUNCTION_COSINE;     // COSINE模式：同时输出cos和sin
    sCordicConfig.Precision = CORDIC_PRECISION_6CYCLES;  // 6次迭代，足够FOC使用
    sCordicConfig.Scale = CORDIC_SCALE_0;                // 无缩放
    sCordicConfig.NbWrite = CORDIC_NBWRITE_1;            // 一次写入（只需角度）
    sCordicConfig.NbRead = CORDIC_NBREAD_2;              // 两次读取（cos和sin）
    sCordicConfig.InSize = CORDIC_INSIZE_32BITS;         // 32位输入（Q1.31）
    sCordicConfig.OutSize = CORDIC_OUTSIZE_32BITS;       // 32位输出（Q1.31）
    HAL_CORDIC_Configure(&hcordic, &sCordicConfig);

    //初始化iq和id的pid
    PID_Inc_Init(&PID_IQ, 0.0001f);
    PID_Inc_Init(&PID_ID, 0.0001f);

    PID_Inc_SetGains(&PID_IQ, KP_IQ, KI_IQ, 0.0f);
    PID_Inc_SetGains(&PID_ID, KP_ID, KI_ID, 0.0f);

    //初始化速度环pid
    PID_Init(&PID_Speed, 0.0001f);
    PID_Speed.integral_max = 2e3f;
    PID_Speed.integral_min = -2e3f;
    PID_Speed.out_max = FOC_MAX_CURRENT;
    PID_Speed.out_min = -FOC_MAX_CURRENT;
    PID_Speed.is_integral_enable = 1;
    PID_SetGains(&PID_Speed, KP_SPEED, KI_SPEED, KD_SPEED);

    //初始化角度环pid
    PID_Init(&PID_Angle, 0.0001f);
    PID_Angle.out_max = FOC_MAX_SPEED;
    PID_Angle.out_min = -FOC_MAX_SPEED;
    PID_Angle.is_integral_enable = 0;//不使用积分限幅
    PID_SetGains(&PID_Angle, KP_ANGLE, KI_ANGLE, KD_ANGLE);

    HAL_TIM_Base_Start_IT(&htim2);   

      // 启动三相PWM输出（必须在CurrentSense_Start之前启动）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // A相PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // B相PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // C相PWM
    
    // 如果使用互补PWM输出（低侧开关），需要启动互补通道
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // A相互补PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // B相互补PWM
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // C相互补PWM
}

//foc校准
//编码器角度是逆时针增大，电角度也是
void FOC_Calibrate(void){
    config_info.calibrated = 0; // 标记为未校准    
    /*1.校准电流偏置*/
    FOC_SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 设置电压为0
    
    config_info.iu_offset = 0.0f;
    config_info.iv_offset = 0.0f;
    config_info.iw_offset = 0.0f;
    for (uint16_t i = 0; i < 200; i++) {
        // 读取电流值，200次平均
        config_info.iu_offset += current_sense.currents.Iu / 200.0f;
        config_info.iv_offset += current_sense.currents.Iv / 200.0f;
        config_info.iw_offset += current_sense.currents.Iw / 200.0f;
        HAL_Delay(1);
    }
    
    /*2.测量相电阻，确定后续校准使用电压*/
    FOC_SetDuty(0.0f, 0.0f, 0.0f);
    float uu = 0.0f;
    float max_current = 1.65f; // GB4310最大电流约1.9A，取90%为1.7A左右
    // 逐渐增加电压，直到电流达到安全值
    for (uu = 0.0f; uu < 0.8f && (current_sense.currents.Iu - config_info.iu_offset) < max_current * 0.9f; ) {
        uu += 0.001f;
        FOC_SetDuty(uu, 0.0f, 0.0f);
        HAL_Delay(1);
    }
    // 测量相电阻，100次平均
    config_info.phase_resistance = 0.0f;
    for (uint16_t i = 0; i < 100; i++) {
        config_info.phase_resistance += uu * foc_ctrl.vbat_voltage / (current_sense.currents.Iu - config_info.iu_offset) / 3.0f * 4.0f / 100.0f;
        HAL_Delay(1);
    }
    FOC_SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 设置电压为0
    // 计算校准电压（限制在合理范围内）
    float voltage_align = uu * 4.0f / 3.0f;
    if (voltage_align > 1.0f) voltage_align = 1.0f;
    if (voltage_align < -1.0f) voltage_align = -1.0f;
    
    /*3.校准编码器正方向，使其与q轴正方向相同*/
    FOC_SetPhaseVoltage(voltage_align * 0.3f, 0.0f, 0.0f);
    // 读取电机角度，100次平均
    HAL_Delay(100);
    float begin_angle = 0.0f;
    for (uint16_t i = 0; i < 100; i++) {
        MT6825_ReadAngleData(&angle_data);
        begin_angle += angle_data.angle_rad / 100.0f;
    }
    //4.42
    // 按q轴正方向硬拖2π电角度 4.84
    for (uint16_t i = 0; i < 500; i++) {
        float angle = FOC_2PI * i / 500.0f;
        FOC_SetPhaseVoltage(voltage_align * 0.3f, 0.0f, angle);
        HAL_Delay(1);
    }
    // 读取电机角度，100次平均
    float end_angle = 0.0f;
    for (uint16_t i = 0; i < 100; i++) {
        MT6825_ReadAngleData(&angle_data);
        end_angle += angle_data.angle_rad / 100.0f;
    }
    FOC_SetPhaseVoltage(0.0f, 0.0f, 0.0f); // 停止电机
    // 判断编码器方向
    //是正方向
    if ((end_angle > begin_angle && end_angle < begin_angle + FOC_PI) ||
        end_angle < begin_angle - FOC_PI) {
        config_info.encoder_direction = 1; // 正向
    } else {
        config_info.encoder_direction = 0; // 反向
    }
    
    /*4.校准电角度零点*/
    float sum_offset_angle = 0.0f;
    for (uint8_t i = 0; i < FOC_POLE_PAIRS; i++) {
        // 按q轴正方向硬拖2π电角度
        for (uint16_t j = 0; j < 250; j++) {
            float angle = FOC_2PI * j / 250.0f;
            FOC_SetPhaseVoltage(voltage_align * 0.3f, 0.0f, angle);
            HAL_Delay(1);
        }
        FOC_SetPhaseVoltage(voltage_align*0.5, 0.0f, 0.0f);
        HAL_Delay(300);
        // 读取角度100次平均
        for (uint16_t j = 0; j < 100; j++) {
            MT6825_ReadAngleData(&angle_data);
            if (config_info.encoder_direction) {
                sum_offset_angle += (FOC_2PI - angle_data.angle_rad) / 100.0f;
            } else {
                sum_offset_angle += angle_data.angle_rad / 100.0f;
            }
            HAL_Delay(1);
        }
    }
    FOC_SetPhaseVoltage(0.0f, 0.0f, 0.0f);//恢复
    config_info.zero_electric_angle = (sum_offset_angle - FOC_PI * (FOC_POLE_PAIRS - 1)) / FOC_POLE_PAIRS;

    config_info.calibrated = 1; // 标记为已校准
}

//设置在当前的角度基础上以多少速度运动多少角度
void FOC_Set_Step_Speed_Angle(float step_angle, float target_speed){
    MT6825_ReadAngleData(&angle_data);//在当前角度的基础上移动一定的角度
    float current_angle = angle_data.angle_rad;
    float target_angle = current_angle + step_angle;
    
    // 归一化到[0, 2π]
    while(target_angle >= FOC_2PI) target_angle -= FOC_2PI;
    while(target_angle < 0.0f) target_angle += FOC_2PI;
    if(step_angle < 0.0f) target_speed = -target_speed;
    
    // 调用速度+角度模式
    FOC_Set_Speed_Angle(target_angle, target_speed);
}

//设置位置速度模式
//偏差为0.0057度
//弧度为0.0001
//这个是移动到角度编码器的绝对位置
//这个位置只有0到2π之间
void FOC_Set_Speed_Angle(float target_angle, float target_speed){
    move_target_angle = target_angle;
    foc_ctrl.mode = FOC_MODE_SPEED_ANGLE_LOOP;
    last_angle = angle_data.angle_rad;//如果不设置last_angle,会由于刚起步的时候速度计算错误，导致iq为最大值，电机会抽搐一下
    low_speed = target_speed;//目标速度
    low_speed_angle = angle_data.angle_rad;//记录初始位置
    move_error = (FOC_2PI *fabs(low_speed) / FOC_UPDATE_FREQ_SPEED_ANGLE/60.0f);//2ms的移动误差
    //如果速度为负的，需要取绝对值
}


//设置三环的参数
//这个是用户设置参数时调用不参与更新
void FOC_Set_Parameter(FOC_Mode_t mode,float value){
    MT6825_ReadAngleData(&angle_data);
    switch(mode){
        case FOC_MODE_LOW_SPEED_LOOP:
            low_speed = value;
            low_speed_angle = angle_data.angle_rad;
            last_angle = angle_data.angle_rad;
            previous_angle = angle_data.angle_rad;
            break;
        case FOC_MODE_STEP_ANGLE_LOOP://步进模式
            if(foc_ctrl.mode == FOC_MODE_STEP_ANGLE_LOOP){//在当前的位置上移动一定角度
                foc_ctrl.target_angle = foc_ctrl.target_angle + value;//如果已经是步进模式那么在之前的值上加上步进的角度
            } else {
                foc_ctrl.target_angle = angle_data.angle_rad + value;//如果第一次设置步进模式那么就设置为当前角度加上步进的角度
            }
            if(foc_ctrl.target_angle > FOC_2PI) foc_ctrl.target_angle -= FOC_2PI;
            else if(foc_ctrl.target_angle < 0.0f) foc_ctrl.target_angle += FOC_2PI;
            break;
        case FOC_MODE_ANGLE_LOOP://控制达到目标角度使用最短路径,不用绕一圈才达到目标角度
            if(value - angle_data.angle_rad > FOC_PI) value -= FOC_2PI;
            else if(value - angle_data.angle_rad < -FOC_PI) value += FOC_2PI;
            foc_ctrl.target_angle = value;
            break;
        case FOC_MODE_SPEED_LOOP:
            foc_ctrl.target_speed = value;
            break;
        case FOC_MODE_CURRENT_LOOP://电流环直接设置iq_ref
            foc_ctrl.iq_ref = value;
            break;
        default:
            break;
    }
    foc_ctrl.mode = mode;
}

//角度环和速度环的更新 1khz
void FOC_Angle_And_Speed_Update(void){
    MT6825_ReadAngleData(&angle_data);
    if(config_info.calibrated == 0){
        previous_angle = angle_data.angle_rad;
        return;//如果没有校准完成不进行角度环和速度环更新
    }
    switch(foc_ctrl.mode){//先位置环，然后速度环最后电流环
        case FOC_MODE_SPEED_ANGLE_LOOP://位置速度模式
            if(fabs(angle_data.angle_rad - move_target_angle) < move_error) {
                low_speed = 0.0f;//如果已经到达目标位置那么就速度为0
                foc_ctrl.mode = FOC_MODE_ANGLE_LOOP;//切换到角度环锁住目标角度
                low_speed_angle = move_target_angle;
            }
        case FOC_MODE_LOW_SPEED_LOOP://低速模式
            low_speed_angle += FOC_2PI *low_speed / FOC_UPDATE_FREQ_SPEED_ANGLE/60.0f;//计算出当前这1ms需要移动的角度
            if(low_speed_angle > FOC_2PI) low_speed_angle -= FOC_2PI;
            else if(low_speed_angle < 0.0f) low_speed_angle += FOC_2PI;
            foc_ctrl.target_angle = low_speed_angle;
        case FOC_MODE_ANGLE_LOOP:
        case FOC_MODE_STEP_ANGLE_LOOP:
            if(previous_angle - angle_data.angle_rad > FOC_PI) previous_angle -= FOC_2PI;
            else if(previous_angle - angle_data.angle_rad < -FOC_PI) previous_angle += FOC_2PI;
            foc_ctrl.target_speed = PID_Update(&PID_Angle, foc_ctrl.target_angle, angle_data.angle_rad);
        case FOC_MODE_SPEED_LOOP://速度环直接输出目标iq
            foc_ctrl.iq_ref = PID_Update(&PID_Speed, foc_ctrl.target_speed, foc_ctrl.current_speed);
            break;
        default:
            break;
    }
    previous_angle = angle_data.angle_rad;
}

/**
 * @brief FOC电流环更新函数（极致优化版）
 * 
 * 主要优化：
 * 1. 移除重复的MT6825_ReadAngleData()调用（使用中断中已读取的angle_data）
 * 2. 只调用一次CORDIC计算sin/cos，所有变换复用该值
 * 3. 内联Clarke和Park变换，避免函数调用开销
 * 4. 优化角度归一化，避免fmod
 * 5. 使用寄存器变量减少内存访问
 * 
 * 预期性能：从100us降至15-20us
 */
void FOC_Current_Loop_Update(void)
{   
    if(config_info.calibrated == 0){
        MT6825_ReadAngleData(&angle_data);
        last_angle = angle_data.angle_rad;
        return;//如果没有校准完成不进行电流环更新
    }

     //计算电角度
     //读取角度
     MT6825_ReadAngleData(&angle_data);
     float mechanical_angle = angle_data.angle_rad;
    
     // 根据编码器方向计算校准后的机械角度（优化：减少FOC_Wrap调用）
     float angle;
     if (config_info.encoder_direction) {
         angle = config_info.zero_electric_angle + mechanical_angle;
     } else {
         angle = config_info.zero_electric_angle - mechanical_angle;
     }
     
     // 快速角度归一化到[0, 2π]
     while (angle >= FOC_2PI) angle -= FOC_2PI;
     while (angle < 0.0f) angle += FOC_2PI;
     
     // 计算电角度（机械角度 × 极对数）
     float electrical_angle = angle * FOC_POLE_PAIRS;
     while (electrical_angle >= FOC_2PI) electrical_angle -= FOC_2PI;
     while (electrical_angle < 0.0f) electrical_angle += FOC_2PI;
     
     foc_ctrl.electrical_angle = electrical_angle;

    
    /** 2. 使用CORDIC硬件一次性计算sin/cos（关键优化！）**/
    float sin_angle, cos_angle;
    FOC_FastSinCos(foc_ctrl.electrical_angle, &sin_angle, &cos_angle);
    
    /** 3. Clarke变换 + Park变换（内联，复用sin/cos）**/
    // 读取校正后的三相电流
    register float iu = current_sense.currents.Iu - config_info.iu_offset;
    register float iv = current_sense.currents.Iv - config_info.iv_offset;
    
    // Clarke变换：abc -> αβ
    register float Ia = iu;  // Iα
    register float Ib = (iu + 2.0f * iv) * INV_SQRT3;  // Iβ

    alpha_beta.alpha = Ia;
    alpha_beta.beta = Ib;
    
    // Park变换：αβ -> dq（复用已计算的sin/cos）
    // 标准Park变换公式：id = Iα*cos(θ) + Iβ*sin(θ), iq = -Iα*sin(θ) + Iβ*cos(θ)
    foc_ctrl.id_actual = Ia * cos_angle + Ib * sin_angle;
    foc_ctrl.iq_actual = -Ia * sin_angle + Ib * cos_angle;

    //计算转速
    
    float temp = angle_data.angle_rad - last_angle;
    if(last_angle-angle_data.angle_rad > FOC_PI) temp+=FOC_2PI;
    else if(last_angle-angle_data.angle_rad < -FOC_PI) temp-=FOC_2PI;
    //temp * 60.0f * FOC_UPDATE_FREQ / FOC_2PI原始值
    foc_ctrl.current_speed = (temp * 60.0f * FOC_UPDATE_FREQ / FOC_2PI)*0.5+foc_ctrl.current_speed*0.5;//使用一阶滤波
    last_angle = angle_data.angle_rad;
    
    /** 5. PID电流控制 **/
    static float ud = 0.0f;
    static float uq = 0.0f;

    // // PID 使用过滤后的值
    ud = PID_Inc_Update(&PID_ID, foc_ctrl.id_ref,foc_ctrl.id_actual);
    uq = PID_Inc_Update(&PID_IQ, foc_ctrl.iq_ref, foc_ctrl.iq_actual);

    foc_ctrl.ud = ud;
    foc_ctrl.uq = uq;//方便打印出ud和uq
    
    // /** 6. 逆Park + 逆Clarke变换并设置PWM（复用sin/cos）**/
    FOC_SetPhaseVoltage_Fast(ud, uq, cos_angle, sin_angle);
}

/**
 * @brief 读取母线电压（通过ADC2 Channel 17）
 * 
 * 该函数读取ADC2的常规通道17（Channel 17），并将其转换为实际电池电压。
 * 注意：
 * 1. 该函数会临时停止ADC注入转换，读取电池电压后重新启动
 * 2. 建议在初始化时调用一次，或者在主循环中以低频率（>100ms）调用
 * 3. 不要在高频中断中调用此函数！
 * 4. 如果电池电压通过分压电路连接到ADC，请根据实际分压比调整 FOC_VBAT_DIVIDER_RATIO
 */
void FOC_Read_Vbat_Voltage(void)
{
    uint32_t adc_value = 0;
    float adc_voltage = 0.0f;

    
    // 1. 启动ADC常规转换
    if (HAL_ADC_Start(&hadc2) != HAL_OK)
    {
        return;
    }
    
    // 2. 等待转换完成（超时时间10ms）
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
    {
        // 3. 读取ADC值（12位，范围0-4095）
        adc_value = HAL_ADC_GetValue(&hadc2);
        
        // 4. 转换为电压值（ADC值 -> 参考电压）
        // ADC电压 = ADC值 / 分辨率 * 参考电压
        adc_voltage = (float)adc_value / FOC_ADC_RESOLUTION * FOC_ADC_VREF;
        
        // 5. 计算实际电池电压（考虑分压比）
        // 实际电池电压 = ADC电压 * 分压比
        foc_ctrl.vbat_voltage = adc_voltage * FOC_VBAT_DIVIDER_RATIO;
    }
    //停止常规转换
    HAL_ADC_Stop(&hadc2);
}