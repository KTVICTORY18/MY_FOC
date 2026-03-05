# MY_FOC - 高性能无刷电机 FOC 控制系统

## 项目概述

这是一个基于 STM32G431 微控制器的**开环 FOC（磁场定向控制）无刷电机驱动系统**，用于控制 GB4310 电机。项目集成了多种硬件外设和控制算法，具有高集成度和高性能的特点。

### 主要特性

- ✅ **开环 FOC 控制**：实现磁场定向控制算法，相位电压精确控制
- ✅ **20KHz PWM 频率**：高频 PWM 驱动，平稳低噪声
- ✅ **PID 多环控制**：位置环、速度环、电流环独立调参
- ✅ **三相电流采样**：实时采集三相电流反馈
- ✅ **编码器反馈**：集成 MT6825GT 磁编码器支持
- ✅ **FDCAN 通信**：高速 CAN 总线通信接口
- ✅ **硬件加速**：利用 CORDIC 硬件加速器进行三角函数运算
- ✅ **存储管理**：Flash 参数存储与恢复
- ✅ **视觉集成**：支持 OpenMV 摄像头模块
- ✅ **UART 调试**：多通道串口通信与调试

---

## 目录结构

```
my_foc/
├── CMakeLists.txt              # CMake 构建配置
├── CMakePresets.json           # CMake 预设配置
├── BSP/                         # 板级支持包 (Board Support Package)
│   ├── foc/                     # FOC 控制核心库
│   │   ├── foc.c / foc.h
│   │   └── motor_gb4310.txt     # GB4310 电机参数文件
│   ├── pid/                     # PID 控制器（通用）
│   │   ├── pid.c / pid.h
│   ├── current_sense/           # 三相电流采样
│   │   ├── current_sense.c / current_sense.h
│   ├── MT6825GT/                # 磁编码器驱动
│   │   ├── MT6825GT.c / MT6825GT.h
│   ├── can_comm/                # FDCAN 通信封装
│   │   ├── can_comm.c / can_comm.h
│   ├── storage/                 # Flash 存储管理
│   │   ├── storage.c / storage.h
│   ├── usart/                   # 串口通信
│   │   ├── myUsart.c / myUsart.h
│   ├── user_protocol/           # 用户通信协议
│   │   ├── user_protocol.c / user_protocol.h
│   └── openmv/                  # OpenMV 摄像头支持
│       ├── openmv.c / openmv.h
├── Core/                        # STM32 核心驱动（由 CubeMX 生成）
│   ├── Inc/                     # 头文件
│   │   ├── main.h
│   │   ├── adc.h, dma.h, fdcan.h, gpio.h, spi.h, tim.h, usart.h
│   │   └── stm32g4xx_hal_conf.h, stm32g4xx_it.h
│   └── Src/                     # 源文件
│       ├── main.c
│       ├── adc.c, cordic.c, dma.c, fdcan.c, gpio.c, spi.c, tim.c, usart.c
│       └── 其他启动和配置文件
├── Drivers/                     # STM32 HAL 驱动库
│   ├── CMSIS/                   # ARM CMSIS 标准库
│   └── STM32G4xx_HAL_Driver/    # STM32G4 HAL 驱动
├── build/                       # CMake 构建输出目录
├── cmake/                       # CMake 工具链配置
├── startup_stm32g431xx.s        # 启动文件
└── STM32G431XX_FLASH.ld         # 链接脚本
```

---

## 硬件配置

### 核心控制器
- **MCU**: STM32G431xx
- **主频**: 170 MHz
- **RAM**: 32 KB
- **Flash**: 128 KB

### 关键外设
| 外设 | 用途 | 频率/配置 |
|------|------|---------|
| **TIM1** | PWM 驱动（三相） | 20 kHz |
| **TIM2** | 时间测量与计数 | - |
| **ADC** | 三相电流采样、电池电压采样 | 12-bit |
| **SPI** | MT6825 编码器通信 | - |
| **FDCAN** | CAN 总线通信 | 高速 CAN |
| **CORDIC** | 三角函数硬件加速 | - |
| **USART** | 串口调试通信 | - |
| **DMA** | 数据传输加速 | - |

### 电机配置
- **电机型号**: GB4310
- **极对数**: 14
- **编码器**: MT6825GT 磁旋转编码器

---

## 软件架构

### FOC 控制流程

```
主循环 (main loop)
    ↓
20kHz PWM 中断 (TIM1 Update Interrupt)
    ├─ 读取三相电流采样 (Current Sense)
    ├─ 读取编码器位置/速度 (MT6825GT)
    ├─ 执行 FOC 算法计算
    │  ├─ Clarke 变换 (三相到两相)
    │  ├─ Park 变换 (两相到旋转坐标系)
    │  ├─ PID 电流环控制
    │  └─ 反 Park 变换回电压指令
    ├─ 输出 PWM 占空比到 TIM1
    └─ 返回
    ↓
1kHz 速度/角度环 (可选定时器)
    ├─ 读取目标速度/角度
    ├─ 执行速度 PID 控制
    ├─ 执行角度 PID 控制（若需要）
    └─ 更新电流参考值
```

### 核心 API 说明

#### 1. FOC 模块 (`BSP/foc/`)

```c
// 初始化 FOC 系统
void FOC_Init(void);

// 设置目标转速（单位：RPM）
void FOC_SetSpeedRPM(float speed_rpm);

// 设置电角度速度
void FOC_SetSpeed(float speed);

// 实时更新 FOC 计算（在 PWM 中断中调用）
void FOC_Update(void);

// 停止电机
void FOC_Stop(void);

// 获取当前转速
float FOC_GetSpeedRPM(void);
```

#### 2. PID 控制模块 (`BSP/pid/`)

```c
// PID 初始化
void PID_Init(PID_t *pid, float Ts);

// 设置 PID 增益
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

// 设置输出限幅
void PID_SetOutputLimit(PID_t *pid, float min, float max);

// 更新 PID（返回控制输出）
float PID_Update(PID_t *pid, float setpoint, float measurement);

// 清空 PID 积分值
void PID_Clear(PID_t *pid);
```

#### 3. 编码器模块 (`BSP/MT6825GT/`)

```c
// 初始化编码器
void MT6825GT_Init(SPI_HandleTypeDef *hspi);

// 读取编码器角度（0-4095）
uint16_t MT6825GT_ReadAngle(void);

// 计算转速
float MT6825GT_GetSpeed(void);

// 获取电角度
float MT6825GT_GetElecAngle(void);
```

#### 4. 三相电流采样 (`BSP/current_sense/`)

```c
// 初始化电流采样
void Current_Sense_Init(void);

// 读取原始 ADC 值
void Current_Sense_ReadRaw(uint16_t *ia, uint16_t *ib, uint16_t *ic);

// 获取校准后的电流值（单位：A）
void Current_Sense_GetCurrent(float *ia, float *ib, float *ic);

// 读取电池电压
float Current_Sense_GetBatVoltage(void);
```

#### 5. CAN 通信 (`BSP/can_comm/`)

```c
// 初始化 CAN 通信
HAL_StatusTypeDef CAN_Comm_Init(void);

// 发送数据
HAL_StatusTypeDef CAN_Comm_Transmit(uint8_t *data, uint8_t length);

// 发送到指定 ID
HAL_StatusTypeDef CAN_Comm_Transmit_To_ID(uint32_t can_id, uint8_t *data, uint8_t length);

// 接收回调（用户实现）
void CAN_Comm_RxCallback(uint32_t can_id, uint8_t *data, uint8_t length);
```

#### 6. 存储管理 (`BSP/storage/`)

```c
// 初始化存储
void Storage_Init(void);

// 保存参数到 Flash
void Storage_Save(void);

// 从 Flash 恢复参数
void Storage_Load(void);
```

---

## 快速开始

### 1. 环境准备

**必需工具**：
- CMake 3.22 或更高版本
- ARM GCC 工具链（arm-none-eabi-gcc）
- VS Code + CMake Tools 扩展
- STM32CubeMX（用于修改硬件配置）

**安装 ARM 工具链**（Windows PowerShell）：
```powershell
# 使用 Chocolatey
choco install cmake arm-none-eabi-gcc

# 或手动下载
# https://developer.arm.com/downloads/-/gnu-rm
```

### 2. 构建项目

```bash
# 在项目根目录执行
cd c:\Users\Administrator\Desktop\myFOC\my_foc

# 配置 CMake（仅需一次）
cmake --preset=default -B build

# 编译
cmake --build build --config Debug

# 清理构建
cmake --build build --target clean
```

### 3. 烧录固件

```bash
# 使用 OpenOCD、JLink 或 STM32CubeProgrammer
# 烧录生成的 .elf 或 .bin 文件到 STM32G431
```

### 4. 在 main.c 中初始化

```c
int main(void)
{
    // 硬件初始化（由 CubeMX 生成）
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DMA_Init();
    MX_FDCAN1_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART1_Init();
    MX_CORDIC_Init();

    // 用户库初始化
    FOC_Init();              // FOC 初始化
    MT6825GT_Init(&hspi1);   // 编码器初始化
    Current_Sense_Init();    // 电流采样初始化
    CAN_Comm_Init();         // CAN 通信初始化
    Storage_Load();          // 恢复存储的参数

    // 启动 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 启动 TIM1 更新中断（触发 FOC 计算）
    HAL_TIM_Base_Start_IT(&htim1);

    // 设置目标转速
    FOC_SetSpeedRPM(500.0f);  // 500 RPM

    // 主循环
    while (1)
    {
        // 可选：速度环和角度环在这里更新（或在定时器中断中）
        // 其他任务：CAN 接收、UART 调试、存储等
    }

    return 0;
}
```

---

## 常见配置参数调整

### FOC 参数 (foc.h)

```c
#define FOC_PWM_PERIOD          4250        // PWM 周期（对应 20kHz）
#define FOC_VOLTAGE_LIMIT       0.3f        // 电压限制（0-1）
#define FOC_POLE_PAIRS          14          // 极对数（GB4310）
#define FOC_MAX_CURRENT         1.65f       // 最大电流（A）
#define FOC_MAX_SPEED           1000.0f     // 最大转速（RPM）
```

### PID 调参建议

1. **电流环** (最内层，最快)
   - 建议更新频率：20 kHz（与 PWM 同步）
   - 参数范围：Kp=0.1~1.0, Ki=0.01~0.1, Kd=0

2. **速度环** (中层)
   - 建议更新频率：1 kHz
   - 参数范围：Kp=0.1~0.5, Ki=0.01~0.05, Kd=0~0.02

3. **角度环** (外层，最慢)
   - 建议更新频率：100~500 Hz
   - 参数范围：Kp=0.5~2.0, Ki=0, Kd=0~0.05

### 调参步骤

1. **禁用积分**：设置 Ki=0
2. **调整 Kp**：逐步增加，直到响应快但不过冲
3. **增加 Ki**：逐步增加消除静差，若振荡则减小
4. **调整 Kd**：若需要，使用很小的值（通常 0）

---

## 故障排查

### 问题：电机不转

- [ ] 检查 PWM 输出是否正常（示波器测量 TIM1 输出）
- [ ] 检查编码器 SPI 通信是否正常
- [ ] 检查电源供电是否充足
- [ ] 通过 UART 查看错误日志

### 问题：电机抖动或噪声大

- [ ] 增大 `FOC_VOLTAGE_LIMIT` 值
- [ ] 减小电流环 Kp 或增加 Ki
- [ ] 检查三相电流采样是否准确
- [ ] 检查编码器偏差补偿

### 问题：响应缓慢

- [ ] 增大电流环 Kp
- [ ] 检查 FOC 更新频率（目标 20kHz）
- [ ] 减小速度环 Kp（若调过大）

---

## 开发建议

### 代码规范

- ✅ 函数命名：`模块名_功能名()`（如 `FOC_SetSpeed`）
- ✅ 变量命名：使用明确的英文名称，避免单字母
- ✅ 注释：关键算法、参数配置必须有中文注释
- ✅ 头文件：使用 `#ifndef` 防止重复包含

### 性能优化

- 📌 时间关键代码放在中断中（如 FOC_Update）
- 📌 使用 CORDIC 硬件加速三角函数
- 📌 使用 DMA 传输 ADC 数据
- 📌 避免在中断中调用 malloc/free

### 调试技巧

1. **UART 日志输出**：使用 `printf` 配合 `myUsart` 模块
2. **示波器观察**：PWM 波形、电流采样波形
3. **CAN 抓包**：使用 CAN 分析工具监控通信
4. **单步调试**：使用 J-Link 或 ST-Link 连接 VS Code

---

## 项目文档

- 📄 **GB4310 电机参数**：见 `BSP/foc/motor_gb4310.txt`
- 📄 **用户协议定义**：见 `BSP/user_protocol/user_protocol.h`
- 📄 **硬件原理图**：（需补充）
- 📄 **STM32G431 数据手册**：（参考官方文档）

---

## 常用命令

```powershell
# 进入项目目录
cd c:\Users\Administrator\Desktop\myFOC\my_foc

# 配置构建（仅一次）
cmake --preset=default -B build

# 增量编译
cmake --build build --config Debug

# 完整重新编译
cmake --build build --clean-first --config Debug

# 生成编译命令数据库（用于代码补全）
cmake --preset=default -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 格式化代码（需要 clang-format）
# find . -name "*.c" -o -name "*.h" | xargs clang-format -i
```


## 联系与支持

如有问题或建议，欢迎反馈！

---

**最后更新**：2026年3月5日  
**项目维护者**：KTVICTORY18
