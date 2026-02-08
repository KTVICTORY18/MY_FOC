/**
 * @file MT6825GT.c
 * @brief MT6825GT磁编码器驱动实现
 */

#include "MT6825GT.h"
#include "stm32g431xx.h"
#include "spi.h"
#include "stm32g4xx_hal_gpio.h"

MT6825_AngleData_t angle_data = {0};

/* 更快的CS控制：直接写BSRR，避免HAL开销 */
#define MT6825_CS_LOW()   (GPIOC->BSRR = (1U << 4) << 16)
#define MT6825_CS_HIGH()  (GPIOC->BSRR = (1U << 4))

/* 已移除：不再需要全局超时常量，使用局部变量 */

/**
 * @brief MT6825初始化
 * 配置CS引脚为输出模式，并初始化SPI通信
 */
void MT6825_Init(void)
{   
    MT6825_CS_HIGH();
}

/**
 * @brief 使用Burst模式一次性读取MT6825三个角度寄存器（快速版本）
 * @param reg03 输出：寄存器0x03的值（角度[17:10]）
 * @param reg04 输出：寄存器0x04的值（角度[9:4] + No_Mag_Warning + PC1）
 * @param reg05 输出：寄存器0x05的值（角度[3:0] + Over_Speed + PC2 + NA）
 * @return 1-读取成功，0-读取失败
 * 
 * MT6825 SPI协议（4线SPI）：
 * - CSN下降沿触发SPI通信，CSN上升沿结束SPI通信
 * - SCK时钟信号由上位机发送，非通信状态下保持SCK为高电平（SPI Mode 3满足）
 * - 数据在SCK下降沿改变，推荐在SCK上升沿采样（SPI Mode 3: CPOL=1, CPHA=1）
 * 
 * Burst模式读取（原子操作，避免CS频繁切换导致的状态机混乱）：
 * - tx[0] = 0x83 (Bit 0=1读操作, Bit 1-7=0x03地址)
 * - tx[1-3] = 0x00 (提供时钟，MT6825会自动递增地址指针)
 * - rx[0] = 命令回显（忽略）
 * - rx[1] = 寄存器0x03的数据（Angle[17:10]）
 * - rx[2] = 寄存器0x04的数据（Angle[9:4] + No_Mag_Warning[bit1] + PC1[bit0]）
 * - rx[3] = 寄存器0x05的数据（Angle[3:0] + Over_Speed[bit3] + PC2[bit2] + NA[bit1:0]）
 */
/**
 * @brief 优化的高速SPI读取函数，目标：5-6us完成读取
 * 优化策略：
 * 1. 使用寄存器变量减少内存访问
 * 2. 优化等待逻辑，减少超时检查
 * 3. 使用紧凑的代码结构
 * 4. 快速CS切换
 */
static uint8_t MT6825_ReadAngleBurst(uint8_t *reg03, uint8_t *reg04, uint8_t *reg05)
{
    SPI_TypeDef *spi = SPI1;
    uint8_t rx_buf[4];
    uint32_t timeout;

    /* 快速状态清理：只清理关键状态 */
    if (spi->SR & SPI_SR_OVR) { (void)spi->DR; (void)spi->SR; }
    if (spi->SR & SPI_SR_RXNE) { (void)spi->DR; }

    /* 确保SPI配置正确 */
    //stm32g431有fifo先进先出的4个队列
    spi->CR2 |= SPI_CR2_FRXTH;
    if (!(spi->CR1 & SPI_CR1_SPE)) spi->CR1 |= SPI_CR1_SPE;
    
    /* CS拉低 */
    MT6825_CS_LOW();
    
    /* 快速连续写入4字节 */
    *(__IO uint8_t *)&spi->DR = 0x83;  // 命令字节
    
    /* 优化：适当增加超时时间，提高高速运动时的可靠性 */
    timeout = 100;  // 增加超时时间（原50）
    while (!(spi->SR & SPI_SR_TXE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    *(__IO uint8_t *)&spi->DR = 0x00;
    
    timeout = 100;  // 增加超时时间（原50）
    while (!(spi->SR & SPI_SR_TXE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    *(__IO uint8_t *)&spi->DR = 0x00;
    
    timeout = 100;  // 增加超时时间（原50）
    while (!(spi->SR & SPI_SR_TXE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    *(__IO uint8_t *)&spi->DR = 0x00;

    /* 快速读取4字节 */
    timeout = 500;  // 增加超时时间（原250）
    while (!(spi->SR & SPI_SR_RXNE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    rx_buf[0] = *(__IO uint8_t *)&spi->DR;
    
    /* 优化：展开循环减少开销 */
    timeout = 500;  // 增加超时时间（原250）
    while (!(spi->SR & SPI_SR_RXNE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    rx_buf[1] = *(__IO uint8_t *)&spi->DR;
    
    timeout = 500;  // 增加超时时间（原250）
    while (!(spi->SR & SPI_SR_RXNE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    rx_buf[2] = *(__IO uint8_t *)&spi->DR;
    
    timeout = 500;  // 增加超时时间（原250）
    while (!(spi->SR & SPI_SR_RXNE) && timeout--);
    if (!timeout) { MT6825_CS_HIGH(); return 0; }
    rx_buf[3] = *(__IO uint8_t *)&spi->DR;

    /* 等待传输完成（通常已经完成） */
    timeout = 100;  // 增加超时时间（原60）
    while ((spi->SR & SPI_SR_BSY) && timeout--);
    
    /* CS拉高 */
    MT6825_CS_HIGH();
    
    /* 输出数据 */
    if (reg03 != NULL) *reg03 = rx_buf[1];
    if (reg04 != NULL) *reg04 = rx_buf[2];
    if (reg05 != NULL) *reg05 = rx_buf[3];
    
    return 1;
}


/**
 * @brief 快速计算奇偶性（1的个数模2）
 * @param data 要计算的字节数据
 * @return 奇偶性（0=偶数个1，1=奇数个1）
 * 使用XOR折叠法，比计算1的个数快得多
 */
static inline uint8_t MT6825_Parity(uint8_t data)
{
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return data & 1;
}


/**
 * @brief 校验PC1奇偶校验位（偶数校验）
 * @param reg03 寄存器0x03的值（Angle[17:10]）
 * @param reg04 寄存器0x04的值（Angle[9:4] + No_Mag_Warning + PC1）
 * @param pc1_read 读取到的PC1值（0x04[0]）
 * @return 1-校验通过，0-校验失败
 * 
 * PC1校验规则（根据MT6825协议文档）：
 * - 检查0x03[7:0]和0x04[7:1]总共的1的个数
 * - 如果偶数个1，PC1应该为0
 * - 如果奇数个1，PC1应该为1
 * 优化：使用XOR直接计算奇偶性，无需计算1的个数
 */
static inline uint8_t MT6825_VerifyPC1(uint8_t reg03, uint8_t reg04, uint8_t pc1_read)
{
    // 计算0x03和0x04[7:1]的奇偶性（排除PC1位）
    // 优化：直接XOR后计算奇偶性，减少中间变量
    uint8_t combined = reg03 ^ (reg04 & 0xFE);  // 清除bit[0]（PC1位）并XOR
    uint8_t pc1_expected = MT6825_Parity(combined);
    
    // 比较读取到的PC1和期望的PC1
    return (pc1_read == pc1_expected);
}

/**
 * @brief 校验PC2奇偶校验位（偶数校验）
 * @param reg05 寄存器0x05的值（Angle[3:0] + Over_Speed + PC2 + NA）
 * @param pc2_read 读取到的PC2值（0x05[2]）
 * @return 1-校验通过，0-校验失败
 * 
 * PC2校验规则（根据MT6825协议文档）：
 * - 检查0x05[7:3]总共的1的个数
 * - 如果偶数个1，PC2应该为0
 * - 如果奇数个1，PC2应该为1
 * 优化：使用XOR直接计算奇偶性，无需计算1的个数
 */
static inline uint8_t MT6825_VerifyPC2(uint8_t reg05, uint8_t pc2_read)
{
    // 提取0x05[7:3]位（排除PC2和低2位）
    uint8_t reg05_bits_7_3 = (reg05 >> 3) & 0x1F;  // 提取bit[7:3]
    uint8_t pc2_expected = MT6825_Parity(reg05_bits_7_3);
    
    // 比较读取到的PC2和期望的PC2
    return (pc2_read == pc2_expected);
}



/**
 * @brief 高速读取角度数据（优化版本，带重试机制）
 * 优化策略：
 * 1. 增加重试机制（最多3次），提高高速运动时的可靠性
 * 2. 使用寄存器变量优化
 * 3. 紧凑的数据处理
 * 4. 移除不必要的范围检查（18位数据不会超过262143）
 * 5. 优化数据提取和计算
 */
void MT6825_ReadAngleData(MT6825_AngleData_t *data)
{
    uint8_t reg03, reg04, reg05;
    uint32_t raw;
    uint8_t pc1_read, pc2_read;
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;  // 最多重试3次
    
    if (data == NULL)
    {
        return;
    }

    data->data_valid = 0;

    /* 带重试的读取机制 */
    while (retry_count < max_retries)
    {
        /* 快速读取 */
        if (!MT6825_ReadAngleBurst(&reg03, &reg04, &reg05))
        {
            retry_count++;
            continue;  // SPI读取失败，重试
        }

        /* 快速提取奇偶校验位 */
        pc1_read = reg04 & 0x01;
        pc2_read = (reg05 >> 2) & 0x01;

        /* 快速奇偶校验 */
        if (!MT6825_VerifyPC1(reg03, reg04, pc1_read) ||
            !MT6825_VerifyPC2(reg05, pc2_read))
        {
            retry_count++;
            continue;  // 校验失败，重试
        }

        /* 校验通过，跳出重试循环 */
        break;
    }

    /* 如果所有重试都失败，返回无效数据 */
    if (retry_count >= max_retries)
    {
        return;
    }

    /* 快速组合18位角度（优化：使用位操作直接组合） */
    raw = ((uint32_t)reg03 << 10) |
          (((uint32_t)(reg04 >> 2) & 0x3F) << 4) |
          ((uint32_t)(reg05 >> 4) & 0x0F);

    /* 移除范围检查：18位数据范围是0-262143，不会超出 */

    /* 快速填充数据 */
    data->angle_raw      = raw;
    data->angle_deg      = (float)raw * MT6825_RAW_TO_DEG_CONST;
    data->angle_rad      = (float)raw * MT6825_RAW_TO_RAD_CONST;
    data->no_mag_warning = (reg04 >> 1) & 0x01;
    data->over_speed     = (reg05 >> 3) & 0x01;
    data->pc1            = pc1_read;
    data->pc2            = pc2_read;
    data->data_valid     = 1;
}






