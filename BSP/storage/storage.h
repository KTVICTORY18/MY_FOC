#ifndef __STORAGE_H__
#define __STORAGE_H__

#include "foc.h"
#include "stm32g4xx_hal.h"

// Flash配置
#define STORAGE_CONFIG_INFO_ADDRESS 0x0801F800  // 使用最后一个页面
#define STORAGE_SIZE 2048                       // 字节 最后一个页面存储2k字节
#define STORAGE_CONFIG_INFO_SIZE 128            // 字节
#define STORAGE_PAGE_NUMBER 63                  // STM32G431最后一个页面编号(0-63)

// 函数声明
HAL_StatusTypeDef Storage_WriteConfig(ConfigInfo_t *config);
HAL_StatusTypeDef Storage_ReadConfig(ConfigInfo_t *config);
HAL_StatusTypeDef Storage_EraseConfig(void);

#endif