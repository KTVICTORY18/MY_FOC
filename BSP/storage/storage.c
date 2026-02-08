/**
 ******************************************************************************
 * @file           : storage.c
 * @brief          : Flash存储管理
 * @description    : 用于在STM32G431的最后一个Flash页面(2KB)存储配置信息
 ******************************************************************************
 */

#include "storage.h"
#include <string.h>

/**
 * @brief  擦除配置信息Flash页面
 * @retval HAL_StatusTypeDef HAL状态
 */
HAL_StatusTypeDef Storage_EraseConfig(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // 解锁Flash
    HAL_FLASH_Unlock();

    // 配置擦除参数
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;//擦除模式，按页擦除
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Page = STORAGE_PAGE_NUMBER;//擦除的页码，最后一个页面
    EraseInitStruct.NbPages = 1;//擦除的页数

    // 执行擦除
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

    // 锁定Flash
    HAL_FLASH_Lock();

    return status;
}

/**
 * @brief  写入配置信息到Flash
 * @param  config: 指向ConfigInfo_t结构体的指针
 * @retval HAL_StatusTypeDef HAL状态
 */
HAL_StatusTypeDef Storage_WriteConfig(ConfigInfo_t *config)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t address = STORAGE_CONFIG_INFO_ADDRESS;
    uint64_t *data = (uint64_t *)config;
    uint32_t data_size = sizeof(ConfigInfo_t);

    // 检查参数
    if (config == NULL) {
        return HAL_ERROR;
    }

    // 先擦除页面
    status = Storage_EraseConfig();
    if (status != HAL_OK) {
        return status;
    }

    // 解锁Flash
    HAL_FLASH_Unlock();

    // 按64位(双字)写入数据
    //一次写入8个字节
    for (uint32_t i = 0; i < data_size / 8; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 8;
    }

    // 处理剩余字节(如果有)
    uint32_t remaining = data_size % 8;
    if (remaining > 0) {
        uint64_t last_data = 0xFFFFFFFFFFFFFFFF;
        memcpy(&last_data, &((uint8_t *)config)[data_size - remaining], remaining);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, last_data);
    }

    // 锁定Flash
    HAL_FLASH_Lock();

    return status;
}

/**
 * @brief  从Flash读取配置信息
 * @param  config: 指向ConfigInfo_t结构体的指针，用于存储读取的数据
 * @retval HAL_StatusTypeDef HAL状态
 */
HAL_StatusTypeDef Storage_ReadConfig(ConfigInfo_t *config)
{
    // 检查参数
    if (config == NULL) {
        return HAL_ERROR;
    }

    // 直接从Flash地址读取数据
    uint8_t *flash_data = (uint8_t *)STORAGE_CONFIG_INFO_ADDRESS;
    memcpy(config, flash_data, sizeof(ConfigInfo_t));

    return HAL_OK;
}
