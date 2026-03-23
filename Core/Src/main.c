/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "MT6825GT.h"
#include "current_sense.h"
#include "myUsart.h"
#include "storage.h"
#include "user_protocol.h"
#include "can_comm.h"
#include "openmv.h"

// 外部变量声明（用于时间测量）
extern TIM_HandleTypeDef htim2;


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Comm_Init();//初始化fdcan通信
  // 初始化MT6825GT编码器（必须在SPI初始化之后）
  MT6825_Init();
  // 初始化FOC
  FOC_Init();
  // 初始化电流采样模块
  CurrentSense_Init();
  //上电读取一下母线电压
  FOC_Read_Vbat_Voltage();
  FOC_Update_PID_Parameter();//根据电压更新pid参数
  // 启动电流采样（会启动ADC注入转换）
  CurrentSense_Start();
  // float electrical_angle = 0.0f;//电角度
  //FOC校准
  Storage_ReadConfig(&config_info);//第一次读取是0xFF calibrated
  if(config_info.calibrated != 1){//检测是否校准过
    config_info.motor_id = 0;//默认id为0
    FOC_Calibrate();//校准
    Storage_WriteConfig(&config_info);//写入配置信息
  }
  OpenMV_Init(&openmv);
  MyUsart_Init_DMA_Receive();//启动串口2的dma接收
  // static uint8_t tx_data[8] = {0xAA, 0x01, 0x02, 0x03, 0xFF};  // 不需要4字节对齐
  // FOC_Set_Parameter(FOC_MODE_LOW_SPEED_LOOP, 10.0f);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // electrical_angle += 0.1f;
    // FOC_SetPhaseVoltage(0.0f, 0.2f, electrical_angle);
    if(config_info.motor_id==0x02){//只有顶部的电机进行openmv数据处理和pid，计算后直接通过can总线发送给电机2
      OpenMV_Process(&openmv);//进行openmv数据处理和pid计算
    }
    if(calibrated_flag == 1){//耗时操作放到主循环中
      FOC_Calibrate();//是否校准的标志位是is_calibrated
      Storage_WriteConfig(&config_info);//写入配置信息
      calibrated_flag = 0;
    }
    if(save_config_flag == 1){
      Storage_WriteConfig(&config_info);//写入配置信息
      save_config_flag = 0;
    }
    MyUsart_SendAllCurrentsFromGlobal();
    // CAN_Comm_Transmit_To_ID(0x001, tx_data, 5);
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  ADC注入转换完成回调函数
 * @param  hadc: ADC句柄
 * @retval None
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        // tim2shi
        // ADC注入转换完成，此时数据已经存储在注入数据寄存器中
        // 在这里同步读取角度和电流，然后执行FOC更新，确保数据时间戳一致
        // 调试：如果这个回调都没有进入，说明ADC中断没有被触发
        // 请检查：1. TIM1是否运行 2. TRGO是否产生 3. ADC注入转换是否启动
        CurrentSense_Update();
        //执行foc的电流环更新
        FOC_Current_Loop_Update();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)// tim2的计时是1khz
  {
    //执行foc的角度环和速度环更新
    FOC_Angle_And_Speed_Update();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
