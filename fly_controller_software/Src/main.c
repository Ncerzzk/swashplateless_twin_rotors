/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20600.h"
#include "i2c_ext.h"
#include "spi_slave.h"
#include "tim_ext.h"
#include "uart_ext.h"
#include "command.h"
#include "control.h"

#include "nrf24l01.h"
#include "can.h"

void SLAVE_Test();
uint16_t RC_ADC_Value[4]={0};
float RC_Control_Value[4]={0};
uint8_t NRF_Cnt=0;
uint8_t NRF_Key=0;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  debug_uart_init(&huart1,DMA,DMA);
  //Delay_Timer_Init(&htim6,84000000); 
  Delay_Us_NOP_Init(168000000);
  I2C_EXT_Init(&hi2c1,GPIOB,GPIO_PIN_8,GPIOB,GPIO_PIN_9,MX_I2C1_Init);
  MPU9250_Init(&MPU9250); 

  uint8_t temp[7]={"hell"};
  CAN_Send_Message(0x10,temp,7);
  uint8_t rx_addr[3][5] = {{19,97,07,15,0},{0},{0}};
  NRF_Init(&hspi1,1,NRF_CSN_GPIO_Port,NRF_CSN_Pin,NRF_CE_GPIO_Port,NRF_CE_Pin);
  //NRF_Set_Rx_Addr(rx_addr,sizeof(rx_addr));

  NRF_Receive_IT();

  uprintf("Init OK,wait 1s for motor drivers to init!\r\n");
  HAL_Delay(1000);

  uprintf("hello,world!\r\n");
  HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    if(buffer_rx_OK){
      UART_Command_Analize_And_Call();
    }
    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  NRF_RX_IRQ_Handler();
  NRF_Receive_IT();
}

/*
void RC_Key_Handle(uint8_t key){
  if(key & 0x01){
    uprintf("LEFT \r\n");

  }
  if(key & 0x02){
    uprintf("UP \r\n");
  }
  if(key &0x04){
    uprintf("RIGHT \r\n");

  }
  if(key &0x08){
    uprintf("DOWN \r\n");
  }
  if(key &0x10){
    uprintf("OK\r\n");
  }
  if(key &0x20){
    uprintf("Cancel\r\n");
  }
}
*/

void NRF_Receive_Callback(uint8_t * data,int len){
  NRF_Cnt = data[0];
  NRF_Key = data[1];

  if(NRF_Key==0xFF){
    return ; // 有时候会收到错误的包，所有位都为0xFF,不知道为什么
  }
  memcpy(RC_ADC_Value,data+2,8);
  for(int i =0;i<4;++i){
    if(RC_ADC_Value[i]!=0xFFFF){
        RC_Control_Value[i]=RC_ADC_Value[i]*1000.0f/4096.0f;
    }
  }

 RC_Key_Handle(NRF_Key);
  //uprintf("cnt:%d %d %d %d %d \r\n",NRF_Key,RC_ADC_Value[0],RC_ADC_Value[1],RC_ADC_Value[2],RC_ADC_Value[3]);
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

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
