/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_ds3231.h"
#include <stdio.h>  //printf()
#include <string.h> // memset()
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BUTTON    B1_GPIO_Port,B1_Pin
#define LED       LD2_GPIO_Port,LD2_Pin

#define HALF_SECOND_DELAYS  5
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t counter;
float temp;
extern I2C_HandleTypeDef hi2c1; // from i2c.c, changes for I2C2

//uint8_t aTxBuffer[100] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
RTC_st rtc = {
    .Year = 21, .Month = 10, .Date = 17,
    .DaysOfWeek = SUNDAY,
    .Hour = 22, .Min = 16, .Sec = 0
};

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  LL_GPIO_SetOutputPin(LED);

  // wait for button to be released.
  printf("waiting: ");
  while (LL_GPIO_IsInputPinSet(BUTTON))
  {
    counter++;
    printf(".");
    HAL_Delay(100);
    if (counter % 20 == 0)
      printf("\n");
  }

  printf("\n");
  LL_GPIO_ResetOutputPin(LED);

//  counter = 0;
//  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)DS3231_ADDR, (uint8_t*)aTxBuffer, sizeof(aTxBuffer), 10000)!= HAL_OK)
//  {
//        /* Error_Handler() function is called when Timeout error occurs.
//       When Acknowledge failure occurs (Slave don't acknowledge its address)
//       Master restarts communication */
//    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
//    {
//      Error_Handler();
//    }
//    counter++;
//  }
//
  // make sure the button is released
  while (!LL_GPIO_IsInputPinSet(BUTTON))
    ;

  printf("looking for DS3231.\n");

  counter = 10;
  while(!DS3231_Init(&hi2c1) && counter--)
  {
    printf("DS3231 init failed!\n");
    printf("%d tries remaining.\n", counter);
    HAL_Delay(100);
    LL_GPIO_TogglePin(LED);
  }

  LL_GPIO_SetOutputPin(LED);
  printf("success! DS3231 answered!\n");

  if (!DS3231_SetTime(&rtc))
    printf("DS3231 time set failed!\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    memset(&rtc,0,sizeof(rtc));
    printf("reading: ");
    if(!DS3231_GetTime(&rtc))
    {
      printf("get time failed\n");
      LL_GPIO_ResetOutputPin(LED);
    }
    else
    {
      LL_GPIO_SetOutputPin(LED);
      printf("year: %d, month %d date %d\n", rtc.Year,rtc.Month,rtc.Date);
      printf("%2d:%2d:%2d\n", rtc.Hour,rtc.Min,rtc.Sec);
    }

    if(DS3231_ReadTemperature(&temp))
    {
      printf("temperature is %2.2fC\n",temp);
    }

    for (int i = 0; i < HALF_SECOND_DELAYS; i++)
    {
      HAL_Delay(500);
      printf(".");
    }
    printf("\n");

    /* USER CODE END WHILE */

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_3, LL_RCC_PLL_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(24000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
