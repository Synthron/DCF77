/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "dcf77.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Pin Defines
#define CS_Inner GPIO_PIN_0
#define CS_Outer GPIO_PIN_1
#define CS_Date GPIO_PIN_2
#define CS_Time GPIO_PIN_3
#define CS_Pulse GPIO_PIN_4
#define Ser_Data GPIO_PIN_8
#define Ser_Clock GPIO_PIN_9
#define Ser_Latch GPIO_PIN_10
#define RTC_Lock GPIO_PIN_15
#define PON GPIO_PIN_14
#define USB_Connect GPIO_PIN_8

//MAX7221 Defines
#define Dig0 0x1
#define Dig1 0x2
#define Dig2 0x3
#define Dig3 0x4
#define Dig4 0x5
#define Dig5 0x6
#define Dig6 0x7
#define Dig7 0x8
#define DecodeMode 0x9
#define Intensity 0xA
#define ScanLimit 0xB
#define Shutdown 0xC
#define DispTest 0xF


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//timestamp variables
uint32_t pulse_time1, pulse_time2, period_time;

//indicator array: 
//CEST-P1-P2-P3-PErr-Mo-Di-Mi-Do-Fr-Sa-So-CET
bool leds[13];

//Minute Telegrams
uint8_t bitno;
bool oldminute[60], newminute[60];

//time data variable
uint8_t hour_tens, hour_ones, 
        minute_tens, minute_ones, 
        second_tens, second_ones, 
        year_tens, year_ones, 
        month_tens, month_ones, 
        day_tens, day_ones, 
        week;
uint16_t period, pulse;

//SPI
uint8_t data_buf[2];

bool ok = false;

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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Module Init and test routines
  IO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void IO_Init(void)
{
  HAL_GPIO_WritePin(GPIOB, PON, 1);
  HAL_GPIO_WritePin(GPIOB, USB_Connect, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  HAL_GPIO_WritePin(GPIOA, Ser_Clock, 0);
  HAL_GPIO_WritePin(GPIOA, Ser_Data, 0);
  HAL_GPIO_WritePin(GPIOA, Ser_Latch, 0);

//Enable Displays
  data_buf[0] = Shutdown;
  data_buf[1] = 0x01;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);

//set 7segments to BCD
  data_buf[0] = DecodeMode;
  data_buf[1] = 0xFF;
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);

//set segment size to 6
  data_buf[0] = ScanLimit;
  data_buf[1] = 0x05;
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);

//set all segments to 0
  data_buf[0] = Dig0;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig1;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig2;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig3;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig4;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig5;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Date, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Time, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig6;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);
  data_buf[0] = Dig7;
  data_buf[1] = 0x00;
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 1);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, CS_Inner, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Outer, 0);
  HAL_GPIO_WritePin(GPIOA, CS_Pulse, 0);

//set decimal points
  MAX7221_Send(CS_Date, Dig2, 0x80);
  MAX7221_Send(CS_Date, Dig4, 0x80);
  MAX7221_Send(CS_Time, Dig2, 0x80);
  MAX7221_Send(CS_Time, Dig4, 0x80);

  for(int i = 0; i < 13; i++)
    leds[i] = 0;
  LED_Indicators(leds);

}

void MAX7221_Send(uint16_t _pin, uint8_t _reg, uint8_t _data)
{
  data_buf[0] = _reg;
  data_buf[1] = _data;
  delay_us(1);
    HAL_GPIO_WritePin(GPIOA, _pin, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 15);
  HAL_GPIO_WritePin(GPIOA, _pin, 0);
}

//Update LED Indicators
void LED_Indicators(bool arr[13])
{
  for(int i = 12; i != 0; i--)
  {
    HAL_GPIO_WritePin(GPIOA, Ser_Data, arr[i]);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, Ser_Clock, 1);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, Ser_Clock, 1);
  }
  delay_us(1);
  HAL_GPIO_WritePin(GPIOA, Ser_Latch, 1);
  delay_us(1);
  HAL_GPIO_WritePin(GPIOA, Ser_Latch, 1);
}

//Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_15)
  {
    //Read Pulse and process times
    pulse_time1 = HAL_GetTick();
    period = pulse_time1 - period_time;
    period_time = pulse_time1;
    while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15));
    pulse_time2 = HAL_GetTick();
    pulse = pulse_time2 - pulse_time1;

    // Use data gathered //

    //New Minute start
    if(period > 1500)
    {
      ok = true;
      bitno = 0;
    }
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
