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

// Pin Defines
#define CS_Inner GPIO_PIN_0
#define CS_Outer GPIO_PIN_1
#define CS_Date GPIO_PIN_3
#define CS_Time GPIO_PIN_2
#define CS_Pulse GPIO_PIN_4
#define Ser_Data GPIO_PIN_8
#define Ser_Clock GPIO_PIN_9
#define Ser_Latch GPIO_PIN_10
#define RTC_Lock GPIO_PIN_15
#define PON GPIO_PIN_14
#define USB_Connect GPIO_PIN_8

// MAX7221 Defines
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

// timestamp variables
uint32_t pulse_time1, pulse_time2, period_time, temp;

// indicator array:
// CEST-P1-P2-P3-PErr-Mo-Di-Mi-Do-Fr-Sa-So-CET
bool leds[13];

// Minute Telegrams
uint8_t bitno;
bool minute[60];

// time data variable
uint8_t hour_tens, hour_ones,
    minute_tens, minute_ones,
    second_tens, second_ones,
    year_tens, year_ones,
    month_tens, month_ones,
    day_tens, day_ones,
    week,
    bitval, digs, bits;
uint16_t period, pulse;

// SPI
uint8_t data_buf[2];

bool pulse_ok = false, period_ok = false, perr = true;
uint32_t minstart = 0;

uint8_t t, h1, h2, z1, z2, e1, e2;
uint8_t out[8];

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
  timer_init();

  // Module Init and test routines
  IO_Init();

  RTC_Read();

  pulse_time1 = HAL_GetTick();
  pulse_time2 = pulse_time1;
  period_time = pulse_time1;
  temp = pulse_time1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    temp = HAL_GetTick();

    if (period_ok)
    {
      sekundentakt();
    }
    if (pulse_ok)
    {
      pulstakt();
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void RTC_Read(void)
{
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BCD);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BCD);

  gDate.Year    = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1); // backup register
  gDate.Month   = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2); // backup register
  gDate.Date    = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3); // backup register
  gDate.WeekDay = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4); // backup register

  hour_tens = gTime.Hours >> 4;
  hour_ones = gTime.Hours & 0x0F;
  minute_tens = gTime.Minutes >> 4;
  minute_ones = gTime.Minutes & 0x0F;
  year_tens = gDate.Year >> 4;
  year_ones = gDate.Year & 0x0F;
  month_tens = gDate.Month >> 4;
  month_ones = gDate.Month & 0x0F;
  day_tens = gDate.Date >> 4;
  day_ones = gDate.Date & 0x0F;

  MAX7221_Send(CS_Time, Dig0, hour_tens);
  MAX7221_Send(CS_Time, Dig1, hour_ones | 0x80);
  MAX7221_Send(CS_Time, Dig2, minute_tens);
  MAX7221_Send(CS_Time, Dig3, minute_ones | 0x80);

  MAX7221_Send(CS_Date, Dig0, day_tens);
  MAX7221_Send(CS_Date, Dig1, day_ones | 0x80);
  MAX7221_Send(CS_Date, Dig2, month_tens);
  MAX7221_Send(CS_Date, Dig3, month_ones | 0x80);
  MAX7221_Send(CS_Date, Dig4, year_tens);
  MAX7221_Send(CS_Date, Dig5, year_ones);
}

void RTC_Write(void)
{
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  HAL_PWR_EnableBkUpAccess();
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  sTime.Hours = (hour_tens << 4) + hour_ones;       // set hours
  sTime.Minutes = (minute_tens << 4) + minute_ones; // set minutes
  sTime.Seconds = 0x00;                             // set seconds
  sDate.WeekDay = week;                             // day
  sDate.Month = (month_tens << 4) + month_ones;     // month
  sDate.Date = (day_tens << 4) + day_ones;          // date
  sDate.Year = (year_tens << 4) + year_ones;        // year

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, sDate.Year); // backup register
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, sDate.Month); // backup register
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, sDate.Date); // backup register
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, sDate.WeekDay); // backup register
  HAL_GPIO_WritePin(GPIOA, RTC_Lock, 1);
}

void sekundentakt(void)
{
  uint32_t temp1 = pulse_time1 - period_time;
  e1 = temp1 % 10;
  z1 = ((temp1 % 100) - e1) / 10;
  h1 = ((temp1 % 1000) - z1) / 100;
  t = (temp1 - h1) / 1000;

  MAX7221_Send(CS_Pulse, Dig0, t | 0x80);
  MAX7221_Send(CS_Pulse, Dig1, h1);
  MAX7221_Send(CS_Pulse, Dig2, z1);
  MAX7221_Send(CS_Pulse, Dig3, e1);

  if (temp1 > 1500)
  {
    minstart++;
    bitno = 0;
    digs = 0;
    bits = 0;
    for (int i = 1; i < 9; i++)
    {
      MAX7221_Send(CS_Outer, i, out[i - 1]);
    }
    for (int i = 1; i < 9; i++)
    {
      MAX7221_Send(CS_Inner, i, 0x0);
    }
    analyze();
    if (((hour_ones + hour_tens) > 0) && ((hour_ones + hour_tens) < 24)) //&& ((minute_ones + minute_tens) == 0))
    {
      RTC_Write();
    }
  }

  if (minstart > 0)
  {
    MAX7221_Send(CS_Time, Dig4, (bitno - (bitno % 10)) / 10);
    MAX7221_Send(CS_Time, Dig5, bitno % 10);
  }
  else
  {
    if (bitno % 2)
    {
      MAX7221_Send(CS_Time, Dig4, 0xF);
      MAX7221_Send(CS_Time, Dig5, 0xF);
    }
    else
    {
      MAX7221_Send(CS_Time, Dig4, 0x0);
      MAX7221_Send(CS_Time, Dig5, 0x0);
    }
  }
  period_ok = false;
}

void pulstakt(void)
{
  uint32_t p = pulse_time2 - pulse_time1;
  e2 = p % 10;
  z2 = (p % 100 - e2) / 10;
  h2 = (p % 1000 - z2) / 100;

  MAX7221_Send(CS_Pulse, Dig4, h2);
  MAX7221_Send(CS_Pulse, Dig5, z2);
  MAX7221_Send(CS_Pulse, Dig6, e2);

  if ((pulse_time2 - pulse_time1) > 120)
  {
    MAX7221_Send(CS_Pulse, Dig7, 1);
    bitval = 1;
  }
  else
  {
    MAX7221_Send(CS_Pulse, Dig7, 0);
    bitval = 0;
  }
  pulse_ok = false;

  if (minstart > 0)
  {
    switch (bitno % 8)
    {
    case 0:
      bits |= bitval << 6;
      break;
    case 1:
      bits |= bitval << 5;
      break;
    case 2:
      bits |= bitval << 4;
      break;
    case 3:
      bits |= bitval << 3;
      break;
    case 4:
      bits |= bitval << 2;
      break;
    case 5:
      bits |= bitval << 1;
      break;
    case 6:
      bits |= bitval << 0;
      break;
    case 7:
      bits |= bitval << 7;
      break;
    }
    digs = (bitno - (bitno % 8)) / 8;
    minute[bitno] = bitval;

    out[digs] = bits;

    MAX7221_Send(CS_Inner, digs + 1, bits);
  }
  bitno++;
  if (!(bitno % 8))
  {
    bits = 0;
  }
}

void analyze(void)
{
  minute_ones = minute[21] +
                minute[22] * 2 +
                minute[23] * 4 +
                minute[24] * 8;
  minute_tens = minute[25] +
                minute[26] * 2 +
                minute[27] * 4;

  hour_ones = minute[29] +
              minute[30] * 2 +
              minute[31] * 4 +
              minute[32] * 8;
  hour_tens = minute[33] +
              minute[34] * 2;

  day_ones = minute[36] +
             minute[37] * 2 +
             minute[38] * 4 +
             minute[39] * 8;
  day_tens = minute[40] +
             minute[41] * 2;

  month_ones = minute[45] +
               minute[46] * 2 +
               minute[47] * 4 +
               minute[48] * 8;
  month_tens = minute[49];

  year_ones = minute[50] +
              minute[51] * 2 +
              minute[52] * 4 +
              minute[53] * 8;
  year_tens = minute[54] +
              minute[55] * 2 +
              minute[56] * 4 +
              minute[57] * 8;

  week = minute[42] +
         minute[43] * 2 +
         minute[44] * 4;

  bool mo = false, di = false, mi = false, d = false, fr = false, sa = false, so = false;
  switch (week)
  {
  case 0:
    mo = true;
    di = false;
    mi = false;
    d = false;
    fr = false;
    sa = false;
    so = false;
    break;
  case 1:
    mo = false;
    di = true;
    mi = false;
    d = false;
    fr = false;
    sa = false;
    so = false;
    break;
  case 2:
    mo = false;
    di = false;
    mi = true;
    d = false;
    fr = false;
    sa = false;
    so = false;
    break;
  case 3:
    mo = false;
    di = false;
    mi = false;
    d = true;
    fr = false;
    sa = false;
    so = false;
    break;
  case 4:
    mo = false;
    di = false;
    mi = false;
    d = false;
    fr = true;
    sa = false;
    so = false;
    break;
  case 5:
    mo = false;
    di = false;
    mi = false;
    d = false;
    fr = false;
    sa = true;
    so = false;
    break;
  case 6:
    mo = false;
    di = false;
    mi = false;
    d = false;
    fr = false;
    sa = false;
    so = true;
    break;
  }

  uint8_t parity = 0;
  for (int i = 21; i < 59; i++)
  {
    parity += minute[i];
  }
  if (parity % 2 == 0)
  {
    perr = false;
  }
  else
  {
    perr = true;
  }
  
  if (!perr && (minstart > 1))
  {
    MAX7221_Send(CS_Time, Dig0, hour_tens);
    MAX7221_Send(CS_Time, Dig1, hour_ones | 0x80);
    MAX7221_Send(CS_Time, Dig2, minute_tens);
    MAX7221_Send(CS_Time, Dig3, minute_ones | 0x80);

    MAX7221_Send(CS_Date, Dig0, day_tens);
    MAX7221_Send(CS_Date, Dig1, day_ones | 0x80);
    MAX7221_Send(CS_Date, Dig2, month_tens);
    MAX7221_Send(CS_Date, Dig3, month_ones | 0x80);
    MAX7221_Send(CS_Date, Dig4, year_tens);
    MAX7221_Send(CS_Date, Dig5, year_ones);
  }
  else
  {
    RTC_Read();
  }

  leds[12] = minute[17];
  leds[11] = minute[28];
  leds[10] = minute[35];
  leds[9] = minute[58];
  leds[8] = perr;
  leds[7] = mo;
  leds[6] = di;
  leds[5] = mi;
  leds[4] = d;
  leds[3] = fr;
  leds[2] = sa;
  leds[1] = so;
  leds[0] = minute[18];

  LED_Indicators(leds);
}

void IO_Init(void)
{
  HAL_GPIO_WritePin(GPIOB, PON, 1);
  HAL_GPIO_WritePin(GPIOB, PON, 0);
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

  // Inner Ring Init
  MAX7221_Send(CS_Inner, Shutdown, 0x01);
  MAX7221_Send(CS_Inner, ScanLimit, 0x07);
  MAX7221_Send(CS_Inner, Dig0, 0xff);
  MAX7221_Send(CS_Inner, Dig1, 0xff);
  MAX7221_Send(CS_Inner, Dig2, 0xff);
  MAX7221_Send(CS_Inner, Dig3, 0xff);
  MAX7221_Send(CS_Inner, Dig4, 0xff);
  MAX7221_Send(CS_Inner, Dig5, 0xff);
  MAX7221_Send(CS_Inner, Dig6, 0xff);
  MAX7221_Send(CS_Inner, Dig7, 0xff);

  // Outer Ring Init
  MAX7221_Send(CS_Outer, Shutdown, 0x01);
  MAX7221_Send(CS_Outer, ScanLimit, 0x07);
  MAX7221_Send(CS_Outer, Dig0, 0xff);
  MAX7221_Send(CS_Outer, Dig1, 0xff);
  MAX7221_Send(CS_Outer, Dig2, 0xff);
  MAX7221_Send(CS_Outer, Dig3, 0xff);
  MAX7221_Send(CS_Outer, Dig4, 0xff);
  MAX7221_Send(CS_Outer, Dig5, 0xff);
  MAX7221_Send(CS_Outer, Dig6, 0xff);
  MAX7221_Send(CS_Outer, Dig7, 0xff);

  // Date Init
  MAX7221_Send(CS_Date, Shutdown, 0x01);
  MAX7221_Send(CS_Date, ScanLimit, 0x05);
  MAX7221_Send(CS_Date, DecodeMode, 0xFF);
  MAX7221_Send(CS_Date, Dig0, 0x00);
  MAX7221_Send(CS_Date, Dig1, 0x80);
  MAX7221_Send(CS_Date, Dig2, 0x00);
  MAX7221_Send(CS_Date, Dig3, 0x80);
  MAX7221_Send(CS_Date, Dig4, 0x00);
  MAX7221_Send(CS_Date, Dig5, 0x00);

  // Time Init
  MAX7221_Send(CS_Time, Shutdown, 0x01);
  MAX7221_Send(CS_Time, ScanLimit, 0x05);
  MAX7221_Send(CS_Time, DecodeMode, 0xFF);
  MAX7221_Send(CS_Time, Dig0, 0x0);
  MAX7221_Send(CS_Time, Dig1, 0x80);
  MAX7221_Send(CS_Time, Dig2, 0x0);
  MAX7221_Send(CS_Time, Dig3, 0x80);
  MAX7221_Send(CS_Time, Dig4, 0x0);
  MAX7221_Send(CS_Time, Dig5, 0x0);

  // Pulse Init
  MAX7221_Send(CS_Pulse, Shutdown, 0x01);
  MAX7221_Send(CS_Pulse, ScanLimit, 0x07);
  MAX7221_Send(CS_Pulse, DecodeMode, 0xFF);
  MAX7221_Send(CS_Pulse, Dig0, 0x80);
  MAX7221_Send(CS_Pulse, Dig1, 0x0);
  MAX7221_Send(CS_Pulse, Dig2, 0x0);
  MAX7221_Send(CS_Pulse, Dig3, 0x0);
  MAX7221_Send(CS_Pulse, Dig4, 0x0);
  MAX7221_Send(CS_Pulse, Dig5, 0x0);
  MAX7221_Send(CS_Pulse, Dig6, 0x0);
  MAX7221_Send(CS_Pulse, Dig7, 0x0);

  for (int i = 0; i < 13; i++)
    leds[i] = 0;
  LED_Indicators(leds);
}

void MAX7221_Send(uint16_t _pin, uint8_t _reg, uint8_t _data)
{
  data_buf[0] = _reg;
  data_buf[1] = _data;
  delay_us(1);
  HAL_GPIO_WritePin(GPIOA, _pin, 1);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data_buf, 2, 100);
  HAL_GPIO_WritePin(GPIOA, _pin, 0);
  delay_us(1);
}

// Update LED Indicators
void LED_Indicators(bool arr[13])
{
  for (int i = 13; i != 0; i--)
  {
    HAL_GPIO_WritePin(GPIOA, Ser_Data, arr[i - 1]);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, Ser_Clock, 1);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, Ser_Clock, 0);
  }
  delay_us(1);
  HAL_GPIO_WritePin(GPIOA, Ser_Latch, 1);
  delay_us(1);
  HAL_GPIO_WritePin(GPIOA, Ser_Latch, 0);
}

// Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_15)
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == 0)
    {
      period_time = pulse_time1;
      pulse_time1 = temp;
      period_ok = true;
    }
    else
    {
      pulse_time2 = temp;
      pulse_ok = true;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
