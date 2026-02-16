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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "matrix.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "sht40.h"
#include "battery.h"
#include "screens.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_ROWS 7
#define NUM_COLS 84
#define TOTAL_SR_BITS 88 // 11 chips * 8 bits

extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== SCREEN RENDER FUNCTIONS ====================
 *
 * Each screen is a function with signature:
 *   void my_screen(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
 *
 * It receives a zeroed buffer and draws into it using the _Buf functions.
 * The screen manager calls this each frame, so content can be dynamic.
 * Sensor data is read in the main loop and stored in globals so the
 * screen functions can access it without doing I2C reads themselves.
 */

// Shared sensor data (updated in main loop, read by screen functions)
static uint8_t g_hours, g_minutes, g_seconds;
static int g_temp_f;
static int g_soc;
static int g_current_mA;
static int g_humidity;
static int g_voltage;
static int g_light;

// Charger status from BQ25120 STAT1/STAT2 pins
// STAT1=HIGH, STAT2=HIGH -> Charge complete / sleep / disabled (VBAT > VRCH)
// STAT1=HIGH, STAT2=LOW  -> Normal charging in progress (incl. auto recharge)
// STAT1=LOW,  STAT2=HIGH -> Recoverable fault (VIN_OVP, TS HOT/COLD, TSHUT, short)
// STAT1=LOW,  STAT2=LOW  -> Non-recoverable / latch-off fault (ILIM/ISET short, BATOCP, safety timer)
typedef enum {
    CHARGER_COMPLETE_OR_DISABLED,   // STAT1=H, STAT2=H
    CHARGER_CHARGING,               // STAT1=H, STAT2=L
    CHARGER_FAULT_RECOVERABLE,      // STAT1=L, STAT2=H (OVP, TS, TSHUT, system short)
    CHARGER_FAULT_LATCHOFF          // STAT1=L, STAT2=L (ILIM/ISET short, BATOCP, safety timer)
} ChargerStatus_t;

static ChargerStatus_t g_charger_status = CHARGER_COMPLETE_OR_DISABLED;
static bool g_charger_fault = false;           // true if any fault detected
static bool g_charger_fault_recoverable = false; // true if recoverable fault
static bool g_charger_fault_latchoff = false;    // true if non-recoverable latch-off fault

/**
 * @brief Read BQ25120 STAT1/STAT2 pins and update charger status flags
 */

static void Charger_UpdateStatus(void)
{
    GPIO_PinState stat1 = HAL_GPIO_ReadPin(STAT1_GPIO_Port, STAT1_Pin);
    GPIO_PinState stat2 = HAL_GPIO_ReadPin(STAT2_GPIO_Port, STAT2_Pin);

    if (stat1 == GPIO_PIN_SET && stat2 == GPIO_PIN_SET) {
        g_charger_status = CHARGER_COMPLETE_OR_DISABLED;
        g_charger_fault = false;
        g_charger_fault_recoverable = false;
        g_charger_fault_latchoff = false;
    } else if (stat1 == GPIO_PIN_SET && stat2 == GPIO_PIN_RESET) {
        g_charger_status = CHARGER_CHARGING;
        g_charger_fault = false;
        g_charger_fault_recoverable = false;
        g_charger_fault_latchoff = false;
    } else if (stat1 == GPIO_PIN_RESET && stat2 == GPIO_PIN_SET) {
        g_charger_status = CHARGER_FAULT_RECOVERABLE;
        g_charger_fault = true;
        g_charger_fault_recoverable = true;
        g_charger_fault_latchoff = false;
    } else {
        // STAT1=LOW, STAT2=LOW
        g_charger_status = CHARGER_FAULT_LATCHOFF;
        g_charger_fault = true;
        g_charger_fault_recoverable = false;
        g_charger_fault_latchoff = true;
    }
}

static int g_scroll_offset = 0;
static uint32_t g_scroll_tick = 0;

static int scroll_screen_index = -1;

void Screen_ScrollMessage(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_ScrollText_Buf(buf, 0,
        "IEEE is a scam! Don't give them money!",
        &g_scroll_offset, 30, &g_scroll_tick);
}

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(buf, kompressor_logo);
}

void Screen_Logo2(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(buf, buy_a_wd);
}

void Screen_Time(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];
    sprintf(str, "%02d:%02d:%02d", g_hours, g_minutes, g_seconds);
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    // Show fault warning if charger has a problem
    if (g_charger_fault_latchoff) {
        sprintf(str, "CHG FAULT!");
    } else if (g_charger_fault_recoverable) {
        sprintf(str, "CHG WARN!");
    } else if (g_current_mA >= 0) {
        sprintf(str, "%2d%% %3dmA \x02", g_soc, g_current_mA);
    } else {
        sprintf(str, "%2d%% %3dmA", g_soc, g_current_mA);
    }
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];
    sprintf(str, "  %2dF   %2d\x01%%", g_temp_f, g_humidity);
    Matrix_DrawText_Buf(buf, 0, 0, str);
}

//void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
//{
//    char str1[32];
//    sprintf(str1, "%02d:%02d:%02d", g_hours, g_minutes, g_seconds);
//    Matrix_DrawText_Buf(buf, 0, 0, str1);
//
//    char str2[32];
//    sprintf(str2, "%2d\x03 %2d\x01", g_temp_f, g_humidity);
//    Matrix_DrawTextRight_Buf(buf, 0, str2);
//}

void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str1[32];
    sprintf(str1, "%02d:%02d:%02d", g_hours, g_minutes, g_seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    char str2[32];
    sprintf(str2, "%2d%%", g_light);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Matrix_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  if (RV3032_Init(&hi2c1)) {
      // Set initial time
      //RV3032_SetTime(40, 49, 2, 5, 13, 2, 2026);  // sec, min, hr, weekday, date, month, year
  }

  if (SHT40_Init(&hi2c1)) {
      // Sensor initialized successfully
  }

  if (LTR_329_Init(&hi2c1)) {
      // Light sensor initialized successfully
  }

  BATTERY_Init();

  // Initialize screen manager and register screens
  Screen_Init();
//  scroll_screen_index = Screen_Register(Screen_ScrollMessage);
//  Screen_Register(Screen_Logo);
//  Screen_Register(Screen_Logo2);
//  Screen_Register(Screen_Time);
//  Screen_Register(Screen_TimeTempHumid);
//  Screen_Register(Screen_Battery);
//  Screen_Register(Screen_TempHumid);
  Screen_Register(Screen_TimeLight);

  Screen_SetAutoCycle(true);
  Screen_SetAutoCycleTransition(TRANSITION_DISSOLVE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  Matrix_Fill();
//  HAL_Delay(10000);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      static uint32_t lastLightUpdate = 0;
      static uint32_t lastLightUpdate2 = 0;
      static uint32_t lastTempUpdate = 0;
      static uint32_t lastTimeUpdate = 0;
      static uint32_t lastBatteryUpdate = 0;
      static uint32_t lastChargerCheck = 0;

      uint32_t now = HAL_GetTick();

      // Update time every 200ms
      if (now - lastTimeUpdate >= 200) {
          if (RV3032_UpdateTime()) {
              uint8_t h = RV3032_GetHours();
              uint8_t m = RV3032_GetMinutes();
              uint8_t s = RV3032_GetSeconds();

              if (h != g_hours || m != g_minutes || s != g_seconds) {
                  g_hours = h;
                  g_minutes = m;
                  g_seconds = s;
                  Screen_MarkDirty();
              }
          }
          lastTimeUpdate = now;
      }

      // Update light reading every 100ms — brightness via timer period (original approach)
      if (now - lastLightUpdate >= 100) {
          LTR_329_UpdateReadings();
          lastLightUpdate = now;

          uint16_t new_period = LTR_329_GetTimerPeriod();
          __HAL_TIM_SET_AUTORELOAD(&htim3, new_period);

          g_light = LTR_329_GetLightPercent();
      }

      // Update light reading every 77ms
      if (now - lastLightUpdate2 >= 77) {
          lastLightUpdate2 = now;

          g_light = LTR_329_GetLightPercent();
      }


      // Update temperature every 3 seconds
      if (now - lastTempUpdate > 300) {
          SHT40_UpdateReadings();
          int new_temp = (int)(SHT40_GetTemperatureF());
          int new_humid = SHT40_GetHumidity();

          if (new_temp != g_temp_f || new_humid != g_humidity) {
              g_temp_f = new_temp;
              g_humidity = new_humid;
              Screen_MarkDirty();
          }
          lastTempUpdate = now;
      }

      // Update battery every 1 second
      if (now - lastBatteryUpdate >= 500) {
          BATTERY_UpdateState();
          int new_soc = BATTERY_GetSOC();
          int new_mA = BATTERY_GetCurrent();
          int new_v = BATTERY_GetVoltage();

          if (new_soc != g_soc || new_mA != g_current_mA) {
              g_soc = new_soc;
              g_current_mA = new_mA;
              g_voltage = new_v;
              Screen_MarkDirty();
          }
          lastBatteryUpdate = now;
      }

      // Check charger STAT pins every 500ms
      if (now - lastChargerCheck >= 500) {
          ChargerStatus_t prev_status = g_charger_status;
          Charger_UpdateStatus();

          // Redraw if charger status changed (especially for fault display)
          if (g_charger_status != prev_status) {
              Screen_MarkDirty();
          }
          lastChargerCheck = now;
      }

//      char str[32];
//      sprintf(str, "            \x02");
//      Matrix_DrawText_Buf(buf, 0, 0, str);

      // Drive screen state machine
      Screen_Update();

      if (Screen_GetCurrent() == scroll_screen_index) {
          Screen_MarkDirty();
      }

      HAL_Delay(5);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 259;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_NVIC_SetPriority(TIM3_IRQn,0,0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RCLK_Pin|SRCLK_Pin|DATA_Pin|A1_Pin
                          |A2_Pin|A3_Pin|A4_Pin|A5_Pin
                          |A6_Pin|A7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPOUT_GPIO_Port, GPOUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RCLK_Pin SRCLK_Pin DATA_Pin */
  GPIO_InitStruct.Pin = RCLK_Pin|SRCLK_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin A2_Pin A3_Pin A4_Pin
                           A5_Pin A6_Pin A7_Pin */
  GPIO_InitStruct.Pin = A1_Pin|A2_Pin|A3_Pin|A4_Pin
                          |A5_Pin|A6_Pin|A7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_B_Pin STAT1_Pin STAT2_Pin */
  GPIO_InitStruct.Pin = ROT_B_Pin|STAT1_Pin|STAT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROT_A_Pin ROT_SW_Pin */
  GPIO_InitStruct.Pin = ROT_A_Pin|ROT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPOUT_Pin */
  GPIO_InitStruct.Pin = GPOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPOUT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
