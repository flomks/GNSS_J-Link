/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : GNSS test with SEGGER J-Link RTT
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SEGGER_RTT.h"
#include "gnss.h"
#include "nmea.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * Select exactly one GNSS output mode here:
 *   APP_GNSS_OUTPUT_PARSED  readable status once per second
 *   APP_GNSS_OUTPUT_RAW     every NMEA sentence as received
 */
#define APP_GNSS_OUTPUT_RAW     1u
#define APP_GNSS_OUTPUT_PARSED  2u

#ifndef APP_GNSS_OUTPUT_MODE
#define APP_GNSS_OUTPUT_MODE APP_GNSS_OUTPUT_PARSED
#endif

#if ((APP_GNSS_OUTPUT_MODE != APP_GNSS_OUTPUT_RAW) && \
     (APP_GNSS_OUTPUT_MODE != APP_GNSS_OUTPUT_PARSED))
#error "APP_GNSS_OUTPUT_MODE must be RAW or PARSED"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED)
/* Timestamp of the last heartbeat line. */
static uint32_t last_heartbeat_time = 0;

/* Navigation state accumulated from the incoming sentences. */
static NMEA_Data nmea_data;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN PFP */

static void GNSS_OnSentence(const char *sentence);

#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED)
static void App_PrintCoordinate(int32_t micro_degrees, char positive, char negative);
static void App_PrintAltitude(int32_t decimetres);
static void App_PrintDecodedStatus(const GNSS_Status *status, uint32_t now);
#endif

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Prints one decoded NMEA sentence.
 *
 * Invoked from GNSS_Poll(), i.e. from the main loop, never from an
 * interrupt -- so a slow RTT write here cannot cost received bytes.
 */
static void GNSS_OnSentence(const char *sentence)
{
#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_RAW)
    SEGGER_RTT_WriteString(0, sentence);
    SEGGER_RTT_WriteString(0, "\r\n");
#else
    NMEA_Parse(&nmea_data, sentence);
#endif
}

#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED)

/**
 * @brief Prints micro-degrees as "48.117300 N".
 *
 * SEGGER's printf has no %f, so the value is split into whole degrees and
 * a six-digit fraction. The zero padding is what keeps 48.007° from being
 * printed as 48.7°.
 */
static void App_PrintCoordinate(int32_t micro_degrees, char positive, char negative)
{
    uint32_t magnitude;
    char hemisphere;

    if (micro_degrees < 0)
    {
        magnitude = (uint32_t)(-micro_degrees);
        hemisphere = negative;
    }
    else
    {
        magnitude = (uint32_t)micro_degrees;
        hemisphere = positive;
    }

    SEGGER_RTT_printf(
        0,
        "%u.%06u %c",
        (unsigned)(magnitude / 1000000u),
        (unsigned)(magnitude % 1000000u),
        hemisphere
    );
}

/** Prints a signed decimetre value without relying on printf float support. */
static void App_PrintAltitude(int32_t decimetres)
{
    uint32_t magnitude;

    if (decimetres < 0)
    {
        SEGGER_RTT_WriteString(0, "-");
        magnitude = (uint32_t)(-(decimetres + 1)) + 1u;
    }
    else
    {
        magnitude = (uint32_t)decimetres;
    }

    SEGGER_RTT_printf(
        0, "%u.%u m",
        (unsigned)(magnitude / 10u),
        (unsigned)(magnitude % 10u)
    );
}

/**
 * @brief Prints receiver diagnostics and decoded navigation data as a table.
 */
static void App_PrintDecodedStatus(const GNSS_Status *status, uint32_t now)
{
    const char *fix_text;
    const char *quality_text;
    const char *nmea_text;

    if (nmea_data.position_valid == 0u)
    {
        fix_text = "none";
    }
    else
    {
        switch (nmea_data.fix_type)
        {
            case NMEA_FIX_2D: fix_text = "2D";    break;
            case NMEA_FIX_3D: fix_text = "3D";    break;
            default:          fix_text = "valid"; break;
        }
    }

    switch (nmea_data.fix_quality)
    {
        case 1u: quality_text = "GPS";       break;
        case 2u: quality_text = "DGPS";      break;
        case 4u: quality_text = "RTK fixed"; break;
        case 5u: quality_text = "RTK float"; break;
        case 6u: quality_text = "estimated"; break;
        default: quality_text = "invalid";   break;
    }

    if ((now - status->last_byte_tick) >= 3000u)
    {
        nmea_text = "KEINE DATEN";
    }
    else if ((now - status->last_sentence_tick) >= 3000u)
    {
        nmea_text = "Bytes, aber keine Saetze";
    }
    else
    {
        nmea_text = "OK";
    }

    SEGGER_RTT_printf(
        0,
        "\r\n--------------------------------------------------\r\n"
        "[LIVE] STM32 laeuft | %u s\r\n"
        "UART        : %u Baud | Fehler: %u | Overflow: %u\r\n"
        "NMEA        : %s | Saetze: %u | CRC-Fehler: %u\r\n",
        (unsigned)(now / 1000u),
        (unsigned)huart1.Init.BaudRate,
        (unsigned)status->uart_errors,
        (unsigned)status->ring_overflows,
        nmea_text,
        (unsigned)status->sentences_received,
        (unsigned)nmea_data.checksum_errors
    );

    if (nmea_data.position_valid != 0u)
    {
        SEGGER_RTT_printf(0, "Fix         : JA (%s, %s)\r\n", fix_text, quality_text);
    }
    else
    {
        SEGGER_RTT_WriteString(0, "Fix         : NEIN\r\n");
    }

    SEGGER_RTT_printf(
        0,
        "Satelliten  : %u genutzt | %u sichtbar\r\n",
        (unsigned)nmea_data.satellites_used,
        (unsigned)nmea_data.satellites_visible
    );

    SEGGER_RTT_WriteString(0, "Position     : ");
    if (nmea_data.position_valid != 0u)
    {
        App_PrintCoordinate(nmea_data.latitude_udeg, 'N', 'S');
        SEGGER_RTT_WriteString(0, ", ");
        App_PrintCoordinate(nmea_data.longitude_udeg, 'E', 'W');
        SEGGER_RTT_WriteString(0, "\r\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "noch nicht verfuegbar\r\n");
    }

    SEGGER_RTT_WriteString(0, "Hoehe        : ");
    if (nmea_data.altitude_valid != 0u)
    {
        App_PrintAltitude(nmea_data.altitude_dm);
        SEGGER_RTT_WriteString(0, "\r\n");
    }
    else
    {
        SEGGER_RTT_WriteString(0, "nicht verfuegbar\r\n");
    }

    SEGGER_RTT_printf(
        0, "HDOP         : %u.%02u\r\n",
        (unsigned)(nmea_data.hdop_milli / 1000u),
        (unsigned)((nmea_data.hdop_milli % 1000u) / 10u)
    );

    SEGGER_RTT_WriteString(0, "Geschw.      : ");
    if (nmea_data.speed_valid != 0u)
    {
        SEGGER_RTT_printf(
            0, "%u.%02u km/h\r\n",
            (unsigned)(nmea_data.speed_kmh_milli / 1000u),
            (unsigned)((nmea_data.speed_kmh_milli % 1000u) / 10u)
        );
    }
    else
    {
        SEGGER_RTT_WriteString(0, "nicht verfuegbar\r\n");
    }

    SEGGER_RTT_WriteString(0, "Kurs         : ");
    if (nmea_data.course_valid != 0u)
    {
        SEGGER_RTT_printf(
            0, "%u.%02u Grad\r\n",
            (unsigned)(nmea_data.course_mdeg / 1000u),
            (unsigned)((nmea_data.course_mdeg % 1000u) / 10u)
        );
    }
    else
    {
        SEGGER_RTT_WriteString(0, "nicht verfuegbar\r\n");
    }

    SEGGER_RTT_WriteString(0, "UTC          : ");
    if (nmea_data.time_valid != 0u)
    {
        SEGGER_RTT_printf(
            0, "%02u:%02u:%02u UTC\r\n",
            (unsigned)nmea_data.utc_hour,
            (unsigned)nmea_data.utc_minute,
            (unsigned)nmea_data.utc_second
        );
    }
    else
    {
        SEGGER_RTT_WriteString(0, "--:--:-- UTC\r\n");
    }

    SEGGER_RTT_WriteString(0, "Datum        : ");
    if (nmea_data.date_valid != 0u)
    {
        SEGGER_RTT_printf(
            0, "%02u.%02u.%04u\r\n",
            (unsigned)nmea_data.utc_day,
            (unsigned)nmea_data.utc_month,
            (unsigned)nmea_data.utc_year
        );
    }
    else
    {
        SEGGER_RTT_WriteString(0, "--.--.----\r\n");
    }

    SEGGER_RTT_WriteString(0, "--------------------------------------------------\r\n");
}

#endif /* APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Bring up the RTT control block before anything is logged. */
  SEGGER_RTT_Init();

  SEGGER_RTT_WriteString(
      0,
      "\r\n"
      "========================================\r\n"
      " STM32H743 GNSS Test\r\n"
      "========================================\r\n"
  );

  /*
   * The baud rate is read back from the HAL configuration rather than
   * hard-coded here, so a mismatch between CubeMX and the generated
   * code shows up immediately in the log.
   */
  SEGGER_RTT_printf(
      0,
      "USART1: %u baud, 8N1\r\n",
      (unsigned)huart1.Init.BaudRate
  );

  SEGGER_RTT_WriteString(
      0,
      "GNSS RX : PB15 / USART1_RX\r\n"
      "GNSS TX : PB14 / USART1_TX\r\n"
      "Debug   : SEGGER J-Link RTT\r\n"
      "\r\n"
      "Waiting for GNSS data...\r\n"
      "\r\n"
  );

#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED)
  last_heartbeat_time = HAL_GetTick();
  NMEA_Reset(&nmea_data);
#endif

  /*
   * Arm reception. From here on the USART1 interrupt fills the receive
   * ring and the main loop only drains it via GNSS_Poll().
   */
  if (GNSS_Init(&huart1, GNSS_OnSentence) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /*
     * Hand over everything the interrupt collected since the last pass.
     * Reception keeps running in the background meanwhile.
     */
    GNSS_Poll();

#if (APP_GNSS_OUTPUT_MODE == APP_GNSS_OUTPUT_PARSED)
    /* Receiver diagnostics and decoded state, once per second. */
    uint32_t now = HAL_GetTick();

    if ((now - last_heartbeat_time) >= 1000u)
    {
        GNSS_Status status;

        last_heartbeat_time = now;
        GNSS_GetStatus(&status);

        App_PrintDecodedStatus(&status, now);
    }
#endif

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2 |
      RCC_CLOCKTYPE_D3PCLK1 |
      RCC_CLOCKTYPE_D1PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(
      &RCC_ClkInitStruct,
      FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/* MPU Configuration */

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  SEGGER_RTT_WriteString(
      0,
      "\r\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
      "[FATAL] Error_Handler() aufgerufen!\r\n"
      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n"
  );

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

  SEGGER_RTT_printf(
      0,
      "\r\n[ASSERT] %s:%u\r\n",
      (const char *)file,
      (unsigned)line
  );

  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
