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
 * ---------------------------------------------------------------------
 * Output switches -- comment a line out to silence that part of the log.
 * ---------------------------------------------------------------------
 * Both are independent. Turning APP_SHOW_RAW_SENTENCES off is the usual
 * choice once the link works, because the raw dump is what drowns the
 * decoded status line. Turning APP_SHOW_DECODED_STATUS off gets you the
 * previous behaviour back.
 */

/** Dump every received NMEA sentence verbatim ("GNSS> $GPGGA,..."). */
#define APP_SHOW_RAW_SENTENCES

/** Print the decoded fix/satellites/position line once per second. */
#define APP_SHOW_DECODED_STATUS

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Timestamp of the last heartbeat line */
static uint32_t last_heartbeat_time = 0;

/* Navigation state accumulated from the incoming sentences */
static NMEA_Data nmea_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN PFP */

static void GNSS_OnSentence(const char *sentence);

#ifdef APP_SHOW_DECODED_STATUS
static void App_PrintCoordinate(int32_t micro_degrees, char positive, char negative);
static void App_PrintDecodedStatus(void);
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
#ifdef APP_SHOW_RAW_SENTENCES
    SEGGER_RTT_WriteString(0, "GNSS> ");
    SEGGER_RTT_WriteString(0, sentence);
    SEGGER_RTT_WriteString(0, "\r\n");
#endif

#ifdef APP_SHOW_DECODED_STATUS
    NMEA_Parse(&nmea_data, sentence);
#endif

#if !defined(APP_SHOW_RAW_SENTENCES) && !defined(APP_SHOW_DECODED_STATUS)
    (void)sentence;
#endif
}

#ifdef APP_SHOW_DECODED_STATUS

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

/**
 * @brief Prints the decoded navigation state as one line.
 *
 * Position and altitude are only shown once a fix actually exists --
 * printing stale zeros would look like a valid position at Null Island.
 */
static void App_PrintDecodedStatus(void)
{
    const char *fix_text;

    switch (nmea_data.fix_type)
    {
        case NMEA_FIX_2D: fix_text = "2D";   break;
        case NMEA_FIX_3D: fix_text = "3D";   break;
        default:          fix_text = "none"; break;
    }

    SEGGER_RTT_printf(
        0,
        "[GNSS] fix=%s | sats=%u/%u | hdop=%u.%03u",
        fix_text,
        (unsigned)nmea_data.satellites_used,
        (unsigned)nmea_data.satellites_visible,
        (unsigned)(nmea_data.hdop_milli / 1000u),
        (unsigned)(nmea_data.hdop_milli % 1000u)
    );

    if ((nmea_data.position_valid != 0u) && (nmea_data.fix_type >= NMEA_FIX_2D))
    {
        SEGGER_RTT_WriteString(0, " | ");
        App_PrintCoordinate(nmea_data.latitude_udeg, 'N', 'S');

        SEGGER_RTT_WriteString(0, " ");
        App_PrintCoordinate(nmea_data.longitude_udeg, 'E', 'W');

        SEGGER_RTT_printf(
            0,
            " | alt=%d.%u m",
            (int)(nmea_data.altitude_dm / 10),
            (unsigned)((nmea_data.altitude_dm < 0 ?
                        -nmea_data.altitude_dm : nmea_data.altitude_dm) % 10)
        );
    }
    else
    {
        SEGGER_RTT_WriteString(0, " | no position yet");
    }

    SEGGER_RTT_printf(
        0,
        " | %02u:%02u:%02u UTC%s\r\n",
        (unsigned)nmea_data.utc_hour,
        (unsigned)nmea_data.utc_minute,
        (unsigned)nmea_data.utc_second,
        (nmea_data.data_valid != 0u) ? "" : " (not valid)"
    );

    if (nmea_data.checksum_errors != 0u)
    {
        SEGGER_RTT_printf(
            0,
            "[GNSS] checksum errors: %u\r\n",
            (unsigned)nmea_data.checksum_errors
        );
    }
}

#endif /* APP_SHOW_DECODED_STATUS */

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

  last_heartbeat_time = HAL_GetTick();

  NMEA_Reset(&nmea_data);

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

    /* Heartbeat, once per second. */
    uint32_t now = HAL_GetTick();

    if ((now - last_heartbeat_time) >= 1000u)
    {
        GNSS_Status status;

        last_heartbeat_time = now;
        GNSS_GetStatus(&status);

        SEGGER_RTT_printf(
            0,
            "[LIVE %u ms] bytes=%u | nmea=%u | uart-err=%u | overflow=%u",
            (unsigned)now,
            (unsigned)status.bytes_received,
            (unsigned)status.sentences_received,
            (unsigned)status.uart_errors,
            (unsigned)status.ring_overflows
        );

        if ((now - status.last_byte_tick) >= 3000u)
        {
            /* Nothing at all on the wire for three seconds. */
            SEGGER_RTT_WriteString(0, " | GNSS: NO DATA");
        }
        else if ((now - status.last_sentence_tick) >= 3000u)
        {
            /*
             * Bytes arrive but never parse. Usually one of:
             * wrong baud rate, a binary protocol, or a UART misconfig.
             * The raw byte is the most useful clue here.
             */
            SEGGER_RTT_printf(
                0,
                " | GNSS: bytes but no NMEA | last=0x%02X",
                (unsigned)status.last_byte
            );
        }
        else
        {
            SEGGER_RTT_WriteString(0, " | GNSS: OK");
        }

        SEGGER_RTT_WriteString(0, "\r\n");

#ifdef APP_SHOW_DECODED_STATUS
        App_PrintDecodedStatus();
#endif
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
