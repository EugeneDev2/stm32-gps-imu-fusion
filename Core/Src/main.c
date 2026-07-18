/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 6D Orientation Example for Nucleo-446RE
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed under the terms found in the LICENSE file
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
#include "accel.h"
#include "gps.h"
#include "geo.h"
#include "kalman.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
UART_TxQueueTypeDef uart2_tx_queue = { .head = 0, .tail = 0, .is_transmitting = 0 };

/* останній відомий стан GPS для CSV: між fix-ами значення повторюються */
static uint8_t geo_origin_set = 0;
static uint8_t gps_fix = 0;
static float geo_east = 0.0f, geo_north = 0.0f, gps_spd = 0.0f;

/* Кальман: стартує разом з origin (перший fix = точка (0,0) ENU) */
static KalmanFilter kf;
static uint8_t kf_started = 0;
static uint32_t kf_last_tick = 0;
static uint32_t kf_last_fix_tick = 0;
static uint8_t kf_gate = 1; /* результат останнього KF_UpdateGated (1=accepted) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_TxQueue_Init(UART_TxQueueTypeDef *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->is_transmitting = 0;
}

uint8_t UART_TxQueue_IsFull(UART_TxQueueTypeDef *queue) {
    return ((queue->head + 1) % UART_TX_QUEUE_SIZE) == queue->tail;
}

uint8_t UART_TxQueue_IsEmpty(UART_TxQueueTypeDef *queue) {
    return queue->head == queue->tail;
}

void UART_TxQueue_Enqueue(UART_TxQueueTypeDef *queue, const char *data) {
    if (!UART_TxQueue_IsFull(queue)) {
        strncpy(queue->buffer[queue->head], data, UART_TX_BUFFER_SIZE - 1);
        queue->buffer[queue->head][UART_TX_BUFFER_SIZE - 1] = '\0';
        queue->head = (queue->head + 1) % UART_TX_QUEUE_SIZE;
    }
}

void UART_TxQueue_TransmitNext(UART_HandleTypeDef *huart, UART_TxQueueTypeDef *queue) {
    if (!queue->is_transmitting && !UART_TxQueue_IsEmpty(queue)) {
        queue->is_transmitting = 1;
        HAL_UART_Transmit_IT(huart, (uint8_t*)queue->buffer[queue->tail], strlen(queue->buffer[queue->tail]));
    }
}

void Safe_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size) {
    char temp[UART_TX_BUFFER_SIZE];
    strncpy(temp, (char*)data, size);
    temp[size] = '\0';
    UART_TxQueue_Enqueue(&uart2_tx_queue, temp);
    UART_TxQueue_TransmitNext(huart, &uart2_tx_queue);
}

/*
 * Розлипання I2C1-шини (PB6=SCL, PB7=SDA) перед HAL_I2C_Init:
 * якщо slave тримає SDA низько, даємо до 9 імпульсів SCL у GPIO-режимі,
 * щоб він дочитав перерваний байт і відпустив шину, потім формуємо STOP.
 * Викликати ДО HAL_I2C_Init: MspInit потім сам поверне піни в режим AF I2C.
 */
static void I2C1_BusRecover(void) {
    GPIO_InitTypeDef gpio = {0};
    const char *result;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SDA (PB7) — вхід: перевіряємо, чи шина взагалі залипла */
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
        result = "I2C bus free\r\n";
    } else {
        /* SCL (PB6) — open-drain вихід, тактуємо вручну */
        gpio.Pin = GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);

        for (int i = 0; i < 9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_Delay(1);
        }

        /* STOP condition: SDA тягнемо вниз і відпускаємо при високому SCL */
        gpio.Pin = GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        HAL_GPIO_Init(GPIOB, &gpio);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_Delay(1);

        /* SDA знову вхід — перевіряємо результат */
        gpio.Pin = GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(GPIOB, &gpio);
        result = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
                     ? "I2C bus recovered\r\n"
                     : "I2C bus stuck\r\n";
    }

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    HAL_UART_Transmit(&huart2, (uint8_t*)result, (uint16_t)strlen(result), 100);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ACCEL_Init(&hi2c1);
  GPS_Init(&huart1);

  /* даємо IT-черзі дослати стартові повідомлення, інакше блокуючий
     вивід заголовка отримає HAL_BUSY і рядок загубиться */
  uint32_t hdr_t0 = HAL_GetTick();
  while (!UART_TxQueue_IsEmpty(&uart2_tx_queue) && (HAL_GetTick() - hdr_t0) < 500) {}
  const char csv_hdr[] = "t_ms,ax,ay,az,gx,gy,gz,fix,east,north,spd,kf_east,kf_north,kf_vE,kf_vN,gate\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)csv_hdr, sizeof(csv_hdr) - 1, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (ACCEL_Read()) {
      /* Predict на кожному читанні IMU з реальним dt; орієнтації ще нема,
         тому aE=aN=0 - фільтр веде тільки модель постійної швидкості */
      if (kf_started) {
        uint32_t now = HAL_GetTick();
        float dt = (float)(now - kf_last_tick) / 1000.0f;
        kf_last_tick = now;
        if (dt > 0.0f && dt < 0.5f) {
          KF_Predict(&kf, dt, 0.0f, 0.0f);
        }
      }

      GPS_RMCData rmc;
      if (GPS_GetRMC(&rmc)) {
        gps_fix = rmc.valid ? 1 : 0;
        if (rmc.valid) {
          if (!geo_origin_set) {
            GEO_SetOrigin(rmc.lat, rmc.lon);
            geo_origin_set = 1;
            KF_Init(&kf, 0.3f, 1.5f); /* origin = (0,0) ENU = старт фільтра */
            kf_started = 1;
            kf_last_tick = HAL_GetTick();
            kf_last_fix_tick = kf_last_tick;
            char msg[64];
            int mlen = snprintf(msg, sizeof(msg), "ORIGIN SET lat=%.5f lon=%.5f\r\n", rmc.lat, rmc.lon);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, (uint16_t)mlen, 100);
          }
          GEO_ToENU(rmc.lat, rmc.lon, &geo_east, &geo_north);
          gps_spd = rmc.speed_mps;
          uint32_t fix_now = HAL_GetTick();
          float dt_fix = (float)(fix_now - kf_last_fix_tick) / 1000.0f;
          kf_last_fix_tick = fix_now;
          kf_gate = (uint8_t)KF_UpdateGated(&kf, geo_east, geo_north, dt_fix, rmc.speed_mps);
        }
      }

      char csv[160];
      int len = snprintf(csv, sizeof(csv), "%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%u,%.2f,%.2f,%.2f",
                         HAL_GetTick(), accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
                         gps_fix, geo_east, geo_north, gps_spd);
      if (kf_started) {
        float pE, pN, vE, vN;
        KF_GetPosition(&kf, &pE, &pN);
        KF_GetVelocity(&kf, &vE, &vN);
        len += snprintf(csv + len, sizeof(csv) - (size_t)len, ",%.3f,%.3f,%.3f,%.3f,%u\r\n",
                        pE, pN, vE, vN, kf_gate);
      } else {
        len += snprintf(csv + len, sizeof(csv) - (size_t)len, ",,,,,\r\n"); /* фільтр ще не стартував */
      }
      HAL_UART_Transmit(&huart2, (uint8_t*)csv, (uint16_t)len, 100);
    }
    HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  I2C1_BusRecover(); /* розлипання шини до HAL_I2C_Init (USART2 вже готовий для логу) */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  GPS_UART_RxEventCallback(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  GPS_UART_ErrorCallback(huart);
}

/* Просуває чергу uart2_tx_queue після завершення IT-передачі —
   без цього колбека is_transmitting ніколи не скидається і черга стає */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uart2_tx_queue.tail = (uart2_tx_queue.tail + 1) % UART_TX_QUEUE_SIZE;
    uart2_tx_queue.is_transmitting = 0;
    UART_TxQueue_TransmitNext(huart, &uart2_tx_queue);
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
