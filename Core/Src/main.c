/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU9250 I2C diagnostic and attitude demo.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "MPUXX50.h"

#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t serialBuf[100];
static char msg[100];
static uint8_t who_am_i = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

int main(void)
{
    HAL_StatusTypeDef status;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();

    HAL_Delay(100);

    /* STEP 1: Verify the MPU9250 responds at 7-bit address 0x68. */
    status = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x68U << 1), 3, 100);

    if (status == HAL_OK) {
        snprintf(msg, sizeof(msg), "I2C device detected at 0x68\r\n");
    } else {
        snprintf(msg, sizeof(msg),
                 "No device at 0x68; HAL status=%d\r\n", (int)status);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);

    /* STEP 2: Read WHO_AM_I (MPU9250 normally returns 0x71). */
    status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(0x68U << 1),
                              0x75, I2C_MEMADD_SIZE_8BIT,
                              &who_am_i, 1, 100);

    if (status == HAL_OK) {
        snprintf(msg, sizeof(msg), "WHO_AM_I = 0x%02X\r\n", who_am_i);
    } else {
        snprintf(msg, sizeof(msg),
                 "WHO_AM_I read failed; HAL status=%d\r\n", (int)status);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);

    /* If address 0x68 failed, stop here so the diagnostic remains clear. */
    if (status != HAL_OK || (who_am_i != WHO_AM_I_9250_ANS &&
                             who_am_i != WHO_AM_I_6050_ANS)) {
        snprintf(msg, sizeof(msg),
                 "Check wiring, power, AD0, and sensor identity.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                          (uint16_t)strlen(msg), HAL_MAX_DELAY);
        while (1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(500);
        }
    }

    /* STEP 3: Initialize the IMU. AD0_LOW means address 0x68. */
    if (MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS,
                  0.98f, 0.004f) != 1U) {
        snprintf(msg, sizeof(msg), "ERROR: MPU initialization failed.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                          (uint16_t)strlen(msg), HAL_MAX_DELAY);
        while (1) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(250);
        }
    }

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    snprintf(msg, sizeof(msg), "MPU initialization successful.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);

    /* STEP 4: Keep the sensor stationary during gyro calibration. */
    snprintf(msg, sizeof(msg), "CALIBRATING; keep sensor still...\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);

    MPU_calibrateGyro(&hi2c1, 500);

    snprintf(msg, sizeof(msg), "Calibration complete.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                      (uint16_t)strlen(msg), HAL_MAX_DELAY);

    /* STEP 5: Start TIM2. Current prescaler/period target is 250 Hz. */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        snprintf(msg, sizeof(msg), "ERROR: TIM2 failed to start.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg,
                          (uint16_t)strlen(msg), HAL_MAX_DELAY);
        while (1) {
        }
    }

    /* Sleep between interrupts; the timer callback emits attitude values. */
    while (1) {
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* Timer callback: calculate and print roll, pitch, yaw. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        MPU_calcAttitude(&hi2c1);

        int n = snprintf((char *)serialBuf, sizeof(serialBuf),
                         "%.1f,%.1f,%.1f\r\n",
                         attitude.r, attitude.p, attitude.y);

        if (n > 0) {
            HAL_UART_Transmit(&huart2, serialBuf, (uint16_t)n, 10);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
