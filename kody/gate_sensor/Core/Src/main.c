/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "bma400.h"
#include "math.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_FIFO_SAMPLES 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t motion_time = 0;
volatile uint16_t flag_2sec = 0;
volatile uint16_t send_2sec = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void enterStandby(void)
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
  	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnterSTANDBYMode();
}

// Configure BMA400 for low power + wake-up on Y-axis motion
void configureBMA400(struct bma400_dev *dev)
{
	if (bma400_set_power_mode(BMA400_MODE_LOW_POWER, dev) != BMA400_OK)
	{
		__NOP();
		Error_Handler();
	}

	struct bma400_device_conf dev_conf[] = {
		{
			.type = BMA400_AUTO_LOW_POWER,	// auto low power after movement stops (350 ms)
			.param.auto_lp = {
				.auto_low_power_trigger = BMA400_AUTO_LP_GEN1_TRIGGER | BMA400_AUTO_LP_TIMEOUT_EN | BMA400_AUTO_LP_TIME_RESET_EN,
				.auto_lp_timeout_threshold = 1500	// 250 x 2.5 ms =  ms
			}
		},
		{
			.type = BMA400_AUTOWAKEUP_INT,	// wake-up interrupt on motion detection on Y axis
			.param.wakeup = {
				.wakeup_ref_update = BMA400_UPDATE_EVERY_TIME,
				.sample_count = BMA400_SAMPLE_COUNT_1,
				.wakeup_axes_en = BMA400_AXIS_Y_EN,
				.int_wkup_threshold = 10,				// 15 mg threshold
				.int_chan = BMA400_INT_CHANNEL_1
			}
		},
		{
			.type = BMA400_INT_PIN_CONF,	// configure interrupt pin
			.param.int_conf = {
				.int_chan = BMA400_INT_CHANNEL_1,
				.pin_conf = BMA400_INT_OPEN_DRAIN_ACTIVE_0
			}
		}
	};

	bma400_set_device_conf(dev_conf, 3, dev);

	struct bma400_sensor_conf sensor_conf = {
		.type = BMA400_ACCEL,	// configure accelerometer
		.param.accel = {
			.range = BMA400_RANGE_2G,
			.data_src = BMA400_DATA_SRC_ACCEL_FILT_2,
			.osr = BMA400_ACCEL_OSR_SETTING_3,
			.odr = BMA400_ODR_100HZ
		}
	};

	bma400_set_sensor_conf(&sensor_conf, 1, dev);
	set_auto_wakeup(BMA400_ENABLE, dev);	// enable auto wake up
}

void configureFifo(struct bma400_dev *dev)
{
	// stream mode, don't stop on full
	struct bma400_device_conf fifo_conf = {
		.type = BMA400_FIFO_CONF,
		.param.fifo_conf = {
			.conf_regs = BMA400_FIFO_Y_EN | BMA400_FIFO_DATA_SRC,
			.conf_status = BMA400_ENABLE
		}
	};

	if (bma400_set_device_conf(&fifo_conf, 1, dev) != BMA400_OK ||
		bma400_set_fifo_flush(dev) != BMA400_OK)
		Error_Handler();
	HAL_Delay(10);
}

uint8_t readFifoY(int16_t *out, uint8_t max_samples, struct bma400_dev *dev)
{
    struct bma400_fifo_data fifo = {0};
    static uint8_t fifo_data[MAX_FIFO_SAMPLES * 3];		// buffer for raw FIFO bytes (1 byte header + 2 bytes per Y sample)

    struct bma400_fifo_sensor_data accel_data[MAX_FIFO_SAMPLES];	// buuffer for decoded samples
    uint16_t accel_count = MAX_FIFO_SAMPLES;

    fifo.data = fifo_data;
    fifo.length = sizeof(fifo_data);

    if (bma400_get_fifo_data(&fifo, dev) != BMA400_OK)	// read raw FIFO data
        return 0;

    if (bma400_extract_accel(&fifo, accel_data, &accel_count, dev) != BMA400_OK)	// decode FIFO data (eg. 1024)
        return 0;

    uint8_t copied = (accel_count > max_samples) ? max_samples : accel_count;	// limit number of samples copied to buffer

    for (uint8_t i = 0; i < copied; i++)
    {
        out[i] = accel_data[i].y;
    }

    return copied;
}

bool detectGateMotion(const int16_t *data)
{
    float sigma[4] = {0};

    // calculate SD from 4 blocks of 8 samples each
    for (uint8_t block = 0; block < 4; block++)
    {
        float sum = 0, mean = 0, sum_diff_squared = 0;
        const int16_t *ptr = &data[block * 8];			// pointer to start of current block

        // calculate mean of the 8 samples
        for (uint8_t i = 0; i < 8; i++)
        {
            sum += ptr[i];
        }
        mean = sum / 8.0f;

        // calculate sum of squared differences from the mean
        for (uint8_t i = 0; i < 8; i++)
        {
        	sum_diff_squared += powf(ptr[i] - mean, 2);
        }

        sigma[block] = sqrtf(sum_diff_squared / 8.0f); // calculate standard deviation
    }

    // count how many times the difference between adjacent sigmas exceeds threshold (5 LSB)
    uint8_t count = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        float diff = fabsf(sigma[i + 1] - sigma[i]);
        if (diff >= 5.0f)
            count++;
    }

    return (count >= 3); // if 3 times threshold was exceeded -> motion detected
}

void Send16BitESP(UART_HandleTypeDef *huart, uint16_t value) {
	char msg[16];
	snprintf(msg, sizeof(msg), "%u\n", value);

	HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, SET);
	HAL_Delay(200);
	HAL_UART_Transmit(huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	bool timer_started = false;
	bool motion_detected = false;

	struct bma400_dev bma400 = {
		.intf = BMA400_I2C_INTF,
		.intf_ptr = &hi2c1,
		.read = user_i2c_read,
		.write = user_i2c_write,
		.delay_us = user_delay_us
	};
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB); // dbg - check power mode state
  bma400_init(&bma400);
  configureBMA400(&bma400);
  configureFifo(&bma400);
  HAL_Delay(10);

  uint16_t fifo_len = 0;
  do {
      uint8_t fifo_len_raw[2];
      bma400_get_regs(0x12, fifo_len_raw, 2, &bma400);
      fifo_len = fifo_len_raw[0] | (fifo_len_raw[1] << 8);
      HAL_Delay(5);
  } while (fifo_len < 96);  // (1 header + 2 data bytes) * 32 samples
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  int16_t y_samples[32];
	  readFifoY(y_samples, 32, &bma400);

	  motion_detected = detectGateMotion(y_samples);

	  if (!motion_detected)
	  {
		  HAL_TIM_Base_Stop_IT(&htim14);
		  if (motion_time >= 1)	// gate stopped while movement
		  {
			  Send16BitESP(&huart1, motion_time);
			  enterStandby();
		  }
		  else
		  {
			  enterStandby();
		  }
	  }
	  else if (motion_detected && flag_2sec == 1)
	  {
		  flag_2sec = 0;
		  Send16BitESP(&huart1, send_2sec);
	  }
	  else if (motion_detected && !timer_started)
	  {
		  motion_time = 0;
		  HAL_TIM_Base_Start_IT(&htim14);
		  timer_started = true;
	  }
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM14)
	{
		motion_time++;

		static uint16_t tick_counter = 0;
		tick_counter++;

		if (tick_counter >= 2000)
		{
			tick_counter = 0;
			flag_2sec = 1;
			send_2sec = motion_time;
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
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
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
