/* USER CODE BEGIN Header */
/**
  * Code for gate opening in one axis - from down to up
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t gate_state = 0;	// 0 = STOP, 1 = OPENING, 2 = CLOSING
volatile uint32_t gate_timer = 0;	// count time after detecing movement
volatile uint8_t percent_closure = 0;		//	percentage value of gate closure
volatile uint8_t send_data_flag = 0;
int16_t acc_z_prev = 0;
int16_t acc_x_prev = 0;
int16_t acc_y_prev = 0;
uint16_t time_up = 0;				// max time of gate opening
uint16_t time_down = 0;				// max time of gate closing
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void enter_standby_mode(void)
{
    HAL_SuspendTick();
    HAL_PWR_EnterSTANDBYMode();
    HAL_ResumeTick();

    // re-init peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
}

void configure_bma400(struct bma400_dev *dev)
{
    uint8_t rslt;
    struct bma400_device_conf dev_conf[3];
    struct bma400_sensor_conf sensor_conf;

    // Set initial power mode to low power
    rslt = bma400_set_power_mode(BMA400_MODE_LOW_POWER, dev);
    if (rslt != BMA400_OK)
    {
        Error_Handler();
    }

    // auto low power after movement stops (1 s)
    dev_conf[0].type = BMA400_AUTO_LOW_POWER;
    dev_conf[0].param.auto_lp.auto_low_power_trigger =
        BMA400_AUTO_LP_GEN1_TRIGGER | BMA400_AUTO_LP_TIMEOUT_EN | BMA400_AUTO_LP_TIME_RESET_EN;
    dev_conf[0].param.auto_lp.auto_lp_timeout_threshold = 400; // 400 × 2.5 ms = 1 s

    // wake-up interrupt on motion detection on X axis
    dev_conf[1].type = BMA400_AUTOWAKEUP_INT;
    dev_conf[1].param.wakeup.wakeup_ref_update = BMA400_UPDATE_ONE_TIME;
    dev_conf[1].param.wakeup.sample_count = BMA400_SAMPLE_COUNT_1;
    dev_conf[1].param.wakeup.wakeup_axes_en = BMA400_AXIS_XYZ_EN;
    dev_conf[1].param.wakeup.int_wkup_threshold = 2;	// mg threshold
    dev_conf[1].param.wakeup.int_wkup_ref_z = 0;
    dev_conf[1].param.wakeup.int_chan = BMA400_INT_CHANNEL_1;

    dev_conf[2].type = BMA400_INT_PIN_CONF;
    dev_conf[2].param.int_conf.int_chan = BMA400_INT_CHANNEL_1;
    dev_conf[2].param.int_conf.pin_conf = BMA400_INT_OPEN_DRAIN_ACTIVE_0;

    rslt = bma400_set_device_conf(dev_conf, 3, dev);
    if (rslt != BMA400_OK)
    {
        Error_Handler();
    }

    // configure accelerometer
    sensor_conf.type = BMA400_ACCEL;
    sensor_conf.param.accel.odr = BMA400_ODR_100HZ;
    sensor_conf.param.accel.range = BMA400_RANGE_2G;
    sensor_conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_LP;
    sensor_conf.param.accel.osr = BMA400_ACCEL_OSR_SETTING_3;
    sensor_conf.param.accel.filt1_bw = BMA400_ACCEL_FILT1_BW_0;

    rslt = bma400_set_sensor_conf(&sensor_conf, 1, dev);
    if (rslt != BMA400_OK) Error_Handler();

    // enable auto wakeup
    rslt = set_auto_wakeup(BMA400_ENABLE, dev);
    if (rslt != BMA400_OK)
    {
        Error_Handler();
    }
}

void check_gate_direction(struct bma400_dev *dev)
{
    struct bma400_sensor_data data;
    bma400_get_accel_data(BMA400_DATA_ONLY, &data, dev);

    int16_t acc_x_now = data.x;
    int16_t delta_x = acc_x_now - acc_x_prev;

    int16_t acc_y_now = data.y;
    int16_t delta_y = acc_y_now - acc_y_prev;

    int16_t acc_z_now = data.z;
    int16_t delta_z = acc_z_now - acc_z_prev;

    if (delta_x > 1 || delta_y > 1 || delta_z > 1)
    {
        gate_state = 1; // opening
    }
    else if (delta_x < -1 || delta_y < -1 || delta_z < -1)
    {
        gate_state = 2; // closing
    }

    acc_x_prev = acc_x_now;
    acc_y_prev = acc_y_now;
    acc_z_prev = acc_z_now;
}


void update_gate_position()
{
    if (gate_state == 1)	// Opening
    {
        if (gate_timer >= time_up)
        {
        	percent_closure = 100;
            gate_state = 0;
        }
        else
        {
        	percent_closure = (gate_timer * 100) / time_up;
        }
    }
    else if (gate_state == 2)	// Closing
    {
        if (gate_timer >= time_down)
        {
        	percent_closure = 0;
            gate_state = 0;
        }
        else
        {
        	percent_closure = 100 - (gate_timer * 100) / time_down;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	struct bma400_dev bma400;

	bma400.intf = BMA400_I2C_INTF;
	bma400.intf_ptr = &hi2c1;
	bma400.read = user_i2c_read;
	bma400.write = user_i2c_write;
	bma400.delay_us = user_delay_us;
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
  bma400_init(&bma400);
  configure_bma400(&bma400);

  HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, SET);
  HAL_Delay(200);

  uint8_t buffer[4];
  HAL_UART_Receive(&huart1, buffer, 4, HAL_MAX_DELAY);
  time_up   = buffer[0] | (buffer[1] << 8);
  time_down = buffer[2] | (buffer[3] << 8);

  HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, RESET);

  HAL_TIM_Base_Start_IT(&htim14);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  check_gate_direction(&bma400);

	  if (send_data_flag)	// if gate is opening or closing
	  {
		  send_data_flag = 0;

		  HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, SET);	// activate ESP8266
		  HAL_Delay(200);

		  char msg[5];
		  snprintf(msg, sizeof(msg), "%d\n", percent_closure);
		  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);	// send data to esp

		  HAL_GPIO_WritePin(EN_IO_GPIO_Port, EN_IO_Pin, RESET);
	  }
	  else if ((gate_state == 1 && gate_timer >= time_up) ||
	           (gate_state == 2 && gate_timer >= time_down))	// if gate has finished opening or closing
	  {
		  gate_state = 0; // STOP
		  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
		  enter_standby_mode();
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
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		if(gate_state != 0)
		{
			gate_timer += 1000;
			update_gate_position();
			send_data_flag = 1;
		}
		else
		{
			__HAL_TIM_SET_COUNTER(&htim14, 0);
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
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
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
