/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMPXX80.h"
#include <stdio.h>
#include <stdint.h>
#include "TM1637.h"

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
uint8_t temp[3];

int received_temperature;
int pressure;
int temp_zad_disp;
int temp_act_disp;
int impulsy, prev_impulsy;
float counter;

float temperature;
float temp_zad;
float pulse = 0.0;
int pwm_pulse = 0;

typedef struct {
	float kp;
	float ki;
	float kd;
	float dt;
	float prev_error;
	float prev_integral;
	float saturation_limit;
} PIDController;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PIDController myPID = {2.03, 0.002 , 0, 0.5, 0, 0};
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
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart3, &temp, 3);
BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
HAL_TIM_Base_Start_IT(&htim10);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
TM1637_SetBrightness(7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(HAL_GPIO_ReadPin(ENC_Btn_GPIO_Port, ENC_Btn_Pin)==1) TM1637_DisplayDecimal(temp_zad_disp, 1);
	  else TM1637_DisplayDecimal(temp_act_disp, 1);
	  impulsy = __HAL_TIM_GET_COUNTER(&htim4);
	 // temperature = BMP280_ReadTemperature();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

/* USER CODE BEGIN 4 */
float calc_pid(PIDController* pid, float set_temp, float meas_temp){
	float u, P, I, D, error, integral, derivative;

	error = set_temp-meas_temp;

	P=pid->kp * error;

	integral = pid->prev_integral + (error+pid->prev_error);
	pid->prev_integral = integral;
	I = pid->ki*integral*(pid->dt/2.0);

	derivative = (error - pid->prev_error)/pid->dt;
	pid->prev_error=error;
	D = pid->kd*derivative;

	u = P+I+D;


	return u;
}

int ConvertTemperature(char hundreds, char tens, char units)
{
  int integer_part = (hundreds - '0') * 100 + (tens - '0') * 10 + (units - '0');
  return integer_part;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART3){
		received_temperature = ConvertTemperature(temp[0], temp[1], temp[2]);
		if (received_temperature==999) {
			uint8_t Data[70];

			sprintf(Data, "Temperatura aktualna wynosi wynosi:", temperature);

			HAL_UART_Transmit_IT(&huart3, (uint8_t*)Data, strlen(Data));
		} else if(received_temperature==997){
			uint8_t Data[70];
			uint16_t size = 0;

			sprintf(Data, "Temperatura zadana wynosi: ", temp_zad);

			HAL_UART_Transmit_IT(&huart3, (uint8_t*)Data, strlen(Data));
		}

		temp_zad = received_temperature / 10.0;
		if(temp_zad>35){
			temp_zad=35;
		}
		else if(temp_zad<25){
			temp_zad=25;
		}
		temp_zad_disp=temp_zad*100;
		HAL_UART_Receive_IT(&huart3, &temp, 3);

	}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    if(__HAL_TIM_GET_COUNTER(&htim4)>prev_impulsy)temp_zad += 0.05;
    else temp_zad += -0.05;
    temp_zad_disp=temp_zad*100; //skalowanie na wyświetlacz
    prev_impulsy =__HAL_TIM_GET_COUNTER(&htim4);
    if(temp_zad>35){
    			temp_zad=35;
    		}
    		else if(temp_zad<25){
    			temp_zad=25;
    		}
      }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	temperature = BMP280_ReadTemperature();
	temp_act_disp=temperature*100;
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	if(temp_zad != 0){
	pulse = (999.0*calc_pid(&myPID, temp_zad, temperature));
	if(pulse > 999.0){
		pwm_pulse = 999;
	}
	else if(pulse <0){
		pwm_pulse = 0;
	}
	else {
		pwm_pulse = (int)pulse;
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_pulse);
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
