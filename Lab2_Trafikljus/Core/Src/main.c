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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


void my_systick_handler(void);


enum state {
    s_init,                // Initial state
    s_car_go,              // Cars go (green light for cars)
    s_pushed_wait,         // Wait after pedestrian button pressed
    s_cars_stopping,       // Cars slowing down (yellow light for cars)
    s_ped_go,              // Pedestrians go (green light for pedestrians)
    s_ped_warning,         // Pedestrian warning (about to switch)
    s_all_red,             // All red (safe state)
    s_cars_starting        // Cars preparing to start (yellow light)
};

enum event {
    ev_none = 0,           // No event
	ev_error = -99,
    ev_button_push,        // Button pressed event
    ev_state_timeout       // Timeout event
};

#define EVQ_SIZE 10
enum event evq[ EVQ_SIZE ];
int evq_count = 0;
int evq_front_ix = 0;
int evq_rear_ix = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int systick_count = 0;
uint32_t ticks_left_in_state = 0;  // Declare this globally



void evq_push_back(enum event e)
{
// if queue is full, ignore e
if ( evq_count < EVQ_SIZE )
{
evq[evq_rear_ix] = e;
evq_rear_ix++;
evq_rear_ix %= EVQ_SIZE;
evq_count++;
}
}
enum event evq_pop_front()
{
enum event e = ev_none;
if ( evq_count > 0 )
{
e = evq[evq_front_ix];
evq[evq_front_ix] = ev_error; // detect stupidity
evq_front_ix++;
evq_front_ix %= EVQ_SIZE;
evq_count--;
}
return e;
}

void evq_init(void) {
    for (int i = 0; i < EVQ_SIZE; i++) {
        evq[i] = ev_error; // Set all entries to ev_error
    }
    evq_count = 0;            // Queue is initially empty
    evq_front_ix = 0;         // Start of the queue
    evq_rear_ix = 0;          // End of the queue
}

void my_systick_handler(void)
{
    systick_count++;

    // Decrease ticks_left_in_state and check if it reaches 0
    if (ticks_left_in_state > 0) {
        ticks_left_in_state--;
    }
    if (systick_count == 1000)
        {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            systick_count = 0;
        }

        // Push timeout event if ticks_left_in_state reaches 0
        if (ticks_left_in_state == 0) {
            evq_push_back(ev_state_timeout);
        }
    }


void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
        evq_push_back(ev_button_push);

	}

}



int is_button_pressed()
{
    // Kontrollera status för pinne 13 på GPIOC (PC13)
    if (GPIOC->IDR & (1 << 13))
    {
        return 0; // Knappen är nedtryckt (bit är satt)
    }
    else
    {
        return 1; // Knappen är inte nedtryckt (bit är ej satt)
    }
}

void push_button_light_on(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);  // Tänd lysdioden
}

void push_button_light_off(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // Släck lysdioden
}

void set_car_lights(int red, int yellow, int green) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, red ? GPIO_PIN_SET : GPIO_PIN_RESET); // Red light
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,
			yellow ? GPIO_PIN_SET : GPIO_PIN_RESET); // Yellow light
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, green ? GPIO_PIN_SET : GPIO_PIN_RESET); // Green light
}

void set_pedestrian_lights(int red, int green) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, red ? GPIO_PIN_SET : GPIO_PIN_RESET); // Red light
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, green ? GPIO_PIN_SET : GPIO_PIN_RESET); // Green light
}

void set_traffic_lights(enum state s) {
	switch (s) {
	case s_init:
		// All lights on during initialization
		set_car_lights(1, 1, 1);       // Red, yellow, and green for cars
		set_pedestrian_lights(1, 1);   // Red and green for pedestrians
		break;

	case s_all_red:
		// All lights red (safety mode)
		set_car_lights(1, 0, 0);       // Red for cars
		set_pedestrian_lights(1, 0);   // Red for pedestrians
		break;

	case s_ped_go:
		// Green light for pedestrians
		set_car_lights(1, 0, 0);       // Red for cars
		set_pedestrian_lights(0, 1);   // Green for pedestrians
		break;

	case s_ped_warning:
		// Warning signal for pedestrians
		set_car_lights(1, 0, 0);       // Red for cars
		set_pedestrian_lights(0, 1);   // Green for pedestrians
		break;

	case s_cars_starting:
		// Preparing for green light for cars
		set_car_lights(0, 1, 0);       // Yellow for cars
		set_pedestrian_lights(1, 0);   // Red for pedestrians
		break;

	case s_car_go:
		// Green light for cars, red for pedestrians
		set_car_lights(0, 0, 1);       // Green for cars
		set_pedestrian_lights(1, 0);   // Red for pedestrians
		break;

	case s_pushed_wait:
		// Wait time after button press
		set_car_lights(0, 0, 1);       // Green for cars
		set_pedestrian_lights(1, 0);   // Red for pedestrians
		break;

	case s_cars_stopping:
		// Yellow light for cars
		set_car_lights(0, 1, 0);       // Yellow for cars
		set_pedestrian_lights(1, 0);   // Red for pedestrians
		break;

	default:
		// Handle undefined states
		set_car_lights(1, 1, 1);       // All lights on as fallback
		set_pedestrian_lights(1, 1);   // All lights on as fallback
		break;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
		push_button_light_off();  // Ensure the LED is off initially

		evq_init();
		enum state st = s_init;    // Initiera state
		enum event ev = ev_none;  // Initiera event

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

		while (1) {
			ev = evq_pop_front(); // Default event

			switch (st) {
			case s_init:
				set_traffic_lights(s_init);
				if (ev == ev_button_push) {
					st = s_all_red; // Övergång till nästa state på knapptryckning
					ticks_left_in_state = 2000; // 2000 ms i nästa tillstånd
				}
				break;

			case s_all_red:
				set_traffic_lights(s_all_red);
				if (ev == ev_state_timeout) {
					st = s_car_go; // Övergång till bilens gröna ljus efter timeout
					ticks_left_in_state = 5000; // 5000 ms för bilar att köra
				}
				break;

			case s_car_go:
				set_traffic_lights(s_car_go);
				if (ev == ev_button_push) {
			        push_button_light_on();
					st = s_pushed_wait;  // Om knappen trycks, vänta
					ticks_left_in_state = 2000;  // Väntetid på 2000 ms
				}
				break;

			case s_pushed_wait:
				set_traffic_lights(s_pushed_wait);
				if (ev == ev_state_timeout) {
					st = s_cars_stopping;  // Bilarna stannar
					ticks_left_in_state = 1000;  // 1000 ms för att stanna
				}
				break;

			case s_cars_stopping:
				set_traffic_lights(s_cars_stopping);
				if (ev == ev_state_timeout) {
					st = s_ped_go;  // Gå för fotgängare
					ticks_left_in_state = 5000;  // 5000 ms för fotgängare
				}
				break;

			case s_ped_go:
				set_traffic_lights(s_ped_go);
		        push_button_light_on();
				if (ev == ev_state_timeout) {
					st = s_ped_warning;  // Fotgängarvarning
					ticks_left_in_state = 2000;  // 2000 ms för varning
				}
				break;

			case s_ped_warning:
				set_traffic_lights(s_ped_warning);
				if (ev == ev_state_timeout) {
					st = s_cars_starting;  // Bilar börjar köra
					ticks_left_in_state = 1000; // 1000 ms för att börja köra
				}
				break;

			case s_cars_starting:
				set_traffic_lights(s_cars_starting);
				if (ev == ev_state_timeout) {
					st = s_car_go;  // Tillbaka till bilarnas gröna ljus
					ticks_left_in_state = 5000;  // 5000 ms för bilkörning
				}
				break;

			default:
				st = s_init;
				break;
			}

		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
