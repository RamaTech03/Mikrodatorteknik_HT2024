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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_print_menu(void);
int uart_get_menu_choice(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Globala variabler
uint16_t button_exti_count = 0;         // Antal gånger avbrottsfunktionen körts
uint16_t button_debounced_count = 0;    // Antal gånger användaren upplever ett giltigt knapptryck
uint32_t last_flank_causing_exti = 0;   // Tidpunkt för den senaste flanken (millisekunder)

// Konstant för debouncing-tid
#define BOUNCE_DELAY_MS 20              // Den tid som behövs för att hantera kontaktstuds


void uart_print_menu()
{
    char menu[] = "\r\n"
                  "This is a very nice menu, yes?\r\n"
                  "Choose your destiny:\r\n"
                  "1. Clock mode\r\n"
                  "2. Button mode\r\n"
                  "Enter your choice: ";

    // Skicka menyn via UART
    HAL_UART_Transmit(&huart2, (uint8_t *)menu, sizeof(menu)-1, HAL_MAX_DELAY);
}

int uart_get_menu_choice()
{
    char str[1] = { '\0' };  // Initierar strängen
    uint16_t str_len = 1;    // Vi vill läsa in ett tecken
    HAL_UART_Receive(&huart2, (uint8_t *)str, str_len, HAL_MAX_DELAY);

    int ret = -1;
    // sscanf tolkar inmatningen som ett heltal
    if (sscanf(str, "%d", &ret) == 1)
    {
        // Returnera valet om det är en giltig siffra
        return ret;
    }
    // Returnera -1 om ingen siffra kunde läsas
    return -1;
}


void uart_print_bad_choice()
{
    char msg[] = "Invalid choice! Please try again.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg)-1, HAL_MAX_DELAY);
}

void clock_mode(void) {
    uint8_t seconds = 45;   // Startar från 23:59:45 för testning
    uint8_t minutes = 59;
    uint8_t hours = 23;
    uint8_t colon_on = 0;   // Kolondiodens status (blinkning)

    // Startar timern i interrupt-läge
    HAL_TIM_Base_Start_IT(&htim2);

    while (1) {
        // Kontrollera om knappen är intryckt
        if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
            // Knappen är intryckt, visa HH:MM
            DisplayTime(hours, minutes);
        } else {
            // Knappen är inte intryckt, visa MM:SS
            DisplayTime(minutes, seconds);
        }

        // Logik för att hålla reda på tiden hanteras i callback
        // Kolondiodens blinkande hanteras också i callback (varje halv sekund)
    }
}


void button_mode()
{
	 int b1_pressed = 0;
	    uint32_t now = 0;

	    while (1) {
	        now = HAL_GetTick();  // Hämta nuvarande tid i millisekunder

	        // Kolla om en ny flank har inträffat och om tillräcklig tid har passerat sedan förra hanteringen
	        if (last_flank_causing_exti != last_managed_exti && (now - last_flank_causing_exti) >= BOUNCE_DELAY_MS) {
	            // Läs GPIO för att kontrollera om knappen är nedtryckt
	            int pressed = GPIO_PIN_RESET == HAL_GPIO_ReadPin(MY_BTN_GPIO_Port, MY_BTN_Pin);

	            // Om knappen fortfarande är nedtryckt efter debouncing-tiden
	            if (pressed) {
	                button_debounced_count++;  // Öka debounced-räknaren för giltigt knapptryck
	            }

	            // Markera denna flank som hanterad
	            last_managed_exti = last_flank_causing_exti;
	        }

	        // Kontrollera om knappen är nedtryckt (aktiv låg på B1-knappen)
	        b1_pressed = GPIO_PIN_RESET == HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

	        // Visa antal knapptryckningar: Om knappen är nedtryckt visas alla studsningar (button_exti_count),
	        // annars visas debounced-värdet (button_debounced_count)
	        qs_put_big_num(b1_pressed ? button_exti_count : button_debounced_count);
	    }
	}

// Avbrottshanterare för knappen
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == MY_BTN_Pin) {
        button_exti_count++;            // Räknar antalet avbrott (flanker)
        last_flank_causing_exti = HAL_GetTick();  // Sparar tidpunkten då avbrottet inträffade
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  uart_print_menu();  // Skriv ut menyn via UART
	      int menu_choice = uart_get_menu_choice();  // Läs användarens val

	      switch (menu_choice)
	      {
	        case 1:
	          clock_mode();  // Om valet är 1, gå in i klockläge
	          break;
	        case 2:
	          button_mode();  // Om valet är 2, gå in i knapp-läge
	          break;
	        default:
	          uart_print_bad_choice();  // Felaktigt val, skriv ut ett felmeddelande
	          break;
	      }
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG_DIO_Pin|SEG_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_DIO_Pin */
  GPIO_InitStruct.Pin = SEG_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_DIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEG_CLK_Pin */
  GPIO_InitStruct.Pin = SEG_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEG_CLK_GPIO_Port, &GPIO_InitStruct);

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
