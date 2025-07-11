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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t ESP_WaitForResponse(char *expected, uint32_t timeout_ms)
{
    char buffer[300] = {0};
    uint32_t timeStart = HAL_GetTick();
    uint16_t i = 0;
    uint8_t ch;

    while ((HAL_GetTick() - timeStart) < timeout_ms && i < sizeof(buffer) - 1)
    {
        if (HAL_UART_Receive(&huart1, &ch, 1, 1) == HAL_OK)
        {
            buffer[i++] = ch;
            buffer[i] = '\0';
            if (strstr(buffer, expected))
            {
                printf("ESP8266: %s\r\n", buffer);
                return 1;
            }
        }
    }

    printf("ESP8266 TIMEOUT OR FAILED: %s\r\n", buffer);
    return 0;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //char api[58] = "https://api.thingspeak.com/update?api_key=BRJE1GXSM5UK5UNL";
//   char api[58] = "/update?api_key=BRJE1GXSM5UK5UNL";
//   uint8_t f1;
//   uint32_t f2;
   char ATcommand[100];
   char toPost[150];
   uint8_t rxBuffer[150] = {0};
   uint8_t ATisOK;

	sprintf(ATcommand,"AT+RST\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
	printf("RESET Response:\r\n%s\r\n", rxBuffer);
	HAL_Delay(500);

	// Set Wi-Fi mode to Station
	  printf("Setting WiFi Mode = Station...\r\n");
	  sprintf(ATcommand, "AT+CWMODE_CUR=1\r\n");
	  memset(rxBuffer, 0, sizeof(rxBuffer));
	  HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);
	  HAL_UART_Receive(&huart1, rxBuffer, sizeof(rxBuffer), 1000);
	  printf("CWMODE Response:\r\n%s\r\n", rxBuffer);

	  printf("Connecting to WiFi: venky...\r\n");
	  sprintf(ATcommand, "AT+CWJAP_CUR=\"venky\",\"11223344\"\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);

	  // Use the new function to wait for full response
	  if (ESP_WaitForResponse("OK", 15000) || ESP_WaitForResponse("WIFI GOT IP", 15000))
	  {
	      printf("WiFi Connected Successfully!\r\n");
	  }
	  else
	  {
	      printf("WiFi Connection Failed!\r\n");
	  }


   ATisOK = 0;
   while(!ATisOK){
     sprintf(ATcommand,"AT+CIPMUX=0\r\n");
     memset(rxBuffer,0,sizeof(rxBuffer));
     HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
     HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
     if (ESP_WaitForResponse("OK", 5000))
     {
    	 ATisOK = 1;
     }

   }

    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   uint32_t counter = 0;

   while (1)
   {
       printf("Connecting to ThingSpeak...\r\n");

       // Start TCP connection
       sprintf(ATcommand, "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
       HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);

       if (ESP_WaitForResponse("CONNECT", 5000) || ESP_WaitForResponse("ALREADY CONNECTED", 5000))
       {
           printf("Connected to ThingSpeak server.\r\n");

           // Prepare HTTP GET string
           sprintf(toPost,
                   "GET /update?api_key=YE5E28DP59ZS9QLX&field1=%lu HTTP/1.1\r\n"
                   "Host: api.thingspeak.com\r\n"
                   "Connection: close\r\n\r\n", counter);

           // Send AT+CIPSEND
           sprintf(ATcommand, "AT+CIPSEND=%d\r\n", strlen(toPost));
           HAL_UART_Transmit(&huart1, (uint8_t *)ATcommand, strlen(ATcommand), 1000);

           if (ESP_WaitForResponse(">", 3000))
           {
               // Send actual GET request
               HAL_UART_Transmit(&huart1, (uint8_t *)toPost, strlen(toPost), 1000);

               if (ESP_WaitForResponse("OK", 5000) || ESP_WaitForResponse("SEND OK", 5000))
               {
                   printf("Data sent to ThingSpeak: Counter = %lu\r\n", counter);
               }
               else
               {
                   printf("Failed to send data.\r\n");
               }
           }
           else
           {
               printf("CIPSEND failed. ESP8266 not ready.\r\n");
           }

           counter++;
           if(counter > 5)
        	   counter = 0;
       }
       else
       {
           printf("ThingSpeak TCP connection failed.\r\n");
       }

       HAL_Delay(15000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
