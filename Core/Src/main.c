/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "stdint.h"
#include "spi.h"
#include "MFRC522.h"
#include "uart.h"
#include "temp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

void led_init()
{
	//enable clock for GPIOD
	__HAL_RCC_GPIOD_CLK_ENABLE();
	//set PD12,13,14 and 15 in OUTPUT (push-pull)
	uint32_t* GPIOD_MODER = (uint32_t*)(0x40020C00);
	*GPIOD_MODER &= ~(0b11111111 << 26);
	*GPIOD_MODER |= (0b01 << 26) | (0b01 << 28) | (0b01 << 30);
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    //LED_GREEN = 12,
    LED_ORANGE = 13,
    LED_RED = 14,
    LED_BLUE = 15,
} led_t;

void led_ctrl(uint8_t led, uint8_t state)
{
    //write state into output data register
	uint32_t* GPIOD_ODR = (uint32_t*)(0x40020C14);
	if(state == 1)
	{
		//set pin led into 1
		*GPIOD_ODR |= (1 << led);
	}
	else
	{
		//set pin led into 1
		*GPIOD_ODR &= ~(1 << led);
	}
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void UART_init()
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    uint32_t* GPIOB_MODER = (uint32_t*)(0x40020400 + 0x00);
    uint32_t* GPIOB_AFRL = (uint32_t*)(0x40020400 + 0x20);

    *GPIOB_MODER &= ~((0b11 << 12) | (0b11 << 14));
    *GPIOB_MODER |= ((0b10 << 12) | (0b10 << 14));
    *GPIOB_AFRL |= (7 << 24) | (7 << 28);

    //baudrate = 9600 ---> use formular to resolve it.
    //parity: even
    //data len: 8 bit
    __HAL_RCC_USART1_CLK_ENABLE();
    uint32_t* USART1_BRR = (uint32_t*)(0x40011000 + 0x08);
    uint32_t* USART1_CR1 = (uint32_t*)(0x40011000 + 0x0C);

    *USART1_BRR |= (104 << 4) | (3 << 0); // set baud rate = 9600
    *USART1_CR1 |= (1 << 12) | (1 << 10);
    *USART1_CR1 &= ~(1 << 9);
    *USART1_CR1 |= (1 << 13)|(1 << 3)|(1 << 2);// Enable UART, Tx và Rx
}

void UART_send_1_byte(char data)
{
    uint32_t* USART1_DR  = (uint32_t*)(0x40011000 + 0x04);
    uint32_t* USART1_SR  = (uint32_t*)(0x40011000 + 0x00);

    while(((*USART1_SR >> 7) &1) !=1); //7: Transmit data register empty
    *USART1_DR = data;
    while(((*USART1_SR >> 6) &1) !=1); //6: Transmission complete
}

void UART_send_string(char* str)
{
    int trl_len = strlen(str);
    for(int i = 0; i < trl_len; i++)
    {
        UART_send_1_byte(str[i]);
    }
}

char UART_recieve_data()
{
	uint32_t* USART1_DR  = (uint32_t*)(0x40011000 + 0x04);
	uint32_t* USART1_SR  = (uint32_t*)(0x40011000 + 0x00);

	while(((*USART1_SR >> 5) &1) !=1);
	uint8_t data = *USART1_DR;

	return data;
}

void printlog(char* format, ...)
{
	char buf[1024] = {0};
	char buf_len =0;
	va_list ap;
	va_start (ap, format);
	vsprintf(buf, format, ap);
	va_end(ap);
	buf_len = strlen(buf);
	for(int i=0; i < buf_len; i++)
	{
		UART_send_1_byte(buf[i]);
	}
}


void delay(int time)
{
	for (int b = 0; b <= time*1000; b++)
	{
		__asm("NOP");
	}
}



void system_init()
{

}

//void custom_printf(uint8_t format)
//{
//	char buf[1024] = [0];
//	va_list _ArgList;
//	va_start (_ArgList, format);
//	va_printf(buf, format,_ArgList);
//	UART_send_string(buf);
//	va_end(_ArgList);
//}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t status;
uint8_t cardstr[MAX_LEN+1];
int dem = 0;
int a = 20;
float temperature;


float adc_get_temp()
{
	int Vsense = adc_measure_vin();
	int V25 = 760;
	float AVG_slope = 2.5;
	temperature = ((Vsense - V25)/AVG_slope) +25;
	return temperature;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	adc_init();
	void delay();
	adc_get_temp();

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  HAL_Init();
  led_init();
  MFRC522_Init();
  UART_init();
  status = Read_MFRC522(VersionReg);

  UART_send_string("Get Ready! \r\n");

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  printlog("Temp = %d Degree C\r\n", temperature);
	  delay(3000);

	  status = MFRC522_Request(PICC_REQIDL, cardstr);
	  if (status == MI_OK)
	  {
		  HAL_Delay(100);
		  status = MFRC522_Anticoll(cardstr);
		  printf("Card Id: %.2x-%.2x-%.2x-%.2x",cardstr[0],cardstr[1],cardstr[2],cardstr[3]);

		  if(cardstr[0] == 136 && cardstr[1] == 4 && cardstr[2] == 148 && cardstr[3] == 200 && cardstr[4] == 208)
		   {
			  dem =0;
			  UART_send_string("Correct! \r\n");
			  for(int j =0; j <2; j++)
			  {
  				 led_ctrl(LED_BLUE, 1);
  				 HAL_Delay(100);
 				 led_ctrl(LED_BLUE, 0);
  				 HAL_Delay(100);
			  }
		  	  led_ctrl(LED_ORANGE,1);
		      HAL_Delay(3500);
		  	  led_ctrl(LED_ORANGE,0);
		  	HAL_Delay(3500);
		  	UART_send_string("Get Ready! \r\n");
		   }
		  else
		  {
  			  dem++;
  			  UART_send_string("Try again \r\n");
  	  		  led_ctrl(LED_RED, 1);
  	  		  led_ctrl(LED_BLUE, 0);
  	  		  HAL_Delay(500);
			  if(dem == 3)
			  {
				 UART_send_string("Door locked! Try again after 20 seconda \r\n");
	  			 for(int c =0; c <10; c++)
	  			 {
	  				 led_ctrl(LED_RED, 1);
	  				 HAL_Delay(1000);
	  				 a--;
	  				 led_ctrl(LED_RED, 0);
	  				 HAL_Delay(1000);
	  				 a--;
	  			 }
	  			 UART_send_string("Try again!");
	  			 dem = 0;
  	  			 a = 20;
	  			 led_ctrl(LED_RED, 0);
	  			 HAL_Delay(100);
			  }

		  }
	  }
	  if (status == MI_ERR)
	  	  {
		  	  led_ctrl(LED_RED, 0);
		  	  led_ctrl(LED_BLUE, 0);
		  	  HAL_Delay(500);
	  	  }
	  HAL_Delay(500);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
