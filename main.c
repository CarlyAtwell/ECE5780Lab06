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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  HAL_Init();

  SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable the GPIOA clock in the RCC
	
	
	// Configure LEDS -------------------------------------
	// Set pins PC6 PC7 PC8 and PC9 to output mode
	GPIOC->MODER |= (1 << 12) |  (1 << 14) | (1 <<16) | (1 << 18);
	GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
	// Set pins PC6 PC7 PC8 and PC9 to output push-pull
	GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) |(1 <<9));
	// Set pins PC6 PC7 PC8 and PC9 to low speed
	GPIOC->OSPEEDR &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 <<18));
	// Set pins PC6 PC7 PC8 and PC9 to no pullup, pulldown
	GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
	
	// select a GPIO pin to use as the ADC input
	// Pin PA1 GPIOA ADC_IN1 - channel 1
	// configure the pin to analog mode, no pull-up/down resistors
	GPIOA->MODER |= (1 << 2) | (1 << 3);
	GPIOA->PUPDR &= ~( (1 << 2) | (1 << 3) );
	
	// enable the ADC1 in the RCC peripheral
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //RCC_APB2ENR_ADCEN ???
	
	// Configure the ADC to desired operating mode, data resolution, and trigger source
	// ADC_CFGR1
	// operating mode = continuous conversion mode
	ADC1->CFGR1 |= (1 << 13);
	// data resolution = 8-bit resolution
	ADC1->CFGR1 |= (1 << 4);
	// trigger source = software triggers only / hardware trigger disabled
	ADC1->CFGR1 &= ~( (1 << 11) | (1 << 10) );
	
	//Select/enable the input pin's channel for ADC conversion
	ADC1->CHSELR |= (1 << 1);
	
	// perform a self calibration, enable, and start the ADC
	// ensure that ADEN = 0 and DMAEN = 0
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
		ADC1->CR |= ADC_CR_ADDIS; /* (2) */
	}
	while ((ADC1->CR & ADC_CR_ADEN) != 0)
	{

	}
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
	ADC1->CR |= ADC_CR_ADCAL; /* (4) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
	{

	}
	/* (1) Ensure that ADRDY = 0 */
	/* (2) Clear ADRDY */
	/* (3) Enable the ADC */
	/* (4) Wait until ADC ready */
	if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) /* (1) */
	{
		ADC1->ISR |= ADC_ISR_ADRDY; /* (2) */
	}
	ADC1->CR |= ADC_CR_ADEN; /* (3) */
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
	{
		
	}

	ADC1->CR |= ADC_CR_ADSTART;
	
	///////////////////////////////////// PART 2: DAC //////////////////////////////////////////
	
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	
	// dac_out1 pin PA4
	//configure the pin to analog mode, no pull-up/down resistors
	GPIOA->MODER |= (1 << 8) | (1 << 9);
	GPIOA->PUPDR &= ~( (1 << 4) | (1 << 8) );
	
	//Set the used DAC channel to software trigger mode. [5:3] 111
	DAC->CR |= (1 << 5) | (1 << 4) | (1 << 3);
	//enable the used DAC channel - channel 1
	DAC->CR |= (1 << 0);	
	
	// Sine Wave: 8-bit, 32 samples/cycle
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244, 
		232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	

  while (1)
  {
		//////////////////////////////////DAC ////////////////////////////////////////
		
		int index;
		volatile uint8_t nextVal;
		for(index = 0; index < 32; index++){
			nextVal = sine_table[index];
			HAL_Delay(1);
			DAC->DHR8R1 = nextVal; 
		}
		
		////////////////////////////// ADC /////////////////////////////////////////////////
		
		// read the ADC data register and turn on/off LEDs depending on the value
    volatile uint16_t adc_data = ADC1->DR;
		
		
		if(adc_data > 50){
			// turn on first led - red
			GPIOC->ODR |= (1 << 6);
		}
		if(adc_data > 100){
			// turn on second led - blue
			GPIOC->ODR |= (1 << 7);
		}
		if(adc_data > 150){
			// turn on third led - orange
			GPIOC->ODR |= (1 << 8);
		}
		if(adc_data > 200){
			// turn on fourth led - green
			GPIOC->ODR |= (1 << 9);
		}
		
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
