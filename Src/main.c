/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define LONG_PRESS_DURATION 500

void TurnOnLed0();
void TurnOffLed0();
void TurnOnLed1();
void TurnOffLed1();
void LEDs_Init();
void SysTick_Init();
void Buttons_Init();
void Interrupts_Init();
volatile int timeButOne = 0;
volatile int timeButTwo = 0;
void delay(uint32_t milis);
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
	LEDs_Init();
	SysTick_Init();
	Buttons_Init();
	Interrupts_Init();

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //empty while loop, no polling
  }
  /* USER CODE END 3 */
}

void Interrupts_Init() {
	RCC->APB2ENR |= 0b1; //enable clock for SYSCFG

	SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI1_PA) |
			(SYSCFG_EXTICR1_EXTI1_PA << SYSCFG_EXTICR1_EXTI1_Pos); //allow PA1 interrupt requests (button one)

	SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0_PB) |
			(SYSCFG_EXTICR1_EXTI0_PB << SYSCFG_EXTICR1_EXTI0_Pos); //allow PB0 interrupt requests  (button two)

	EXTI->FTSR |= EXTI_FTSR_TR1; //enable falling edge detection for line 1
	EXTI->FTSR |= EXTI_FTSR_TR0; //enable falling edge detection for line 0

	EXTI->RTSR |= EXTI_RTSR_TR1; //enable rising edge detection for line 1
	EXTI->RTSR |= EXTI_RTSR_TR0; //enable rising edge detection for line 0

	EXTI->IMR |= EXTI_IMR_MR1; //unmask line 1
	EXTI->IMR |= EXTI_IMR_MR0; //unmask line 0

	NVIC_EnableIRQ(EXTI1_IRQn); //enable interrupts on line 1
	NVIC_EnableIRQ(EXTI0_IRQn); //enable interrupts on line 0

	EXTI->PR |= EXTI_PR_PR1; // reset flag line 1
	EXTI->PR |= EXTI_PR_PR0; // reset flag line 0

}

// interrupt handler for button one
void EXTI1_IRQHandler(void) {
	EXTI->PR |= EXTI_PR_PR1; // reset flag
	int ButtonOne = GPIOA->IDR & GPIO_IDR_1;

	if (ButtonOne) { // interrupt triggered by rising edge here
		delay(20); //debounce delay
		ButtonOne = GPIOA->IDR & GPIO_IDR_1; // read button state again
		if (ButtonOne) { // debounced input
			TurnOnLed1();
			timeButOne = 0; //reset timer to potentially turn off LED in falling edge
		}
	}
	else { // interrupt triggered by falling edge here
		delay(20); // debounce falling edge delay
		ButtonOne= GPIOA->IDR & GPIO_IDR_1;
		if ((timeButOne > LONG_PRESS_DURATION) && (!ButtonOne)) { // check if more than 500ms have passed since button has been pressed
			TurnOffLed1();
		}
	}
}

// interrupt handler for button two, works analogically to button one handler
void EXTI0_IRQHandler(void) {
	EXTI->PR |= EXTI_PR_PR0;
	int ButtonTwo = GPIOB->IDR & GPIO_IDR_0;

	if (ButtonTwo) {
		delay(20);
		ButtonTwo = GPIOB->IDR & GPIO_IDR_0;
		if (ButtonTwo) {
			TurnOnLed0();
			timeButTwo = 0;
		}
	}
	else {
		delay(20);
		ButtonTwo = GPIOB->IDR & GPIO_IDR_0;
		if ((timeButTwo > LONG_PRESS_DURATION) && (!ButtonTwo)){
			TurnOffLed0();
		}
	}
}

void Buttons_Init() {
	// External button A3 (PB0)
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER0) |
			(0b00 << GPIO_MODER_MODER0_Pos); //input mode
	GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR0) |
			(0b10 << GPIO_PUPDR_PUPDR0_Pos); //pull down resistor

	// External button A1
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER1) |
			(0b00 << GPIO_MODER_MODER1_Pos); // Set mode for external button A1 to input
	GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR1) |
			(0b10 << GPIO_PUPDR_PUPDR1_Pos); // configure internal pull-down resistor for the button
}

void TurnOnLed0() {
	GPIOA->ODR |= GPIO_ODR_5; //set bit PA5
}

void TurnOffLed0() {
	GPIOA->ODR &= ~GPIO_ODR_5; //clear bit PA5
}

void TurnOnLed1() {
	GPIOA->ODR |= GPIO_ODR_0; //set bit PA5
}

void TurnOffLed1() {
	GPIOA->ODR &= ~GPIO_ODR_0; //clear bit PA5
}
void SysTick_Init() {
	SysTick->CTRL = 0x4; // disable interrupts while configuring and disable counter
	SysTick->LOAD = 15999; //reload value of 15999 gives us a tick each milisecond, since the clock frequency is locked to 16MHz
	SysTick->VAL = 0; // reset counter and counter flag
	SysTick->CTRL = 0x7; // enable interrupts and counter
}

void LEDs_Init() {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable clock for port a

	GPIOA->MODER &= ~GPIO_MODER_MODER5; //clear pin 5
	GPIOA->MODER |= GPIO_MODER_MODER5_0; //set pin 5 to output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5; //set output to push pull

	GPIOA->MODER &= ~GPIO_MODER_MODER0; //clear pin 0
	GPIOA->MODER |= GPIO_MODER_MODER0_0; //set pin 0 to output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_0; //set output to push pull
}

void delay(uint32_t milis) { //blocking delay for debouncing
	int32_t counter = 0;
	while (counter < milis) {
		if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == SysTick_CTRL_COUNTFLAG_Msk) { //check flag
			counter++; //increment time
		}
	}
}

void SysTick_Handler(void) // this internal interrupt is generated once every 1ms
{
	timeButOne++;
	timeButTwo++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
