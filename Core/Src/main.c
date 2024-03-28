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
char r = 0; //This is for the character typed into the terminal
int usart_flag = 0; //Flag for if a character has been read by UART3

int lastCommand = 0; //This variable holds the last transmitted MIDI command for implementing running status
// MIDI notes for each button
#define NOTE_BUTTON_1 0x40 // E3
#define NOTE_BUTTON_2 0x41 // F3
#define NOTE_BUTTON_3 0x42 // F#3(Gb3)
#define NOTE_BUTTON_4 0x43 // G3
#define NOTE_BUTTON_5 0x44 // G#3(Ab3)
#define NOTE_BUTTON_6 0x45 // A3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint32_t readADC(void); // Function to read ADC value
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setupUART3(void) {
	
	// Set PC4 (USART3_TX) and PC5 (USART3_RX) to alternate function mode
	GPIOC->MODER |= (1 << 11) | (1 << 9);
	GPIOC->MODER &= ~((1 << 10) | (1 << 8));
	// Set to push pull output type
	GPIOC->OTYPER &= ~((1 << 5) | (1 << 4));
	// Set to low speed
	GPIOC->OSPEEDR &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8));
	
	//Select AF1 for PC4 and PC5
	GPIOC->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL4_Pos;
	GPIOC->AFR[0] |= 0x01 << GPIO_AFRL_AFSEL5_Pos;
	
	//set baud rate to about 31250
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 31250;
	
	//enable transmitter and receiver
	USART3->CR1 |= (1 << 3) | (1 << 2);
	
	//enable RXNE interrupt
	USART3->CR1 |= (1 << 5);
	NVIC_EnableIRQ (USART3_4_IRQn); //enable NVIC interrupt
	NVIC_SetPriority (USART3_4_IRQn, 3);
	
	//enable USART 3
	USART3->CR1 |= (1 << 0);
}

//This function takes a command (x), note (y), and velocity (z) as inputs and transmits them to the UART interface
void sendMIDI(int x, int y, int z) {
	
	
	//If last command is not same as previous (running status), send command once transmit register is empty
	if (x != lastCommand) {
		while (1) {
			if (USART3->ISR & (1 << 7)) {
				break;
			}
		}
		USART3->TDR = x;
	}
	
	//send note once transmit register is empty
	while (1) {
		if (USART3->ISR & (1 << 7)) {
			break;
		}
	}
	USART3->TDR = y;
	
	//send velocity once transmit register is empty
	while (1) {
		if (USART3->ISR & (1 << 7)) {
			break;
		}
	}
	USART3->TDR = z;
	
	//update last command variable
	lastCommand = x;
}

//This is the USART3 interrupt handler that puts the read character into char r and sets the usart_flag
void USART3_4_IRQHandler(void) {
	r = USART3->RDR;
	usart_flag = 1;
}


//set up pin PC0 as analog input 10 for ADC
void setupADC(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; //Enable peripheral clock to ADC1
	//Set to analog mode
	GPIOC->MODER |= (1 << 1) | (1 << 0);
	// Set to push pull output type
	GPIOC->OTYPER &= ~(1 << 0);
	// Set to low speed
	GPIOC->OSPEEDR &= ~(1 << 0);
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~((1 << 1) | (1 << 0));
	
	//Set ADC to 8 bit resolution, continuous conversion, hardware triggers disabled
	ADC1->CFGR1 |= (1 << 13) | (1 << 4);
	ADC1->CFGR1 &= ~((1 << 11) | (1 << 10) | (1 << 3));
	//Select channel 10
	ADC1->CHSELR |= (1 << 10);
	
	//Perform self-calibration
	ADC1->CR |= (1 << 31);
	//wait for self-calibration to complete
	while((ADC1->CR & (1 << 31)) != 0) {
		HAL_Delay(100);
	}
	//Enable ADC peripheral
	ADC1->CR |= (1 << 0);
	//wait until ADC is ready
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
		HAL_Delay(100);
	}
	//start ADC by setting ADSTART bit
	ADC1->CR |= (1 << 2);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	

	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable peripheral clock to GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable peripheral clock to GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	
	setupUART3();
	setupADC();
	
	
	//Initialize 4 byte array for note on command
	//Most significant byte is command, then channel, then note, then velocity
	int noteOnCommand = 0x90;
	int noteOffCommand = 0x80;//note off
	int note = 0x40;
	int velocityOn = 0x40;
	int velocityOff = 0x00;

	
		
		
//	Note On and Off are each comprised of three bytes

//	0xnc, 0xkk, 0xvv
//	Where

//	n is the command (note on (0x9) or off(0x8))
//	c is the channel (1 to 16)
//	kk is the key number (0 to 127, where middle C is key number 60)
//	vv is the striking velocity (0 to 127)
		
		
		
	//	uint8_t prevState = 0;
    //uint8_t currentState = 0;
    //uint8_t debounceDelay = 50; // Debounce delay
		uint8_t prevState[6] = {0}; // One for each button
    uint8_t currentState;
    uint8_t debounceDelay = 50; // Debounce delay

		

      while (1) {
    // Loop over each button and check its state
    for (int button = 0; button < 6; button++) {
       switch (button) {
        case 0: currentState = GPIOA->IDR & (1 << 0); break; // Original button
        case 1: currentState = GPIOA->IDR & (1 << 1); break; // Have to check for actual pins on GPIOA
        case 2: currentState = GPIOA->IDR & (1 << 2); break;
        case 3: currentState = GPIOA->IDR & (1 << 3); break;
        case 4: currentState = GPIOA->IDR & (1 << 4); break;
        case 5: currentState = GPIOA->IDR & (1 << 5); break;
    }

    if (currentState != prevState[button]) {
        HAL_Delay(debounceDelay); // Debounce
        // Recheck state after delay
        switch (button) {
            case 0: currentState = GPIOA->IDR & (1 << 0); break;
            case 1: currentState = GPIOA->IDR & (1 << 1); break; // Repeat for rechecking
            case 2: currentState = GPIOA->IDR & (1 << 2); break;
            case 3: currentState = GPIOA->IDR & (1 << 3); break;
            case 4: currentState = GPIOA->IDR & (1 << 4); break;
            case 5: currentState = GPIOA->IDR & (1 << 5); break;
        }

            if (currentState != prevState[button]) {
                prevState[button] = currentState;
                int note = NOTE_BUTTON_1 + button; // Assign the note based on button index
                if (currentState) {
                    // Button pressed
                    sendMIDI(noteOnCommand, note, velocityOn);
                } else {
                    // Button released
                    sendMIDI(noteOffCommand, note, velocityOff);
                }
            }
        }
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