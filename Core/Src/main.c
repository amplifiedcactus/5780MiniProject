
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

//GLOBAL VARIABLES
int sequence[8] = {60, 60, 60, 60, 60, 60, 60, 60}; //Sequence array of 8 notes
int velocityArray[8] = {0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40};
int sequenceCount = 0; //Integer for counting through sequence array
int sequenceCount16 = 0; //Integer for counting through sequence array with note off commands
int lastCommand = 0; //This variable holds the last transmitted MIDI command for implementing running status
int noteMapping[6] = {0x3C, 60, 62, 64, 67, 69}; //array to map button indices to MIDI note values
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

//This function sets up the GPIOC pins for using the LEDs
void setupLED(void) {
	// Initialize pins PC6 (red), PC7 (blue), PC8 (orange), PC9 (green)
	// Set to general purpose output mode
	GPIOC->MODER |= (1 << 18) | (1 << 16) | (1 << 14) | (1 << 12);
	GPIOC->MODER &= ~((1 << 19) |(1 << 17) |(1 << 15) | (1 << 13));
	// Set to push pull output type
	GPIOC->OTYPER &= ~(0x000FF000); //0000 0000 0000 1111 1111 0000 0000 0000
	// Set to low speed
	GPIOC->OSPEEDR &= ~(0x000FF000);
	// Set to no pullup/down resistor
	GPIOC->PUPDR &= ~(0x000FF000);
	//To turn on LEDs:
	//GPIOC->ODR |= (1 << 7); //Turn on blue LED
	//To toggle LEDs:
	//GPIOC->ODR ^= (1 << 8); //Toggle orange LED
	//To turn off LEDs:
	//GPIOC->ODR &= ~(1 << 6); //Turn off red LED
	//GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
}

//This function sets up the USART3 peripheral for use as a MIDI data output
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

//set up pin PC0 as analog input 10 for ADC to read potentiometer values
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

//This is a function for reading the ADC value
uint32_t readADC(void) {
    // Start ADC conversion
    ADC1->CR |= ADC_CR_ADSTART;
    // Wait for conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    return ADC1->DR; // Read ADC value
}

//This function adds notes to the sequence array
void addNoteToSequence(int button, int velocity) {
    if (button >= 0 && button < sizeof(noteMapping) / sizeof(noteMapping[0])) { //check if the button index is within the array of note mapping
        int noteValue = noteMapping[button]; //fetch note values
        sequence[sequenceCount] = noteValue;
				velocityArray[sequenceCount] = velocity;
        sequenceCount = (sequenceCount + 1) % 8; //ensuring it stays within the sequence array, 8 in our case
    }
}



//Timer 2 interrupt handler for sequencer
void TIM2_IRQHandler(void) {
	//Send sequence note to MIDI interface
	//if sequence count is even, send note on command. If it is odd, send note off command
	if (sequenceCount16 % 2)
		sendMIDI(0x90, sequence[sequenceCount16/2], velocityArray[sequenceCount16/2]);
	else
		sendMIDI(0x90, sequence[(sequenceCount16-1)/2], 0);
	
	//Add to sequence count, if it is 16, go back to 0
	if (sequenceCount16 > 15)
		sequenceCount16 = 0;
	sequenceCount16 = sequenceCount16 + 1;
	
	//Update ARR value based on ADC input to change tempo
	int read = readADC();
	if (read > 0)
		TIM2->ARR = read*2;
	
	NVIC->ICPR[0U]	|= (1 << 0); //clear NVIC pending flag
	TIM2->SR &= ~(1 << 0); //clear update interrupt flag
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
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable peripheral clock to GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable peripheral clock to USART3
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //Enable peripheral clock to Timer 2
	
	//call setup functions
	setupUART3();
	setupADC();
	setupLED();
	
	//set up button inputs
	GPIOA->PUPDR |= 0xAAA8;
	GPIOC->PUPDR |= (1 << 3) | (1 << 5);
	GPIOB->MODER &= ~0xF;
	GPIOB->OTYPER &= ~0x3;
	
	//set up timer 2
	TIM2->PSC = 7999; //Set PSC to 7999 to Set frequency to 4Hz 
	TIM2->ARR = 250; //Set auto-reload register to 250 to set frequency to 4Hz
	TIM2->DIER |= (1 << 0); //Enable Update interrupt
	TIM2->CR1 |= (1 << 0); //Set CEN bit to enable counter

	
	//Initialize 4 byte array for note on command
	//Most significant byte is command, then channel, then note, then velocity
	int noteOnCommand = 0x90;
	int noteOffCommand = 0x80;//note off
	int note = 0x40;
	int velocityOn = 0x40;
	int velocityOff = 0x00;
	int volCommand = 0xB0;
	int volCtrl = 0x07;
	
	uint32_t debouncer = 0; //Debouncer variable for button 0

	//Variables for debouncing code
	uint8_t prevState[6] = {0}; // One for each button
	uint8_t currentState;
	uint8_t debounceDelay = 5; // Debounce delay - play with this
	uint8_t ADCState; //State for potentiometer
	uint8_t prevADCState = 0; 
	

	int switchMode = 0; //This is used to break out of two loops to go to Play mode
	
	//Main infinite while loop
	while (1) {
		
		//Pause mode while loop
		while (1) {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)); //turn off other LEDs
			GPIOC->ODR |= (1 << 6); //Turn on red LED to show that Pause mode is active
			
			NVIC_DisableIRQ (TIM2_IRQn); //disable NVIC interrupt to stop sequencer

			// Loop over each button and check its state
			for (int button = 0; button < 6; button++) {
				 switch (button) {
					case 0: currentState = GPIOA->IDR & (1 << 0); break; // Original button
					case 1: currentState = GPIOA->IDR & (1 << 1); break; // Have to check for actual pins on GPIOA
					case 2: currentState = GPIOC->IDR & (1 << 1); break;
					case 3: currentState = GPIOC->IDR & (1 << 2); break;
					case 4: currentState = GPIOA->IDR & (1 << 4); break;
					case 5: currentState = GPIOA->IDR & (1 << 5); break;
				}
				if (currentState != prevState[button]) {
					HAL_Delay(debounceDelay); // Debounce - Change from HAL delay
					// Recheck state after delay
					switch (button) {
							case 0: currentState = GPIOA->IDR & (1 << 0); break;
							case 1: currentState = GPIOA->IDR & (1 << 1); break; // Repeat for rechecking
							case 2: currentState = GPIOC->IDR & (1 << 1); break;
							case 3: currentState = GPIOC->IDR & (1 << 2); break;
							case 4: currentState = GPIOA->IDR & (1 << 4); break;
							case 5: currentState = GPIOA->IDR & (1 << 5); break;
					}
					if (currentState != prevState[button]) {
						prevState[button] = currentState;
						//int note = NOTE_BUTTON_1 + button; // Assign the note based on button index
						if (currentState) {
								// Button pressed
								if (button == 0) {
									switchMode = 1;
									break;
								}
								sendMIDI(noteOnCommand, noteMapping[button], velocityOn);
								addNoteToSequence(button, readADC()/4);
						} 
						else {
								// Button released
								sendMIDI(noteOnCommand, noteMapping[button], velocityOff);
						}
					}
				}
			}
			 
			
			//read ADC and change velocity
			velocityOn = readADC()/4;

			//switch modes when button is pressed
			if (switchMode == 1){	
				switchMode = 0;
				break;
			}
		}
		
		HAL_Delay(200); //pause between play and pause mode
		
		//Play while loop
		while (1) {
			GPIOC->ODR &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9)); //turn off other LEDs
			GPIOC->ODR |= (1 << 9); //Turn on green LED to show that play mode is active
			
			
			NVIC_EnableIRQ (TIM2_IRQn); //enable NVIC interrupt for sequencer
			NVIC_SetPriority (TIM2_IRQn, 3); //set EXTI0 interrupt priority to 3

			//Break out of while loop if button 0 is pressed:
			debouncer = (debouncer << 1); // Always shift every loop iteration
			if (GPIOA->IDR & (1 << 0)) { // If input signal is set/high
			debouncer |= 0x01; // Set lowest bit of bit-vector
			}
			if (debouncer == 0xFFFFFFFF) {
			// This code triggers repeatedly when button is steady high!
			}
			if (debouncer == 0x00000000) {
			// This code triggers repeatedly when button is steady low!
			}
			if (debouncer == 0x7FFFFFFF) {
			// This code triggers only once when transitioning to steady high!
				break;
			}
		}
	}
}
		
  /* USER CODE END 3 */


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