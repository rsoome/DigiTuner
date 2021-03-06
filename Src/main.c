/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019	STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "yin.h"

#define BUFLEN 1024
#define ACCDIVIDER 128
#define SEMITONE_CONST 1.05946309
#define SEMITONE_CONST_INVERSE 0.94387431
#define EVALSAMPLES = 10
#define A4 440
#define NUMBEROFNOTES 12

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

struct LED{
	int pin;
	GPIO_TypeDef* port;
	volatile uint32_t* pwm;
};

struct LED* TIM4ITLEDs[] = {NULL, NULL, NULL, NULL};

const int SER = 6;
const int SER_CLK = 7;
const int SER_STR = 8;
const int AGC = 15;
const int AATT = 16;

int numbers[] = {
		1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2,						//0
		1 << 5 | 1 << 6,																								//1
		1 << 7 | 1 << 6 | 1 << 1 | 1 << 3 | 1 << 4,											//2
		1 << 7 | 1 << 5 | 1 << 6 | 1 << 4 | 1 << 1,											//3
		1 << 6 | 1 << 5 | 1 << 2 | 1 << 1,															//4
		1 << 7 | 1 << 5 | 1 << 4 | 1 << 2 | 1 << 1,											//5
		1 << 1 | 1 << 7 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2,						//6
		1 << 7 | 1 << 6 | 1 << 5,																				//7
		1 << 7 | 1 <<  1 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 5 | 1 << 6,	//8
		1 << 7 | 1 << 6 | 1 << 5 | 1 << 2 | 1 << 1,											//9
		1 << 7 | 1 <<  1 | 1 << 2 | 1 << 3 | 1 << 5 | 1 << 6,						//A
		1 << 1 | 1 << 5 | 1 << 4 | 1 << 3 | 1 << 2,											//b
		1 << 7 | 1 << 4 | 1 << 3 | 1 << 2,															//C
		1 << 3 | 1 << 5 | 1 << 6 | 1 << 4 | 1 << 1,											//d
		1 << 7 | 1 << 2 | 1 << 3 | 1 << 4 | 1 << 1,											//E
		1 << 7 | 1 <<  1 | 1 << 2 | 1 << 3,															//F
		1 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 2 | 1 << 1};						//g

int ptr = 0;
uint16_t adc_val = 0;
int16_t adc_buf[BUFLEN];
uint16_t adc_peak = 0;
uint16_t adc_low = 0xFFFF;
uint8_t bufFull = 0;
uint16_t bufPos = 0;
uint16_t agc_midpoint = 2048;

int notebuffer[3];
float notebuffer_temp[3];

void getLowestOctave(float a4, float minFreq){
	float currentNote = a4;
	int octave = 4;

	while(currentNote >= minFreq * 2){
		currentNote /= 2;
		octave -= 1;
	}
	notebuffer_temp[0] = currentNote;
	notebuffer_temp[1] = octave;
}

void getLowestNote(float freq, float fMin){
	if(fMin < 0) fMin = 0;

	int octave = 0;
	int semitonePos = 9;

	float currentNote = freq;
	float prevNote = currentNote * SEMITONE_CONST_INVERSE;

	while(prevNote >= fMin){
		currentNote = prevNote;
		semitonePos -= 1;

		if(semitonePos < 0){
			octave -= 1;
			semitonePos = NUMBEROFNOTES + semitonePos;
		}

		prevNote = currentNote * SEMITONE_CONST_INVERSE;
	}

	notebuffer_temp[0] = currentNote;
	notebuffer_temp[1] = semitonePos;
	notebuffer_temp[2] = octave;
}

void getLowestFreq(float currentNote, float accuracy, float minFreq){
	float step = accuracy/ACCDIVIDER;

	float prevNote = currentNote * SEMITONE_CONST_INVERSE;
	float currentFreq = currentNote;
	float prevFreq = currentNote;
	int distFromNote = 0;
	int prevDistFromNote = distFromNote;
	int semitonePos = 0;

	float diff = (currentNote - prevNote)*step;
	while(currentFreq >= minFreq){
		prevDistFromNote = distFromNote;
		distFromNote -= accuracy;

		if(distFromNote == -ACCDIVIDER - accuracy){
			semitonePos -= 1;
			distFromNote = ACCDIVIDER - accuracy;
		}

		prevFreq = currentFreq;
		currentFreq -= diff;
	}

	notebuffer_temp[0] = prevFreq;
	notebuffer_temp[1] = prevDistFromNote;
	notebuffer_temp[2] = semitonePos;
}

void getHighestOctave(float a4, float maxFreq){
	float currentNote = a4;
	int octave = 4;

	while(currentNote <= maxFreq * 2){
		currentNote *= 2;
		octave += 1;
	}
	notebuffer_temp[0] = currentNote;
	notebuffer_temp[1] = octave;
}

void getHighestNote(float freq, float fMax){

	int octave = 0;
	int semitonePos = 9;

	float currentNote = freq;
	float nextNote = currentNote * SEMITONE_CONST;

	while(nextNote <= fMax){
		currentNote = nextNote;
		semitonePos += 1;

		if(semitonePos >= NUMBEROFNOTES){
			octave += 1;
			semitonePos = 0;
		}

		nextNote = currentNote * SEMITONE_CONST;
	}

	notebuffer_temp[0] = currentNote;
	notebuffer_temp[1] = semitonePos;
	notebuffer_temp[2] = octave;
}

void getHighestFreq(float currentNote, float accuracy, float maxFreq){
	float step = accuracy/ACCDIVIDER;

	float nextNote = currentNote * SEMITONE_CONST;
	float currentFreq = currentNote;
	float prevFreq = currentNote;
	int distFromNote = 0;
	int prevDistFromNote = distFromNote;
	int semitonePos = 0;

	float diff = (currentNote - nextNote)*step;
	while(currentFreq <= maxFreq){
		prevFreq = currentFreq;
		currentFreq -= diff;
		prevDistFromNote = distFromNote;
		distFromNote -= accuracy;

		if(distFromNote == ACCDIVIDER){
			semitonePos += 1;
			distFromNote *= -1;
		}
	}

	notebuffer_temp[0] = prevFreq;
	notebuffer_temp[1] = prevDistFromNote;
	notebuffer_temp[2] = semitonePos;
}

void getNote(float freq, int accuracy, float a4){

	if(freq < a4){
		getLowestOctave(A4, freq);
		float currentNote = notebuffer_temp[0];
		int octave = notebuffer_temp[1];

		getLowestNote(currentNote, freq);
		currentNote = notebuffer_temp[0];
		int semitonePos = notebuffer_temp[1];
		octave += notebuffer_temp[2];

		getLowestFreq(currentNote, accuracy, freq);
		float foundFreq = notebuffer_temp[0];
		float distFromNote = notebuffer_temp[1];
		semitonePos += notebuffer_temp[2];

		if(semitonePos < 0){
			octave -= 1;
			semitonePos = NUMBEROFNOTES + semitonePos;
		}

		notebuffer[0] = semitonePos;
		notebuffer[1] = octave;
		notebuffer[2] = distFromNote;
	}
	else {
		getHighestOctave(A4, freq);
		float currentNote = notebuffer_temp[0];
		int octave = notebuffer_temp[1];

		getHighestNote(currentNote, freq);
		currentNote = notebuffer_temp[0];
		int semitonePos = notebuffer_temp[1];
		octave += notebuffer_temp[2];

		getHighestFreq(currentNote, accuracy, freq);
		float foundFreq = notebuffer_temp[0];
		float distFromNote = notebuffer_temp[1];
		semitonePos += notebuffer_temp[2];

		if(semitonePos < 0){
			octave -= 1;
			semitonePos = NUMBEROFNOTES + semitonePos;
		}

		notebuffer[0] = semitonePos;
		notebuffer[1] = octave;
		notebuffer[2] = distFromNote;
	}
}
//Quick hack, some amount of delay
void ms_delay(int ms)
{
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0)
         __asm("NOP");
   }
}

void send7seg(const int inchr){
	int chr = inchr;
	GPIOB->ODR &=  (1 << SER_STR)^0xFFFF;
	ms_delay(5);
	for(int i = 0; i < 8; i++){
		int val = chr & 0x01;
		if(val == 0){
			GPIOB->ODR &= (1 << SER)^0xFFFF;
		}
		else{
			GPIOB->ODR |= 1 << SER;
		}
		ms_delay(5);
		GPIOB->ODR |= 1 << SER_CLK;
		chr = chr >> 1;
		ms_delay(5);
		GPIOB->ODR &= (1 << SER_CLK)^0xFFFF;
		ms_delay(5);
	}
}

void ADC_IRQHandler(){
	adc_val = HAL_ADC_GetValue(&hadc1);
	if(bufPos < BUFLEN) adc_buf[bufPos] = adc_val - agc_midpoint;
	if(bufPos + 1 < BUFLEN) bufPos++;
	else{
		bufFull = 1;
		HAL_NVIC_DisableIRQ(TIM3_IRQn);
	}
	if(adc_val > adc_peak) adc_peak = adc_val;
	if(adc_val < adc_low) adc_low = adc_val;
	HAL_NVIC_ClearPendingIRQ(ADC_IRQn);
}

void TIM3_IRQHandler(void) {
	HAL_ADC_Start_IT(&hadc1);
	TIM3->SR = 0;
}

void TIM4_IRQHandler(void) {
	
	int states = (TIM4->SR) & (1 | 1 << 1 | 1 << 2 | 1 << 3 | 1 << 4);

	if((states & 1) == 1){
			for(int i = 0; i < 4; i++){
				struct LED current = *(TIM4ITLEDs[i]);
				if(*(current.pwm)){
					current.port->ODR |= 1 << (*(TIM4ITLEDs[i])).pin;
				}
			}
			TIM4->SR &= (1)^0xFFFF;
		}

	for(int i = 0; i < 4; i++){
		int state = states >> (i + 1);
		state &= 1;
		if(state == 1){
			(*(TIM4ITLEDs[i])).port->ODR &= (1 << (*(TIM4ITLEDs[i])).pin)^0xFFFF;
			TIM4->SR &= (1 << (i+1))^0xFFFF;
		}
	}
}

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
	int NOTES[] = {
			numbers[12],         //C
			numbers[12] | 1,     //C#
			numbers[13],         //D
			numbers[13] | 1,     //D#
			numbers[14],         //E
			numbers[15],         //F
			numbers[15] | 1,     //F#
			numbers[16],         //G
			numbers[16] | 1,     //G#
			numbers[10],         //A
			numbers[10] | 1,     //A#
			numbers[11]          //B
	};
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
	
	//TIM4->DIER = TIM_DIER_UIE;

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
	MX_TIM3_Init();
	
  /* USER CODE BEGIN 2 */

	struct LED red;
	struct LED green;
	struct LED blue;
	
  red.pin = 9;
  red.port = GPIOB;
  red.pwm = &(TIM4->CCR1);
  TIM4ITLEDs[0] = &red;

  blue.pin = 0;
  blue.port = GPIOE;
  blue.pwm = &(TIM4->CCR2);
  TIM4ITLEDs[1] = &blue;
		
  green.pin = 1;
  green.port = GPIOE;
  green.pwm = &(TIM4->CCR3);
  TIM4ITLEDs[2] = &green;
		
  send7seg(numbers[8] | 1);
  send7seg(numbers[8] | 1);
  GPIOB->ODR |= 1 << SER_STR;
  ms_delay(1000);
  send7seg(0);
  send7seg(0);
  GPIOB->ODR |= 1 << SER_STR;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int intensity = 0;
	int mask = 0x8;//F0;
	

	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_SetPriority(TIM4_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);
	//HAL_ADC_Start(&hadc1);
	Yin yin;
	float pitch;
	Yin_init(&yin, BUFLEN, 0.05);
	int validSamples = 0;

	*(red.pwm) = 750;
	ms_delay(1000);
	*(red.pwm) = 0;
	*(blue.pwm) = 750;
	ms_delay(1000);
	*(blue.pwm) = 0;
	*(green.pwm) = 750;
	ms_delay(1000);
	*(green.pwm) = 0;

  while (1)
  {
	if(bufFull){
		pitch = Yin_getPitch(&yin, adc_buf);
		if(pitch > 50){
			getNote(pitch, 1, A4);
			int note = NOTES[notebuffer[0]];
			int octave = numbers[notebuffer[1]];
			float distFromNote = notebuffer[2];
			if(distFromNote > 1){
				*(green.pwm) = 0;
				*(red.pwm) = 0;
				*(blue.pwm) = (volatile uint32_t) ((distFromNote/(ACCDIVIDER/2.0))*1000);
			}
			else if(distFromNote < -1){
				*(green.pwm) = 0;
				*(red.pwm) = (volatile uint32_t) (((-1 * distFromNote)/(ACCDIVIDER/2.0))*1000);
				*(blue.pwm) = 0;
			} else {
				*(green.pwm) = 1000;
				*(red.pwm) = 0;
				*(blue.pwm) = 0;
			}
			send7seg(octave);
			send7seg(note);
			GPIOB->ODR |= 1 << SER_STR;
		}
		else{
			uint16_t g_ADCValue = (uint16_t) pitch;//adc_peak - adc_low;
			uint32_t div = 1000000000;
			int startH = 1;
			int startL = 1;
			if(g_ADCValue == 0){
				startH = 0;
				startL = 0;
			}
			while(div > 0){
			  int lower = g_ADCValue/(div/10);
			  int higher = g_ADCValue/div;
			  if(lower > 0 && startL) startL = 0;
			  if(higher > 0 && startH) startH = 0;
			  if(div/10 > 0 && !startL){
				uint16_t nbr = numbers[lower%10];
				send7seg(nbr);
				} else {
				send7seg(0);
				}
			  if(!startH){
				  send7seg(numbers[higher%10]);
			  } else {
				  send7seg(0);
			  }
			  div /= 10;
				GPIOB->ODR |= 1 << SER_STR;
				ms_delay(150);
			}
			send7seg(0);
			send7seg(0);
			GPIOB->ODR |= 1 << SER_STR;
	}
	bufPos = 0;
	bufFull = 0;
	adc_peak = 0;
	adc_low = 0xFFFF;
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
	__ADC1_CLK_ENABLE();

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	

  /* USER CODE END ADC1_Init 2 */

}

static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */
	
	__HAL_RCC_TIM3_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	int sysclk = HAL_RCC_GetSysClockFreq();
	int sr = YIN_SAMPLING_RATE;
  htim3.Init.Period = HAL_RCC_GetSysClockFreq()/YIN_SAMPLING_RATE;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = HAL_RCC_GetSysClockFreq()/100000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
	
	/* Enable ADC mode one gpio C */
	
	GPIO_InitTypeDef gpioInit;
	gpioInit.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpioInit.Mode = GPIO_MODE_ANALOG;
  gpioInit.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpioInit);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	uint16_t gc_initial_state = 0xFFFF;
	HAL_GPIO_WritePin(GPIOD, gc_initial_state, 1);
	uint8_t gc_temp = gc_initial_state;
	uint8_t resistors_in = 0;
	float initialResistance = 1000;
	for(int i = 0; i < 8; i++){
		if(!(gc_temp & 1)){
			resistors_in++;
		}
		gc_temp >>= 1;
	}
	agc_midpoint = (uint16_t) (agc_midpoint * (1 + 100/(initialResistance/resistors_in)));

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
	send7seg(numbers[14]);
	send7seg(numbers[14]);
	GPIOB->ODR |= 1 << SER_STR;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
