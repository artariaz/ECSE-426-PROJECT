/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const int BUFFER_SIZE = 24000;
const int SEND_BUF_SIZE = 40;
const int REC_TIME = 3; //s

const int order = 2;
const int filterWindowSize = 3;

//float bufferA[BUFFER_SIZE];
//float bufferB[BUFFER_SIZE];
//float filteredA[BUFFER_SIZE];
//float filteredB[BUFFER_SIZE];

uint8_t sendBuffer[SEND_BUF_SIZE];
uint8_t bufferA[BUFFER_SIZE];
uint8_t adcInterruptFlag;
struct FIR_coeff{
		float coefficent[order * 2 + 1];
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int IIR_CMSIS(float* InputArray, float* OutputArray, struct FIR_coeff* coeff, int Length);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//	struct FIR_coeff coeff;	
//	coeff.coefficent[0] = 0.26404913056292794;
//	coeff.coefficent[1] = 0.5280982611258559;
//	coeff.coefficent[2] = 0.26404913056292794;
//	coeff.coefficent[3] = -0.11782701552717954;
//	coeff.coefficent[4] = 0.17402353777889132;	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

	//For displaying the digits
	//Step(1): Start the Timer as interrupt
	HAL_TIM_Base_Start_IT(&htim2);

	uint8_t adcVal;
	int ind;
	int index;

	//Initialize the buffer size
	for(int i = 0; i < BUFFER_SIZE; i++){
		bufferA[i] = 0;
	}
	for(int i = 0; i < SEND_BUF_SIZE; i++){
		sendBuffer[i] = 0;
	}

//	for(int i = 0; i < BUFFER_SIZE; i++){
//		filteredA[i] = 0;
//	}
//	for(int i = 0; i < BUFFER_SIZE; i++){
//		filteredB[i] = 0;
//	}
	
	//To confirm that the sytem began working
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); //Blue LED
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//uint8_t start[5] = {0x73,0x74,0x61,0x72,0x74};
	//HAL_UART_Transmit(&huart3, &start[0], 5, 200);
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){
		
		uint8_t start[5] = {0x73,0x74,0x61,0x72,0x74};
		HAL_UART_Transmit(&huart3, &start[0], 5, 2000);
		HAL_StatusTypeDef status = HAL_TIMEOUT;
		
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); //Red LED

		HAL_Delay(1000);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);	//Red LED
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 		//Orange LED
			
		HAL_Delay(1000);	
	
  	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);	//Orange LED
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 		//Green LED
		
		HAL_ADC_Start(&hadc2);

			ind = 0;	
			while(ind < BUFFER_SIZE){	
				
				while(adcInterruptFlag){
					adcInterruptFlag = 0;
					
					adcVal = HAL_ADC_GetValue(&hadc2);
					bufferA[ind] = adcVal;
					ind++;
				}

			}
			
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 		//Green LED
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET); 		//Orange LED			
//		//FILTERING	
//			
//		for(int i = 0; i < BUFFER_SIZE; i++){
//			filterIn[i] = (float)bufferA[i];
//		}
//					
//		IIR_CMSIS(filterIn, filterOut, &coeff, BUFFER_SIZE);
//		
//		for(int i = 0; i < BUFFER_SIZE; i++){
//			bufferA[i] = (uint8_t)filterOut[i];
//		}
//			
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); 		//Orange LED			
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);	//Red LED
			
		for(int k=0;k<BUFFER_SIZE; k+=SEND_BUF_SIZE)
		{
			for(int j=0; j<SEND_BUF_SIZE; j++){
				sendBuffer[j] = bufferA[k+j];
			}
			
			while(status != HAL_OK){
				status = HAL_UART_Transmit(&huart3, &sendBuffer[0], SEND_BUF_SIZE, 2000);
			}
			HAL_Delay(10);

			status = HAL_TIMEOUT;
		}	
					
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); 		//Red LED
		
		HAL_ADC_Stop(&hadc2);

/*
			for(int i=0;i<REC_TIME*3;i++){
				
				for(int i = 0; i < BUFFER_SIZE; i++){
					adcVal = HAL_ADC_GetValue(&hadc2);
					bufferA[i] = adcVal;
					//TODO SEND BY UART

//					if (adcVal > 0){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//					}
//					if (adcVal > 100){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//					}
//					if (adcVal > 200){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//					}
//					if (adcVal < 1){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//					}
//					if (adcVal < 101){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//					}
//					if (adcVal < 201){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//					}

				}
				for(int i = 0; i < BUFFER_SIZE; i++){
					adcVal = HAL_ADC_GetValue(&hadc2);
					bufferB[i] = adcVal;
					//TODO SEND BY UART
//					if (adcVal > 0){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//					}
//					if (adcVal > 100){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
//					}
//					if (adcVal > 200){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//					}
//					if (adcVal < 1){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//					}
//					if (adcVal < 101){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
//					}
//					if (adcVal < 201){
//						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//					}
				}	
			}
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 		//Green LED
			
		if(bufferA[1] == bufferA[15]){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		}
		if(bufferA[7] == bufferB[2]){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}
		if(bufferB[9] == bufferB[13]){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		}
*/			
		}
		

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
//int IIR_CMSIS(float* InputArray, float* OutputArray, struct FIR_coeff* coeff, int Length){
//	
//	//Initialize the variables required for the arm biquad filter
//	arm_biquad_casd_df1_inst_f32 S;
//	float stateArray[4] = {0,0,0,0};
//	S.numStages = 1;
//	S.pCoeffs = coeff ->coefficent;
//	S.pState = stateArray;
//	
//	//Call arm biquad filter
//	arm_biquad_cascade_df1_f32(&S,InputArray,OutputArray,Length);
//	
//	return 0;
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
