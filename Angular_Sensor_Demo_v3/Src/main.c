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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_Read_Counter 10
#define ADC_Channel 1
#define ADC_RW_LEN ADC_Read_Counter * ADC_Channel

#define StringRW 64

char uartTxMessage[StringRW];
char printMsg[StringRW];

float   ADC1_FLOAT = 0;
float   ADC2_FLOAT = 0;
float   ADC3_FLOAT = 0;
float   ADC4_FLOAT = 0;
uint32_t ADC1_U32, ADC2_U32, ADC3_U32, ADC4_U32;
uint32_t ADC1_U32_ARR[ADC_RW_LEN] = {0};
uint32_t ADC2_U32_ARR[ADC_RW_LEN] = {0};
uint32_t ADC3_U32_ARR[ADC_RW_LEN] = {0};
uint32_t ADC4_U32_ARR[ADC_RW_LEN] = {0};

uint8_t i;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void get_ADCValue(float *ADC_FLOAT, uint32_t *ADC_U32, uint32_t *ADC_U32_ARR);
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_OPAMP4_Init();
  MX_OPAMP1_Init();

  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_UART5_Init();


  /* USER CODE BEGIN 2 */


  /*---- OPAMP Init ----------------- */
  OPAMP_Calibration();
  OPAMP_Start();

  /*---- ADC Init ----------------- */
  ADC_Calibration();
  HAL_TIM_Base_Start_IT(&htim3);
  

  /*---- DAC Init ----------------- */
  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LED_TOGGLE(LD2_Pin);


    // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)aTxMessage, sizeof(aTxMessage));

    // printfDMA("******** ADC DMA Example ********\r\n");
    // sprintf(printMsg, "Temp value = %1.3f  \r\n", interTemp);

    printf("ADC1 = %6d, %1.3fV \r\n", ADC1_U32, ADC1_FLOAT);
    printf("ADC2 = %6d, %1.3fV \r\n", ADC2_U32, ADC2_FLOAT);
    printf("ADC3 = %6d, %1.3fV \r\n", ADC3_U32, ADC3_FLOAT);
    printf("ADC4 = %6d, %1.3fV \r\n", ADC4_U32, ADC4_FLOAT);
    printf("\r\n");

    HAL_Delay(200);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // timer3 1kHz : HAL_ADC_Start_DMA, HAL_ADC_Stop_DMA
  // timer6 100Hz : DAC_TRIANGLEAMPLITUDE_255
  if (htim->Instance == htim3.Instance)
  {
    // maybe use DMA complete callback event is better than start and stop in timer.
    // In this way, do this method
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1_U32_ARR, ADC_RW_LEN);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&ADC2_U32_ARR, ADC_RW_LEN);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC3_U32_ARR, ADC_RW_LEN);
    HAL_ADC_Start_DMA(&hadc4, (uint32_t*)&ADC4_U32_ARR, ADC_RW_LEN);

    get_ADCValue(&ADC1_FLOAT, &ADC1_U32, ADC1_U32_ARR);
    get_ADCValue(&ADC2_FLOAT, &ADC2_U32, ADC2_U32_ARR);
    get_ADCValue(&ADC3_FLOAT, &ADC3_U32, ADC3_U32_ARR);
    get_ADCValue(&ADC4_FLOAT, &ADC4_U32, ADC4_U32_ARR);

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
    HAL_ADC_Stop_DMA(&hadc4);
  }
}

#define ADC2VOLTAGE 3.3f / 4096
void get_ADCValue(float *ADC_FLOAT, uint32_t *ADC_U32, uint32_t *ADC_U32_ARR)
{
  *ADC_U32 = 0;
  for (uint8_t i = 0; i < ADC_RW_LEN; i++)
  {
    *ADC_U32 += ADC_U32_ARR[i];
  }
  *ADC_U32 /= ADC_Read_Counter;
  *ADC_FLOAT = *ADC_U32 * ADC2VOLTAGE;
}

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
  while (1)
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
