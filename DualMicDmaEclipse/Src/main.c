/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "errno.h"
#include "stm32f4xx_hal_tim.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_BUFFER_LENGTH 4000
#define MAXDELAY 2000
#define STARTDELAY -2000
#define TIMEDELAY 0.000003665
uint32_t ADCBuffer1[ADC_BUFFER_LENGTH]={0};

//error errno solution "undefined reference to `__errno'"
int errno;
int* __errno(void){return &errno;}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void ExportValues(uint32_t buffer[])
{
	char bufferprint[50] = {""};
	uint32_t twaalfbit = 4095;

	for(int i=0; i<ADC_BUFFER_LENGTH/2; i++)
	{
		  buffer[i] &= twaalfbit;
		  sprintf(bufferprint,"%05d",(int) buffer[i]);
		  HAL_UART_Transmit(&huart6,(uint8_t*) bufferprint,strlen(bufferprint),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
	}
	  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
	  sprintf(bufferprint,"....Finished....");
	  HAL_UART_Transmit(&huart6,(uint8_t*) bufferprint,strlen(bufferprint),HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
}
void Uart_transmit(uint32_t dma_buffer[])
{
	int i = 0;
	int count = 0;
	char buffer[50] = {"Empty.."};
	uint32_t twaalfbit = 4095;
	//int acht = 8;
	for(i=0; i<ADC_BUFFER_LENGTH;i++)
	{
		  if(i%2 ==0)
		  {

			  dma_buffer[i] &= twaalfbit;
			  sprintf(buffer,/*pin a1:%d ->*/"%05d"   ,(int) dma_buffer[i]);
			  HAL_UART_Transmit(&huart6,(uint8_t*) buffer,strlen(buffer),HAL_MAX_DELAY);
			  count++;
		  }
		  else
		  {
			  dma_buffer[i] &= twaalfbit;
			  sprintf(buffer,"pin a2:%d -> %05d   ",count,(int) dma_buffer[i]);
			  HAL_UART_Transmit(&huart6,(uint8_t*) buffer,strlen(buffer),HAL_MAX_DELAY);
			  count++;
		  }
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
	}
	HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
}
void Uart_array(double array[], int length)
{
	HAL_UART_Transmit(&huart6,(uint8_t*)"...Transmitting...",strlen("...Transmitting..."),HAL_MAX_DELAY);
	char buffer[50] = {""};
	for(int i = 0; i<length; i++)
	{
		  sprintf(buffer,"%f",array[i]);
		  HAL_UART_Transmit(&huart6,(uint8_t*) buffer,strlen(buffer),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
	}
}
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	HAL_GPIO_WritePin(GPIOD,LD4_Pin,GPIO_PIN_SET);//ORANGE
}*/
void Correlate(uint32_t buffer1[], uint32_t buffer2[],double *correlation)
{
	double mean1, mean2, s1, s2, s, denom, r;
	int count = 0;
	int n = ADC_BUFFER_LENGTH/2;
	uint32_t twaalfbit = 4095;
	//mean berekenen
	mean1 = 0;
	mean2 = 0;
	for (int i = 0;i < n;i++)
	{
		buffer1[i] = buffer1[i] & twaalfbit;
		buffer2[i] = buffer2[i] & twaalfbit;
		mean1 += buffer1[i];
		mean2 += buffer2[i];
	}
	mean1 /= n;
	mean2 /= n;

	//denominator
	s1 = 0;
	s2 = 0;
	for (int i = 0;i < n;i++) {
	  s1 += (buffer1[i] - mean1) * (buffer1[i] - mean1);
	  s2 += (buffer2[i] - mean2) * (buffer2[i] - mean2);
	}
	denom = sqrt(s1*s2);

	HAL_UART_Transmit(&huart6,(uint8_t*)"...Denom done...\r\n",strlen("...Denom done...\r\n"),HAL_MAX_DELAY);
	//correlatie reeks

	for (int delay= -4000; delay<4000; delay++)
	{
	  s = 0;
	  for (int i=0; i<n; i++)
	  {
		  //first approach (non-circular)
		 int j = i + delay;
		 if (j < 0 || j >= n)
		 {
			continue;
		 }
		 else
		 {
			s += (buffer1[i] - mean1) * (buffer2[j] - mean2);
		 }
	  }
	  r = s / denom;
	  correlation[count] = r;
	  count++;
	  /* r is the correlation coefficient at "delay" */

	}
}
int ScanHigh(double * correlation)
{
	int top = 5000;
	double value = 0.0;
	for(int i = 0; i<7999; i++)
	{
		if(correlation[i] > value)
		{
			value = correlation[i];
			top = i;
		}
	}
	return top;
}
double berekenHoek(int verschuiving)
{
	int d = verschuiving - 4000;
	int c = 343;
	float x = 0.07;
	double hoek = 500;
	hoek = acos((d * TIMEDELAY * c)/x);
	hoek = hoek * 57.2957795; //radialen omzetten naar graden
	return hoek;
}
void servoMove(double hoek)
{
	int move = 500 + 2.77777 * hoek;
	//__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1210);

	  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,move);
	  HAL_Delay(2000);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//static int __errno = 1;
//static int errno = 1;
//static int _errno = 1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
char tempbuffer[50]= {""};
uint32_t bufferPin1[ADC_BUFFER_LENGTH/2]={0};
uint32_t bufferPin2[ADC_BUFFER_LENGTH/2]={0};
double correlatieArray[8000]={0};
HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0))
	  {
		  //Blink twice then constant on -> recording
		  for(int i = 0; i<5;i++)
		  {
			  HAL_Delay(500);
			  HAL_GPIO_TogglePin(GPIOD,LD6_Pin);//BLUE
		  }
		  //start recording samples
		  for(int i = 0;i<ADC_BUFFER_LENGTH;i++)
		  {
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADCBuffer1[0], ADC_BUFFER_LENGTH);
		  }
		  //signal recording stopped, start uart transmit
		  HAL_GPIO_TogglePin(GPIOD,LD6_Pin);//BLUE
		  HAL_GPIO_TogglePin(GPIOD,LD5_Pin);//RED

		  /*sprintf(tempbuffer,"printcount: %d   ",printCount);
		  HAL_UART_Transmit(&huart6,(uint8_t*) tempbuffer,strlen(tempbuffer),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
		  printCount++;
		  Uart_transmit(ADCBuffer1);*/

		  HAL_Delay(500);
		  int count1 = 0, count2 = 0;
		  for(int i = 0; i<ADC_BUFFER_LENGTH; i++)
		  {
			  if(i%2==0)
			  {
				  bufferPin1[count1] = ADCBuffer1[i];
				  count1++;
			  }
			  else
			  {
				  bufferPin2[count2] = ADCBuffer1[i];
				  count2++;
				  if(count2 >= ADC_BUFFER_LENGTH/2) count2 = 0;
			  }
		  }
		  /*sprintf(tempbuffer,"Microfoon 1 (pin a1)/r/n");
		  HAL_UART_Transmit(&huart6,(uint8_t*) tempbuffer,strlen(tempbuffer),HAL_MAX_DELAY);
		  ExportValues(bufferPin1);
		  sprintf(tempbuffer,"Microfoon 2 (pin a2)/r/n");
		  HAL_UART_Transmit(&huart6,(uint8_t*) tempbuffer,strlen(tempbuffer),HAL_MAX_DELAY);
		  ExportValues(bufferPin2);*/
		  HAL_GPIO_TogglePin(GPIOD,LD5_Pin);//RED
		  HAL_GPIO_TogglePin(GPIOD,LD4_Pin);//Yellow
		  Correlate(bufferPin1, bufferPin2,correlatieArray);
		  //arm_correlate_f32(bufferPin1, ADC_BUFFER_LENGTH/2, bufferPin2, ADC_BUFFER_LENGTH/2, correlatieArray);
		  //arm_correlate_fast_q31(bufferPin1,ADC_BUFFER_LENGTH/2, bufferPin2,ADC_BUFFER_LENGTH/2, correlatieArray);
		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(GPIOD,LD4_Pin);//Yellow
		  HAL_GPIO_TogglePin(GPIOD,LD5_Pin);//RED
		  //drukt correlatie array af
		  //Uart_array(correlatieArray, 8000);
		  //end of operations
		  HAL_Delay(500);
		  HAL_GPIO_TogglePin(GPIOD,LD5_Pin);//RED
		  HAL_GPIO_TogglePin(GPIOD,LD3_Pin);//ORANGE
		  int sweetspot;
		  char special[50];
		  sweetspot = ScanHigh(correlatieArray);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
		  sprintf(special,"sweetspot: %d -> %f  ",sweetspot,correlatieArray[sweetspot]);
		  HAL_UART_Transmit(&huart6,(uint8_t*) special,strlen(special),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"...Done...",strlen("...Done..."),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);
		  double hoek = 500;
		  hoek = berekenHoek(sweetspot);
		  servoMove(hoek);
		  sprintf(special,"Hoek: %f  ",hoek);
		  HAL_UART_Transmit(&huart6,(uint8_t*) special,strlen(special),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"...Done...",strlen("...Done..."),HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart6,(uint8_t*)"\r\n",strlen("\r\n"),HAL_MAX_DELAY);

	  }
	//HAL_ADC_ConvCpltCallback(&hadc1);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

  HAL_TIM_MspPostInit(&htim2);

}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart6);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PB12   ------> I2S2_WS
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_SCK_Pin PC12 */
  GPIO_InitStruct.Pin = I2S3_SCK_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
