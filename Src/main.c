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

#include "stm32f1xx_hal.h"
#include "dwt_stm32_delay.h"
#include "stm32_hal_legacy.h"

#include "config.h"
#include "rs485.h"
    
   
uint32_t g_ADCBuffer[ADC_BUFFER_LENGTH];
    
extern tUart data;    
extern uint32_t myAddr;

int sendEvent = 0;    

int readyToReceive = 0;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ADC_HandleTypeDef myADC;
ADC_HandleTypeDef g_AdcHandle;
DMA_HandleTypeDef g_DmaHandle;


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

int transferCallback = 0;
int readCallback = 0;


void SystemClock_Config(void);


static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init();

void ADC_CAL_Start(void);
void ADC_Get_Value(void);
void ADC_Process(void);
void ConfigureADC();

int test=0;
int test1=0;
int timerValue=-1;
static int count = 0;
int realCallback = 0;
int serialData = 0;
uint32_t sendData = 0;
int sentBufferEmpty = 1;
int transferErrorCount = 0;
int writeErrorCount = 0;
int readErrorCount = 0;

uint8_t nTIM5_ADC = 0;
uint8_t ucADC_Event = 0;

uint16_t unpADC_Result[1] = {0};  
uint16_t unpADC_Filtered[1] = {0};

uint32_t g_ADCValue = 0;
uint32_t g_ADCValueDMA[2];
int g_MeasurementNumber = 0;

int ucReceive_Event = 0;
int reqReceived = 0;
int dataReady = 0;
static int led = 0;
uint8_t myChar = 0;
int lastByte = 0;



int ret = 0;
uint32_t errCode[256];


extern uint8_t  ucpRx1Buffer  [RX1BUFFERSIZE]; // ??? ??? ??

void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT);

uint32_t timeout = 1000000;

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
  
  //InitializeTimer();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */  
  
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  //MX_TIM2_Init();
  ConfigureADC();
  //MX_ADC1_Init();
  //ADC_CAL_Start();
 
  //MX_TIM7_Init();
  //init_variable_SCIA();

  
  //if(DWT_Delay_Init())
  //{
    //Error_Handler(); 
  //}


  
  int toggle = 0;
  
  
  
  
  rs485_Init();
  
  
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);

  
   GetAddr();
  
  //HAL_ADC_Start_IT(&g_AdcHandle); 
  //HAL_ADC_Start(&g_AdcHandle);
  
   if ((ret=HAL_ADC_Start_DMA(&g_AdcHandle, g_ADCBuffer, ADC_BUFFER_LENGTH)) != HAL_OK) {
    Error_Handler();
   }
  
  
  RequestRecv();
  
  //if((HAL_UART_Receive_IT(&huart1, (uint8_t *)&myChar, 1)) != HAL_OK) 
    //Error_Handler();
  
  
  uint32_t tickstart = 0U;
  for (;;)
  {
       
      if(dataReady == 0) {
        ProcessInput();     
      }
      else
        __NOP();
      
      if( reqReceived == 1 || 1) {
      
        if(sentBufferEmpty == 1 && dataReady == 1) {
          dataReady = 0;
          
          HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
          HAL_GPIO_WritePin(RS485_ENABLE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
          __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
          SendData();
          HAL_Delay(10);
          
        } else if (sentBufferEmpty && dataReady == 0 ) {
          
          
          __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
          USART_ClearITPendingBit(&huart1, UART_IT_TC);
          
          RequestRecv();
          
          

  
    
        }
           
      }
      
      
      if(myAddr & 0x8)  {          // msb of addr is set
        if(HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_3) == GPIO_PIN_SET) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          
        }
        else {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
         
        }
      }
      else {
        if(HAL_GPIO_ReadPin(ADDR_PORT, ADDR_PIN_3) == GPIO_PIN_SET) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          
        }
        else {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
         
        }
        
      }
     
     count ++;
      
    
  
    
         
  }
  return 0;
}




void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==htim2.Instance) 
  {
    
    nTIM5_ADC ++;
  }
  
  //test =100;
  
}

static void MX_TIM2_Init(void)
{

  
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 35;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if ((ret=HAL_TIM_Base_Init(&htim2)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}




void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if ((ret=HAL_RCC_OscConfig(&RCC_OscInitStruct)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if ((ret=HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
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



static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600; 
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  
  
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if ((ret=HAL_UART_Init(&huart1)) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_RE_PIN | RS485_DE_PIN, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = RS485_RE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_ENABLE_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = RS485_DE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_ENABLE_PORT, &GPIO_InitStruct);
  

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  
  GPIO_InitStruct.Pin = ADDR_PIN_3 | ADDR_PIN_2 | ADDR_PIN_1 | ADDR_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ADDR_PORT, &GPIO_InitStruct);
  
  ///////////// for LED /////////////
  
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  
  ///////////////////////////////////


}


/*
void ConfigureADC()
{
    GPIO_InitTypeDef gpioInit;
 
    __GPIOA_CLK_ENABLE();
      
    __ADC1_CLK_ENABLE();
 
    gpioInit.Pin = GPIO_PIN_1;
    gpioInit.Mode = GPIO_MODE_ANALOG;
    gpioInit.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpioInit);
 
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
 
    ADC_ChannelConfTypeDef adcChannel;
 
    g_AdcHandle.Instance = ADC1;
 
    //g_AdcHandle.Init..ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
    //g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
    g_AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
    g_AdcHandle.Init.ContinuousConvMode = ENABLE;
    g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
    g_AdcHandle.Init.NbrOfDiscConversion = 1;
    //g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_AdcHandle.Init.NbrOfConversion = 1;
    //g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
    //g_AdcHandle.Init.EOCSelection = DISABLE;
 
    //HAL_ADC_Init(&g_AdcHandle);
    
  if (HAL_ADC_Init(&g_AdcHandle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


    adcChannel.Channel = ADC_CHANNEL_1;
    adcChannel.Rank = 1;
    adcChannel.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;//ADC_SAMPLETIME_55CYCLES_5;
    
    


 
    if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}
*/



void ConfigureADC()
{
    
 
  ADC_ChannelConfTypeDef adcChannel;

  g_AdcHandle.Instance = ADC1;

  //g_AdcHandle.Init..ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  //g_AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  g_AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;
  g_AdcHandle.Init.ContinuousConvMode = ENABLE;
  g_AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  g_AdcHandle.Init.NbrOfDiscConversion = 0;
  //g_AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  g_AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  g_AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  g_AdcHandle.Init.NbrOfConversion = 2;
  //g_AdcHandle.Init.DMAContinuousRequests = ENABLE;
  //g_AdcHandle.Init.EOCSelection = DISABLE;

  //HAL_ADC_Init(&g_AdcHandle);

  if (HAL_ADC_Init(&g_AdcHandle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


  adcChannel.Channel = SENSOR1_CHANNEL;
  adcChannel.Rank = 1;
  adcChannel.SamplingTime = SENSOR1_SAMPLING_TIME;//ADC_SAMPLETIME_239CYCLES_5;//ADC_SAMPLETIME_55CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  adcChannel.Channel = SENSOR2_CHANNEL;
  adcChannel.Rank = 2;
  adcChannel.SamplingTime = SENSOR2_SAMPLING_TIME;//ADC_SAMPLETIME_239CYCLES_5;//ADC_SAMPLETIME_55CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&g_AdcHandle, &adcChannel) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/*

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  
  if(huart == &huart1)
  {
    

      //sentBufferEmpty = 0;
      transferCallback ++;
      
      sentBufferEmpty = 1;
      
    //__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);
  
      
    
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  
  
  if(sentBufferEmpty == 0)
    writeErrorCount ++;
  else
    readErrorCount ++;
  
  errCode[transferErrorCount%256] = huart->ErrorCode;
  transferErrorCount ++;
  sentBufferEmpty = 1;
  
  //UART_RxAgain(huart);
}




//USART_ClearITPendingBit(USARTx, USART_IT_TC)

void USART_ClearITPendingBit(UART_HandleTypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  /* The CTS interrupt is not available for UART4 and UART5 */
  if (USART_IT == UART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
  
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->Instance->SR = (uint16_t)~itmask;
}
  

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
  
  uint32_t sum[2]={0, 0};
  int i;
  for(i=0; i<ADC_BUFFER_LENGTH; i++) {
    sum[i%2] += g_ADCBuffer[i];
  }
  g_ADCValueDMA[0] = sum[0]*2/ADC_BUFFER_LENGTH;
  g_ADCValueDMA[1] = sum[1]*2/ADC_BUFFER_LENGTH;
  //__NOP();
}
