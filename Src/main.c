
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stdlib.h"
#include "stm32f0xx_hal.h"
#include "STM32R_SignalMainSwitch.h"
#include "PCF8563.h"
#include "string.h"
#include "LPF.h"

/* Private macros ------------------------------------------------------------*/

//Ez egy teszt

/* Private types--------------------------------------------------------------*/


typedef enum SystemState_t_
{
	SystemState_SystemInit = 0,
	
	SystemState_WakeUp,
	SystemState_NormalOperation,
	SystemState_Charging,
	SystemState_ToolOff,
	SystemState_Battery_Low,
	
} SystemState_t;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
 
static SystemState_t SystemState;
static volatile uint16_t ADC_Results[5];
static volatile uint16_t ADC_Filtered_Results[5];
uint8_t ChargerCheck_Bit = 0;
uint8_t Charging_Enabled = 0;
uint16_t VBAT_Blanking = 0;
uint16_t VCHG_Blanking = 0;
uint32_t Tick_Main = 0;
uint32_t Charger_Tick = 0;
uint32_t Charger_Check_Tick = 0;
uint32_t BQ_Load_Tick = 0;
uint16_t I2C_Read = 0;
uint16_t UART_Send = 0;
LPF32type Battery_Voltage__Filtered;
LPF32type BatteryPack_Current__Filtered;
LPF32type BatteryPack_Voltage__Filtered;
LPF32type BatteryPack_Temperature__Filtered;
LPF32type ChargerVoltage__Filtered;

PCF8563_Write_Buffer_t PCF8563_Write;
PCF8563_Raw_Buffer_t PCF8563;
PCF8563_BCD_Buffer_t PCF8563_BCD;

char date[] = __DATE__;
char time[] = __TIME__;
char UART_Values[75];

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);

static void WakeUp(void);
static void NormalOperation(void);
static void Charging_Control(void);
static void Shutdown(void);
static void ToolOff(void);
static void SystemUpdateTask(void);
static void Switch_ADC_UpdateTask(void);
static void BatteryLow(void);

static void I2C_RTC_Read(void);
static void UART_OPENLOG_Send(void);

uint16_t BatteryPack_Current(void);
uint16_t ChargerVoltage(void);
uint16_t BatteryPack_Voltage(void);
uint16_t BatteryPack_Temperature(void);
uint16_t Battery_Voltage(void);

uint16_t Battery_Voltage_Filtered(void);
uint16_t Battery_Voltage_Filtered_Result(void);
uint16_t BatteryPack_Current_Filtered(void);
uint16_t BatteryPack_Current_Filtered_Result(void);
uint16_t BatteryPack_Voltage_Filtered(void);
uint16_t BatteryPack_Voltage_Filtered_Result(void);
uint16_t BatteryPack_Temperature_Filtered(void);
uint16_t BatteryPack_Temperature_Filtered_Result(void);
uint16_t ChargerVoltage_Filtered(void);
uint16_t ChargerVoltage_Filtered_Result(void);
uint8_t KeepAliveConditions(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	SystemState = SystemState_SystemInit;
	
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_Results, 5);
    LPF32_Init (&Battery_Voltage__Filtered, 0, 3);
	LPF32_Init (&BatteryPack_Current__Filtered, 0, 1);
	LPF32_Init (&BatteryPack_Voltage__Filtered, 0, 1);
	LPF32_Init (&BatteryPack_Temperature__Filtered, 0, 1);
	LPF32_Init (&ChargerVoltage__Filtered, 0, 3);	

	BQ_EN_OFF;
	ADC_V_EN_ON;

	SystemState = SystemState_WakeUp;

  /* USER CODE END 2 */

	HAL_Delay(500); 		// delay kondik miatt

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
	Switch_ADC_UpdateTask();

	  
  /* USER CODE END WHILE */
	switch(SystemState)
	{
		case SystemState_WakeUp:			WakeUp();				break;
		case SystemState_NormalOperation: 	NormalOperation();		break;
		case SystemState_ToolOff:			ToolOff();				break;
		case SystemState_Battery_Low:		BatteryLow();			break;
		default:							Shutdown();				break;
	}
		

//	if (SystemState != SystemState_WakeUp)
//	{
		SystemUpdateTask();
//	}


  /* USER CODE BEGIN 3 */                                    

  }
  /* USER CODE END 3 */

}


void Switch_ADC_UpdateTask(void)
{
	SMSW_PeriodicUpdate();
	LPF32(&Battery_Voltage__Filtered, Battery_Voltage());
	LPF32(&BatteryPack_Voltage__Filtered, BatteryPack_Voltage());
	LPF32(&BatteryPack_Current__Filtered, BatteryPack_Current());
	LPF32(&BatteryPack_Temperature__Filtered, BatteryPack_Temperature());
	LPF32(&ChargerVoltage__Filtered, ChargerVoltage());
	
	if (Tick_Main)
	{
		Tick_Main = 0;

		if (SystemState != SystemState_Charging)
		{
			if (VOLTTODIGIT_CHG(0.2) < ChargerVoltage())
			{
				if (VCHG_Blanking) VCHG_Blanking--;
				else 
				{
					SHOLD_ON;
					Charging_Enabled = 1;
					SystemState = SystemState_NormalOperation;
				}
			}
			else VCHG_Blanking = 50;
		}
		
		if (SystemState != SystemState_Battery_Low)
		{
			if (VOLTTODIGIT_BAT(BATTERY_LOW_VOLTAGE) > Battery_Voltage())
			{
				if (VBAT_Blanking) VBAT_Blanking--;
				else SystemState = SystemState_Battery_Low;			
			}
			else VBAT_Blanking = 50;	
		}
		
		if (SystemState != SystemState_WakeUp)
		{
			if (KeepAliveConditions() == 0) SystemState = SystemState_ToolOff;
		}
	}
}



void SystemUpdateTask(void)
{	

	
}

	
void WakeUp(void)
{
	if (SMSWState() == 1)		
	{	
		SHOLD_ON;
		Mem_Write();
		HAL_I2C_Mem_Write(&hi2c1, PCF8563_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, &PCF8563_Write.RTCData[0], 8, 50);
		SystemState = SystemState_NormalOperation;
	}
}
	
	
void Charging_Control(void)
{
	if (Charger_Tick)
	{
		if (Charging_Enabled == 1)
		{
			if (VOLTTODIGIT_CHG(5.5)> ChargerVoltage() && VOLTTODIGIT_CHG(4.5) < ChargerVoltage())
			{
				BQ_EN_ON;
				HAL_Delay(200);		
				if (!BQCHG) 
					{
						LED_YELLOW_OFF;
						LED_RED_OFF;
						LED_GREEN_TOGGLE;
						HAL_Delay(500);
					}
					else
					{
						LED_GREEN_ON;
						BQ_EN_OFF;
						SystemState = SystemState_ToolOff; 
					}
				
			}
			else if (VOLTTODIGIT_CHG(5.5) < ChargerVoltage() || VOLTTODIGIT_CHG(4.5) > ChargerVoltage())
			{	
				Charging_Enabled = 0;
			}
		}
	}
}

void NormalOperation(void)
{
	Charging_Control();
	
	if (SMSWState() == 0) SystemState = SystemState_ToolOff;
	if ((BatteryPack_Current_Filtered_Result() > 0 
		|| BatteryPack_Voltage_Filtered_Result() > 0)  
		&& VOLTTODIGIT_CHG(5.5)> ChargerVoltage() 
		&& VOLTTODIGIT_CHG(4.5) < ChargerVoltage())
	{
		BQ_EN_ON;
		HAL_Delay(500);
		if(!BQCHG)
		{
			LED_GREEN_TOGGLE;
			HAL_Delay(500);
		}
		else
		{
			BQ_EN_OFF;
		}
	}

	I2C_RTC_Read();
	UART_OPENLOG_Send();
	Battery_Voltage_Filtered_Result();
	BatteryPack_Current_Filtered_Result();
	BatteryPack_Voltage_Filtered_Result();
	BatteryPack_Temperature_Filtered_Result();
	ChargerVoltage_Filtered_Result();
}

void BatteryLow(void)
{
	LED_GREEN_OFF;
	LED_YELLOW_OFF;
	if(VOLTTODIGIT_BAT(BATTERY_LOW_VOLTAGE))
	{
		SHOLD_OFF;
		LED_RED_TOGGLE;
		HAL_Delay(500);
	}
	else
	{
		SystemState = SystemState_WakeUp;
	}
}

void ToolOff(void)
{
	LED_YELLOW_OFF;
	LED_RED_OFF;
	LED_GREEN_OFF;
	ADC_V_EN_OFF;
	SHOLD_OFF;
	BQ_EN_OFF;
	while(1);
}

void Shutdown(void)
{
	;
}

uint8_t KeepAliveConditions(void)
{
	return 1;
}

void HAL_SYSTICK_Callback(void)
{
	SysTimer_1();
	I2C_Read++;
	UART_Send++;
	Tick_Main++;
	Charger_Tick++;
	Charger_Check_Tick++;
	BQ_Load_Tick++;
}

void I2C_RTC_Read(void)
{
	if(I2C_Read > 1000)
	{	
		I2C_Read = 0;	
		HAL_I2C_Mem_Read(&hi2c1, PCF8563_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, &PCF8563.RTCData[0], 8, 50);
		Mem_Read();
	}		
}

void UART_OPENLOG_Send(void)
{
	if (UART_Send > 1000)
	{

	HAL_ADC_Stop_DMA(&hadc);	
	UART_Send = 0;

	HAL_UART_Transmit(&huart1, (uint8_t*) UART_Values, 	sprintf(UART_Values, " %d. %0.2d. %0.2d. %0.2d:%0.2d:%0.2d \r",\
						PCF8563_BCD.BCD_Data.Years,\
						PCF8563_BCD.BCD_Data.Century_months,\
						PCF8563_BCD.BCD_Data.Days,\
						PCF8563_BCD.BCD_Data.Hours,\
						PCF8563_BCD.BCD_Data.Minutes,\
						PCF8563_BCD.BCD_Data.VL_seconds), 50);

	HAL_UART_Transmit(&huart1, (uint8_t*) UART_Values, sprintf(UART_Values, " Ibat=%d mA Vpack=%d mV Bat_temp=%d Raw A/D Vbat=%d mV \r",\
						BatteryPack_Current_Filtered_Result(),\
						BatteryPack_Voltage_Filtered_Result(),\
						BatteryPack_Temperature(),\
						Battery_Voltage_Filtered_Result()), 50);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_Results, 5);
		
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

 /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* EXTI2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t BatteryPack_Current(void)
{
	return (ADC_Results[0]);
}

uint16_t ChargerVoltage(void)
{
	return (ADC_Results[1]);
}

uint16_t BatteryPack_Voltage(void)
{
	return (ADC_Results[2]);
}

uint16_t BatteryPack_Temperature(void)
{
	return (ADC_Results[3]);
}

uint16_t Battery_Voltage(void)
{
	return (ADC_Results[4]);
}

uint16_t Battery_Voltage_Filtered(void)
{
	return Battery_Voltage__Filtered.X;
}

 uint16_t Battery_Voltage_Filtered_Result(void)
{
	uint16_t Battery_Voltage_Result = (((Battery_Voltage_Filtered()*3300)>>12)*(INV_ADC_BAT_GAIN)); //mV
	return (ADC_Filtered_Results[4] = Battery_Voltage_Result);
}


uint16_t BatteryPack_Current_Filtered(void)
{
	return BatteryPack_Current__Filtered.X;
}

uint16_t BatteryPack_Current_Filtered_Result(void)
{
	uint16_t BatteryPack_Current_Result = ((((((BatteryPack_Current_Filtered()*3300)>>12)-770))*5000))>>8; //mA
	return (ADC_Filtered_Results[0] = BatteryPack_Current_Result);
}

uint16_t BatteryPack_Voltage_Filtered(void)
{
	return BatteryPack_Voltage__Filtered.X;
}
uint16_t BatteryPack_Voltage_Filtered_Result(void)
{
	uint16_t BatteryPack_Voltage_Result = (((BatteryPack_Voltage_Filtered()*3300)>>12)*(INV_ADC_VOLT_GAIN)); //mV
	return (ADC_Filtered_Results[2] = BatteryPack_Voltage_Result);
}

uint16_t BatteryPack_Temperatue_Filtered(void)
{
	return BatteryPack_Temperature__Filtered.X;
}

uint16_t BatteryPack_Temperature_Filtered_Result(void)
{
	uint16_t BatteryPack_Temperature_Result = ((BatteryPack_Temperatue_Filtered()*3300)>>12); //mV
	return (ADC_Filtered_Results[3] = BatteryPack_Temperature_Result);
}

uint16_t ChargerVoltage_Filtered(void)
{
	return ChargerVoltage__Filtered.X;
}

uint16_t ChargerVoltage_Filtered_Result(void)
{
	uint16_t ChargerVoltage_Result = ((ChargerVoltage_Filtered()*3300)>>13); //mV
	return (ADC_Filtered_Results[1] = ChargerVoltage_Result);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
