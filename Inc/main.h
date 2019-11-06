/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "stdint.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define PWR2(EXP)				(1ul << (EXP))

/** Position of result in ADC_Results[]. */
 #define ADC_RESULT_POS_CUR					0
 #define ADC_RESULT_POS_CHG					1
 #define ADC_RESULT_POS_VOLT				2
 #define ADC_RESULT_POS_TEMP				3
 #define ADC_RESULT_POS_BAT					4
 
 #define ADC__FILTERED_RESULT_POS_CUR		0
 #define ADC_FILTERED_RESULT_POS_CHG		1
 #define ADC_FILTERED_RESULT_POS_VOLT		2
 #define ADC_FILTERED_RESULT_POS_TEMP		3
 #define ADC_FILTERED_RESULT_POS_BAT		4

 #define ADC_REFERENCEVOLTAGE 		(3.3)
 #define ADC_RESOLUTION 			(12)
 #define CHG_ADC_STEPS 				PWR2(ADC_RESOLUTION)

 #define BATTERY_LOW_VOLTAGE			3.0 	// eddig merülhet az akksi
	 
/**		Gain of measurement analog path.
 *		[V/V] */
 #define  ADC_CUR_GAIN				(1+(249/1.5))			//167
 #define  ADC_CHG_GAIN				(10.0/(10.0+10.0))		//0.5
 #define  ADC_VOLT_GAIN				(10.0/(62.0+10.0))		//0.1388
 #define  ADC_TEMP_GAIN				1
 #define  ADC_BAT_GAIN				(22.0/(9.1+22.0))		//0.7073
 #define  INV_ADC_BAT_GAIN			((22+9.1)/22)
 #define  INV_ADC_VOLT_GAIN			((62+10)/10)

#define VOLTTODIGIT_CUR(V)						(int32_t)((float)(V)/(ADC_REFERENCEVOLTAGE/(CHG_ADC_STEPS*ADC_CUR_GAIN)))
#define VOLTTODIGIT_CHG(V)						(int32_t)((float)(V)/(ADC_REFERENCEVOLTAGE/(CHG_ADC_STEPS*ADC_CHG_GAIN)))
#define VOLTTODIGIT_VOLT(V)						(int32_t)((float)(V)/(ADC_REFERENCEVOLTAGE/(CHG_ADC_STEPS*ADC_VOLT_GAIN)))
#define VOLTTODIGIT_TEMP(V)						(int32_t)((float)(V)/(ADC_REFERENCEVOLTAGE/(CHG_ADC_STEPS*ADC_TEMP_GAIN)))
#define VOLTTODIGIT_BAT(V)						(int32_t)((float)(V)/(ADC_REFERENCEVOLTAGE/(CHG_ADC_STEPS*ADC_BAT_GAIN)))


#define SHOLD_Pin 				GPIO_PIN_5
#define SHOLD_GPIO_Port 		GPIOB

#define LED1_Pin 				GPIO_PIN_11
#define LED1_GPIO_Port 			GPIOA
#define LED2_Pin 				GPIO_PIN_12
#define LED2_GPIO_Port 			GPIOA
#define LED3_Pin 				GPIO_PIN_15
#define LED3_GPIO_Port 			GPIOA
#define BQ_EN_Pin				GPIO_PIN_4
#define BQ_EN_GPIO_Port			GPIOA
#define BQCHG_Pin				GPIO_PIN_3
#define BQCHG_GPIO_Port			GPIOA
#define SW_Pin					GPIO_PIN_1
#define SW_GPIO_Port			GPIOB
#define ADC_V_EN_Pin			GPIO_PIN_0
#define ADC_V_EN_GPIO_Port		GPIOB

// PB5 (out)  -> uC selfhold
#define SHOLD_ON			SHOLD_GPIO_Port->BSRR = SHOLD_Pin
#define SHOLD_OFF			SHOLD_GPIO_Port->BRR  = SHOLD_Pin

#define BQ_EN_ON			BQ_EN_GPIO_Port->BRR = BQ_EN_Pin
#define BQ_EN_OFF			BQ_EN_GPIO_Port->BSRR= BQ_EN_Pin

#define BQCHG 				HAL_GPIO_ReadPin(BQCHG_GPIO_Port, BQCHG_Pin)
#define SW_READ	  			HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)

#define ADC_V_EN_ON			ADC_V_EN_GPIO_Port->BSRR = ADC_V_EN_Pin
#define ADC_V_EN_OFF		ADC_V_EN_GPIO_Port->BRR  = ADC_V_EN_Pin
		
// PA11 (out)  -> LED1
#define LED_RED_TOGGLE		LED1_GPIO_Port->ODR  ^= LED1_Pin
#define LED_RED_ON			LED1_GPIO_Port->BRR  = LED1_Pin		//	set pin to "1" 
#define LED_RED_OFF			LED1_GPIO_Port->BSRR = LED1_Pin		//  reset pin to "0"
// PA12 (out)  -> LED2
#define LED_YELLOW_TOGGLE	LED2_GPIO_Port->ODR  ^= LED2_Pin
#define LED_YELLOW_ON		LED2_GPIO_Port->BRR  = LED2_Pin		//	set pin to "1" 
#define LED_YELLOW_OFF		LED2_GPIO_Port->BSRR = LED2_Pin		//  reset pin to "0"
// PA15 (out)  -> LED3
#define LED_GREEN_TOGGLE	LED3_GPIO_Port->ODR  ^= LED3_Pin
#define LED_GREEN_ON		LED3_GPIO_Port->BRR  = LED3_Pin		//	set pin to "1" 
#define LED_GREEN_OFF		LED3_GPIO_Port->BSRR = LED3_Pin		//  reset pin to "0"

uint16_t BatteryPack_Current(void);
uint16_t ChargerVoltage(void);
uint16_t BatteryPack_Voltage(void);
uint16_t BatteryPack_Temperature(void);
uint16_t Battery_Voltage(void);

uint16_t Battery_Voltage_Filtered(void);
uint16_t Battery_Voltage_Filtered_Result(void);
uint32_t BatteryPack_Current_Filtered(void);
uint32_t BatteryPack_Current_Filtered_Result(void);
uint16_t BatteryPack_Voltage_Filtered(void);
uint16_t BatteryPack_Voltage_Filtered_Result(void);
uint16_t BatteryPack_Temperature_Filtered(void);
uint16_t BatteryPack_Temperature_Filtered_Result(void);
uint16_t ChargerVoltage_Filtered(void);
uint16_t ChargerVoltage_Filtered_Result(void);
uint8_t KeepAliveConditions(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
