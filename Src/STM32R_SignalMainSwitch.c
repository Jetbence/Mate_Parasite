/* Includes ------------------------------------------------------------------*/

#include "stdint.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "STM32R_SignalMainSwitch.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static uint8_t SMSW_SwitchOn_Tick = 0;
uint16_t SMSW_SwitchOn_Counter = 3000;
uint16_t SMSW_IdleOff_Counter = 10000;
uint16_t SMSW_SwitchOff_Counter = 3000;

typedef enum
{
	SMSWState_Released_Startup = 0,
	SMSWState_Released,
	SMSWState_Pressed,
	SMSWState_RePressed
} SMSWState_t;

static SMSWState_t SMSWState_ = SMSWState_Released_Startup;

/* Private function prototypes -----------------------------------------------*/

uint8_t Lock_Button_State(void);

/* Public function definitions -----------------------------------------------*/

/* Private function definitions ----------------------------------------------*/

void SysTimer_1(void)
{
	SMSW_SwitchOn_Tick = 1;
}

/**
 *	\brief	Signal main switch 3 seconds detect function.
 *
 *	\param	None
 *	\return	None
 */
void SMSW_PeriodicUpdate(void)
{
	if (SMSW_SwitchOn_Tick)
	{
		SMSW_SwitchOn_Tick = 0;
		
		switch(SMSWState_)
		{
			case SMSWState_Released_Startup:
				if (SwitchState() == 1)
				{
					if (SMSW_SwitchOn_Counter) SMSW_SwitchOn_Counter--;
					else SMSWState_ = SMSWState_Pressed;
				}			
			break;
			
			case SMSWState_Released:
				if (SwitchState() == 1)
				{
					if (SMSW_SwitchOff_Counter) SMSW_SwitchOff_Counter--;
					else SMSWState_ = SMSWState_RePressed;
					SMSW_IdleOff_Counter = 10000;
				}
#if 0
				else
				{
					if (SMSW_IdleOff_Counter) SMSW_IdleOff_Counter--;
					else SMSWState_ = SMSWState_RePressed;
					SMSW_SwitchOff_Counter = 3000;
				}
#endif
			break;
			
			case SMSWState_Pressed:
				if (SwitchState() == 0) SMSWState_ = SMSWState_Released;

			break;
			
			case SMSWState_RePressed:
				
			break;
		}
	}
}

/**
 *	\brief	Brief description.
 *
 *	\param	None
 *	\return	None
 */
uint8_t SwitchState(void)
{
	if(HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin)) return 1;
	else return 0;
}

/**
 *	\brief	Brief description.
 *
 *	\param	None
 *	\return	None
 */
uint8_t SMSWState(void)
{
	if(SMSWState_ == SMSWState_Pressed || SMSWState_ == SMSWState_Released) return 1;
	else return 0;
}

