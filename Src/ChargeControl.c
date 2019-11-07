
#include "ChargeControl.h"

extern uint16_t LED_Green_Blinking;
extern uint16_t LED_Red_Blinking;
extern uint16_t LED_Yellow_Blinking;
extern uint8_t Switch;

Charging_t Charging;

void Variable_Init(void)
{
	Charging.Check_Bit = 0;
	Charging.Enabled = 0;
	Charging.Error = 0;
	Charging.On_Delay = 0;
	Charging.Bad_Voltage_Blanking = 200;
	Charging.Disconnect_Blanking = 50;
	Charging.Tick = 0;
	Charging.VCHG_Blanking = 200;
}

void Charging_Control(void)
{
	if (Charging.Tick)
	{
		Charging.Tick = 0;

		if (Charging.Enabled == 1)
		{
			if (VOLTTODIGIT_CHG(5.5)> ChargerVoltage() && VOLTTODIGIT_CHG(4.5) < ChargerVoltage())
			{
				BQ_EN_ON;

				if (Charging.On_Delay) Charging.On_Delay--;
				else
				{
					if (!BQCHG) 		// if charging is ongoing
					{
						if (LED_Green_Blinking) LED_Green_Blinking--;
						else
						{
							LED_Green_Blinking = 500;
							LED_GREEN_TOGGLE;
						}
					}
					else if (VOLTTODIGIT_BAT(4.10) < Battery_Voltage_Filtered())		// charging is completed
					{
						LED_GREEN_ON;
						BQ_EN_OFF;
					}
					else		// charging fault
					{
						if (Charging.Disconnect_Blanking) Charging.Disconnect_Blanking--;
						else
						{	
//							Charging.Disconnect_Blanking = 50;

							if (LED_Red_Blinking) LED_Red_Blinking--;
							else
							{
								LED_Red_Blinking = 500;
								if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == 1) LED_RED_ON;
								else LED_RED_OFF;
							}
							BQ_EN_OFF;
						}
						
					}						
				}
			}
			else if (VOLTTODIGIT_CHG(5.5) < ChargerVoltage() || VOLTTODIGIT_CHG(4.5) > ChargerVoltage())
			{
				if (VOLTTODIGIT_CHG(0.2) > ChargerVoltage())		// charger is disconnected
				{
					if (Charging.Disconnect_Blanking) Charging.Disconnect_Blanking--;
					else
					{
						Charging.Disconnect_Blanking = 50;
						BQ_EN_OFF;
						LED_RED_OFF;
						LED_GREEN_OFF;	
						Charging.Enabled = 0;
						Switch = 0;
					}
				}
				else
				{
					if (Charging.Bad_Voltage_Blanking) Charging.Bad_Voltage_Blanking--;
					else
					{	
						if (LED_Red_Blinking) LED_Red_Blinking--;
						else
						{
							LED_Red_Blinking = 500;
							if (HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == 1) LED_RED_ON;
							else LED_RED_OFF;
						}
						BQ_EN_OFF;
//						Charging.Enabled = 0;
					}		
				}				
			}
		}
	}
}


