#include "stdint.h"
#include "main.h"
#include "stm32f0xx_hal.h"

#ifndef __ChargerControl_H__
#define __ChargerControl_H__

void Variable_Init(void);
void Charging_Control(void);

typedef struct 
{

	uint8_t 	Check_Bit;
	uint8_t 	Enabled;
	uint8_t 	Error;
	uint8_t 	Bad_Voltage_Blanking;
	uint16_t 	On_Delay;
	uint16_t 	Disconnect_Blanking;
	uint16_t	VCHG_Blanking;
	uint32_t 	Tick;
	
}Charging_t;


#endif /* __ChargerControl_H__ */
