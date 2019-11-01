#ifndef TEMPLATE_H_
#define TEMPLATE_H_

/* Includes ------------------------------------------------------------------*/

#include "stdint.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------- */

void SignalMainSwitch_Init(void);
uint8_t SwitchState(void);
void SysTimer_1(void);
void SMSW_PeriodicUpdate(void);
uint8_t SMSWState(void);

#endif /* TEMPLATE_H_ */
