#ifndef __PCF8563_H__
#define __PCF8563_H__

#include "stdint.h"
#include "main.h"
#include "stm32f0xx_hal.h"

#define PCF8563_ADDRESS			0xA2

#define BUILD_YEAR_CH0 (__DATE__[ 7])
#define BUILD_YEAR_CH1 (__DATE__[ 8])
#define BUILD_YEAR_CH2 (__DATE__[ 9])
#define BUILD_YEAR_CH3 (__DATE__[10])

#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')


#define BUILD_MONTH_CH0 \
    ((BUILD_MONTH_IS_OCT || BUILD_MONTH_IS_NOV || BUILD_MONTH_IS_DEC) ? '1' : '0')

#define BUILD_MONTH_CH1 \
    ( \
        (BUILD_MONTH_IS_JAN) ? '1' : \
        (BUILD_MONTH_IS_FEB) ? '2' : \
        (BUILD_MONTH_IS_MAR) ? '3' : \
        (BUILD_MONTH_IS_APR) ? '4' : \
        (BUILD_MONTH_IS_MAY) ? '5' : \
        (BUILD_MONTH_IS_JUN) ? '6' : \
        (BUILD_MONTH_IS_JUL) ? '7' : \
        (BUILD_MONTH_IS_AUG) ? '8' : \
        (BUILD_MONTH_IS_SEP) ? '9' : \
        (BUILD_MONTH_IS_OCT) ? '0' : \
        (BUILD_MONTH_IS_NOV) ? '1' : \
        (BUILD_MONTH_IS_DEC) ? '2' : \
        /* error default */    '?' \
    )

#define BUILD_DAY_CH0 ((__DATE__[4] >= '0') ? (__DATE__[4]) : '0')
#define BUILD_DAY_CH1 (__DATE__[ 5])

#define BUILD_HOUR_CH0 (__TIME__[0])
#define BUILD_HOUR_CH1 (__TIME__[1])

#define BUILD_MIN_CH0 (__TIME__[3])
#define BUILD_MIN_CH1 (__TIME__[4])

#define BUILD_SEC_CH0 (__TIME__[6])
#define BUILD_SEC_CH1 (__TIME__[7])

struct MyStruct
{
	uint8_t VL_seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Days;
	uint8_t Weekdays;
	uint8_t Century_months;
	uint16_t Years;
	
	uint8_t YearCH0;
	uint8_t YearCH1;
	uint8_t YearCH2;
	uint8_t YearCH3;
	
	uint8_t MonthCH0;
	uint8_t MonthCH1;
	
	uint8_t DayCH0;
	uint8_t DayCH1;
	
	uint8_t HourCH0;
	uint8_t HourCH1;
	uint8_t MinCH0;
	uint8_t MinCH1;
	uint8_t SecCH0;
	uint8_t SecCH1;	
};

#ifndef PCF8563_Raw_Data

typedef struct
{
	uint8_t VL_seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Days;
	uint8_t Weekdays;
	uint8_t Century_months;
	uint16_t Years;
} PCF8563_Raw_Data_t;

#endif

#ifndef PCF8563_BCD_Data

typedef struct
{
	uint16_t Years;
	uint8_t Century_months;
	uint8_t Weekdays;
	uint8_t Days;
	uint8_t Hours;
	uint8_t Minutes;
	uint8_t VL_seconds;
} PCF8563_BCD_Data_t;

#endif 

#ifndef PCF8563_Write_Data 

typedef struct
{
	uint8_t VL_seconds;
	uint8_t Minutes;
	uint8_t Hours;
	uint8_t Days;
	uint8_t Weekdays;
	uint8_t Century_months;
	uint16_t Years;
} PCF8563_Write_Data_t;

#endif

#ifndef PCF8563_Raw_Buffer_t

typedef union
{
	PCF8563_Raw_Data_t Raw_Data;
	uint8_t RTCData[sizeof(PCF8563_Raw_Data_t)];
} PCF8563_Raw_Buffer_t;

#endif

#ifndef PCF8563_BCD_Buffer_t

typedef union
{
	PCF8563_BCD_Data_t BCD_Data;
	uint8_t RTCData[sizeof(PCF8563_BCD_Data_t)];
} PCF8563_BCD_Buffer_t;

#endif

#ifndef PCF8563_Write_Buffer_t

typedef union
{
	PCF8563_Write_Data_t Write_Data;
	uint8_t RTCData[sizeof(PCF8563_Write_Data_t)];
} PCF8563_Write_Buffer_t;

#endif

void Mem_Write(void);
void Mem_Read(void);

#endif
