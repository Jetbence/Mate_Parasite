#include "main.h"
#include "stdlib.h"
#include "stm32f0xx_hal.h"
#include "PCF8563.h"

char RTC_printf[50];
char bcd2bin(char bcd_value);

int8_t bin2bcd(int8_t value);

struct MyStruct StringToData;

extern PCF8563_Write_Buffer_t PCF8563_Write;
extern PCF8563_Raw_Buffer_t PCF8563;
extern PCF8563_BCD_Buffer_t PCF8563_BCD;

void Mem_Write(void)
{
			StringToData.YearCH0 = BUILD_YEAR_CH0-48;
			StringToData.YearCH1 = BUILD_YEAR_CH1-48;
			StringToData.YearCH2 = BUILD_YEAR_CH2-48;
			StringToData.YearCH3 = BUILD_YEAR_CH3-48;
			
			StringToData.Years = (uint16_t)StringToData.YearCH0*1000+StringToData.YearCH1+StringToData.YearCH2*10+StringToData.YearCH3;
			
			StringToData.MonthCH0 = BUILD_MONTH_CH0-48;
			StringToData.MonthCH1 = BUILD_MONTH_CH1-48;
	
			StringToData.Century_months = (uint8_t)StringToData.MonthCH0*10+StringToData.MonthCH1;
	
			StringToData.DayCH0 = BUILD_DAY_CH0-48;
			StringToData.DayCH1 = BUILD_DAY_CH1-48;
	
			StringToData.Days = (uint8_t)StringToData.DayCH0*10+StringToData.DayCH1;

			StringToData.HourCH0 = BUILD_HOUR_CH0-48;
			StringToData.HourCH1 = BUILD_HOUR_CH1-48;
	
			StringToData.Hours = (uint8_t)StringToData.HourCH0*10+StringToData.HourCH1;
	
			StringToData.MinCH0 = BUILD_MIN_CH0-48;
			StringToData.MinCH1 = BUILD_MIN_CH1-48;
			
			StringToData.Minutes = (uint8_t)StringToData.MinCH0*10+StringToData.MinCH1;
			
			StringToData.SecCH0 = BUILD_SEC_CH0-48;
			StringToData.SecCH1 = BUILD_SEC_CH1-48;

			StringToData.VL_seconds = (uint8_t)StringToData.SecCH0*10+StringToData.SecCH1;
			
			PCF8563_Write.Write_Data.VL_seconds = bin2bcd(StringToData.VL_seconds);
			PCF8563_Write.Write_Data.Minutes = bin2bcd(StringToData.Minutes);
			PCF8563_Write.Write_Data.Hours = bin2bcd(StringToData.Hours);
			PCF8563_Write.Write_Data.Days = bin2bcd(StringToData.Days);
			PCF8563_Write.Write_Data.Century_months = bin2bcd(StringToData.Century_months);
			PCF8563_Write.Write_Data.Years = bin2bcd(StringToData.Years-2000);
			
}

void Mem_Read(void)
{

		PCF8563_BCD.BCD_Data.Years = 			bcd2bin(PCF8563.RTCData[6]+PCF8563.RTCData[7]);
		PCF8563_BCD.BCD_Data.Century_months = 	bcd2bin(PCF8563.RTCData[5] & 0x1F);
		PCF8563_BCD.BCD_Data.Days = 			bcd2bin(PCF8563.RTCData[3] & 0x3F);
		PCF8563_BCD.BCD_Data.Hours = 			bcd2bin(PCF8563.RTCData[2] & 0x3F);
		PCF8563_BCD.BCD_Data.Minutes = 			bcd2bin(PCF8563.RTCData[1]);	
		PCF8563_BCD.BCD_Data.VL_seconds = 		bcd2bin(PCF8563.RTCData[0]);
}

int8_t bin2bcd(int8_t value)
{
char retval;

retval = 0;

while(1)
  {
   if(value >= 10)
     {
      value -= 10;
      retval += 0x10;
     }
   else 
     {
      retval += value;
      break;
     }
   }

return(retval);
}

char bcd2bin(char bcd_value)
{
char temp;

temp = bcd_value;

return(((temp >> 4)*10) + (bcd_value & 0x0f));
}


