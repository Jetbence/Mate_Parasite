/***********************************************************************************//**
 * Included files
***************************************************************************************/

#include "stdint.h"

#include "LPF.h"

/***********************************************************************************//**
 * Functions
***************************************************************************************/

/**
 * \brief	16 bit low-pass filter.
 * 			Usage example:
 * 			...
 *			LPF16type BatVoltageFilter;
 *			LPF16_Init(&BatVoltageFilter, 0, 3);
 *			Battery.Voltage = LPF16(&BatVoltageFilter, BatteryVoltage_ADC_Value);
 *			...
 *
 * \param[in,out]	filter	Pointer to the filter structure.
 * \param[in]		Xraw	New X value.
 * \return			Filtered X value.
 */
int16_t LPF16(LPF16type *filter, int16_t Xraw)
{
	uint8_t n = filter->n;
	int16_t Y_prev = filter->Y;			// Previous Y value
	int16_t X_prev = filter->X;			// Previous X value
	Y_prev = Y_prev - X_prev + Xraw;	// New Y value
	filter->Y = Y_prev;					// Write back new Y value
	if(Y_prev < 0)
	{
		Y_prev = -Y_prev;
		do {Y_prev = Y_prev >> 1;} while(--n != 0);	// Calculate new X value
		Y_prev = -Y_prev;
	}
	else
	{
		do {Y_prev = Y_prev >> 1;} while(--n != 0);	// Calculate new X value
	}
	filter->X = Y_prev;					// Write back new X value
	return Y_prev;						// Returns filtered X value (Xfilt). No extra code if return value is not used!
}

/******************************************************************************/

/**
 * \brief	Initializes 16 bit low-pass filter.
 * 			Usage example:
 * 			...
 *			LPF16type BatVoltageFilter;
 *			LPF16_Init(&BatVoltageFilter, 0, 3);
 *			Battery.Voltage = LPF16(&BatVoltageFilter, BatteryVoltage_ADC_Value);
 *			...
 *
 * \param[in,out]	filter	Pointer to the filter structure.
 * \param[in]		Xinit	Initial X value.
 * \param[in]		n		Grade of LPF.
 * \return			None
 */
void LPF16_Init(LPF16type *filter, int16_t Xinit, uint8_t n)
{
	filter->n = n;
	filter->X = Xinit;
	do {Xinit *= 2;} while(--n != 0);
	filter->Y = Xinit;
}

/******************************************************************************/

/**
 * \brief	32 bit low-pass filter.
 * 			Usage example:
 * 			...
 *			LPF32type BatVoltageFilter;
 *			LPF32_Init(&BatVoltageFilter, 0, 3);
 *			Battery.Voltage = LPF32(&BatVoltageFilter, BatteryVoltage_ADC_Value);
 *			...
 *
 * \param[in,out]	filter	Pointer to the filter structure.
 * \param[in]		Xraw	New X value.
 * \return			Filtered X value.
 */
int32_t LPF32(LPF32type *filter, int32_t Xraw)
{
	uint16_t n = filter->n;
	int32_t Y_prev = filter->Y;			// Previous Y value
	int32_t X_prev = filter->X;			// Previous X value
	Y_prev = Y_prev - X_prev + Xraw;	// New Y value
	filter->Y = Y_prev;					// Write back new Y value
	if(Y_prev < 0)
	{
		Y_prev = -Y_prev;
		do {Y_prev = Y_prev >> 1;} while(--n != 0);	// Calculate new X value
		Y_prev = -Y_prev;
	}
	else
	{
		do {Y_prev = Y_prev >> 1;} while(--n != 0);	// Calculate new X value
	}
	filter->X = Y_prev;					// Write back new X value
	return Y_prev;						// Returns filtered X value (Xfilt). No extra code if return value is not used!
}

/******************************************************************************/

/**
 * \brief	Initializes 32 bit low-pass filter.
 * 			Usage example:
 * 			...
 *			LPF32type BatVoltageFilter;
 *			LPF32_Init(&BatVoltageFilter, 0, 3);
 *			Battery.Voltage = LPF32(&BatVoltageFilter, BatteryVoltage_ADC_Value);
 *			...
 *
 * \param[in,out]	filter	Pointer to the filter structure.
 * \param[in]		Xinit	Initial X value.
 * \param[in]		n		Grade of LPF.
 * \return			None
 */
void LPF32_Init(LPF32type *filter, int32_t Xinit, uint16_t n)
{
	filter->n = n;
	filter->X = Xinit;
	do {Xinit *= 2;} while(--n != 0);
	filter->Y = Xinit;
}

/***********************************************************************************//**
 * End of file
***************************************************************************************/

