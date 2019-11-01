#include "main.h"

#ifndef LPF_H_
#define LPF_H_

/***********************************************************************************//**
 * Included files
***************************************************************************************/

/***********************************************************************************//**
 * Typedefs
***************************************************************************************/

/**
 * \brief	Structure of 16bit low-pass filter.
 */
typedef volatile struct
{
	int16_t X;			//!< Filtered value.
	int16_t Y;			//!< Filter variable.
	uint8_t n;			//!< Filter grade.
} LPF16type;

/**
 * \brief	Structure of 32bit low-pass filter.
 */
typedef volatile struct
{
	int32_t X;			//!< Filtered value.
	int32_t Y;			//!< Filter variable.
	uint16_t n;			//!< Filter grade.
} LPF32type;

/***********************************************************************************//**
 * Function prototypes
***************************************************************************************/

int16_t			LPF16(LPF16type *filter, int16_t Xraw);
void			LPF16_Init(LPF16type *filter, int16_t Xinit, uint8_t n);
int32_t			LPF32(LPF32type *filter, int32_t Xraw);
void			LPF32_Init(LPF32type *filter, int32_t Xinit, uint16_t n);

/***********************************************************************************//**
 * End of file
***************************************************************************************/
#endif /* LPF_H_ */
