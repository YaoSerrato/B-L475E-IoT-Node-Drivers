/*
 * stm32l475xx_rcc_driver.h
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#ifndef INC_STM32L475XX_RCC_DRIVER_H_
#define INC_STM32L475XX_RCC_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
  /* INCLUDES */
/******************************************************************************/

/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */
#include <stm32l475xx.h>

/* Here go the own includes */

/******************************************************************************/
  /* DEFINES */
/******************************************************************************/
#define	RCC_MSISPEED_100K		(0)
#define	RCC_MSISPEED_200K		(1)
#define	RCC_MSISPEED_400K		(2)
#define	RCC_MSISPEED_800K		(3)
#define	RCC_MSISPEED_1M			(4)
#define	RCC_MSISPEED_2M			(5)
#define	RCC_MSISPEED_4M			(6)
#define	RCC_MSISPEED_8M			(7)
#define	RCC_MSISPEED_16M		(8)
#define	RCC_MSISPEED_24M		(9)
#define	RCC_MSISPEED_32M		(10)
#define	RCC_MSISPEED_48M		(11)

/******************************************************************************/
  /* TYPEDEFS */
/******************************************************************************/
typedef struct
{
	uint32_t	OscillatorType;
	uint32_t	HSEState;
	uint32_t	LSEState;
	uint32_t	HSIState;
	uint32_t	HSICalibrationValue;
	uint32_t	LSIState;
	uint32_t	MSIState;
	uint32_t	MSICalibrationValue;
	uint32_t	MSIClockRange;
	/* PLL struct configuration */
}RCC_OscConfig_t;

typedef struct
{
	RCC_RegDef_t		*pRCC;					/**< This will hold the base address of the RCC peripheral */
	RCC_OscConfig_t		RCC_OscConfig_t;		/**< This is for configuring the oscillator */
}RCC_Handle_t;


void RCC_Config_MSI(uint8_t MSIspeed);
void RCC_Config_LSI(void);
void RCC_Config_HSI(void);
void RCC_Config_PLLCLK(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_RCC_DRIVER_H_ */
