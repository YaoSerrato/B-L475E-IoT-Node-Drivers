/**************************************************************************//**
 * @file    stm32l475xx_flash_driver.c
 * @brief   This file contains the function definitions for the Flash driver
 *          for the STM32L475VG microcontroller.
 *
 * This file has 1 function definition (input parameters omitted):
 *      <br>1) FLASH_SetLatency()  - Enables the GPIO peripheral clock. </br>
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    24/February/2020
 ******************************************************************************
 * @section License
 ******************************************************************************
 *
 *
 *****************************************************************************/

/*****************************************************************************/
  /* INCLUDES */
/*****************************************************************************/
/* Here go the system header files */

/* Here go the project includes */

/* Here go the own includes */
#include <stm32l475xx_flash_driver.h>

/*****************************************************************************/
  /* DEFINES */
/*****************************************************************************/

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* PUBLIC VARIABLES */
/*****************************************************************************/

/*****************************************************************************/
  /* STATIC VARIABLES */
/*****************************************************************************/

/*****************************************************************************/
  /* DEPENDENCIES */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DEFINITIONS */
/*****************************************************************************/

/**************************************************************************//**
* @brief       This function modifies the access wait states of flash memory
*              depending on the system clock frequency.
*
* @param       freq_HCLK        System clock frequency.
******************************************************************************/
void FLASH_SetLatency(uint32_t freq_HCLK)
{
	/* uint32_t LatencyValue = 0; */

	/* Clearing register */
	FLASH->FLASH_ACR &= ~(0x7 << 0);

	/* For voltage Range 1 */
	if((freq_HCLK > 0) & (freq_HCLK <= 16000000UL))
	{
		/* 0 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_ZERO_WAITSTATE;
		/* LatencyValue = FLASH_LATENCY_ZERO_WAITSTATE; */
	}
	else if((freq_HCLK > 16000000UL) & (freq_HCLK <= 32000000UL))
	{
		/* 1 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_ONE_WAITSTATE;
		/* LatencyValue = FLASH_LATENCY_ONE_WAITSTATE; */
	}
	else if((freq_HCLK > 32000000UL) & (freq_HCLK <= 48000000UL))
	{
		/* 2 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_TWO_WAITSTATE;
		/* LatencyValue = FLASH_LATENCY_TWO_WAITSTATE; */
	}
	else if((freq_HCLK > 48000000UL) & (freq_HCLK <= 64000000UL))
	{
		/* 3 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_THREE_WAITSTATE;
		/* LatencyValue = FLASH_LATENCY_THREE_WAITSTATE; */
	}
	else if((freq_HCLK > 64000000UL) & (freq_HCLK <= 80000000UL))
	{
		/* 4 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_FOUR_WAITSTATE;
		/* LatencyValue = FLASH_LATENCY_FOUR_WAITSTATE; */
	}
	else
	{
		/* Frequency out of range */
	}

	/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
	//while((FLASH->FLASH_ACR)&(0x00000007) != LatencyValue);
}
