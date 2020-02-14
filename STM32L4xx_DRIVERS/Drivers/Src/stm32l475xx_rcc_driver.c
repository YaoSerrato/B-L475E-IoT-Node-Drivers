/*
 * stm32l475xx_rcc_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#include <stm32l475xx_rcc_driver.h>

void RCC_Config_MSI(uint32_t MSIspeed, uint8_t CalValue)
{
	/* Configure MSI range */
	RCC->RCC_CR |= 0x00000008;			// MSIRGSEL
	RCC->RCC_CR	|= (MSIspeed << 4);		// MSIRANGE

	/* Trim/calibrate the MSI oscillator */
	RCC->RCC_ICSCR |= ((uint32_t) CalValue) << 8;

	/* Wait for MSI clock signal to stabilize */
	while((RCC->RCC_CR & 0x00000002 >> 1) == 0){}		// MSIRDY

	/* Select MSI as SYSCLK source clock */
	RCC->RCC_CFGR |= 0x0;

	/* Enable MSI clock source */
	RCC->RCC_CR |= (0x1 << 0);			// MSION

	/* Registers to modify */
	// RCC_CR
		// MSIPLLEN
	// RCC_ICSCR
	// RCC_CFGR
		// STOPWUCK
		// HPRE
	// RCC_CSR
		// MSISRANGE
}
