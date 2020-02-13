/*
 * stm32l475xx_rcc_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#include <stm32l475xx_rcc_driver.h>

void RCC_Config_MSI(uint8_t MSIspeed)
{
	/* Enable MSI clock source */
	/* Configure MSI range */
	/* Wait for MSI clock signal to stabilize */
	/* Trim/calibrate the MSI oscillator */

	/* Registers to modify */
	// RCC_CR
		// MSIRANGE
		// MSIRGSEL
		// MSIPLLEN
		// MSIRDY
		// MSION
	// RCC_ICSCR
		// MSITRIM
		// MSICAL
	// RCC_CFGR
		// STOPWUCK
		// HPRE
		// SW
	// RCC_CSR
		// MSISRANGE
}
