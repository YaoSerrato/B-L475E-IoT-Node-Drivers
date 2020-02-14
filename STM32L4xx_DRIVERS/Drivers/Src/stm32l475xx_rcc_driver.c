/*
 * stm32l475xx_rcc_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#include <stm32l475xx_rcc_driver.h>

void RCC_Config_MSI(uint32_t MSIspeed, uint8_t CalValue)
{
	if(READ_REG_BIT(RCC->RCC_CR, 0) == 0 || READ_REG_BIT(RCC->RCC_CR, 1))
	{
		/* MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1).
		 * MSIRANGE must NOT be modified when MSI is ON and NOT ready (MSION=1 and MSIRDY=0) */

		/* Configure MSI range */
		RCC->RCC_CR &= ~(0xF << 4);
		RCC->RCC_CR	|= (MSIspeed << 4);		// MSIRANGE
		SET_REG_BIT(RCC->RCC_CR, 3);		// MSIRGSEL

		/* Trim/calibrate the MSI oscillator */
		RCC->RCC_ICSCR |= ((uint32_t) CalValue) << 8;

		/* Select MSI as SYSCLK source clock */
		CLR_REG_BIT(RCC->RCC_CFGR, 0);
		CLR_REG_BIT(RCC->RCC_CFGR, 1);

		/* Enable MSI clock source */
		SET_REG_BIT(RCC->RCC_CR, 0);			// MSION

		/* Wait for MSI clock signal to stabilize */
		while(READ_REG_BIT(RCC->RCC_CR, 1) == 0);		// MSIRDY

	}

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

void RCC_Config_MCO(uint8_t MCOprescaler, uint8_t MCOoutput)
{
	RCC->RCC_CFGR |= (MCOprescaler << 28);
	RCC->RCC_CFGR |= (MCOoutput << 24);
}
