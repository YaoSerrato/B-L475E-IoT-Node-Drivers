/*
 * stm32l475xx_rcc_driver.c
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#include <stm32l475xx_rcc_driver.h>

RCC_STATUS RCC_Config_MSI(uint32_t MSIspeed, uint32_t MSICalibrationValue)
{
	RCC_STATUS status;

	if(READ_REG_BIT(RCC->RCC_CR, REG_BIT_0) == 0 || READ_REG_BIT(RCC->RCC_CR, REG_BIT_1))
	{
		/* MSIRANGE can be modified when MSI is OFF (MSION=0) or when MSI is ready (MSIRDY=1).
		 * MSIRANGE must NOT be modified when MSI is ON and NOT ready (MSION=1 and MSIRDY=0) */

		/* Configure MSI range */
		RCC->RCC_CR &= ~(0xF << 4);
		RCC->RCC_CR	|= (MSIspeed << 4);		// MSIRANGE

		/* MSI clock range selection */
		SET_REG_BIT(RCC->RCC_CR, REG_BIT_3);		// MSIRGSEL

		/* Trim/calibrate the MSI oscillator */
		RCC->RCC_ICSCR &= ~(0xFF << 8);
		RCC->RCC_ICSCR |= (MSICalibrationValue) << 8;

		/* Select MSI as SYSCLK source clock */
		RCC->RCC_CFGR &= ~(0x3 << 0);
		RCC->RCC_CFGR |= RCC_SYSCLK_MSI;

		/* Enable MSI clock source */
		SET_REG_BIT(RCC->RCC_CR, 0);			// MSION

		/* Wait for MSI clock signal to stabilize */
		while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_1) == 0);		// MSIRDY

		status = RCC_STATUS_OK;
	}
	else
	{
		status = RCC_STATUS_ERROR;
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

	return status;
}

void RCC_Config_MCO(uint8_t MCOprescaler, uint8_t MCOoutput)
{
	/* MCO clock pre-scaler */
	RCC->RCC_CFGR &= ~(0x7 << 28);
	RCC->RCC_CFGR |= (MCOprescaler << 28);

	/* MCO clock selection */
	RCC->RCC_CFGR &= ~(0xF << 24);
	RCC->RCC_CFGR |= (MCOoutput << 24);
}
