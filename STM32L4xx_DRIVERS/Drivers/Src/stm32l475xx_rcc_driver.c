/**************************************************************************//**
 * @file    stm32l475xx_rcc_driver.c
 * @brief   This file contains the function definitions for the RCC driver
 *          for the STM32L475VG microcontroller.
 *
 * This file has 8 functions definitions (input parameters omitted):
 *      <br>1) RCC_Config_MSI()         - Configures MSI as system clock. </br>
 *      <br>2) RCC_Config_HSI()         - Configures HSI as system clock. </br>
 *      <br>3) RCC_Config_PLLCLK()      - Configures PLL as system clock. </br>
 *      <br>4) RCC_Config_LSI()         - Configures LSI. </br>
 *      <br>5) RCC_Config_MCO()         - Configures MCO pin. </br>
 *      <br>6) RCC_GetSYSCLK()          - Gets system clock value. </br>
 *      <br>7) RCC_GetHCLK()            - Gets HCLK clock value. </br>
 *      <br>8) RCC_GetMSIfreq()         - Gets the MSI range. </br>
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    24/February/2019
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
#include <stm32l475xx_rcc_driver.h>
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
uint32_t MSIfrequencies[12] = {100000U,   200000U,   400000U,   800000U,  1000000U,  2000000U, 4000000U, \
                              8000000U, 16000000U, 24000000U, 32000000U, 48000000U};

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
* @brief       This function configures the MSI oscillator range.
*              The function sets the MSI as system clock and configures the HCLK.
*
* @param       MSIspeed                 MSI range from the 12 options.
* @param       MSICalibrationValue      Calibration value for MSI.
* @param       AHB_Prescaler            AHB prescaler for HCLK.
*
* @return      RCC_STATUS_OK or RCC_STATUS_ERROR
******************************************************************************/
RCC_STATUS RCC_Config_MSI(uint32_t MSIspeed, uint32_t MSICalibrationValue, uint32_t AHB_Prescaler)
{
	RCC_STATUS status = RCC_STATUS_OK;
	uint32_t freq_new_HCLK = 0;
	uint32_t freq_current_SYSCLK = 0;
	uint32_t freq_current_HCLK = 0;

	/* Determining new desired frequency of HCLK */
	if(AHB_Prescaler == RCC_AHBPRESCALER_DIV1)
	{
		freq_new_HCLK = MSIfrequencies[MSIspeed];
	}
	else if((AHB_Prescaler >= RCC_AHBPRESCALER_DIV2) & (AHB_Prescaler <= RCC_AHBPRESCALER_DIV512))
	{
		freq_new_HCLK = MSIfrequencies[MSIspeed];
		freq_new_HCLK = (freq_new_HCLK) >> (AHB_Prescaler - 0x7);
	}
	else
	{
		/* An invalid pre scaler was entered */
		status = RCC_STATUS_ERROR;
		return status;
	}

	/* Get current SYSCLK */
	freq_current_SYSCLK = RCC_GetSYSCLK();

	/* Get current HCLK */
	freq_current_HCLK = RCC_GetHCLK();

	/* Comparing frequencies */
	if(freq_current_HCLK != freq_new_HCLK)
	{
		/* New HCLK frequency is different from the already set */
		if(freq_current_HCLK < freq_new_HCLK)
		{
			/* Increasing frequency */
			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);

			/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
			/* Select MSI as SYSCLK source clock */
			RCC->RCC_CFGR &= ~(0x3 << 0);
			RCC->RCC_CFGR |= RCC_SYSCLK_MSI;

			while(((RCC->RCC_CFGR)&(0xC) >> (2)) != RCC_SYSCLK_MSI);

			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);

			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

			/* Now, you can increase the CPU frequency. */
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

		}
		else
		{
			/* Decreasing frequency  */
			/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
			/* Select MSI as SYSCLK source clock */
			RCC->RCC_CFGR &= ~(0x3 << 0);
			RCC->RCC_CFGR |= RCC_SYSCLK_MSI;

			while(((RCC->RCC_CFGR)&(0xC) >> 2) != RCC_SYSCLK_MSI);

			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);

			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);

			/* Now, you can decrease the CPU frequency. */
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
		}
	}
	else
	{
		/* Frequencies are equal */
		/* Configure same frequency with new parameters (MSIspeed, MSICalibrationValue, AHB_Prescaler)  */

		/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
		/* Select MSI as SYSCLK source clock */
		RCC->RCC_CFGR &= ~(0x3 << 0);
		RCC->RCC_CFGR |= RCC_SYSCLK_MSI;

		while(((RCC->RCC_CFGR)&(0xC) >> 2) != RCC_SYSCLK_MSI);

		/* Set the AHB Prescaler */
		RCC->RCC_CFGR &= ~(0xF << 4);
		RCC->RCC_CFGR |= (AHB_Prescaler << 4);

		while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

		/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
		/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
		FLASH_SetLatency(freq_new_HCLK);

		/* Now, you can decrease the CPU frequency. */
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

			/* Enable MSI clock source */
			SET_REG_BIT(RCC->RCC_CR, 0);			// MSION

			/* Wait for MSI clock signal to stabilize */
			while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_1) == 0);		// MSIRDY

			status = RCC_STATUS_OK;
		}

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

/**************************************************************************//**
* @brief       The function sets the HSI as system clock and configures the HCLK.
*
* @param       AHB_Prescaler            AHB prescaler for HCLK.
*
* @return      RCC_STATUS_OK or RCC_STATUS_ERROR
******************************************************************************/
RCC_STATUS RCC_Config_HSI(uint32_t AHB_Prescaler)
{
	RCC_STATUS status = RCC_STATUS_OK;
	uint32_t freq_new_HCLK = 0;
	uint32_t freq_current_SYSCLK = 0;
	uint32_t freq_current_HCLK = 0;

	/* Determining new desired frequency of HCLK */
	if(AHB_Prescaler == RCC_AHBPRESCALER_DIV1)
	{
		freq_new_HCLK = RCC_HSI16_VALUE;
	}
	else if((AHB_Prescaler >= RCC_AHBPRESCALER_DIV2) & (AHB_Prescaler <= RCC_AHBPRESCALER_DIV512))
	{
		freq_new_HCLK = RCC_HSI16_VALUE;
		freq_new_HCLK = (freq_new_HCLK) >> (AHB_Prescaler - 0x7);
	}
	else
	{
		/* An invalid pre scaler was entered */
		status = RCC_STATUS_ERROR;
		return status;
	}

	/* Get current SYSCLK */
	freq_current_SYSCLK = RCC_GetSYSCLK();

	/* Get current HCLK */
	freq_current_HCLK = RCC_GetHCLK();

	/* Comparing frequencies */
	if(freq_current_HCLK != freq_new_HCLK)
	{
		/* New HCLK frequency is different from the already set */
		if(freq_current_HCLK < freq_new_HCLK)
		{
			/* Increasing frequency */

			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);

			/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
			/* Select HSI as SYSCLK source clock */
			RCC->RCC_CFGR &= ~(0x3 << 0);
			RCC->RCC_CFGR |= RCC_SYSCLK_HSI16;
			while(((RCC->RCC_CFGR)&(0xC) >> 2) != RCC_SYSCLK_HSI16);

			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);
			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

			/* Enable HSI clock source */
			SET_REG_BIT(RCC->RCC_CR, REG_BIT_8);			// HSION

			/* Wait for HSI clock signal to stabilize */
			while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_10) == 0);		// HSIRDY

			status = RCC_STATUS_OK;

		}
		else
		{
			/* Decreasing frequency  */

			/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
			/* Select HSI as SYSCLK source clock */
			RCC->RCC_CFGR &= ~(0x3 << 0);
			RCC->RCC_CFGR |= RCC_SYSCLK_HSI16;
			while(((RCC->RCC_CFGR)&(0xC) >> 2) != RCC_SYSCLK_HSI16);

			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);
			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);

			/* Enable HSI clock source */
			SET_REG_BIT(RCC->RCC_CR, REG_BIT_8);			// HSION

			/* Wait for HSI clock signal to stabilize */
			while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_10) == 0);		// HSIRDY

			status = RCC_STATUS_OK;
		}
	}
	else
	{
		/* Frequencies are equal */
	}

	return status;
}

/**************************************************************************//**
* @brief       The function sets the PLL as system clock and configures the HCLK.
*
* @param       ClockSource              Clock source for the PLL (HSE, HSI or MSI).
* @param       ClockSourceFrequency     Clock source frequency.
* @param       PLLM                     PLLM prescaler.
* @param       PLLN                     PLLN prescaler.
* @param       PLLR                     PLLR prescaler.
* @param       AHB_Prescaler            AHB prescaler for HCLK.
*
* @return      RCC_STATUS_OK or RCC_STATUS_ERROR
******************************************************************************/
RCC_STATUS RCC_Config_PLLCLK(uint32_t ClockSource, uint32_t ClockSourceFrequency, uint32_t PLLM, uint32_t PLLN, uint32_t PLLR, uint32_t AHB_Prescaler)
{
	RCC_STATUS status = RCC_STATUS_OK;
	uint32_t freq_new_HCLK = 0;
	uint32_t freq_current_SYSCLK = 0;
	uint32_t freq_current_HCLK = 0;
	uint32_t PLL_M, PLL_N, PLL_R;

	/* Determining new desired frequency of HCLK */

	PLL_M = PLLM + 1U;
	PLL_N = PLLN;
	PLL_R = (PLLR + 1U)*(2U);

	switch(ClockSource)
	{
		case RCC_PLLSRC_NOCLK:
			status = RCC_STATUS_ERROR;
			return status;
		case RCC_PLLSRC_MSI:
			freq_new_HCLK = MSIfrequencies[ClockSourceFrequency];
			break;
		case RCC_PLLSRC_HSI16:
			freq_new_HCLK = RCC_HSI16_VALUE;
			break;
		case RCC_PLLSRC_HSE:
			freq_new_HCLK = RCC_HSE_VALUE;
			break;
		default:
			status = RCC_STATUS_ERROR;
			return status;
	}

	freq_new_HCLK = (freq_new_HCLK*PLL_N)/(PLL_M*PLL_R);

	if((AHB_Prescaler >= RCC_AHBPRESCALER_DIV2) & (AHB_Prescaler <= RCC_AHBPRESCALER_DIV512))
	{
		freq_new_HCLK = (freq_new_HCLK) >> (AHB_Prescaler - 0x7);
	}

	/* Get current SYSCLK */
	freq_current_SYSCLK = RCC_GetSYSCLK();

	/* Get current HCLK */
	freq_current_HCLK = RCC_GetHCLK();

	/* Comparing frequencies */
	if(freq_current_HCLK != freq_new_HCLK)
	{
		/* New HCLK frequency is different from the already set */
		if(freq_current_HCLK < freq_new_HCLK)
		{
			/* Increasing frequency */
			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);

			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);
			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));
		}
		else
		{
			/* Decreasing frequency  */
			/* Set the AHB Prescaler */
			RCC->RCC_CFGR &= ~(0xF << 4);
			RCC->RCC_CFGR |= (AHB_Prescaler << 4);
			while(((RCC->RCC_CFGR)&(0xF << 4)) != (AHB_Prescaler << 4));

			/* Program the wait states according to Dynamic Voltage Range selected and the new frequency */
			/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
			FLASH_SetLatency(freq_new_HCLK);
		}
	}
	else
	{
		/* Frequencies are equal */
		/* Configure same frequency with new parameters  */
	}

	/* Configure MSI, HSI16 or HSE first. Turn them on. */
	switch(ClockSource)
	{
		case RCC_PLLSRC_NOCLK:
			status = RCC_STATUS_ERROR;
			return status;

		case RCC_PLLSRC_MSI:
			/* Configuring MSI */

			if(READ_REG_BIT(RCC->RCC_CR, REG_BIT_0) == 0 || READ_REG_BIT(RCC->RCC_CR, REG_BIT_1))
			{
				/* Configure MSI range */
				RCC->RCC_CR &= ~(0xF << 4);
				RCC->RCC_CR	|= (ClockSourceFrequency << 4);		// MSIRANGE

				/* MSI clock range selection */
				SET_REG_BIT(RCC->RCC_CR, REG_BIT_3);		// MSIRGSEL

				/* Enable MSI clock source */
				SET_REG_BIT(RCC->RCC_CR, 0);			// MSION

				/* Wait for MSI clock signal to stabilize */
				while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_1) == 0);		// MSIRDY

			}
			else
			{
				status = RCC_STATUS_ERROR;
				return status;
			}
			break;

		case RCC_PLLSRC_HSI16:
			/* Configuring HSI */

			/* Enable HSI clock source */
			SET_REG_BIT(RCC->RCC_CR, REG_BIT_8);			// HSION

			/* Wait for HSI clock signal to stabilize */
			while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_10) == 0);		// HSIRDY

			break;

		case RCC_PLLSRC_HSE:
			status = RCC_STATUS_ERROR;
			return status;

		default:
			status = RCC_STATUS_ERROR;
			return status;
	}

	/* Disable the PLL by setting PLLON to 0 in Clock control register (RCC_CR). */
	CLR_REG_BIT(RCC->RCC_CR, REG_BIT_24);

	/* Wait until PLLRDY is cleared. The PLL is now fully stopped. */
	while(READ_REG_BIT(RCC->RCC_CR, REG_BIT_25) == 1);

	/* Change the input clock source (MSI, HSI16, HSE). */
	RCC->RCC_PLLCFGR &= ~(0x3 << 0U);
	RCC->RCC_PLLCFGR |= (ClockSource << 0U);

	/* Change this parameters: PLLM, PLLN, PLLR. */
	RCC->RCC_PLLCFGR &= ~(0x7 << 4U);
	RCC->RCC_PLLCFGR |= (PLLM << 4U);

	RCC->RCC_PLLCFGR &= ~(0x7F << 8U);
	RCC->RCC_PLLCFGR |= (PLLN << 8U);

	RCC->RCC_PLLCFGR &= ~(0x3 << 25U);
	RCC->RCC_PLLCFGR |= (PLLR << 25U);

	/* Enable the PLL again by setting PLLON to 1. */
	SET_REG_BIT(RCC->RCC_CR, REG_BIT_24);

	/* Enable the desired PLL outputs by configuring PLLPEN, PLLQEN, PLLREN in RCC_PLLCFGR. */
	SET_REG_BIT(RCC->RCC_PLLCFGR, REG_BIT_24);

	/* Modify the CPU clock source by writing the SW bits in the RCC_CFGR register */
	/* Select PLL as SYSCLK source clock */
	RCC->RCC_CFGR &= ~(0x3 << 0);
	RCC->RCC_CFGR |= RCC_SYSCLK_PLL;
	while(((RCC->RCC_CFGR)&(0xC) >> 2) != RCC_SYSCLK_PLL);

	return status;
}

/**************************************************************************//**
* @brief       The function enables/disables the LSI.
*
* @param       LSI_Enabler              Used for enable/disable the LSI.
*
* @return      RCC_STATUS_OK or RCC_STATUS_ERROR
******************************************************************************/
RCC_STATUS RCC_Config_LSI(uint32_t LSI_Enabler)
{
	/* LSI RC can be switched on and off using the LSION (RCC_CSR) */
	if(LSI_Enabler == SET)
	{
		/* Turn on LSI */
		SET_REG_BIT(RCC->RCC_CSR, REG_BIT_0);

		/* The LSIRDY flag in the Control/status register (RCC_CSR) indicates if the LSI oscillator is ready */
		while(READ_REG_BIT(RCC->RCC_CSR, REG_BIT_1) == 0);		// HSIRDY
	}
	else
	{
		/* Turn off LSI */
		CLR_REG_BIT(RCC->RCC_CSR, REG_BIT_0);
	}

	return RCC_STATUS_OK;
}

/**************************************************************************//**
* @brief       The function configures the MCO for measuring the SYSCLK.
*
* @param       MCOprescaler             Prescaler for the MCO signal.
* @param       MCOoutput                MCO output selection.
*
* @return      RCC_STATUS_OK or RCC_STATUS_ERROR
******************************************************************************/
void RCC_Config_MCO(uint8_t MCOprescaler, uint8_t MCOoutput)
{
	/* MCO clock pre-scaler */
	RCC->RCC_CFGR &= ~(0x7 << 28);
	RCC->RCC_CFGR |= (MCOprescaler << 28);

	/* MCO clock selection */
	RCC->RCC_CFGR &= ~(0xF << 24);
	RCC->RCC_CFGR |= (MCOoutput << 24);
}

/**************************************************************************//**
* @brief       The function calculates the SYSCLK.
*
* @return      SYSCLK frequency.
******************************************************************************/
uint32_t RCC_GetSYSCLK(void)
{
	uint32_t SYSCLK = 0;			/* Here the System Clock will be stored */
	uint32_t system_clock = 0;
	uint32_t pll_clocksrc = 0;
	uint32_t msi_range = 0;
	uint32_t pll_clkin = 0;
	uint32_t PLLM = 1, PLLN = 0, PLLR = 1;

	/* Read bits SWS from RCC_CFGR */
	system_clock = ((RCC->RCC_CFGR) & (0xC)) >> (2);

	switch(system_clock)
	{
		case RCC_CFGR_SWS_MSI:
			if(READ_REG_BIT(RCC->RCC_CR, REG_BIT_3) == 0x0)
			{
				/* MSI Range is provided by MSISRANGE[3:0] in RCC_CSR register  */
				msi_range = ((RCC->RCC_CSR) & (0xF00)) >> (8);
				SYSCLK = MSIfrequencies[msi_range];
			}
			else
			{
				/* MSI Range is provided by MSIRANGE[3:0] in the RCC_CR register */
				msi_range = ((RCC->RCC_CR) & (0xF0)) >> (4);
				SYSCLK = MSIfrequencies[msi_range];
			}
			break;

		case RCC_CFGR_SWS_HSI16:
			SYSCLK = RCC_HSI16_VALUE;
			break;

		case RCC_CFGR_SWS_HSE:
			SYSCLK = RCC_HSE_VALUE;
			break;

		case RCC_CFGR_SWS_PLL:

			/* Read bits PLLSRC from RCC_PLLCFGR */
			pll_clocksrc = (RCC->RCC_PLLCFGR) & (0x3);

			/* Determining PLL clock source value */
			switch(pll_clocksrc)
			{
				case RCC_PLLSRC_NOCLK:
					pll_clkin = 0;
					break;
				case RCC_PLLSRC_MSI:
					if(READ_REG_BIT(RCC->RCC_CR, REG_BIT_3) == 0x0)
					{
						/* MSI Range is provided by MSISRANGE[3:0] in RCC_CSR register  */
						msi_range = ((RCC->RCC_CSR) & (0xF00)) >> (8);
						pll_clkin = MSIfrequencies[msi_range];
					}
					else
					{
						/* MSI Range is provided by MSIRANGE[3:0] in the RCC_CR register */
						msi_range = ((RCC->RCC_CR) & (0xF0)) >> (4);
						pll_clkin = MSIfrequencies[msi_range];
					}
					break;
				case RCC_PLLSRC_HSI16:
					pll_clkin = RCC_HSI16_VALUE;
					break;
				case RCC_PLLSRC_HSE:
					pll_clkin = RCC_HSE_VALUE;
					break;
				default:
					pll_clkin = 0;
					break;
			}

			/* Determining values for PLLM, PLLN and PLLR */
			PLLM = (((RCC->RCC_PLLCFGR) & (0x70U)) >> 4U) + 1U;
			PLLN = ((RCC->RCC_PLLCFGR) & (0x7F00U)) >> 8U;
			PLLR = ((((RCC->RCC_PLLCFGR) & (0x6000000U)) >> 25U) + 1U)*(2U);

			/* Determining system clock */
			SYSCLK = (pll_clkin*PLLN)/(PLLM*PLLR);

			break;

		default:
			SYSCLK = 0;
	}

	return SYSCLK;
}

/**************************************************************************//**
* @brief       The function calculates the HCLK.
*
* @return      HCLK frequency.
******************************************************************************/
uint32_t RCC_GetHCLK(void)
{
	uint32_t SYSCLK = RCC_GetSYSCLK();
	uint32_t HCLK;
	uint32_t AHBPRESC;

	AHBPRESC = ((RCC->RCC_CFGR) & (0xF0)) >> (4);

	if((AHBPRESC >= 8) & (AHBPRESC <= 15))
	{
		HCLK = (SYSCLK) >> (AHBPRESC - 0x7);
	}
	else
	{
		HCLK = SYSCLK;
	}


	return HCLK;
}

uint32_t RCC_GetMSIfreq(uint32_t RCC_MSISPEED)
{
	uint32_t freq = 0;

	switch(RCC_MSISPEED)
	{
		case RCC_MSISPEED_100K:
			freq = (uint32_t)100000;
			break;
		case RCC_MSISPEED_200K:
			freq = (uint32_t)200000;
			break;
		case RCC_MSISPEED_400K:
			freq = (uint32_t)400000;
			break;
		case RCC_MSISPEED_800K:
			freq = (uint32_t)800000;
			break;
		case RCC_MSISPEED_1M:
			freq = (uint32_t)1000000;
			break;
		case RCC_MSISPEED_2M:
			freq = (uint32_t)2000000;
			break;
		case RCC_MSISPEED_4M:
			freq = (uint32_t)4000000;
			break;
		case RCC_MSISPEED_8M:
			freq = (uint32_t)8000000;
			break;
		case RCC_MSISPEED_16M:
			freq = (uint32_t)16000000;
			break;
		case RCC_MSISPEED_24M:
			freq = (uint32_t)24000000;
			break;
		case RCC_MSISPEED_32M:
			freq = (uint32_t)32000000;
			break;
		case RCC_MSISPEED_48M:
			freq = (uint32_t)48000000;
			break;
		default:
			freq = (uint32_t)0;
			break;
	}

	return freq;
}
