/*
 * stm32l475xx_pwr_driver.c
 *
 *  Created on: Feb 17, 2020
 *      Author: H369169
 */

#include <stm32l475xx_pwr_driver.h>

PWR_STATUS PWR_ControlVoltageScaling(uint32_t VoltageScaling)
{
	PWR_STATUS status;

	if(VoltageScaling == PWR_VOLTAGE_RANGE_1)
	{
		if(((PWR->PWR_CR1 & 0x600) >> 9) != PWR_VOLTAGE_RANGE_1)
		{
			/* Means probably we are coming from Range 2 */
			/* Process to change from Range 2 to Range 1 */

			/* Need to set PWR Range 1. Program the VOS bits to “01” in the PWR_CR1 register */
			/* Wait until the VOSF flag is cleared in the PWR_SR2 register */
			status = PWR_STATUS_OK;
		}
		else if(((PWR->PWR_CR1 & 0x600) >> 9) == PWR_VOLTAGE_RANGE_1)
		{
			/* We are already in Range 1 */
			status = PWR_STATUS_OK;
		}
		else
		{
			status = PWR_STATUS_ERROR;
		}
	}
	else if(VoltageScaling == PWR_VOLTAGE_RANGE_2)
	{
		if(((PWR->PWR_CR1 & 0x600) >> 9) != PWR_VOLTAGE_RANGE_2)
		{
			/* Means I am coming from Range 1 to Range 2 */
			/* Set Range 2. Program the VOS bits to “10” in the PWR_CR1 register */
		}
		else if(((PWR->PWR_CR1 & 0x600) >> 9) == PWR_VOLTAGE_RANGE_2)
		{
			/* We are already in Range 2 */
			status = PWR_STATUS_OK;
		}
		else
		{
			status = PWR_STATUS_ERROR;
		}
	}
	else
	{
		/* A different value from 1 and 2 for VoltageScaling was entered */
		status = PWR_STATUS_ERROR;
	}

	return status;
}
