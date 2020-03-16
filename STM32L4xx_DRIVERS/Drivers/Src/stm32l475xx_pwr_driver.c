/**************************************************************************//**
 * @file    stm32l475xx_pwr_driver.c
 * @brief   This file contains the function definitions for the PWR driver
 *          for the STM32L475VG microcontroller.
 *
 * This file has 1 function definitions (input parameters omitted):
 *      <br>1) PWR_ControlVoltageScaling()  - Enables the GPIO peripheral clock. </br>
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
#include <stm32l475xx_pwr_driver.h>

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
* @brief       This function sets the Voltage Dynamic Range according to
*              system clock frequency
*
* @param       VoltageScaling   Voltage range (1 or 2).
*
* @return      PWR_STATUS_OK or PWR_STATUS_ERROR
******************************************************************************/
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
			PWR->PWR_CR1 &= ~(0x3 << 9);
			CLR_REG_BIT(PWR->PWR_CR1, REG_BIT_10);
			SET_REG_BIT(PWR->PWR_CR1, REG_BIT_9);

			/* Wait until the VOSF flag is cleared in the PWR_SR2 register */
			while(READ_REG_BIT(PWR->PWR_SR2, REG_BIT_10) != 0x1U);

			status = PWR_STATUS_OK;
		}
		else
		{
			/* We are already in Range 1 */
			status = PWR_STATUS_OK;
		}
	}
	else if(VoltageScaling == PWR_VOLTAGE_RANGE_2)
	{
		if(((PWR->PWR_CR1 & 0x600) >> 9) != PWR_VOLTAGE_RANGE_2)
		{
			/* Means I am coming from Range 1 to Range 2 */
			/* Set Range 2. Program the VOS bits to “10” in the PWR_CR1 register */
			PWR->PWR_CR1 &= ~(0x3 << 9);
			SET_REG_BIT(PWR->PWR_CR1, REG_BIT_10);
			CLR_REG_BIT(PWR->PWR_CR1, REG_BIT_9);

			status = PWR_STATUS_OK;
		}
		else
		{
			/* We are already in Range 2 */
			status = PWR_STATUS_OK;
		}
	}
	else
	{
		/* A wrong value for VoltageScaling was entered */
		status = PWR_STATUS_ERROR;
	}

	return status;
}
