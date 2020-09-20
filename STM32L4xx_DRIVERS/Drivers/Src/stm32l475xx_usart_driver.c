/**************************************************************************//**
 * @file    stm32l475xx_usart_driver.c
 * @brief   This file contains the function definitions for the USART driver
 *          for the STM32L475VG microcontroller.
 *
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    07/September/2020
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
#include <stm32l475xx_usart_driver.h>

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
* @brief       This function enables/disables the USART/UART module.
*
* @param       pUSARTx  Base address of respective USARTx.
* @param       Enabler  Determines whether the USARTx module must be enabled
* 						or disabled.
******************************************************************************/
void	USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t Enabler)
{
	if(Enabler == ENABLE)
	{
		pUSARTx->USART_CR1 |= (1U << REG_BIT_0);
	}
	else
	{
		pUSARTx->USART_CR1 &= ~(1U << REG_BIT_0);
	}
}

/**************************************************************************//**
* @brief       This function enables/disables the USART/UART peripheral clock.
*
* @param       pUSARTx  Base address of respective USARTx.
* @param       Enabler  Determines whether the USARTx peripheral clock must be
* 						enabled or disabled.
******************************************************************************/
void	USART_PeriphClkControl(USART_RegDef_t *pUSARTx, uint8_t Enabler)
{
	if(Enabler == ENABLE)
	{
		/* USARTx will be enabled */

		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else
		{
			/* TODO: Should return an error and should not let the code compile. */
		}
	}
	else
	{
		/* USARTx will be disabled */

		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else
		{
			/* TODO: Should return an error and should not let the code compile. */
		}
	}
}

/**************************************************************************//**
* @brief       This function returns the bit value of the specified Flag.
*
* @param       pUSARTx  Base address of respective USARTx.
* @param       Flag		Flag offset.
*
* @return      Bit value of the Flag specified.
******************************************************************************/
uint8_t	USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t Flag)
{
	return READ_REG_BIT(pUSARTx->USART_ISR, Flag);
}

/**************************************************************************//**
* @brief       This function clears the bit value of the specified Flag.
*
* @param       pUSARTx  Base address of respective USARTx.
* @param       Flag		Flag offset.
******************************************************************************/
void	USART_ClearFlag(USART_RegDef_t *pUSARTx, uint8_t Flag)
{
	CLR_REG_BIT(pUSARTx->USART_ISR, Flag);
}




