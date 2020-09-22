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
* @brief       This function enables/disables the USART/UART peripheral clock.
*
* @param       pUSARTx  Base address of respective USARTx.
* @param       Enabler  Determines whether the USARTx peripheral clock must be
* 						enabled or disabled.
******************************************************************************/
void	USART_PeriphClkControl(USART_RegDef_t *pUSARTx, uint8_t ClockSource, uint8_t Enabler)
{
	uint8_t	offset = 0;

	if(Enabler == ENABLE)
	{
		/* USARTx will be enabled */

		if(pUSARTx == USART1)
		{
			/* TODO: configure the clock source based on ClockSource input parameter before enabling the USART clock */
			/* Hardcoded to HSI for the moment */
			offset = 0;
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			/* TODO: configure the clock source based on ClockSource input parameter before enabling the USART clock */
			/* Hardcoded to HSI for the moment */
			offset = 2;
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			/* TODO: configure the clock source based on ClockSource input parameter before enabling the USART clock */
			/* Hardcoded to HSI for the moment */
			offset = 4;
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			/* TODO: configure the clock source based on ClockSource input parameter before enabling the USART clock */
			/* Hardcoded to HSI for the moment */
			offset = 6;
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			/* TODO: configure the clock source based on ClockSource input parameter before enabling the USART clock */
			/* Hardcoded to HSI for the moment */
			offset = 8;
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

	/* Selecting corresponding clock source */
	RCC->RCC_CCIPR &= ~(3U << offset);
	RCC->RCC_CCIPR |= (ClockSource << offset);
}

/**************************************************************************//**
* @brief       This function initialises the USART/UART module.
*
* @param       pUSARThandle  USART/UART handle containing the peripheral base
* 							 address and the configuration parameters structure.
******************************************************************************/
void	USART_Init(USART_Handle_t *pUSARThandle)
{
	uint32_t USARTDIV = 0;
												/* Configuration of USART_CR1 */
	/* Configuration of USART mode */
	switch(pUSARThandle->USART_config.USART_Mode)
	{
		case USART_MODE_TXONLY:
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_2);
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_3);
			break;

		case USART_MODE_RXONLY:
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_3);
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_2);
			break;

		case USART_MODE_TX_RX:
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_2);
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_3);
			break;

		default:
			/* TODO: should return an error - Invalid USART mode */
			break;
	}

	/* Configuration of USART parity */
	if(pUSARThandle->USART_config.USART_ParityControl == USART_PARITY_ENABLED)
	{
		SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_10);

		switch(pUSARThandle->USART_config.USART_ParitySelection)
		{
			case USART_PARITY_EVEN:
				CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_9);
				break;

			case USART_PARITY_ODD:
				SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_9);
				break;

			default:
				/* TODO: should return an error - Invalid parity */
				break;
		}
	}

	/* Configuration of word length */
	switch(pUSARThandle->USART_config.USART_WordLength)
	{
		case USART_WORDLENGTH_8:
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_28);
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_12);
			break;

		case USART_WORDLENGTH_9:
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_28);
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_12);
			break;

		case USART_WORDLENGTH_7:
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_28);
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_12);
			break;

		default:
			/* TODO: should return an error - Invalid word length */
			break;
	}

	/* Configuration of oversampling mode and baudrate (USART clock is hardcoded to HSI16) */
	USARTDIV = (USART_CLOCKSOURCE_VALUE)/(pUSARThandle->USART_config.USART_Baudrate);

	switch(pUSARThandle->USART_config.USART_Oversampling)
	{
		case USART_OVERSAMPLING_8:
			SET_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_15);

			USARTDIV = (USARTDIV << 1); /* Multiplied by 2 */

			if(USARTDIV >= USART_BRR_USARTDIV_LIMIT)
			{
				pUSARThandle->pUSARTx->USART_BRR = (USARTDIV & 0x0000FFF0U) | ((USARTDIV & 0x0000000FU) >> 1U);
			}
			else
			{
				/* TODO: should return an error - Invalid USARTDIV value */
			}

			break;

		case USART_OVERSAMPLING_16:
			CLR_REG_BIT(pUSARThandle->pUSARTx->USART_CR1, REG_BIT_15);

			if(USARTDIV >= USART_BRR_USARTDIV_LIMIT)
			{
				pUSARThandle->pUSARTx->USART_BRR = (USARTDIV & (uint32_t)0x0000FFFF);
			}
			else
			{
				/* TODO: should return an error - Invalid USARTDIV value */
			}

			break;

		default:
			/* TODO: should return an error - invalid oversampling mode */
			break;
	}

												/* Configuration of USART_CR2 */
	/* Configuration of stop bits*/
	pUSARThandle->pUSARTx->USART_CR2 &= ~(3U << BIT_POS_12);
	pUSARThandle->pUSARTx->USART_CR2 |= (pUSARThandle->USART_config.USART_StopBits << BIT_POS_12);

}

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
		SET_REG_BIT(pUSARTx->USART_CR1, REG_BIT_0);
	}
	else
	{
		CLR_REG_BIT(pUSARTx->USART_CR1, REG_BIT_0);
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




