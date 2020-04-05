/**************************************************************************//**
 * @file    stm32l475xx_spi_driver.c
 * @brief   This file contains the function definitions for the SPI driver
 *          for the STM32L475VG microcontroller.
 *
 * This file has 7 functions definitions (input parameters omitted):
 *      <br>1) SPI_PeriphClkControl()  - Enables the SPI peripheral clock. </br>
 *      <br>2) SPI_Init()              - Initializes a SPI peripheral with the given configuration. </br>
 *      <br>3) SPI_DeInit()            - Deinitializes a SPI peripheral. </br>
 *      <br>4) SPI_Transmit()          - Send date through SPI bus. </br>
 *      <br>5) SPI_Receive()           - Receive data over the SPI bus. </br>
 *      <br>6) SPI_IRQConfig()         - Configures the SPI interrupts. </br>
 *      <br>7) SPI_IRQHandling()       - Handles the SPI interrupt. </br>
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    01/April/2020
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
#include <stm32l475xx_spi_driver.h>

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
* @brief        Brief explanation of this API.
*
* @param        param1		Explanation.
* @param        param1		Explanation.
*
* @return       Explanation.
******************************************************************************/
void SPI_PeriphClkControl(SPI_RegDef_t* pSPIx, uint8_t Enabler)
{
	  if(Enabler == ENABLE)
	  {
		  if(pSPIx == SPI1)
		  {
			  SPI1_PCLK_EN();
		  }
		  else if(pSPIx == SPI2)
		  {
			  SPI2_PCLK_EN();
		  }
		  else if(pSPIx == SPI3)
		  {
			  SPI3_PCLK_EN();
		  }
		  else
		  {
			  /* Error, no SPI detected. */
		  }
	  }
	  else
	  {
		  if(pSPIx == SPI1)
		  {
			  SPI1_PCLK_DI();
		  }
		  else if(pSPIx == SPI2)
		  {
			  SPI2_PCLK_DI();
		  }
		  else if(pSPIx == SPI3)
		  {
			  SPI3_PCLK_DI();
		  }
		  else
		  {
			  /* Error, no SPI detected. */
		  }
	  }
}

/**************************************************************************//**
* @brief        Brief explanation of this API.
*
* @param        param1		Explanation.
* @param        param1		Explanation.
*
* @return       Explanation.
******************************************************************************/
void SPI_Init(SPI_Handle_t* pSPIHandle)
{
	/* Configure the serial clock baud rate */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(SPI_CR1_MASK_BR);
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPIConfig.SPI_SCLKspeed << SPI_CR1_BR_2_0);

	/* Configure clock polarity (CPOL) and clock phase (PHA) */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(SPI_CR1_MASK_CPOL);
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	pSPIHandle->pSPIx->SPI_CR1 &= ~(SPI_CR1_MASK_CPHA);
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	/* Configure the SPI bus (fullduplex, halfduplex or simplex) */
	switch(pSPIHandle->SPIConfig.SPI_BusConfig)
	{
	case SPI_BUSCONFIG_FULLDUPLEX:
		//BIDIMODE should be cleared
		CLR_REG_BIT(pSPIHandle->pSPIx->SPI_CR1, SPI_CR1_BIDIMODE);
		break;

	case SPI_BUSCONFIG_HALFDUPLEX:
		//BIDIMODE should be set
		SET_REG_BIT(pSPIHandle->pSPIx->SPI_CR1, SPI_CR1_BIDIMODE);
		break;

	case SPI_BUSCONFIG_SIMPLEX_RXONLY:
		//BIDIMODE should be cleared
		CLR_REG_BIT(pSPIHandle->pSPIx->SPI_CR1, SPI_CR1_BIDIMODE);
		//RXONLY must be set
		SET_REG_BIT(pSPIHandle->pSPIx->SPI_CR1, SPI_CR1_RXONLY);
		break;

	default:
		/* Should give an error */
	}

	/* Configure the data frame */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(SPI_CR1_MASK_LSBFIRST);
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPIConfig.SPI_FirstBit << SPI_CR1_LSBFIRST);

	pSPIHandle->pSPIx->SPI_CR2 &= ~(SPI_CR2_MASK_DS);
	pSPIHandle->pSPIx->SPI_CR2 |= (pSPIHandle->SPIConfig.SPI_DataLength << SPI_CR2_DS_3_0);

	/* Configure the software slave management */

	/* Configure the device mode (master or slave) */
	pSPIHandle->pSPIx->SPI_CR1 &= ~(SPI_CR1_MASK_MSTR);
	pSPIHandle->pSPIx->SPI_CR1 &= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
}










