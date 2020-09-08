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
	/**/
}









