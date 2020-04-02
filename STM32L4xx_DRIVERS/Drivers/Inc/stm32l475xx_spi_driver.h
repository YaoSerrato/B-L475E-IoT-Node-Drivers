/**************************************************************************//**
 * @file    stm32l475xx_spi_driver.h
 * @brief   Header file for stm32l475xx_spi_driver.c
 *
 * This file has 7 functions declarations (input parameters omitted):
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

/* Include guard */
#ifndef INC_STM32L475XX_SPI_DRIVER_H_
#define INC_STM32L475XX_SPI_DRIVER_H_

/* For C++ */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
  /* INCLUDES */
/******************************************************************************/

/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */

/* Here go the own includes */
#include <stm32l475xx.h>

/*****************************************************************************/
  /* DEFINES */
/*****************************************************************************/

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef struct  /**< Structure for a SPI peripheral configuration */
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SCLKspeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_SSM;
}SPI_Config_t;

typedef struct  /**< Structure for a SPI peripheral configuration */
{
	SPI_RegDef_t		*pSPIx;				/**< Base address of the SPI peripheral */
	SPI_Config_t		SPIConfig;			/**< Structure for a SPI peripheral configuration */
}SPI_Handle_t;

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/

void SPI_PeriphClkControl(SPI_RegDef_t* pSPIx, uint8_t Enabler);

void SPI_Init(SPI_Handle_t* pSPIHandle);
void SPI_DeInit(SPI_RegDef_t* pSPIx);

void SPI_Transmit(SPI_RegDef_t* pSPIx, uint8_t* pTxBuffer, uint32_t DataLenght);
void SPI_Receive(SPI_RegDef_t* pSPIx, uint8_t* pRxBuffer, uint32_t DataLenght);

void SPI_IRQConfig(uint8_t IRQnumber, uint8_t IRQpriority, uint8_t Enabler);
void SPI_IRQHandling(SPI_Handle_t* pSPIHandle);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_SPI_DRIVER_H_ */
