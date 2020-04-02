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

/** @name SPI device modes.
 */
///@{
#define	SPI_DEVICE_MODE_SLAVE	(0UL)
#define	SPI_DEVICE_MODE_MASTER	(1UL)
///@}

/** @name SPI bus configurations.
 */
///@{
#define	SPI_BUSCONFIG_FULLDUPLEX		(0UL)
#define SPI_BUSCONFIG_HALFDUPLEX		(1UL)
#define SPI_BUSCONFIG_SIMPLEX_TXONLY	(2UL)
#define SPI_BUSCONFIG_SIMPLEX_RXONLY	(3UL)
///@}

/** @name SPI SCLK pre scalers.
 */
///@{
#define	SPI_PCLK_DIV2			(0UL)
#define	SPI_PCLK_DIV4			(1UL)
#define	SPI_PCLK_DIV8			(2UL)
#define	SPI_PCLK_DIV16			(3UL)
#define	SPI_PCLK_DIV32			(4UL)
#define	SPI_PCLK_DIV64			(5UL)
#define	SPI_PCLK_DIV128			(6UL)
#define	SPI_PCLK_DIV256			(7UL)
///@}

/** @name SPI data frame format.
 */
///@{
#define	SPI_DATAFRAME_4_BITS		(3UL)
#define	SPI_DATAFRAME_5_BITS		(4UL)
#define	SPI_DATAFRAME_6_BITS		(5UL)
#define	SPI_DATAFRAME_7_BITS		(6UL)
#define	SPI_DATAFRAME_8_BITS		(7UL)
#define	SPI_DATAFRAME_9_BITS		(8UL)
#define	SPI_DATAFRAME_10_BITS		(9UL)
#define	SPI_DATAFRAME_11_BITS		(10UL)
#define	SPI_DATAFRAME_12_BITS		(11UL)
#define	SPI_DATAFRAME_13_BITS		(12UL)
#define	SPI_DATAFRAME_14_BITS		(13UL)
#define	SPI_DATAFRAME_15_BITS		(14UL)
#define	SPI_DATAFRAME_16_BITS		(15UL)

#define	SPI_DATAFRAME_MSB_FIRST		(0UL)
#define	SPI_DATAFRAME_LSB_FIRST		(1UL)
///@}

/** @name SPI clock polarity.
 */
///@{
#define	SPI_CPOL_IDLE_LOW			(0UL)
#define	SPI_CPOL_IDLE_HIGH			(1UL)
///@}

/** @name SPI clock phase.
 */
///@{
#define	SPI_CPHA_FIRST_EDGE			(0UL)
#define	SPI_CPHA_SECOND_EDGE		(1UL)
///@}

/** @name SPI slave select management.
 */
///@{
#define	SPI_SSM_DISABLE			(0UL)
#define	SPI_SSM_ENABLE			(1UL)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef struct  /**< Structure for a SPI peripheral configuration */
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SCLKspeed;
	uint8_t	SPI_DataLength;
	uint8_t SPI_MSBLSB;
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
