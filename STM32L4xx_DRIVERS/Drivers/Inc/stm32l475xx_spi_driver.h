/**************************************************************************//**
 * @file    stm32l475xx_spi_driver.h
 * @brief   Header file for stm32l475xx_spi_driver.c
 *
 * This file has 10 functions declarations (input parameters omitted):
 *      <br>1) GPIO_PeriphClkControl()  - Enables the GPIO peripheral clock. </br>
 *      <br>2) GPIO_Init()              - Initializes a GPIO pin with the given configuration. </br>
 *      <br>3) GPIO_DeInit()            - Returns every GPIO register to its default value. </br>
 *      <br>4) GPIO_ReadPin()           - Reads the state of a GPIO pin. </br>
 *      <br>5) GPIO_ReadPort()          - Reads the state of a GPIO port. </br>
 *      <br>6) GPIO_WritePin()          - Writes the state of a GPIO pin. </br>
 *      <br>7) GPIO_WritePort()         - Writes the state of a GPIO port. </br>
 *      <br>8) GPIO_TogglePin()         - Toggles the state of a GPIO pin. </br>
 *      <br>9) GPIO_IRQConfig()         - Not implemented yet. </br>
 *      <br>10) GPIO_IRQHandling()      - Not implemented yet. </br>
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

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_SPI_DRIVER_H_ */
