/**************************************************************************//**
 * @file    stm32l475xx_pwr_driver.h
 * @brief   Header file for stm32l475xx_pwr_driver.c
 *
 * This file has 1 function declarations (input parameters omitted):
 *      <br>1) PWR_ControlVoltageScaling()  - Enables the GPIO peripheral clock. </br>
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    24/February/2020
 ******************************************************************************
 * @section License
 ******************************************************************************
 *
 *
 *****************************************************************************/

/* Include guard */
#ifndef INC_STM32L475XX_PWR_DRIVER_H_
#define INC_STM32L475XX_PWR_DRIVER_H_

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
#include <stm32l475xx.h>

/* Here go the own includes */

/*****************************************************************************/
  /* DEFINES */
/*****************************************************************************/

/** @name Power voltage ranges.
 */
///@{
#define	PWR_VOLTAGE_RANGE_1		(1U)
#define	PWR_VOLTAGE_RANGE_2		(2U)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef enum
{
  PWR_STATUS_OK,
  PWR_STATUS_ERROR
}PWR_STATUS;

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/
PWR_STATUS PWR_ControlVoltageScaling(uint32_t VoltageScaling);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_PWR_DRIVER_H_ */
