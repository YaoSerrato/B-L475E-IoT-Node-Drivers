/**************************************************************************//**
 * @file    stm32l475xx_flash_driver.h
 * @brief   Header file for stm32l475xx_flash_driver.c
 *
 * This file has 1 function definition (input parameters omitted):
 *      <br>1) FLASH_SetLatency()  - Enables the GPIO peripheral clock. </br>
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

/* Include guard */
#ifndef INC_STM32L475XX_FLASH_DRIVER_H_
#define INC_STM32L475XX_FLASH_DRIVER_H_

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

/** @name Flash latency wait states.
 */
///@{
#define	FLASH_LATENCY_ZERO_WAITSTATE		(0)
#define	FLASH_LATENCY_ONE_WAITSTATE			(1)
#define	FLASH_LATENCY_TWO_WAITSTATE			(2)
#define	FLASH_LATENCY_THREE_WAITSTATE		(3)
#define	FLASH_LATENCY_FOUR_WAITSTATE		(4)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef enum
{
	FLASH_STATUS_OK = 0,
	FLASH_STATUS_ERROR = 1
}FLASH_STATUS;

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/
void FLASH_SetLatency(uint32_t freq_HCLK);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_FLASH_DRIVER_H_ */
/* This is for hotfix_test */
