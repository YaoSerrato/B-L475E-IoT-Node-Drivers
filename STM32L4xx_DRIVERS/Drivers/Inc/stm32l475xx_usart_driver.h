/**************************************************************************//**
 * @file    stm32l475xx_usart_driver.h
 * @brief   Header file for stm32l475xx_usart_driver.c
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

/* Include guard */
#ifndef INC_STM32L475XX_USART_DRIVER_H_
#define INC_STM32L475XX_USART_DRIVER_H_

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
typedef struct	/**< Structure with configuration parameters for USART instance */
{
	unsigned char USART_Mode;
	unsigned char USART_Baudrate;
	unsigned char USART_StopBits;
	unsigned char USART_WordLength;
	unsigned char USART_Parity;
	unsigned char USART_HWFlowControl;
}USART_Config_t;

typedef struct	/**< Structure for handling a USART instance */
{
	USART_RegDef_t	*pUSARTx;
	USART_Config_t	USART_config;
}USART_Handle_t;


/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/



/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/



#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_USART_DRIVER_H_ */
