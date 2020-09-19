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

/** @name USART word length macro definitions.
 */
///@{
#define	USART_WORDLENGTH_7			(2U)
#define	USART_WORDLENGTH_8			(0U)
#define	USART_WORDLENGTH_9			(1U)
///@}

/** @name USART oversampling macro definitions.
 */
///@{
#define	USART_OVERSAMPLING_8		(1U)
#define	USART_OVERSAMPLING_16		(0U)
///@}

/** @name USART parity macro definitions.
 */
///@{
#define	USART_PARITY_ENABLED		(1U)
#define	USART_PARITY_DISABLED		(0U)
#define	USART_PARITY_EVEN			(0U)
#define	USART_PARITY_ODD			(1U)
///@}

/** @name USART stop bits macro definitions.
 */
///@{
#define	USART_STOPBITS_05			(1U)
#define	USART_STOPBITS_10			(0U)
#define	USART_STOPBITS_15			(3U)
#define	USART_STOPBITS_20			(2U)
///@}


/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef struct	/**< Structure with configuration parameters for USART instance */
{
	uint8_t USART_Mode;
	uint8_t USART_Baudrate;
	uint8_t USART_StopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_ParitySelection;
	uint8_t USART_HWFlowControl;
	uint8_t USART_Oversampling;
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
