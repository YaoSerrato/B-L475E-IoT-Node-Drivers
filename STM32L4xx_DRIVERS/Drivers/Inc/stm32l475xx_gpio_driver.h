/**************************************************************************//**
 * @file    stm32l475xx_gpio_driver.h
 * @brief   Header file for stm32l475xx_gpio_driver.c
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
 * @date    24/February/2019
 ******************************************************************************
 * @section License
 ******************************************************************************
 *
 *
 *****************************************************************************/

/* Include guard */
#ifndef INC_STM32L475XX_GPIO_DRIVER_H_
#define INC_STM32L475XX_GPIO_DRIVER_H_

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

/** @name GPIO pin macro definitions.
 */
///@{
#define	GPIO_PIN_0		(0UL)
#define	GPIO_PIN_1		(1UL)
#define	GPIO_PIN_2		(2UL)
#define	GPIO_PIN_3		(3UL)
#define	GPIO_PIN_4		(4UL)
#define	GPIO_PIN_5		(5UL)
#define	GPIO_PIN_6		(6UL)
#define	GPIO_PIN_7		(7UL)
#define	GPIO_PIN_8		(8UL)
#define	GPIO_PIN_9		(9UL)
#define	GPIO_PIN_10		(10UL)
#define	GPIO_PIN_11		(11UL)
#define	GPIO_PIN_12		(12UL)
#define	GPIO_PIN_13		(13UL)
#define	GPIO_PIN_14		(14UL)
#define	GPIO_PIN_15		(15UL)
///@}

/** @name GPIO mode macro definitions.
*/
///@{
#define	GPIO_MODE_INPUT		(0UL)
#define	GPIO_MODE_OUTPUT	(1UL)
#define	GPIO_MODE_ALTFN		(2UL)
#define	GPIO_MODE_ANALOG	(3UL)
#define GPIO_MODE_ITFE		(4UL)
#define GPIO_MODE_ITRE		(5UL)
#define GPIO_MODE_ITFRE		(6UL)
///@}

/** @name GPIO output type macro definitions.
*/
///@{
#define	GPIO_OTYPE_PP		(0UL)
#define	GPIO_OTYPE_OD		(1UL)
///@}

/** @name GPIO output speed macro definitions.
*/
///@{
#define	GPIO_OSPEED_LOW		(0UL)
#define	GPIO_OSPEED_MEDIUM	(1UL)
#define	GPIO_OSPEED_HIGH	(2UL)
#define	GPIO_OSPEED_VERYHIGH    (3UL)
///@}

/** @name GPIO pull-up/pull-down macro definitions.
*/
///@{
#define	GPIO_PUPD_NONE		(0UL)
#define	GPIO_PUPD_PU		(1UL)
#define	GPIO_PUPD_PD		(2UL)
///@}

/** @name GPIO alternate function macro definitions.
*/
///@{
#define	GPIO_ALTFN_AF0		(0UL)
#define	GPIO_ALTFN_AF1		(1UL)
#define	GPIO_ALTFN_AF2		(2UL)
#define	GPIO_ALTFN_AF3		(3UL)
#define	GPIO_ALTFN_AF4		(4UL)
#define	GPIO_ALTFN_AF5		(5UL)
#define	GPIO_ALTFN_AF6		(6UL)
#define	GPIO_ALTFN_AF7		(7UL)
#define	GPIO_ALTFN_AF8		(8UL)
#define	GPIO_ALTFN_AF9		(9UL)
#define	GPIO_ALTFN_AF10		(10UL)
#define	GPIO_ALTFN_AF11		(11UL)
#define	GPIO_ALTFN_AF12		(12UL)
#define	GPIO_ALTFN_AF13		(13UL)
#define	GPIO_ALTFN_AF14		(14UL)
#define	GPIO_ALTFN_AF15		(15UL)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef struct  /**< Structure for a GPIO pin configuration */
{
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct  /**< Structure for a GPIO peripheral configuration */
{
	GPIO_RegDef_t		*pGPIOx;		/**< Base address of the GPIO peripheral */
	GPIO_PinConfig_t	GPIO_PinConfig;		/**< Structure for a GPIO pin configuration */
}GPIO_Handle_t;

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/

void GPIO_PeriphClkControl(GPIO_RegDef_t* pGPIOx, uint8_t Enabler);
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);
uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t PinValue);
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t PortValue);
void GPIO_TogglePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
void GPIO_IRQConfig(uint8_t IRQnumber, uint8_t IRQpriority, uint8_t Enabler);
void GPIO_IRQHandling(uint8_t PinNumber);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_GPIO_DRIVER_H_ */
/* Third commit on develop */
