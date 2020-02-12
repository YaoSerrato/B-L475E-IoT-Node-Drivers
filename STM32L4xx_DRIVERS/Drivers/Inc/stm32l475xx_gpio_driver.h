#ifndef INC_STM32L475XX_GPIO_DRIVER_H_
#define INC_STM32L475XX_GPIO_DRIVER_H_
/**************************************************************************//**
 * @file    -----------------------------
 * @brief   -----------------------------
 *
 * Here should start the detailed description. This .c file was made to test
 * -----------------------------
 * -----------------------------
 * -----------------------------
 * -----------------------------
 * -----------------------------
 *
 * @version -----------------------------
 *
 * @author  -----------------------------
 *
 * @date    -----------------------------
 ******************************************************************************
 * @section License
 * <b> COPYRIGHT (C) 2019 BY: INNCOM International, Inc. </b>
 *******************************************************************************
 *
 * Here goes the text of the Honeywell License Agreement, or any related
 * text.
 *
 ******************************************************************************/

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


/******************************************************************************/
  /* DEFINES */
/******************************************************************************/
#define	GPIO_PIN_0				0
#define	GPIO_PIN_1				1
#define	GPIO_PIN_2				2
#define	GPIO_PIN_3				3
#define	GPIO_PIN_4				4
#define	GPIO_PIN_5				5
#define	GPIO_PIN_6				6
#define	GPIO_PIN_7				7
#define	GPIO_PIN_8				8
#define	GPIO_PIN_9				9
#define	GPIO_PIN_10				10
#define	GPIO_PIN_11				11
#define	GPIO_PIN_12				12
#define	GPIO_PIN_13				13
#define	GPIO_PIN_14				14
#define	GPIO_PIN_15				15

#define	GPIO_MODE_INPUT			0
#define	GPIO_MODE_OUTPUT		1
#define	GPIO_MODE_ALTFN			2
#define	GPIO_MODE_ANALOG		3
#define GPIO_MODE_ITFE			4
#define GPIO_MODE_ITRE			5
#define GPIO_MODE_ITFRE			6

#define	GPIO_OTYPE_PP			0
#define	GPIO_OTYPE_OD			1

#define	GPIO_OSPEED_LOW			0
#define	GPIO_OSPEED_MEDIUM		1
#define	GPIO_OSPEED_HIGH		2
#define	GPIO_OSPEED_VERYHIGH	3

#define	GPIO_PUPD_NONE			0
#define	GPIO_PUPD_PU			1
#define	GPIO_PUPD_PD			2

#define	GPIO_ALTFN_AF0			0
#define	GPIO_ALTFN_AF1			1
#define	GPIO_ALTFN_AF2			2
#define	GPIO_ALTFN_AF3			3
#define	GPIO_ALTFN_AF4			4
#define	GPIO_ALTFN_AF5			5
#define	GPIO_ALTFN_AF6			6
#define	GPIO_ALTFN_AF7			7
#define	GPIO_ALTFN_AF8			8
#define	GPIO_ALTFN_AF9			9
#define	GPIO_ALTFN_AF10			10
#define	GPIO_ALTFN_AF11			11
#define	GPIO_ALTFN_AF12			12
#define	GPIO_ALTFN_AF13			13
#define	GPIO_ALTFN_AF14			14
#define	GPIO_ALTFN_AF15			15

/******************************************************************************/
  /* TYPEDEFS */
/******************************************************************************/
typedef struct							/**< Structure for a GPIO pin configuration */
{
	uint8_t	GPIO_PinNumber;
	uint8_t	GPIO_PinMode;
	uint8_t	GPIO_PinSpeed;
	uint8_t	GPIO_PinPuPdControl;
	uint8_t	GPIO_PinOType;
	uint8_t	GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct							/**< Structure for a GPIO peripheral configuration */
{
	GPIO_RegDef_t		*pGPIOx;			/**< This will hold the base address of the GPIO peripheral */
	GPIO_PinConfig_t	GPIO_PinConfig;		/**< This is for configuring a GPIO pin */
}GPIO_Handle_t;

/******************************************************************************/
  /* CONSTANTS */
/******************************************************************************/



/******************************************************************************/
  /* FUNCTION DECLARATIONS */
/******************************************************************************/

/**************************************************************************//**
* @brief GPIO peripheral clock setup.
* API for configuring the clock for the GPIO peripheral.
* This function enables or disables the GPIO clock for a given GPIO peripheral.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param   Enabler		Enabler. To enable or disable the GPIO clock.
*
* @return  No return.
*****************************************************************************/
void GPIO_PeriphClkControl(GPIO_RegDef_t* pGPIOx, uint8_t Enabler);

/**************************************************************************//**
* @brief GPIO peripheral initialization.
* API for pin & port initializations for a given GPIO peripheral.
* This function initializes the registers of a given GPIO peripheral.
*
* @param   pGPIOHandle	Pointer to the GPIO structure for a GPIO peripheral configuration.
*
* @return  No return.
*****************************************************************************/
void GPIO_Init(GPIO_Handle_t* pGPIOHandle);

/**************************************************************************//**
* @brief GPIO peripheral deinitialization.
* API for pin & port deinitializations for a given GPIO peripheral.
* This function deinitializes the registers of a given GPIO peripheral.
* This means that the registers are assigned with their reset values.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
*
* @return  No return.
*****************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/**************************************************************************//**
* @brief Read process of a GPIO pin.
* This function reads the state of a GPIO pin.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param   PinNumber	Number of the pin to read.
*
* @return  Pin state stored in a uint8_t (1 == pin is set, 0 == pin is cleared).
*****************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/**************************************************************************//**
* @brief Read process of a GPIO port.
* This function reads the state of a GPIO port.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
*
* @return  Port state stored in a uint16_t (1 == pin is set, 0 == pin is cleared).
*****************************************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx);

/**************************************************************************//**
* @brief Write process of a GPIO pin.
* This function writes the state of a GPIO pin.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param   PinNumber	Number of the pin to write to.
* @param   PinValue		Value to write to the pin (1 == pin is set, 0 == pin is cleared).
*
* @return  No return.
*****************************************************************************/
void GPIO_WritePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t PinValue);

/**************************************************************************//**
* @brief Write process of a GPIO port.
* This function writes the state of a GPIO port.
*
* @param   pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param   PortValue	Value to write to the port.
*
* @return  No return.
*****************************************************************************/
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t PortValue);

void GPIO_TogglePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);					/* API for toggle output port */
void GPIO_IRQConfig(uint8_t IRQnumber, uint8_t IRQpriority, uint8_t Enabler);					/* API for configuring the IRQ */
void GPIO_IRQHandling(uint8_t PinNumber);				/* API for processing the IRQ */

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_GPIO_DRIVER_H_ */
