/**************************************************************************//**
 * @file    stm32l475xx_rcc_driver.h
 * @brief   Header file for stm32l475xx_rcc_driver.c
 *
 * This file has 8 functions definitions (input parameters omitted):
 *      <br>1) RCC_Config_MSI()         - Configures MSI as system clock. </br>
 *      <br>2) RCC_Config_HSI()         - Configures HSI as system clock. </br>
 *      <br>3) RCC_Config_PLLCLK()      - Configures PLL as system clock. </br>
 *      <br>4) RCC_Config_LSI()         - Configures LSI. </br>
 *      <br>5) RCC_Config_MCO()         - Configures MCO pin. </br>
 *      <br>6) RCC_GetSYSCLK()          - Gets system clock value. </br>
 *      <br>7) RCC_GetHCLK()            - Gets HCLK clock value. </br>
 *      <br>8) RCC_GetMSIfreq()         - Gets the MSI range. </br>
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
#ifndef INC_STM32L475XX_RCC_DRIVER_H_
#define INC_STM32L475XX_RCC_DRIVER_H_

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

/******************************************************************************/
  /* DEFINES */
/******************************************************************************/

/** @name RCC speed values.
 */
///@{
#define	RCC_MSISPEED_100K		(0UL)
#define	RCC_MSISPEED_200K		(1UL)
#define	RCC_MSISPEED_400K		(2UL)
#define	RCC_MSISPEED_800K		(3UL)
#define	RCC_MSISPEED_1M			(4UL)
#define	RCC_MSISPEED_2M			(5UL)
#define	RCC_MSISPEED_4M			(6UL)
#define	RCC_MSISPEED_8M			(7UL)
#define	RCC_MSISPEED_16M		(8UL)
#define	RCC_MSISPEED_24M		(9UL)
#define	RCC_MSISPEED_32M		(10UL)
#define	RCC_MSISPEED_48M		(11UL)
///@}

/** @name RCC MCO prescalers.
 */
///@{
#define	RCC_MCOPRE_DIV1			(0UL)
#define	RCC_MCOPRE_DIV2			(1UL)
#define	RCC_MCOPRE_DIV4			(2UL)
#define	RCC_MCOPRE_DIV8			(3UL)
#define	RCC_MCOPRE_DIV16		(4UL)
///@}

/** @name RCC MCO output selection.
 */
///@{
#define	RCC_MCOSEL_NOMCO		(0UL)
#define	RCC_MCOSEL_SYSCLK		(1UL)
#define	RCC_MCOSEL_MSI			(2UL)
#define	RCC_MCOSEL_HSI16		(3UL)
#define	RCC_MCOSEL_HSE			(4UL)
#define	RCC_MCOSEL_PLLCLK		(5UL)
#define	RCC_MCOSEL_LSI			(6UL)
#define	RCC_MCOSEL_LSE			(7UL)
#define	RCC_MCOSEL_HSI48		(8UL)
///@}

/** @name RCC system clock selection.
 */
///@{
#define	RCC_SYSCLK_MSI			(0UL)
#define	RCC_SYSCLK_HSI16		(1UL)
#define	RCC_SYSCLK_HSE			(2UL)
#define	RCC_SYSCLK_PLL			(3UL)
///@}

/** @name RCC SWS bits.
 */
///@{
#define RCC_CFGR_SWS_MSI		(0UL)
#define RCC_CFGR_SWS_HSI16		(1UL)
#define RCC_CFGR_SWS_HSE		(2UL)
#define RCC_CFGR_SWS_PLL		(3UL)
///@}

/** @name RCC HSI and HSE clock values.
 */
///@{
#define RCC_HSI16_VALUE			((uint32_t)16000000UL)
#define RCC_HSE_VALUE			((uint32_t)8000000UL)
///@}

/** @name RCC AHB prescaler values.
 */
///@{
#define	RCC_AHBPRESCALER_DIV1		(0)
#define	RCC_AHBPRESCALER_DIV2		(8)
#define	RCC_AHBPRESCALER_DIV4		(9)
#define	RCC_AHBPRESCALER_DIV8		(0xA)
#define	RCC_AHBPRESCALER_DIV16		(0xB)
#define	RCC_AHBPRESCALER_DIV64		(0xC)
#define	RCC_AHBPRESCALER_DIV128		(0xD)
#define	RCC_AHBPRESCALER_DIV256		(0xE)
#define	RCC_AHBPRESCALER_DIV512		(0xF)
///@}

/** @name RCC PLL oscillator source.
 */
///@{
#define	RCC_PLLSRC_NOCLK		(0UL)
#define	RCC_PLLSRC_MSI			(1UL)
#define	RCC_PLLSRC_HSI16		(2UL)
#define	RCC_PLLSRC_HSE			(3UL)
///@}

/** @name RCC PLL presclaer R and M values.
 */
///@{
#define RCC_PLLR_2			(0UL)
#define RCC_PLLR_4			(1UL)
#define RCC_PLLR_6			(2UL)
#define RCC_PLLR_8			(3UL)

#define RCC_PLLM_1			(0UL)
#define RCC_PLLM_2			(1UL)
#define RCC_PLLM_3			(2UL)
#define RCC_PLLM_4			(3UL)
#define RCC_PLLM_5			(4UL)
#define RCC_PLLM_6			(5UL)
#define RCC_PLLM_7			(6UL)
#define RCC_PLLM_8			(7UL)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/
typedef enum    /**< enum of RCC function status */
{
  RCC_STATUS_OK = 0,            /**< RCC status OK */
  RCC_STATUS_ERROR = 1          /**< RCC status ERROR */
}RCC_STATUS;

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/

RCC_STATUS RCC_Config_MSI(uint32_t MSIspeed, uint32_t MSICalibrationValue, uint32_t AHB_Prescaler);
RCC_STATUS RCC_Config_LSI(uint32_t LSI_Enabler);
RCC_STATUS RCC_Config_HSI(uint32_t AHB_Prescaler);
RCC_STATUS RCC_Config_PLLCLK(uint32_t ClockSource, uint32_t ClockSourceFrequency, uint32_t PLLM, uint32_t PLLN, uint32_t PLLR, uint32_t AHB_Prescaler);
void RCC_Config_MCO(uint8_t MCOprescaler, uint8_t MCOoutput);
uint32_t RCC_GetSYSCLK(void);
uint32_t RCC_GetHCLK(void);
uint32_t RCC_GetMSIfreq(uint32_t RCC_MSISPEED);

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_RCC_DRIVER_H_ */

