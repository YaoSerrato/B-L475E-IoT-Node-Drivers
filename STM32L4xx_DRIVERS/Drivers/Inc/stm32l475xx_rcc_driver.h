/*
 * stm32l475xx_rcc_driver.h
 *
 *  Created on: Feb 13, 2020
 *      Author: H369169
 */

#ifndef INC_STM32L475XX_RCC_DRIVER_H_
#define INC_STM32L475XX_RCC_DRIVER_H_

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
#define	RCC_MSISPEED_100K		(0)
#define	RCC_MSISPEED_200K		(1)
#define	RCC_MSISPEED_400K		(2)
#define	RCC_MSISPEED_800K		(3)
#define	RCC_MSISPEED_1M			(4)
#define	RCC_MSISPEED_2M			(5)
#define	RCC_MSISPEED_4M			(6)
#define	RCC_MSISPEED_8M			(7)
#define	RCC_MSISPEED_16M		(8)
#define	RCC_MSISPEED_24M		(9)
#define	RCC_MSISPEED_32M		(10)
#define	RCC_MSISPEED_48M		(11)

#define	RCC_MCOPRE_DIV1			(0)
#define	RCC_MCOPRE_DIV2			(1)
#define	RCC_MCOPRE_DIV4			(2)
#define	RCC_MCOPRE_DIV8			(3)
#define	RCC_MCOPRE_DIV16		(4)

#define	RCC_MCOSEL_NOMCO		(0)
#define	RCC_MCOSEL_SYSCLK		(1)
#define	RCC_MCOSEL_MSI			(2)
#define	RCC_MCOSEL_HSI16		(3)
#define	RCC_MCOSEL_HSE			(4)
#define	RCC_MCOSEL_PLLCLK		(5)
#define	RCC_MCOSEL_LSI			(6)
#define	RCC_MCOSEL_LSE			(7)
#define	RCC_MCOSEL_HSI48		(8)

#define	RCC_SYSCLK_MSI			(0)
#define	RCC_SYSCLK_HSI16		(1)
#define	RCC_SYSCLK_HSE			(2)
#define	RCC_SYSCLK_PLL			(3)

#define RCC_CFGR_SWS_MSI		(0)
#define RCC_CFGR_SWS_HSI16		(1)
#define RCC_CFGR_SWS_HSE		(2)
#define RCC_CFGR_SWS_PLL		(3)

#define RCC_HSI16_VALUE			((uint32_t)16000000U)
#define RCC_HSE_VALUE			((uint32_t)8000000U)

#define	RCC_AHBPRESCALER_DIV1		(0)
#define	RCC_AHBPRESCALER_DIV2		(8)
#define	RCC_AHBPRESCALER_DIV4		(9)
#define	RCC_AHBPRESCALER_DIV8		(0xA)
#define	RCC_AHBPRESCALER_DIV16		(0xB)
#define	RCC_AHBPRESCALER_DIV64		(0xC)
#define	RCC_AHBPRESCALER_DIV128		(0xD)
#define	RCC_AHBPRESCALER_DIV256		(0xE)
#define	RCC_AHBPRESCALER_DIV512		(0xF)

#define	RCC_PLLSRC_NOCLK			(0)
#define	RCC_PLLSRC_MSI				(1)
#define	RCC_PLLSRC_HSI16			(2)
#define	RCC_PLLSRC_HSE				(3)

#define RCC_PLLR_2					(0)
#define RCC_PLLR_4					(1)
#define RCC_PLLR_6					(2)
#define RCC_PLLR_8					(3)

#define RCC_PLLM_1					(0)
#define RCC_PLLM_2					(1)
#define RCC_PLLM_3					(2)
#define RCC_PLLM_4					(3)
#define RCC_PLLM_5					(4)
#define RCC_PLLM_6					(5)
#define RCC_PLLM_7					(6)
#define RCC_PLLM_8					(7)

/******************************************************************************/
  /* TYPEDEFS */
/******************************************************************************/
typedef enum
{
	RCC_STATUS_OK = 0,
	RCC_STATUS_ERROR = 1
}RCC_STATUS;


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
