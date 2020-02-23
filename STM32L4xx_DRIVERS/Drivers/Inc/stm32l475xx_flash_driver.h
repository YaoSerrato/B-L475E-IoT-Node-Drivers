/*
 * stm32l475xx_flash_driver.h
 *
 *  Created on: Feb 18, 2020
 *      Author: H369169
 */

#ifndef INC_STM32L475XX_FLASH_DRIVER_H_
#define INC_STM32L475XX_FLASH_DRIVER_H_

/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */
#include <stm32l475xx.h>

#define	FLASH_LATENCY_ZERO_WAITSTATE		(0)
#define	FLASH_LATENCY_ONE_WAITSTATE			(1)
#define	FLASH_LATENCY_TWO_WAITSTATE			(2)
#define	FLASH_LATENCY_THREE_WAITSTATE		(3)
#define	FLASH_LATENCY_FOUR_WAITSTATE		(4)

typedef enum
{
	FLASH_STATUS_OK = 0,
	FLASH_STATUS_ERROR = 1
}FLASH_STATUS;

void FLASH_SetLatency(uint32_t freq_HCLK);

#endif /* INC_STM32L475XX_FLASH_DRIVER_H_ */
