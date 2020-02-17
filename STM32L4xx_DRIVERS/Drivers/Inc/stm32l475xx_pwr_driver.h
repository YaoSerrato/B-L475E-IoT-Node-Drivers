/*
 * stm32l475xx_pwr_driver.h
 *
 *  Created on: Feb 17, 2020
 *      Author: H369169
 */

#ifndef INC_STM32L475XX_PWR_DRIVER_H_
#define INC_STM32L475XX_PWR_DRIVER_H_

/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */
#include <stm32l475xx.h>

#define	PWR_VOLTAGE_RANGE_1		(1U)
#define	PWR_VOLTAGE_RANGE_2		(2U)

typedef enum
{
	PWR_STATUS_OK,
	PWR_STATUS_ERROR
}PWR_STATUS;

PWR_STATUS PWR_ControlVoltageScaling(uint32_t VoltageScaling);

#endif /* INC_STM32L475XX_PWR_DRIVER_H_ */
