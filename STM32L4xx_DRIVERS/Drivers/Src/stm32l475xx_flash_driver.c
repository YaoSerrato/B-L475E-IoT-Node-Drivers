/*
 * stm32l475xx_flash_driver.c
 *
 *  Created on: Feb 18, 2020
 *      Author: H369169
 */

#include <stm32l475xx_flash_driver.h>

void FLASH_SetLatency(uint32_t freq_HCLK)
{
	uint32_t LatencyValue = 0;

	/* Clearing register */
	FLASH->FLASH_ACR &= ~(0x7 << 0);

	/* For voltage Range 1 */
	if((freq_HCLK > 0) & (freq_HCLK <= 16000000UL))
	{
		/* 0 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_ZERO_WAITSTATE;
		LatencyValue = FLASH_LATENCY_ZERO_WAITSTATE;
	}
	else if((freq_HCLK > 16000000UL) & (freq_HCLK <= 32000000UL))
	{
		/* 1 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_ONE_WAITSTATE;
		LatencyValue = FLASH_LATENCY_ONE_WAITSTATE;
	}
	else if((freq_HCLK > 32000000UL) & (freq_HCLK <= 48000000UL))
	{
		/* 2 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_TWO_WAITSTATE;
		LatencyValue = FLASH_LATENCY_TWO_WAITSTATE;
	}
	else if((freq_HCLK > 48000000UL) & (freq_HCLK <= 64000000UL))
	{
		/* 3 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_THREE_WAITSTATE;
		LatencyValue = FLASH_LATENCY_THREE_WAITSTATE;
	}
	else if((freq_HCLK > 64000000UL) & (freq_HCLK <= 80000000UL))
	{
		/* 4 wait state */
		FLASH->FLASH_ACR |= FLASH_LATENCY_FOUR_WAITSTATE;
		LatencyValue = FLASH_LATENCY_FOUR_WAITSTATE;
	}
	else
	{
		/* Frequency out of range */
	}

	/* Check if this new setting is being taken into account by reading the LATENCY bits in the FLASH_ACR register */
	//while((FLASH->FLASH_ACR)&(0x00000007) != LatencyValue);

}
