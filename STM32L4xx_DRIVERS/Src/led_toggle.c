/*
 * 001_led_toggle.c
 *
 *  Created on: Feb 12, 2020
 *      Author: H369169
 */

#include <stm32l475xx.h>
#include <led_toggle.h>
#include <stm32l475xx_gpio_driver.h>
#include <stm32l475xx_rcc_driver.h>
#include <stm32l475xx_pwr_driver.h>

int main()
{
	uint32_t freq_SYSCLK = 0;
	uint32_t freq_HCLK = 0;

	App_RCC_Init();
	App_GPIO_Init();

	freq_SYSCLK = RCC_GetSYSCLK();
	freq_HCLK = RCC_GetHCLK();

	while(1)
	{
		GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		delay();
		//GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		//delay();
		//GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//delay();
		//GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//delay();
	}
}

void delay(void)
{
	for(uint64_t i = 0 ; i < 20000 ; i++);
}

void App_RCC_Init(void)
{
	/* Setting the dynamic voltage range to the range that gets up to 80 MHz (Range 1). */
	if(PWR_ControlVoltageScaling(PWR_VOLTAGE_RANGE_1) != PWR_STATUS_OK)
	{
		Error_Handler();
	}

	/* Configuring oscillator */
	//if(RCC_Config_MSI(RCC_MSISPEED_8M, 0x0U, RCC_AHBPRESCALER_DIV8) != RCC_STATUS_OK)
	if(RCC_Config_HSI(RCC_AHBPRESCALER_DIV16) != RCC_STATUS_OK)
	//if(RCC_Config_LSI(SET) != RCC_STATUS_OK)
	{
		Error_Handler();
	}

	/* Configuring MCO pin */
	RCC_Config_MCO(RCC_MCOPRE_DIV1, RCC_MCOSEL_SYSCLK);
}


void App_GPIO_Init(void)
{
	GPIO_Handle_t GPIO_LED1;
	GPIO_Handle_t GPIO_LED2;
	GPIO_Handle_t GPIO_MCO;

	GPIO_LED1.pGPIOx = GPIOA;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	GPIO_PeriphClkControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_LED1);

	GPIO_LED2.pGPIOx = GPIOB;
	GPIO_LED2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_LED2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO_LED2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
	GPIO_LED2.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GPIO_LED2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	GPIO_PeriphClkControl(GPIOB, ENABLE);
	GPIO_Init(&GPIO_LED2);

	GPIO_MCO.pGPIOx = GPIOA;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF0;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VERYHIGH;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GPIO_MCO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	GPIO_PeriphClkControl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_MCO);
}

void Error_Handler(void)
{
	while(1);
}
