/*
 * 001_led_toggle.c
 *
 *  Created on: Feb 12, 2020
 *      Author: H369169
 */

#include <stm32l475xx.h>
#include <stm32l475xx_gpio_driver.h>
#include <stm32l475xx_rcc_driver.h>

void delay(void);
void App_RCC_Init(void);
void App_GPIO_Init(void);

int main()
{
	App_RCC_Init();
	App_GPIO_Init();

	while(1)
	{
		GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		delay();
	}
}

void delay(void)
{
	for(uint64_t i = 0 ; i < 20000 ; i++);
}

void App_RCC_Init(void)
{
	RCC_Config_MSI(RCC_MSISPEED_48M, (uint8_t) 0);
	RCC_Config_MCO(0, 2);
}

void App_GPIO_Init(void)
{
	GPIO_Handle_t GPIO_LED1;
	GPIO_Handle_t GPIO_MCO;

	GPIO_LED1.pGPIOx = GPIOB;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
	GPIO_LED1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
	GPIO_PeriphClkControl(GPIOB, ENABLE);
	GPIO_Init(&GPIO_LED1);

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
