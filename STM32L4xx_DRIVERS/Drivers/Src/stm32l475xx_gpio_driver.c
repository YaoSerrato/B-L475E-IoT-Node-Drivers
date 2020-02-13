/*
 * STM32L475xx_GPIO_DRIV.c
 *
 *  Created on: Feb 11, 2020
 *      Author: yao
 */

#include <stm32l475xx_gpio_driver.h>

void GPIO_PeriphClkControl(GPIO_RegDef_t* pGPIOx, uint8_t Enabler)
{
	if(Enabler == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;

	/* 1. Configure the mode. */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		/* This means that the mode will be from 00 to 11 only */
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->GPIO_MODER &= ~(0x3 << 2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//pGPIOHandle->pGPIOx->GPIO_MODER &= (0xCFFFFFFF);
		//pGPIOHandle->pGPIOx->GPIO_MODER |= 0x10000000;
		pGPIOHandle->pGPIOx->GPIO_MODER |= temp;
	}
	else
	{
		/* TBCL = To be coded later */
	}
	temp = 0;

	/* 2. Configure the speed */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIO_OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIO_OSPEEDR |= temp;
	temp = 0;

	/* 3. Configure the pull-up/pull-down */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIO_PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIO_PUPDR |= temp;
	temp = 0;

	/* 4. Configure the output type */
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIO_OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIO_OTYPER |= temp;
	temp = 0;

	/* 5. Configure the alternate functionality */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber >= 0) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 7))
		{
			pGPIOHandle->pGPIOx->GPIO_AFRL &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->GPIO_AFRL |= temp;
		}
		else if((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber > 7) && (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <= 15))
		{
			pGPIOHandle->pGPIOx->GPIO_AFRH &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle->pGPIOx->GPIO_AFRH |= temp;
		}
	}

}

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}

uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (((pGPIOx->GPIO_IDR) >> PinNumber) & (0x00000001));
	return value;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->GPIO_IDR;
	return value;
}

void GPIO_WritePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t PinValue)
{
	if(PinValue == GPIO_PIN_SET)
	{
		pGPIOx->GPIO_ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->GPIO_ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t PortValue)
{
	pGPIOx->GPIO_ODR = PortValue;
}

void GPIO_TogglePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->GPIO_ODR ^= (1 << PinNumber);
}

void GPIO_IRQConfig(uint8_t IRQnumber, uint8_t IRQpriority, uint8_t Enabler);
void GPIO_IRQHandling(uint8_t PinNumber);
