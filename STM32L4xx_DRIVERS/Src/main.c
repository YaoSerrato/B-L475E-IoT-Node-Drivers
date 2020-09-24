/**************************************************************************//**
 * @file    main.c
 * @brief   This is the first version of the RCC module for STM32L475VG
 *          microcontroller.
 *
 * This file has two configuration functions:
 *      <br>1)  App_GPIO_Init()     - configures the GPIO pins.</br>
 *      <br>2)  App_RCC_Init()      - configures the SYSCLK and HCLK.</br>
 *
 * @version 1.0.0.0
 *
 * @author  Yaoctzin Serrato
 *
 * @date    24/February/2020
 ******************************************************************************
 * @section License
 ******************************************************************************
 *
 *
 *****************************************************************************/

/*****************************************************************************/
  /* INCLUDES */
/*****************************************************************************/
/* Here go the system header files */
#include <stdint.h>

/* Here go the project includes */
/* Like CMSIS or something */

/* Here go the own includes */
#include <stm32l475xx.h>
#include <main.h>
#include <stm32l475xx_gpio_driver.h>
#include <stm32l475xx_rcc_driver.h>
#include <stm32l475xx_pwr_driver.h>
#include <stm32l475xx_usart_driver.h>

/*****************************************************************************/
  /* DEFINES */
/*****************************************************************************/

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* PUBLIC VARIABLES */
/*****************************************************************************/
/* uint32_t freq_SYSCLK = 0; */
/* uint32_t freq_HCLK = 0; */

/*****************************************************************************/
  /* STATIC VARIABLES */
/*****************************************************************************/

/*****************************************************************************/
  /* DEPENDENCIES */
/*****************************************************************************/

int main()
{
	/* Data to send */
	uint8_t BufferTx[] = "All about that base!\n\r";

	/* Initialization of peripherals */
	App_RCC_Init();
	App_GPIO_Init();
	App_EXTI_Init();
	App_USART1_Init();

	/* freq_SYSCLK = RCC_GetSYSCLK(); */
	/* freq_HCLK = RCC_GetHCLK(); */

	while(1)
	{
		GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		USART_SendData(USART1,  BufferTx, sizeof(BufferTx));
		delay();
	}
}

/*****************************************************************************/
  /* FUNCTION DEFINITIONS */
/*****************************************************************************/

 /*************************************************************************//**
 * @brief       This function gives a delay for the toggling of LEDs. The
 *              function does not receives any parameter, neither returns
 *              anything.
 *****************************************************************************/
void delay(void)
{
  for(uint64_t i = 0 ; i < 80000 ; i++);
}

 /*************************************************************************//**
 * @brief       This function configures the system clock.
 *****************************************************************************/
void App_RCC_Init(void)
{
  /* Setting the dynamic voltage range to the range that gets up to 80 MHz (Range 1). */
  if(PWR_ControlVoltageScaling(PWR_VOLTAGE_RANGE_1) != PWR_STATUS_OK)
  {
          Error_Handler();
  }

  /* Configuring oscillator */
  //if(RCC_Config_MSI(RCC_MSISPEED_4M, 0x0U, RCC_AHBPRESCALER_DIV4) != RCC_STATUS_OK)
  if(RCC_Config_HSI(RCC_AHBPRESCALER_DIV1) != RCC_STATUS_OK)
  //if(RCC_Config_PLLCLK(RCC_PLLSRC_MSI, RCC_MSISPEED_32M, RCC_PLLM_6, 13, RCC_PLLR_2, RCC_AHBPRESCALER_DIV1) != RCC_STATUS_OK)
  //if(RCC_Config_LSI(SET) != RCC_STATUS_OK)
  {
          Error_Handler();
  }

  /* Configuring MCO pin */
  RCC_Config_MCO(RCC_MCOPRE_DIV1, RCC_MCOSEL_SYSCLK);
}

 /*************************************************************************//**
 * @brief       This function configures the GPIO pins. It sets their mode,
 *              speed, output type, enables their pull-up/pull-down resistors
 *              and enables their peripheral clocks.
 *****************************************************************************/
void App_GPIO_Init(void)
{
  GPIO_Handle_t GPIO_LED2;
  GPIO_Handle_t GPIO_MCO;
  GPIO_Handle_t GPIO_BUTTON;
  GPIO_Handle_t GPIO_USART1_TX;

  /* PORT B pins */
  /* Configuring user led */
  GPIO_LED2.pGPIOx = GPIOB;
  GPIO_LED2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
  GPIO_LED2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
  GPIO_LED2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_LOW;
  GPIO_LED2.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
  GPIO_LED2.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

  /* Configuring USART1 pins */
  GPIO_USART1_TX.pGPIOx = GPIOB;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF7;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
  GPIO_USART1_TX.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

  GPIO_PeriphClkControl(GPIOB, ENABLE);
  GPIO_Init(&GPIO_LED2);
  GPIO_Init(&GPIO_USART1_TX);

  /* PORT A pins */
  /* Configuring MCO pin */
  GPIO_MCO.pGPIOx = GPIOA;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF0;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_VERYHIGH;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
  GPIO_MCO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
  GPIO_PeriphClkControl(GPIOA, ENABLE);
  GPIO_Init(&GPIO_MCO);

  /* PORT C pins */
  /* Configuring button */
  GPIO_BUTTON.pGPIOx = GPIOC;
  GPIO_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
  GPIO_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ITFE;
  GPIO_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HIGH;
  GPIO_BUTTON.GPIO_PinConfig.GPIO_PinOType = GPIO_OTYPE_PP;
  GPIO_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;
  GPIO_PeriphClkControl(GPIOC, ENABLE);
  GPIO_Init(&GPIO_BUTTON);

}

 /*************************************************************************//**
 * @brief       This function configures the external interrupts.
 *****************************************************************************/
void App_EXTI_Init(void)
{
  GPIO_IRQConfig(IRQ_NO_EXTI15_10, 0, ENABLE);
}

/*************************************************************************//**
* @brief       This function configures the USART peripheral.
*****************************************************************************/
void App_USART1_Init(void)
{
	USART_Handle_t	USART1_VCP;

	/* Configuring USART1 */
	USART1_VCP.pUSARTx = USART1;
	USART1_VCP.USART_config.USART_Mode = USART_MODE_TXONLY;
	USART1_VCP.USART_config.USART_Baudrate = USART_STD_BAUDRATE_9600;
	USART1_VCP.USART_config.USART_Oversampling = USART_OVERSAMPLING_16;
	USART1_VCP.USART_config.USART_ParityControl = USART_PARITY_DISABLED;
	USART1_VCP.USART_config.USART_StopBits = USART_STOPBITS_10;
	USART1_VCP.USART_config.USART_WordLength = USART_WORDLENGTH_8;
	USART_PeriphClkControl(USART1, USART_CLKSOURCE_HSI16, ENABLE);
	USART_Init(&USART1_VCP);
}

void Error_Handler(void)
{
  while(1){};
}

void EXTI15_10_IRQHandler(void)
{
  /* ISR code for Handling the Interrupt */
  GPIO_TogglePin(GPIOB, GPIO_PIN_14);

  /* Clear the EXTI Pending Register */
  GPIO_IRQHandling(GPIO_PIN_13);
}

