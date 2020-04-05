/**************************************************************************//**
 * @file    stm32l475xx_gpio_driver.c
 * @brief   This file contains the function definitions for the GPIO driver
 *          for the STM32L475VG microcontroller.
 *
 * This file has 10 functions definitions (input parameters omitted):
 *      <br>1) GPIO_PeriphClkControl()  - Enables the GPIO peripheral clock. </br>
 *      <br>2) GPIO_Init()              - Initializes a GPIO pin with the given configuration. </br>
 *      <br>3) GPIO_DeInit()            - Returns every GPIO register to its default value. </br>
 *      <br>4) GPIO_ReadPin()           - Reads the state of a GPIO pin. </br>
 *      <br>5) GPIO_ReadPort()          - Reads the state of a GPIO port. </br>
 *      <br>6) GPIO_WritePin()          - Writes the state of a GPIO pin. </br>
 *      <br>7) GPIO_WritePort()         - Writes the state of a GPIO port. </br>
 *      <br>8) GPIO_TogglePin()         - Toggles the state of a GPIO pin. </br>
 *      <br>9) GPIO_IRQConfig()         - Not implemented yet. </br>
 *      <br>10) GPIO_IRQHandling()      - Not implemented yet. </br>
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

/* Here go the project includes */

/* Here go the own includes */
#include <stm32l475xx_gpio_driver.h>

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

/*****************************************************************************/
  /* STATIC VARIABLES */
/*****************************************************************************/

/*****************************************************************************/
  /* DEPENDENCIES */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DEFINITIONS */
/*****************************************************************************/

/**************************************************************************//**
* @brief       This function enables/dienables the clock for each GPIO.
*              The function enables/disables the peripheral clock by calling the
*              corresponding GPIOx_PCLK_EN()/GPIOx_PCLK_DI() macro.
*
* @param       pGPIOx   Base address of respective GPIOx.
* @param       Enabler  Determines whether the clock must be enabled or disabled.
******************************************************************************/
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

/**************************************************************************//**
* @brief       GPIO peripheral initialization.
*              API for pin & port initializations for a given GPIO peripheral.
*              This function initializes the registers of a given GPIO peripheral.
*
* @param       pGPIOHandle      Pointer to the GPIO structure for a GPIO peripheral configuration.
******************************************************************************/
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
  uint32_t temp = 0;

  /* 1. Configure the mode. */
  if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
  {
      /* This means that the mode will be from 00 to 11 only */
      temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
      pGPIOHandle->pGPIOx->GPIO_MODER &= ~(0x3 << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
      pGPIOHandle->pGPIOx->GPIO_MODER |= temp;
  }
  else
  {
    /* This means that an interrupt mode was configured */

    /* Configure the pin as input (since we are receiving an interrupt) */
    temp = GPIO_MODE_INPUT << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->GPIO_MODER &= ~(0x3 << (2*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)));
    pGPIOHandle->pGPIOx->GPIO_MODER |= temp;

    /* Configure the rising/falling edge detection */
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ITFE)
    {
      /* Configure the FTSR - Falling Trigger Selection Register */
      SET_REG_BIT(EXTI->EXTI_FTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      CLR_REG_BIT(EXTI->EXTI_RTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ITRE)
    {
      /* Configure the RTSR - Rising Trigger Selection Register */
      SET_REG_BIT(EXTI->EXTI_RTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      CLR_REG_BIT(EXTI->EXTI_FTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
    else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ITFRE)
    {
      /* Configure both: FTSR and RTSR */
      SET_REG_BIT(EXTI->EXTI_FTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
      SET_REG_BIT(EXTI->EXTI_RTSR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }
    else
    {
      /* No valid GPIO mode selected, should throw an error like GPIO_STATUS_ERROR */
    }

    /* Configure the GPIO port selection in SYSCFG_EXTICR */
    SYSCFG_PCLK_EN();
    uint8_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);

    switch(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)
    {
      case GPIO_PIN_0:
        SYSCFG->SYSCFG_EXTICR1 &= ~(0xF << 0);
        SYSCFG->SYSCFG_EXTICR1 |=  (portcode << 0);
        break;
      case GPIO_PIN_1:
        SYSCFG->SYSCFG_EXTICR1 &= ~(0xF << 4);
        SYSCFG->SYSCFG_EXTICR1 |=  (portcode << 4);
        break;
      case GPIO_PIN_2:
        SYSCFG->SYSCFG_EXTICR1 &= ~(0xF << 8);
        SYSCFG->SYSCFG_EXTICR1 |=  (portcode << 8);
        break;
      case GPIO_PIN_3:
        SYSCFG->SYSCFG_EXTICR1 &= ~(0xF << 12);
        SYSCFG->SYSCFG_EXTICR1 |=  (portcode << 12);
        break;

      case GPIO_PIN_4:
        SYSCFG->SYSCFG_EXTICR2 &= ~(0xF << 0);
        SYSCFG->SYSCFG_EXTICR2 |=  (portcode << 0);
        break;
      case GPIO_PIN_5:
        SYSCFG->SYSCFG_EXTICR2 &= ~(0xF << 4);
        SYSCFG->SYSCFG_EXTICR2 |=  (portcode << 4);
        break;
      case GPIO_PIN_6:
        SYSCFG->SYSCFG_EXTICR2 &= ~(0xF << 8);
        SYSCFG->SYSCFG_EXTICR2 |=  (portcode << 8);
        break;
      case GPIO_PIN_7:
        SYSCFG->SYSCFG_EXTICR2 &= ~(0xF << 12);
        SYSCFG->SYSCFG_EXTICR2 |=  (portcode << 12);
        break;

      case GPIO_PIN_8:
        SYSCFG->SYSCFG_EXTICR3 &= ~(0xF << 0);
        SYSCFG->SYSCFG_EXTICR3 |=  (portcode << 0);
        break;
      case GPIO_PIN_9:
        SYSCFG->SYSCFG_EXTICR3 &= ~(0xF << 4);
        SYSCFG->SYSCFG_EXTICR3 |=  (portcode << 4);
        break;
      case GPIO_PIN_10:
        SYSCFG->SYSCFG_EXTICR3 &= ~(0xF << 8);
        SYSCFG->SYSCFG_EXTICR3 |=  (portcode << 8);
        break;
      case GPIO_PIN_11:
        SYSCFG->SYSCFG_EXTICR3 &= ~(0xF << 12);
        SYSCFG->SYSCFG_EXTICR3 |=  (portcode << 12);
        break;

      case GPIO_PIN_12:
        SYSCFG->SYSCFG_EXTICR4 &= ~(0xF << 0);
        SYSCFG->SYSCFG_EXTICR4 |=  (portcode << 0);
        break;
      case GPIO_PIN_13:
        SYSCFG->SYSCFG_EXTICR4 &= ~(0xF << 4);
        SYSCFG->SYSCFG_EXTICR4 |=  (portcode << 4);
        break;
      case GPIO_PIN_14:
        SYSCFG->SYSCFG_EXTICR4 &= ~(0xF << 8);
        SYSCFG->SYSCFG_EXTICR4 |=  (portcode << 8);
        break;
      case GPIO_PIN_15:
        SYSCFG->SYSCFG_EXTICR4 &= ~(0xF << 12);
        SYSCFG->SYSCFG_EXTICR4 |=  (portcode << 12);
        break;
    }

    /* Enable the EXTI interrupt delivery to the processor using IMR */
    SET_REG_BIT(EXTI->EXTI_IMR1, pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

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

/**************************************************************************//**
* @brief        GPIO peripheral deinitialization.
*               API for pin & port deinitializations for a given GPIO peripheral.
*               This function deinitializes the registers of a given GPIO peripheral.
*               This means that the registers are assigned with their reset values.
*
* @param        pGPIOx  Base address. The pointer to the base address of a GPIO.
******************************************************************************/
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

/**************************************************************************//**
* @brief        Read process of a GPIO pin.
*               This function reads the state of a GPIO pin.
*
* @param        pGPIOx          Base address. The pointer to the base address of a GPIO.
* @param        PinNumber	Number of the pin to read.
*
* @return       Pin state stored in a uint8_t (1 == pin is set, 0 == pin is cleared).
******************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
  uint8_t value;
  value = (uint8_t) (((pGPIOx->GPIO_IDR) >> PinNumber) & (0x00000001));
  return value;
}

/**************************************************************************//**
* @brief        Read process of a GPIO port.
*               This function reads the state of a GPIO port.
*
* @param        pGPIOx          Base address. The pointer to the base address of a GPIO.
*
* @return       Port state stored in a uint16_t (1 == pin is set, 0 == pin is cleared).
******************************************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t* pGPIOx)
{
  uint16_t value;
  value = (uint16_t) pGPIOx->GPIO_IDR;
  return value;
}

/**************************************************************************//**
* @brief        Write process of a GPIO pin.
*               This function writes the state of a GPIO pin.
*
* @param        pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param        PinNumber	Number of the pin to write to.
* @param        PinValue        Value to write to the pin (1 == pin is set, 0 == pin is cleared).
******************************************************************************/
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

/**************************************************************************//**
* @brief        Write process of a GPIO port.
*               This function writes the state of a GPIO port.
*
* @param        pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param        PortValue	Value to write to the port.
******************************************************************************/
void GPIO_WritePort(GPIO_RegDef_t* pGPIOx, uint16_t PortValue)
{
  pGPIOx->GPIO_ODR = PortValue;
}

/**************************************************************************//**
* @brief        API for toggling a GPIO pin.
*
* @param        pGPIOx		Base address. The pointer to the base address of a GPIO.
* @param        PinNumber	Number of the pin to toggle.
******************************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
  pGPIOx->GPIO_ODR ^= (1 << PinNumber);
}

/**************************************************************************//**
* @brief        API for configuring the interruptions on the processor side.
*
* @param        IRQnumber       Interrupt Request Number to configure.
* @param        IRQpriority	Priority of the IRQ.
* @param        Enabler         Enabler of the IRQ to the processor.
******************************************************************************/
void GPIO_IRQConfig(uint8_t IRQnumber, uint8_t IRQpriority, uint8_t Enabler)
{
  /* Pin must be in input mode because you are receiving an interrupt */
  /* Configure the edge detection (rising edge, falling edge, both) */
  /* Enable interrupt delivery from peripheral to processor (in peripheral side) */
  /* Identify the IRQ number on which the processor accepts the interrupt from that pin */
  /* Configure the IRQ priority for the identified IRQ number (processor side on NVIC registers) */
  /* Enable interrupt reception on that IRQ number (processor side in NVIC registers) */
  /* Implement IRQ handler */

  /* Enabling/Disabling an IRQ number */
  if(Enabler == ENABLE)
  {
    /* The IRQ number interrupt will be enabled */
    if(IRQnumber <= 31)
    {
      /* Program NVIC_ISER0 */
      *NVIC_ISER0 |= (0x1UL << IRQnumber);
    }
    else if((IRQnumber > 31) & (IRQnumber < 64))
    {
      /* Program NVIC_ISER1 */
      *NVIC_ISER1 |= (0x1UL << (IRQnumber%32));
    }
    else if((IRQnumber >= 64) & (IRQnumber < 96))
    {
      /* Program NVIC_ISER2 */
      *NVIC_ISER2 |= (0x1UL << (IRQnumber%64));
    }
  }
  else
  {
    /* The IRQ number interrupt will be disabled */
    if(IRQnumber <= 31)
    {
      /* Program NVIC_ICER0 */
      *NVIC_ICER0 |= (0x1UL << IRQnumber);
    }
    else if((IRQnumber > 31) & (IRQnumber < 64))
    {
      /* Program NVIC_ICER1 */
      *NVIC_ICER1 |= (0x1UL << (IRQnumber%32));
    }
    else if((IRQnumber >= 64) & (IRQnumber < 96))
    {
      /* Program NVIC_ICER2 */
      *NVIC_ICER2 |= (0x1UL << (IRQnumber%64));
    }
  }

  /* Setting the priority for the given IRQ number */
  uint8_t IPRx = IRQnumber / 4;
  uint8_t IPRx_offset = IRQnumber % 4;
  uint8_t shift_amount = (8*IPRx_offset) + (8 - NO_PR_BITS_IMPLEMENTED);

  *(NVIC_PRIORITY_BASE_ADDRESS + (IPRx*4)) &= ~(0xFF << (8*IPRx_offset));
  *(NVIC_PRIORITY_BASE_ADDRESS + (IPRx*4)) |= (IRQpriority << shift_amount);

}

/**************************************************************************//**
* @brief        GPIO IRQ handling. This API clears the Pending Register.
*
* @param        PinNumber       Interrupt Request Number to configure.
******************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
  /* Clear the EXTI Pending Register */
  if(EXTI->EXTI_PR1 & (1 << PinNumber))
  {
    /* You clear the register by writing a 1 */
    EXTI->EXTI_PR1 |= (1 << PinNumber);
  }
}
