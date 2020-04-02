/**************************************************************************//**
 * @file    stm32l475xx.h
 * @brief   Header file with definitions for STM32L475xx microcontrollers.
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

/* Include guard */
#ifndef INC_STM32L475XX_H_
#define INC_STM32L475XX_H_

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


/*****************************************************************************/
  /* DEFINES */
/*****************************************************************************/

/** @name General macro definitions.
 */
///@{
#define	__vo				volatile
#define	ENABLE				(1UL)
#define	DISABLE				(0UL)
#define	SET				ENABLE
#define	RESET				DISABLE
#define	GPIO_PIN_SET			SET
#define	GPIO_PIN_RESET			RESET
///@}

/** @name Cortex-M4 NVIC registers addresses
 */
///@{
#define NVIC_ISER0                      (__vo uint32_t*)0xE000E100
#define NVIC_ISER1                      (__vo uint32_t*)0xE000E104
#define NVIC_ISER2                      (__vo uint32_t*)0xE000E108

#define NVIC_ICER0                      (__vo uint32_t*)0xE000E180
#define NVIC_ICER1                      (__vo uint32_t*)0xE000E184
#define NVIC_ICER2                      (__vo uint32_t*)0xE000E188

#define NVIC_PRIORITY_BASE_ADDRESS      (__vo uint32_t*)0xE000E400

#define NO_PR_BITS_IMPLEMENTED          (4)
///@}

/** @name Macros for operations with registers.
 */
///@{
#define READ_REG_BIT(REG, N)		((((unsigned) REG) >> (N)) & (1UL))
#define SET_REG_BIT(REG, N)		(REG |=  (0x1UL << N))
#define CLR_REG_BIT(REG, N) 		(REG &= ~(0x1UL << N))
///@}

/** @name Register bits macro definitions.
 */
///@{
#define	REG_BIT_0			(0UL)
#define	REG_BIT_1			(1UL)
#define	REG_BIT_2			(2UL)
#define	REG_BIT_3			(3UL)
#define	REG_BIT_4			(4UL)
#define	REG_BIT_5			(5UL)
#define	REG_BIT_6			(6UL)
#define	REG_BIT_7			(7UL)
#define	REG_BIT_8			(8UL)
#define	REG_BIT_9			(9UL)
#define	REG_BIT_10			(10UL)
#define	REG_BIT_11			(11UL)
#define	REG_BIT_12			(12UL)
#define	REG_BIT_13			(13UL)
#define	REG_BIT_14			(14UL)
#define	REG_BIT_15			(15UL)
#define	REG_BIT_16			(16UL)
#define	REG_BIT_17			(17UL)
#define	REG_BIT_18			(18UL)
#define	REG_BIT_19			(19UL)
#define	REG_BIT_20			(20UL)
#define	REG_BIT_21			(21UL)
#define	REG_BIT_22			(22UL)
#define	REG_BIT_23			(23UL)
#define	REG_BIT_24			(24UL)
#define	REG_BIT_25			(25UL)
#define	REG_BIT_26			(26UL)
#define	REG_BIT_27			(27UL)
#define	REG_BIT_28			(28UL)
#define	REG_BIT_29			(29UL)
#define	REG_BIT_30			(30UL)
#define	REG_BIT_31			(31UL)
///@}

/** @name IRQ numbers for STM32L475VG.
 */
///@{
#define IRQ_NO_EXTI0                    (6)
#define IRQ_NO_EXTI1                    (7)
#define IRQ_NO_EXTI2                    (8)
#define IRQ_NO_EXTI3                    (9)
#define IRQ_NO_EXTI4                    (10)
#define IRQ_NO_EXTI9_5                  (23)
#define IRQ_NO_EXTI15_10                (40)
///@}

/** @name Base addresses for Flash and SRAM memories.
 */
///@{
#define	FLASH_BASE_ADDRESS		(0x08000000U)
#define SRAM1_BASE_ADDRESS		(0x20000000U)
#define SRAM2_BASE_ADDRESS		(0x10000000U)
///@}

/** @name Base addresses for Peripheral Buses.
 */
///@{
#define	PERIPH_BASE_ADDRESS		(0x40000000U)
#define APB1PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x0U)
#define APB2PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x00010000U)
#define AHB1PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x00020000U)
#define AHB2PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x08000000U)
///@}

/** @name Base addresses of AHB1 Peripherals.
 */
///@{
#define DMA1_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x0000U)
#define DMA2_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x0400U)
#define RCC_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x1000U)
#define FLASHREG_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x2000U)
#define CRC_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x3000U)
#define TSC_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x4000U)
///@}

/** @name Base addresses of AHB2 Peripherals.
 */
///@{
#define GPIOA_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x0000U)
#define GPIOB_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x0400U)
#define GPIOC_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x0800U)
#define GPIOD_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x0C00U)
#define GPIOE_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x1000U)
#define GPIOF_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x1400U)
#define GPIOG_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x1800U)
#define GPIOH_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x1C00U)
#define	OTGFS_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x08000000U)
#define	ADC_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x08040000U)
#define	RNG_BASE_ADDRESS		(AHB2PERIPH_BASE_ADDRESS + 0x08060800U)
///@}

/** @name Base addresses of APB2 Peripherals.
 */
///@{
#define	SYSCFG_BASE_ADDRESS		(APB2PERIPH_BASE_ADDRESS + 0x0000U)
#define EXTI_BASE_ADDRESS               (APB2PERIPH_BASE_ADDRESS + 0x400U)
///@}

/** @name Base addresses of APB1 Peripherals.
 */
///@{
#define	PWR_BASE_ADDRESS		(APB1PERIPH_BASE_ADDRESS + 0x7000U)
///@}

/*****************************************************************************/
  /* TYPEDEFS */
/*****************************************************************************/

typedef struct  /**< Peripheral register definition structure for GPIOx */
{
  __vo uint32_t GPIO_MODER;		/* Address offset:	0x00 */
  __vo uint32_t GPIO_OTYPER;		/* Address offset:	0x04 */
  __vo uint32_t GPIO_OSPEEDR;		/* Address offset:	0x08 */
  __vo uint32_t GPIO_PUPDR;		/* Address offset:	0x0C */
  __vo uint32_t GPIO_IDR;		/* Address offset:	0x10 */
  __vo uint32_t GPIO_ODR;		/* Address offset:	0x14 */
  __vo uint32_t GPIO_BSRR;		/* Address offset:	0x18 */
  __vo uint32_t GPIO_LCKR;		/* Address offset:	0x1C */
  __vo uint32_t GPIO_AFRL;		/* Address offset:	0x20 */
  __vo uint32_t GPIO_AFRH;		/* Address offset:	0x24 */
  __vo uint32_t GPIO_BRR;		/* Address offset:	0x28 */
  __vo uint32_t GPIO_ASCR;		/* Address offset:	0x2C */
}GPIO_RegDef_t;

/** @name GPIO registers base addresses.
 */
///@{
#define	GPIOA				((GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)
#define	GPIOB				((GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)
#define	GPIOC				((GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)
#define	GPIOD				((GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)
#define	GPIOE				((GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)
#define	GPIOF				((GPIO_RegDef_t*) GPIOF_BASE_ADDRESS)
#define	GPIOG				((GPIO_RegDef_t*) GPIOG_BASE_ADDRESS)
#define	GPIOH				((GPIO_RegDef_t*) GPIOH_BASE_ADDRESS)
///@}

typedef struct  /**< Peripheral register definition structure for RCC */
{
  __vo uint32_t RCC_CR;			/* Address offset:	0x00 */
  __vo uint32_t RCC_ICSCR;		/* Address offset:	0x04 */
  __vo uint32_t RCC_CFGR;		/* Address offset:	0x08 */
  __vo uint32_t RCC_PLLCFGR;		/* Address offset:	0x0C */
  __vo uint32_t RCC_PLLSAI1CFGR;	/* Address offset:	0x10 */
  __vo uint32_t RCC_PLLSAI2CFGR;	/* Address offset:	0x14 */
  __vo uint32_t RCC_CIER;		/* Address offset:	0x18 */
  __vo uint32_t RCC_CIFR;		/* Address offset:	0x1C */
  __vo uint32_t RCC_CICR;		/* Address offset:	0x20 */
  uint32_t      RESERVED1;		/* Address offset:	0x24 */
  __vo uint32_t RCC_AHB1RSTR;		/* Address offset:	0x28 */
  __vo uint32_t RCC_AHB2RSTR;		/* Address offset:	0x2C */
  __vo uint32_t RCC_AHB3RSTR;		/* Address offset:	0x30 */
  uint32_t      RESERVED2;		/* Address offset:	0x34 */
  __vo uint32_t RCC_APB1RSTR1;		/* Address offset:	0x38 */
  __vo uint32_t RCC_APB1RSTR2;		/* Address offset:	0x3C */
  __vo uint32_t RCC_APB2RSTR;		/* Address offset:	0x40 */
  uint32_t 	RESERVED3;		/* Address offset:	0x44 */
  __vo uint32_t RCC_AHB1ENR;		/* Address offset:	0x48 */
  __vo uint32_t RCC_AHB2ENR;		/* Address offset:	0x4C */
  __vo uint32_t RCC_AHB3ENR;		/* Address offset:	0x50 */
  uint32_t 	RESERVED4;		/* Address offset:	0x54 */
  __vo uint32_t RCC_APB1ENR1;		/* Address offset:	0x58 */
  __vo uint32_t RCC_APB1ENR2;		/* Address offset:	0x5C */
  __vo uint32_t RCC_APB2ENR;		/* Address offset:	0x60 */
  uint32_t 	RESERVED5;		/* Address offset:	0x64 */
  __vo uint32_t RCC_AHB1SMENR;		/* Address offset:	0x68 */
  __vo uint32_t RCC_AHB2SMENR;		/* Address offset:	0x6C */
  __vo uint32_t RCC_AHB3SMENR;		/* Address offset:	0x70 */
  uint32_t 	RESERVED6;		/* Address offset:	0x74 */
  __vo uint32_t RCC_APB1SMENR1;		/* Address offset:	0x78 */
  __vo uint32_t RCC_APB1SMENR2;		/* Address offset:	0x7C */
  __vo uint32_t RCC_APB2SMENR;		/* Address offset:	0x80 */
  uint32_t 	RESERVED7;		/* Address offset:	0x84 */
  __vo uint32_t RCC_CCIPR;		/* Address offset:	0x88 */
  uint32_t	RESERVED8;		/* Address offset:	0x8C */
  __vo uint32_t RCC_BDCR;		/* Address offset:	0x90 */
  __vo uint32_t RCC_CSR;		/* Address offset:	0x94 */
  __vo uint32_t RCC_CRRCR;		/* Address offset:	0x98 */
  __vo uint32_t RCC_CCIPR2;		/* Address offset:	0x9C */
}RCC_RegDef_t;

/** @name RCC registers base address.
 */
///@{
#define	RCC				((RCC_RegDef_t*) RCC_BASE_ADDRESS)
///@}

typedef struct  /**< Peripheral register definition structure for Flash registers */
{
  __vo uint32_t FLASH_ACR;		/* Address offset:	0x00 */
  __vo uint32_t FLASH_PDKEYR;		/* Address offset:	0x04 */
  __vo uint32_t FLASH_KEYR;		/* Address offset:	0x08 */
  __vo uint32_t FLASH_OPTKEYR;		/* Address offset:	0x0C */
  __vo uint32_t FLASH_SR;		/* Address offset:	0x10 */
  __vo uint32_t FLASH_CR;		/* Address offset:	0x14 */
  __vo uint32_t FLASH_ECCR;		/* Address offset:	0x18 */
  uint32_t 	RESERVED1;		/* Address offset:	0x1C */
  __vo uint32_t FLASH_OPTR;		/* Address offset:	0x20 */
  __vo uint32_t FLASH_PCROP1SR;		/* Address offset:	0x24 */
  __vo uint32_t FLASH_PCROP1ER;		/* Address offset:	0x28 */
  __vo uint32_t FLASH_WRP1AR; 		/* Address offset:	0x2C */
  __vo uint32_t FLASH_WRP1BR;		/* Address offset:	0x30 */
  uint32_t      RESERVED2;		/* Address offset:	0x34 */
  uint32_t 	RESERVED3;		/* Address offset:	0x38 */
  uint32_t 	RESERVED4;		/* Address offset:	0x3C */
  uint32_t 	RESERVED5;		/* Address offset:	0x40 */
  __vo uint32_t FLASH_PCROP2SR;		/* Address offset:	0x44 */
  __vo uint32_t FLASH_PCROP2ER;		/* Address offset:	0x48 */
  __vo uint32_t FLASH_WRP2AR;		/* Address offset:	0x4C */
  __vo uint32_t FLASH_WRP2BR;		/* Address offset:	0x50 */
}FLASH_RegDef_t;

/** @name FLASH registers base address.
 */
///@{
#define	FLASH						((FLASH_RegDef_t*) FLASHREG_BASE_ADDRESS)
///@}

typedef struct  /**< Peripheral register definition structure for Power Control registers */
{
  __vo uint32_t PWR_CR1;		/* Address offset:	0x00 */
  __vo uint32_t PWR_CR2;		/* Address offset:	0x04 */
  __vo uint32_t PWR_CR3;		/* Address offset:	0x08 */
  __vo uint32_t PWR_CR4;		/* Address offset:	0x0C */
  __vo uint32_t PWR_SR1;		/* Address offset:	0x10 */
  __vo uint32_t PWR_SR2;		/* Address offset:	0x14 */
  __vo uint32_t PWR_SCR;		/* Address offset:	0x18 */
  uint32_t	RESERVED1;		/* Address offset:	0x1C */
  __vo uint32_t PWR_PUCRA;		/* Address offset:	0x20 */
  __vo uint32_t PWR_PDCRA;		/* Address offset:	0x24 */
  __vo uint32_t PWR_PUCRB;		/* Address offset:	0x28 */
  __vo uint32_t PWR_PDCRB;		/* Address offset:	0x2C */
  __vo uint32_t PWR_PUCRC;		/* Address offset:	0x30 */
  __vo uint32_t PWR_PDCRC;		/* Address offset:	0x34 */
  __vo uint32_t PWR_PUCRD;		/* Address offset:	0x38 */
  __vo uint32_t PWR_PDCRD;		/* Address offset:	0x3C */
  __vo uint32_t PWR_PUCRE;		/* Address offset:	0x40 */
  __vo uint32_t PWR_PDCRE;		/* Address offset:	0x44 */
  __vo uint32_t PWR_PUCRF;		/* Address offset:	0x48 */
  __vo uint32_t PWR_PDCRF;		/* Address offset:	0x4C */
  __vo uint32_t PWR_PUCRG;		/* Address offset:	0x50 */
  __vo uint32_t PWR_PDCRG;		/* Address offset:	0x54 */
  __vo uint32_t PWR_PUCRH;		/* Address offset:	0x58 */
  __vo uint32_t PWR_PDCRH;		/* Address offset:	0x5C */
  __vo uint32_t PWR_PUCRI;		/* Address offset:	0x60 */
  __vo uint32_t PWR_PDCRI;		/* Address offset:	0x64 */
}PWR_RegDef_t;

/** @name PWR registers base address.
 */
///@{
#define	PWR							((PWR_RegDef_t*) PWR_BASE_ADDRESS)
///@}

typedef struct  /**< Peripheral register definition structure for EXTI */
{
  __vo uint32_t EXTI_IMR1;              /* Address offset: 0x00 */
  __vo uint32_t EXTI_EMR1;              /* Address offset: 0x04 */
  __vo uint32_t EXTI_RTSR1;             /* Address offset: 0x08 */
  __vo uint32_t EXTI_FTSR1;             /* Address offset: 0x0C */
  __vo uint32_t EXTI_SWIER1;            /* Address offset: 0x10 */
  __vo uint32_t EXTI_PR1;               /* Address offset: 0x14 */
  uint32_t      RESERVED1;              /* Address offset: 0x18 */
  uint32_t      RESERVED2;              /* Address offset: 0x1C */
  __vo uint32_t EXTI_IMR2;              /* Address offset: 0x20 */
  __vo uint32_t EXTI_EMR2;              /* Address offset: 0x24 */
  __vo uint32_t EXTI_RTSR2;             /* Address offset: 0x28 */
  __vo uint32_t EXTI_FTSR2;             /* Address offset: 0x2C */
  __vo uint32_t EXTI_SWIER2;            /* Address offset: 0x30 */
  __vo uint32_t EXTI_PR2;               /* Address offset: 0x34 */
}EXTI_RegDef_t;

/** @name EXTI registers base address.
 */
///@{
#define	EXTI							((EXTI_RegDef_t*) EXTI_BASE_ADDRESS)
///@}

typedef struct  /**< Peripheral register definition structure for SYSCFG */
{
  __vo uint32_t SYSCFG_MEMRMP;          /* Address offset: 0x00 */
  __vo uint32_t SYSCFG_CFGR1;           /* Address offset: 0x04 */
  __vo uint32_t SYSCFG_EXTICR1;         /* Address offset: 0x08 */
  __vo uint32_t SYSCFG_EXTICR2;         /* Address offset: 0x0C */
  __vo uint32_t SYSCFG_EXTICR3;         /* Address offset: 0x10 */
  __vo uint32_t SYSCFG_EXTICR4;         /* Address offset: 0x14 */
  __vo uint32_t SYSCFG_SCSR;            /* Address offset: 0x18 */
  __vo uint32_t SYSCFG_CFGR2;           /* Address offset: 0x1C */
  __vo uint32_t SYSCFG_SWPR;            /* Address offset: 0x20 */
  __vo uint32_t SYSCFG_SKR;             /* Address offset: 0x24 */
  __vo uint32_t SYSCFG_SWPR2;           /* Address offset: 0x28 */
}SYSCFG_RegDef_t;

/** @name SYSCFG registers base address.
 */
///@{
#define	SYSCFG							((SYSCFG_RegDef_t*) SYSCFG_BASE_ADDRESS)
///@}


/** @name Clock enable/disable macros for GPIOx peripherals.
 */
///@{
#define	GPIOA_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 0))
#define	GPIOB_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 1))
#define	GPIOC_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 2))
#define	GPIOD_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 3))
#define	GPIOE_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 4))
#define	GPIOF_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 5))
#define	GPIOG_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 6))
#define	GPIOH_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 7))

#define	GPIOA_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 0))
#define	GPIOB_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 1))
#define	GPIOC_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 2))
#define	GPIOD_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 3))
#define	GPIOE_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 4))
#define	GPIOF_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 5))
#define	GPIOG_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 6))
#define	GPIOH_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 7))
///@}

/** @name Clock enable/disable macros for PWR peripheral.
 */
///@{
#define	PWR_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1 << 28))

#define	PWR_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1 << 28))
///@}

/** @name Clock enable/disable macros for PWR peripheral.
 */
///@{
#define SYSCFG_PCLK_EN()                        (RCC->RCC_APB2ENR |= (1 << 0))

#define SYSCFG_PCLK_DI()                        (RCC->RCC_APB2ENR &= ~(1 << 0))
///@}

/** @name GPIO registers reset macros.
 */
///@{
#define GPIOA_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 0)); (RCC->RCC_AHB2RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 1)); (RCC->RCC_AHB2RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 2)); (RCC->RCC_AHB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 3)); (RCC->RCC_AHB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 4)); (RCC->RCC_AHB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 5)); (RCC->RCC_AHB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 6)); (RCC->RCC_AHB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 7)); (RCC->RCC_AHB2RSTR &= ~(1 << 7)); }while(0)
///@}

/** @name Macro that converts GPIO Base Address to a binary number from 0 to 15.
 */
///@{
#define GPIO_BASEADDRESS_TO_CODE(x)             ((x == GPIOA) ? 0:\
                                                 (x == GPIOB) ? 1:\
                                                 (x == GPIOC) ? 2:\
                                                 (x == GPIOD) ? 3:\
                                                 (x == GPIOE) ? 4:\
                                                 (x == GPIOF) ? 5:\
                                                 (x == GPIOG) ? 6:\
                                                 (x == GPIOH) ? 7:0)
///@}

/*****************************************************************************/
  /* CONSTANTS */
/*****************************************************************************/

/*****************************************************************************/
  /* FUNCTION DECLARATIONS */
/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32L475XX_H_ */
