/*
 * STM32L475xx.h
 *
 *  Created on: Feb 10, 2020
 *      Author: Yao
 */

#ifndef INC_STM32L475XX_H_
#define INC_STM32L475XX_H_

#include <stdint.h>

/* Generic macros */
#define	__vo						volatile
#define	ENABLE						(1U)
#define	DISABLE						(0U)

#define	SET							ENABLE
#define	RESET						DISABLE

#define	GPIO_PIN_SET				SET
#define	GPIO_PIN_RESET				RESET

#define READ_REG_BIT(REG, N)		((((unsigned) REG) >> (N)) & (1U))
#define SET_REG_BIT(REG, N)			(REG |=  (1U << N))
#define CLR_REG_BIT(REG, N) 		(REG &= ~(1U << N))

#define	REG_BIT_0					(0U)
#define	REG_BIT_1					(1U)
#define	REG_BIT_2					(2U)
#define	REG_BIT_3					(3U)
#define	REG_BIT_4					(4U)
#define	REG_BIT_5					(5U)
#define	REG_BIT_6					(6U)
#define	REG_BIT_7					(7U)
#define	REG_BIT_8					(8U)
#define	REG_BIT_9					(9U)
#define	REG_BIT_10					(10U)
#define	REG_BIT_11					(11U)
#define	REG_BIT_12					(12U)
#define	REG_BIT_13					(13U)
#define	REG_BIT_14					(14U)
#define	REG_BIT_15					(15U)
#define	REG_BIT_16					(16U)
#define	REG_BIT_17					(17U)
#define	REG_BIT_18					(18U)
#define	REG_BIT_19					(19U)
#define	REG_BIT_20					(20U)
#define	REG_BIT_21					(21U)
#define	REG_BIT_22					(22U)
#define	REG_BIT_23					(23U)
#define	REG_BIT_24					(24U)
#define	REG_BIT_25					(25U)
#define	REG_BIT_26					(26U)
#define	REG_BIT_27					(27U)
#define	REG_BIT_28					(28U)
#define	REG_BIT_29					(29U)
#define	REG_BIT_30					(30U)
#define	REG_BIT_31					(31U)

/* Base addresses for Flash and SRAM memories */
#define	FLASH_BASE_ADDRESS			(0x08000000U)
#define SRAM1_BASE_ADDRESS			(0x20000000U)
#define SRAM2_BASE_ADDRESS			(0x10000000U)

/* Base addresses for Peripheral Buses */
#define	PERIPH_BASE_ADDRESS			(0x40000000U)
#define APB1PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x0U)
#define APB2PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x00010000U)
#define AHB1PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x00020000U)
#define AHB2PERIPH_BASE_ADDRESS		(PERIPH_BASE_ADDRESS + 0x08000000U)

/* Base addresses of AHB1 Peripherals */
#define DMA1_BASE_ADDRESS			(AHB1PERIPH_BASE_ADDRESS + 0x0000U)
#define DMA2_BASE_ADDRESS			(AHB1PERIPH_BASE_ADDRESS + 0x0400U)
#define RCC_BASE_ADDRESS			(AHB1PERIPH_BASE_ADDRESS + 0x1000U)
#define FLASHREG_BASE_ADDRESS		(AHB1PERIPH_BASE_ADDRESS + 0x2000U)
#define CRC_BASE_ADDRESS			(AHB1PERIPH_BASE_ADDRESS + 0x3000U)
#define TSC_BASE_ADDRESS			(AHB1PERIPH_BASE_ADDRESS + 0x4000U)

/* Base addresses of AHB2 Peripherals */
#define GPIOA_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x0000U)
#define GPIOB_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x0400U)
#define GPIOC_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x0800U)
#define GPIOD_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x0C00U)
#define GPIOE_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x1000U)
#define GPIOF_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x1400U)
#define GPIOG_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x1800U)
#define GPIOH_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x1C00U)
#define	OTGFS_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x08000000U)
#define	ADC_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x08040000U)
#define	RNG_BASE_ADDRESS			(AHB2PERIPH_BASE_ADDRESS + 0x08060800U)

/* Base addresses of APB2 Peripherals */
#define	SYSCFG_BASE_ADDRESS			(APB2PERIPH_BASE_ADDRESS + 0x0000U)

/* Base addresses of APB1 Peripherals */
#define	PWR_BASE_ADDRESS			(APB1PERIPH_BASE_ADDRESS + 0x7000U)

/* ------------------------------------------------------------------------------------------------------------------------- */
/* Peripheral register definition structure for GPIOx */
typedef struct
{
	__vo uint32_t GPIO_MODER;				/* Address offset:	0x00 */
	__vo uint32_t GPIO_OTYPER;				/* Address offset:	0x04 */
	__vo uint32_t GPIO_OSPEEDR;				/* Address offset:	0x08 */
	__vo uint32_t GPIO_PUPDR;				/* Address offset:	0x0C */
	__vo uint32_t GPIO_IDR;					/* Address offset:	0x10 */
	__vo uint32_t GPIO_ODR;					/* Address offset:	0x14 */
	__vo uint32_t GPIO_BSRR;				/* Address offset:	0x18 */
	__vo uint32_t GPIO_LCKR;				/* Address offset:	0x1C */
	__vo uint32_t GPIO_AFRL;				/* Address offset:	0x20 */
	__vo uint32_t GPIO_AFRH;				/* Address offset:	0x24 */
	__vo uint32_t GPIO_BRR;					/* Address offset:	0x28 */
	__vo uint32_t GPIO_ASCR;				/* Address offset:	0x2C */
}GPIO_RegDef_t;

#define	GPIOA						((GPIO_RegDef_t*) GPIOA_BASE_ADDRESS)
#define	GPIOB						((GPIO_RegDef_t*) GPIOB_BASE_ADDRESS)
#define	GPIOC						((GPIO_RegDef_t*) GPIOC_BASE_ADDRESS)
#define	GPIOD						((GPIO_RegDef_t*) GPIOD_BASE_ADDRESS)
#define	GPIOE						((GPIO_RegDef_t*) GPIOE_BASE_ADDRESS)
#define	GPIOF						((GPIO_RegDef_t*) GPIOF_BASE_ADDRESS)
#define	GPIOG						((GPIO_RegDef_t*) GPIOG_BASE_ADDRESS)
#define	GPIOH						((GPIO_RegDef_t*) GPIOH_BASE_ADDRESS)

/* Peripheral register definition structure for RCC */
typedef struct
{
	__vo uint32_t RCC_CR;				/* Address offset:	0x00 */
	__vo uint32_t RCC_ICSCR;			/* Address offset:	0x04 */
	__vo uint32_t RCC_CFGR;				/* Address offset:	0x08 */
	__vo uint32_t RCC_PLLCFGR;			/* Address offset:	0x0C */
	__vo uint32_t RCC_PLLSAI1CFGR;		/* Address offset:	0x10 */
	__vo uint32_t RCC_PLLSAI2CFGR;		/* Address offset:	0x14 */
	__vo uint32_t RCC_CIER;				/* Address offset:	0x18 */
	__vo uint32_t RCC_CIFR;				/* Address offset:	0x1C */
	__vo uint32_t RCC_CICR;				/* Address offset:	0x20 */
	uint32_t	  RESERVED1;			/* Address offset:	0x24 */
	__vo uint32_t RCC_AHB1RSTR;			/* Address offset:	0x28 */
	__vo uint32_t RCC_AHB2RSTR;			/* Address offset:	0x2C */
	__vo uint32_t RCC_AHB3RSTR;			/* Address offset:	0x30 */
	uint32_t 	  RESERVED2;			/* Address offset:	0x34 */
	__vo uint32_t RCC_APB1RSTR1;		/* Address offset:	0x38 */
	__vo uint32_t RCC_APB1RSTR2;		/* Address offset:	0x3C */
	__vo uint32_t RCC_APB2RSTR;			/* Address offset:	0x40 */
	uint32_t 	  RESERVED3;			/* Address offset:	0x44 */
	__vo uint32_t RCC_AHB1ENR;			/* Address offset:	0x48 */
	__vo uint32_t RCC_AHB2ENR;			/* Address offset:	0x4C */
	__vo uint32_t RCC_AHB3ENR;			/* Address offset:	0x50 */
	uint32_t 	  RESERVED4;			/* Address offset:	0x54 */
	__vo uint32_t RCC_APB1ENR1;			/* Address offset:	0x58 */
	__vo uint32_t RCC_APB1ENR2;			/* Address offset:	0x5C */
	__vo uint32_t RCC_APB2ENR;			/* Address offset:	0x60 */
	uint32_t 	  RESERVED5;			/* Address offset:	0x64 */
	__vo uint32_t RCC_AHB1SMENR;		/* Address offset:	0x68 */
	__vo uint32_t RCC_AHB2SMENR;		/* Address offset:	0x6C */
	__vo uint32_t RCC_AHB3SMENR;		/* Address offset:	0x70 */
	uint32_t 	  RESERVED6;			/* Address offset:	0x74 */
	__vo uint32_t RCC_APB1SMENR1;		/* Address offset:	0x78 */
	__vo uint32_t RCC_APB1SMENR2;		/* Address offset:	0x7C */
	__vo uint32_t RCC_APB2SMENR;		/* Address offset:	0x80 */
	uint32_t 	  RESERVED7;			/* Address offset:	0x84 */
	__vo uint32_t RCC_CCIPR;			/* Address offset:	0x88 */
	uint32_t	  RESERVED8;			/* Address offset:	0x8C */
	__vo uint32_t RCC_BDCR;				/* Address offset:	0x90 */
	__vo uint32_t RCC_CSR;				/* Address offset:	0x94 */
	__vo uint32_t RCC_CRRCR;			/* Address offset:	0x98 */
	__vo uint32_t RCC_CCIPR2;			/* Address offset:	0x9C */
}RCC_RegDef_t;

#define	RCC							((RCC_RegDef_t*) RCC_BASE_ADDRESS)

/* Peripheral register definition structure for Flash registers */
typedef struct
{
	__vo uint32_t FLASH_ACR;			/* Address offset:	0x00 */
	__vo uint32_t FLASH_PDKEYR;			/* Address offset:	0x04 */
	__vo uint32_t FLASH_KEYR;			/* Address offset:	0x08 */
	__vo uint32_t FLASH_OPTKEYR;		/* Address offset:	0x0C */
	__vo uint32_t FLASH_SR;				/* Address offset:	0x10 */
	__vo uint32_t FLASH_CR;				/* Address offset:	0x14 */
	__vo uint32_t FLASH_ECCR;			/* Address offset:	0x18 */
	uint32_t 	  RESERVED1;			/* Address offset:	0x1C */
	__vo uint32_t FLASH_OPTR;			/* Address offset:	0x20 */
	__vo uint32_t FLASH_PCROP1SR;		/* Address offset:	0x24 */
	__vo uint32_t FLASH_PCROP1ER;		/* Address offset:	0x28 */
	__vo uint32_t FLASH_WRP1AR; 		/* Address offset:	0x2C */
	__vo uint32_t FLASH_WRP1BR;			/* Address offset:	0x30 */
	uint32_t	  RESERVED2;			/* Address offset:	0x34 */
	uint32_t 	  RESERVED3;			/* Address offset:	0x38 */
	uint32_t 	  RESERVED4;			/* Address offset:	0x3C */
	uint32_t 	  RESERVED5;			/* Address offset:	0x40 */
	__vo uint32_t FLASH_PCROP2SR;		/* Address offset:	0x44 */
	__vo uint32_t FLASH_PCROP2ER;		/* Address offset:	0x48 */
	__vo uint32_t FLASH_WRP2AR;			/* Address offset:	0x4C */
	__vo uint32_t FLASH_WRP2BR;			/* Address offset:	0x50 */
}FLASH_RegDef_t;

#define	FLASH						((FLASH_RegDef_t*) FLASHREG_BASE_ADDRESS)

/* Peripheral register definition structure for Power Control registers */
typedef struct
{
	__vo uint32_t PWR_CR1;				/* Address offset:	0x00 */
	__vo uint32_t PWR_CR2;				/* Address offset:	0x04 */
	__vo uint32_t PWR_CR3;				/* Address offset:	0x08 */
	__vo uint32_t PWR_CR4;				/* Address offset:	0x0C */
	__vo uint32_t PWR_SR1;				/* Address offset:	0x10 */
	__vo uint32_t PWR_SR2;				/* Address offset:	0x14 */
	__vo uint32_t PWR_SCR;				/* Address offset:	0x18 */
	uint32_t	  RESERVED1;			/* Address offset:	0x1C */
	__vo uint32_t PWR_PUCRA;			/* Address offset:	0x20 */
	__vo uint32_t PWR_PDCRA;			/* Address offset:	0x24 */
	__vo uint32_t PWR_PUCRB;			/* Address offset:	0x28 */
	__vo uint32_t PWR_PDCRB;			/* Address offset:	0x2C */
	__vo uint32_t PWR_PUCRC;			/* Address offset:	0x30 */
	__vo uint32_t PWR_PDCRC;			/* Address offset:	0x34 */
	__vo uint32_t PWR_PUCRD;			/* Address offset:	0x38 */
	__vo uint32_t PWR_PDCRD;			/* Address offset:	0x3C */
	__vo uint32_t PWR_PUCRE;			/* Address offset:	0x40 */
	__vo uint32_t PWR_PDCRE;			/* Address offset:	0x44 */
	__vo uint32_t PWR_PUCRF;			/* Address offset:	0x48 */
	__vo uint32_t PWR_PDCRF;			/* Address offset:	0x4C */
	__vo uint32_t PWR_PUCRG;			/* Address offset:	0x50 */
	__vo uint32_t PWR_PDCRG;			/* Address offset:	0x54 */
	__vo uint32_t PWR_PUCRH;			/* Address offset:	0x58 */
	__vo uint32_t PWR_PDCRH;			/* Address offset:	0x5C */
	__vo uint32_t PWR_PUCRI;			/* Address offset:	0x60 */
	__vo uint32_t PWR_PDCRI;			/* Address offset:	0x64 */
}PWR_RegDef_t;

#define	PWR							((PWR_RegDef_t*) PWR_BASE_ADDRESS)

/* ------------------------------------------------------------------------------------------------------------------------- */
/* Clock enable macros for GPIOx peripherals */
#define	GPIOA_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 0))
#define	GPIOB_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 1))
#define	GPIOC_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 2))
#define	GPIOD_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 3))
#define	GPIOE_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 4))
#define	GPIOF_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 5))
#define	GPIOG_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 6))
#define	GPIOH_PCLK_EN()				(RCC->RCC_AHB2ENR |= (1 << 7))

/* Clock disable macros for GPIOx peripherals */
#define	GPIOA_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 0))
#define	GPIOB_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 1))
#define	GPIOC_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 2))
#define	GPIOD_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 3))
#define	GPIOE_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 4))
#define	GPIOF_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 5))
#define	GPIOG_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 6))
#define	GPIOH_PCLK_DI()				(RCC->RCC_AHB2ENR &= ~(1 << 7))

/* Clock enable macro for PWR peripheral */
#define	PWR_PCLK_EN()				(RCC->RCC_APB1ENR1 |= (1 << 28))

/* Clock disable macro for PWR peripheral */
#define	PWR_PCLK_DI()				(RCC->RCC_APB1ENR1 &= ~(1 << 28))

/* GPIO registers reset macro */
#define GPIOA_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 0)); (RCC->RCC_AHB2RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 1)); (RCC->RCC_AHB2RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 2)); (RCC->RCC_AHB2RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 3)); (RCC->RCC_AHB2RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 4)); (RCC->RCC_AHB2RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 5)); (RCC->RCC_AHB2RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 6)); (RCC->RCC_AHB2RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()			do{ (RCC->RCC_AHB2RSTR |= (1 << 7)); (RCC->RCC_AHB2RSTR &= ~(1 << 7)); }while(0)


#endif /* INC_STM32L475XX_H_ */
