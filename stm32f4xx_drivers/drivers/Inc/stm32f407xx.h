/*
 * stm32f407xx.h
 *
 *  Created on: Mar 28, 2023
 *      Author: Mahdi Kaffel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>


#define __vo	volatile
#define __weak	__attribute__((weak))
/****************************************************** Processor Specific Details***********************************************************************/
/*
 *  ARM Cortex Mx Processor NVIC ISEx register Addresses
 */

#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)

/*
 *  ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)

/*
 *  ARM Cortex Mx Processor Priority register Calculation
 */

#define NVIC_PR_BASEADDR	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 			4


/************************************************************ MCU Specific Details***********************************************************************/
/*
 *  Base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR						0X08000000U 		/* Flash Memory Base address */
#define SRAM1_BASEADDR						0X20000000U 		/* SRAM1 Memory Base address */
#define SRAM2_BASEADDR						0x2001C000U 		/* SRAM2 Memory Base address */
#define ROM									0x1FFF0000U 		/* ROM Memory Base address */
#define SRAM								SRAM1_BASEADDR 		/* SRAM Memory Base address */

/*
 *  AHBx and APBx Peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

/*
 * Base addresses of Peripherals hanging on AHB1 bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR						(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of Peripherals hanging on APB1 bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of Peripherals hanging on APB2 bus
 */

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

/************** Peripheral register definition structures **************/

/*
 *  Note : Registers of a peripheral are specific to STM32F4 family of MCUs
 *  in case of using a different family of MCUs such as STM32Lx / STM32F0x
 *  Please check your device reference manual
 */

typedef struct
{
	__vo uint32_t MODER;			/* GPIOx_MODER Register 											Offset Address = 0x00 */
	__vo uint32_t OTYPER;			/* GPIOx_OTYPER Register 											Offset Address = 0x04 */
	__vo uint32_t OSPEEDR;			/* GPIOx_OSPEEDR Register 											Offset Address = 0x08 */
	__vo uint32_t PUPDR;			/* GPIOx_PUPDR Register 											Offset Address = 0x0C */
	__vo uint32_t IDR;				/* GPIOx_IDR Register 												Offset Address = 0x10 */
	__vo uint32_t ODR;				/* GPIOx_ODR Register 												Offset Address = 0x14 */
	__vo uint32_t BSRR;				/* GPIOx_BSRR Register 												Offset Address = 0x18 */
	__vo uint32_t LCKR;				/* GPIOx_LCKR Register 												Offset Address = 0x1C */
	__vo uint32_t AFR[2];			/* GPIOx_AFR = AFR[0]/AFRH = AFR[1] Register						Offset Address = 0x20/0x24 */

}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t IMR;				/* GPIOx_MODER Register 											Offset Address = 0x00 */
	__vo uint32_t EMR;				/* GPIOx_OTYPER Register 											Offset Address = 0x04 */
	__vo uint32_t RTSR;				/* GPIOx_OSPEEDR Register 											Offset Address = 0x08 */
	__vo uint32_t FTSR;				/* GPIOx_PUPDR Register 											Offset Address = 0x0C */
	__vo uint32_t SWIER;			/* GPIOx_IDR Register 												Offset Address = 0x10 */
	__vo uint32_t PR;				/* GPIOx_AFR = AFR[0]/AFRH = AFR[1] Register						Offset Address = 0x20 */
}EXTI_RegDef_t;


typedef struct
{
	__vo uint32_t MEMRMP;			/* SYSCFG memory re-map register 									Offset Address = 0x00 */
	__vo uint32_t PMC;				/* SYSCFG peripheral mode configuration register					Offset Address = 0x04 */
	__vo uint32_t EXTICR[4];		/* SYSCFG external interrupt configuration register 1				Offset Address = 0x08-0x14 */
	uint32_t	  RESERVED1[2];		/* SYSCFG peripheral mode configuration register					Offset Address = 0x18-0x1C */
	__vo uint32_t CMPCR;			/* Compensation cell control register								Offset Address = 0x20 */
}SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t CR;				/* RCC clock control register 										Offset Address = 0x00 */
	__vo uint32_t PLLCFGR;			/* RCC PLL configuration register 									Offset Address = 0x04 */
	__vo uint32_t CFGR;				/* RCC clock configuration register 								Offset Address = 0x08 */
	__vo uint32_t CIR;				/* RCC clock interrupt register										Offset Address = 0x0C */
	__vo uint32_t AHB1RSTR;			/* RCC AHB1 peripheral reset register 								Offset Address = 0x10 */
	__vo uint32_t AHB2RSTR;			/* RCC AHB2 peripheral reset register 								Offset Address = 0x14 */
	__vo uint32_t AHB3RSTR;			/* RCC AHB3 peripheral reset register 								Offset Address = 0x18 */
	__vo uint32_t RESERVED;			/* Reserved	0x1C						 								 				  */
	__vo uint32_t APB1RSTR;			/* RCC APB1 peripheral reset register 								Offset Address = 0x20 */
	__vo uint32_t APB2RSTR;			/* RCC APB2 peripheral reset register 								Offset Address = 0x24 */
	__vo uint32_t RESERVED1[2];		/* Reserved	0x28-0x2C						 								 			  */
	__vo uint32_t AHB1ENR;			/* RCC AHB1 peripheral Enable register 								Offset Address = 0x30 */
	__vo uint32_t AHB2ENR;			/* RCC AHB2 peripheral Enable register 								Offset Address = 0x34 */
	__vo uint32_t AHB3ENR;			/* RCC AHB3 peripheral Enable register 								Offset Address = 0x38 */
	__vo uint32_t RESERVED2;		/* Reserved	0x3C						 								 				  */
	__vo uint32_t APB1ENR;			/* RCC AHB3 peripheral Enable register 								Offset Address = 0x40 */
	__vo uint32_t APB2ENR;			/* RCC AHB3 peripheral Enable register 								Offset Address = 0x44 */
	__vo uint32_t RESERVED3[2];		/* Reserved	0x48-0x4C						 								 			  */
	__vo uint32_t AHB1LPENR;		/* RCC AHB1 peripheral clock enable in low power mode register 		Offset Address = 0x50 */
	__vo uint32_t AHB2LPENR;		/* RCC AHB2 peripheral clock enable in low power mode register 		Offset Address = 0x54 */
	__vo uint32_t AHB3LPENR;		/* RCC AHB3 peripheral clock enable in low power mode register 		Offset Address = 0x58 */
	__vo uint32_t RESERVED4;		/* Reserved	0x5C						 								 				  */
	__vo uint32_t APB1LPENR;		/* RCC APB1 peripheral clock enable in low power mode register 		Offset Address = 0x60 */
	__vo uint32_t APB2LPENR;		/* RCC APB2 peripheral clock enable in low power mode register 		Offset Address = 0x64 */
	__vo uint32_t RESERVED5[2];		/* Reserved	0x68-0x6C						 								 			  */
	__vo uint32_t BDCR;				/* RCC Backup domain control register						 		Offset Address = 0x70 */
	__vo uint32_t CSR;				/* RCC clock control & status register 								Offset Address = 0x74 */
	__vo uint32_t RESERVED6[2];		/* Reserved	0x78-0x7C						 								 			  */
	__vo uint32_t SSCGR;			/* RCC spread spectrum clock generation register 					Offset Address = 0x80 */
	__vo uint32_t PLLI2SCFGR;		/* RCC PLLI2S configuration register						 		Offset Address = 0x84 */
}RCC_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;				/* SPI control register 1 											Offset Address = 0x00 */
	__vo uint32_t CR2;				/* SPI control register 2 											Offset Address = 0x04 */
	__vo uint32_t SR;				/* SPI status register 												Offset Address = 0x08 */
	__vo uint32_t DR;				/* SPI data register 												Offset Address = 0x0C */
	__vo uint32_t CRCPR;			/* SPI CRC polynomial register 										Offset Address = 0x10 */
	__vo uint32_t RXCRCR;			/* SPI RX CRC register												Offset Address = 0x14 */
	__vo uint32_t TXCRCR;			/* SPI TX CRC register												Offset Address = 0x18 */
	__vo uint32_t I2SCFGR;			/* SPI_I2S configuration register									Offset Address = 0x1C */
	__vo uint32_t I2SPR;			/* SPI_I2S prescaler register										Offset Address = 0x20 */
}SPI_RegDef_t;


typedef struct
{
	__vo uint32_t CR1;				/* I2C control register 1 											Offset Address = 0x00 */
	__vo uint32_t CR2; 				/* I2C control register 2 											Offset Address = 0x04 */
	__vo uint32_t OAR1;				/* Own address register 1 											Offset Address = 0x08 */
	__vo uint32_t OAR2;				/* Own address register 2 											Offset Address = 0x0C */
	__vo uint32_t DR;				/* I2C Data register 												Offset Address = 0x10 */
	__vo uint32_t SR1;				/* I2C Status register 1 											Offset Address = 0x14 */
	__vo uint32_t SR2;				/* I2C Status register 2 											Offset Address = 0x18 */
	__vo uint32_t CCR;				/* I2C Clock control register 										Offset Address = 0x1C */
	__vo uint32_t TRISE;			/* I2C TRISE register 												Offset Address = 0x20 */
	__vo uint32_t FLTR;				/* I2C FLTR register 												Offset Address = 0x24 */
}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;				/* UART Status register												Offset Address = 0x00 */
	__vo uint32_t DR;				/* UART Data register												Offset Address = 0x04 */
	__vo uint32_t BRR;				/* UART Baud rate register											Offset Address = 0x08 */
	__vo uint32_t CR1;				/* UART Control register 1											Offset Address = 0x0C */
	__vo uint32_t CR2;				/* UART Control register 2											Offset Address = 0x10 */
	__vo uint32_t CR3;				/* UART Control register 3											Offset Address = 0x14 */
	__vo uint32_t GTPR;				/* UART Control register 3											Offset Address = 0x18 */

}USART_RegDef_t;

/*
 *  Peripheral definitions ( Peripheral Base Addresses Type_Casted to xxx_RegDef_t structure)
 */

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)
/*
 *  RCC definitions ( RCC Base Addresses Type_Casted to RCC_RegDef_t structure)
 */
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
/*
 *  EXTI definitions ( EXTI Base Addresses Type_Casted to EXTI_RegDef_t structure)
 */
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
/*
 *  SYSCFG definitions ( SYSCFG Base Addresses Type_Casted to SYSCFG_RegDef_t structure)
 */
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 *  SPI definitions ( SPI Base Addresses Type_Casted to SPI_RegDef_t structure)
 */
#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

/*
 *  I2C definitions ( I2C Base Addresses Type_Casted to I2C_RegDef_t structure)
 */
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)
/*
 *  UART definitions ( UART Base Addresses Type_Casted to USART_RegDef_t structure)
 */
#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)
/*
 * Clock Enable Macros for GPOIx peripherals
 */

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 8 ) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 15 ) )
#define SPI4_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 13 ) )
/*
 * Clock Enable Macros for SPIx peripherals
 */

#define USART1_PCLK_EN()		( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()		( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()			( RCC->APB1ENR |= ( 1 << 5 ) )

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN() 	( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock Disable Macros for GPOIx peripherals
 */

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 8 ) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 13 ) )

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PCLK_DI()		( RCC->APB1ENR &= ~( 1 << 5 ) )

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI() 	( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * Macros to reset GPIOx peripherals
 */
#define  GPIOA_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 0))	; ( RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define  GPIOB_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 1))	; ( RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define  GPIOC_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 2))	; ( RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define  GPIOD_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 3))	; ( RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define  GPIOE_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 4))	; ( RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define  GPIOF_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 5))	; ( RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define  GPIOG_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 6))	; ( RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define  GPIOH_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 7))	; ( RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define  GPIOI_REG_RESET()	do{( RCC->AHB1RSTR |= (1 << 8))	; ( RCC->AHB1RSTR &= ~(1 << 8));}while(0)


#define GPIO_BASEADDR_TO_CODE(x)  (	(x == GPIOA)?0:\
									(x == GPIOB)?1:\
									(x == GPIOC)?2:\
									(x == GPIOD)?3:\
									(x == GPIOE)?4:\
									(x == GPIOF)?5:\
									(x == GPIOG)?6:\
									(x == GPIOH)?7:\
									(x == GPIOI)?8:0 )

/*
 * IRQ(Interrupt Request) Number of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53
#define IRQ_NO_USART6			71

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34

#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

/*		NVIC Priority Macros		*/

#define NVIC_IRQ_PRI_0		0
#define NVIC_IRQ_PRI_1		1
#define NVIC_IRQ_PRI_2		2
#define NVIC_IRQ_PRI_3		3
#define NVIC_IRQ_PRI_4		4
#define NVIC_IRQ_PRI_5		5
#define NVIC_IRQ_PRI_6		6
#define NVIC_IRQ_PRI_7		7
#define NVIC_IRQ_PRI_8		8
#define NVIC_IRQ_PRI_9		9
#define NVIC_IRQ_PRI_10		10
#define NVIC_IRQ_PRI_11		11
#define NVIC_IRQ_PRI_12		12
#define NVIC_IRQ_PRI_13		13
#define NVIC_IRQ_PRI_14		14
#define NVIC_IRQ_PRI_15		15

/*
 * Generic Macros
 */
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

/**********************************************************************************
 * 	Bit Position definitions of SPI peripheral
 **********************************************************************************/
// SPI Control Register 1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15
// SPI Control Register 2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
// SPI status register (SPI_SR)
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
// Reset SPI Peripherals
#define  SPI1_REG_RESET()	do{( RCC->APB2ENR |= (1 << 12))	; ( RCC->APB2ENR &= ~( 1 << 12 ));}while(0)
#define  SPI2_REG_RESET()	do{( RCC->APB1ENR |= (1 << 14))	; ( RCC->APB1ENR &= ~( 1 << 14 ));}while(0)
#define  SPI3_REG_RESET()	do{( RCC->APB1ENR |= (1 << 15))	; ( RCC->APB1ENR &= ~( 1 << 15 ));}while(0)
#define  SPI4_REG_RESET()	do{( RCC->APB2ENR |= (1 << 13))	; ( RCC->APB2ENR &= ~( 1 << 13 ));}while(0)
/**********************************************************************************
 * 	Bit Position definitions of I2C peripheral
 **********************************************************************************/
// I2C Control Register 1
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15
// I2C Control Register 2
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12
// I2C Own address register 1
#define I2C_OAR1_ADD0			0
#define I2C_OAR1_ADD			1
#define I2C_OAR1_ADD2			8
#define I2C_OAR1_ADDMODE		15
// I2C Own address register 2
#define I2C_OAR2_ENDUAL			0
#define I2C_OAR2_ADD2			1
// I2C Status register 1
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15
// I2C Status register 2
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8
// I2C Clock control register
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15
// I2C TRISE register
#define I2C_TRISE_TRISE			0
// Reset SPI Peripherals
#define  I2C1_REG_RESET()	do{( RCC->APB1ENR |= (1 << 21))	; ( RCC->APB2ENR &= ~( 1 << 21 ));}while(0)
#define  I2C2_REG_RESET()	do{( RCC->APB1ENR |= (1 << 22))	; ( RCC->APB1ENR &= ~( 1 << 22 ));}while(0)
#define  I2C3_REG_RESET()	do{( RCC->APB1ENR |= (1 << 23))	; ( RCC->APB1ENR &= ~( 1 << 23 ));}while(0)
/**********************************************************************************
 * 	Bit Position definitions of USART peripheral
 **********************************************************************************/
// USART Control register 1
#define USART_CR1_SBK				0
#define USART_CR1_RWU				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M					12
#define USART_CR1_UE				13
#define USART_CR1_OVER8				15
// USART Control register 2
#define USART_CR2_ADD				0
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14
// USART Control register 3
#define USART_CR3_EIE				0
#define USART_CR3_IREN				1
#define USART_CR3_IRLP				2
#define USART_CR3_HDSEL				3
#define USART_CR3_NACK				4
#define USART_CR3_SCEN				5
#define USART_CR3_DMAR				6
#define USART_CR3_DMAT				7
#define USART_CR3_RTSE				8
#define USART_CR3_CTSE				9
#define USART_CR3_CTSIE				10
#define USART_CR3_ONEBIT			11
// USART Status register
#define USART_SR_PE					0
#define USART_SR_FE					1
#define USART_SR_NF					2
#define USART_SR_ORE				3
#define USART_SR_IDLE				4
#define USART_SR_RXNE				5
#define USART_SR_TC					6
#define USART_SR_TXE				7
#define USART_SR_LBD				8
#define USART_SR_CTS				9
// USART Status register
#define USART_BRR_DIV_FRAC			0
#define USART_BRR_DIV_MANT			4
// USART Data register
#define USART_DR					0
// Reset USART Peripherals
#define  USART1_REG_RESET()	do{( RCC->APB1ENR |= (1 << 4))	; ( RCC->APB2ENR &= ~( 1 << 4 ));}while(0)
#define  USART2_REG_RESET()	do{( RCC->APB1ENR |= (1 << 17))	; ( RCC->APB1ENR &= ~( 1 << 17 ));}while(0)
#define  USART3_REG_RESET()	do{( RCC->APB1ENR |= (1 << 18))	; ( RCC->APB1ENR &= ~( 1 << 18 ));}while(0)
#define  UART4_REG_RESET()	do{( RCC->APB1ENR |= (1 << 19))	; ( RCC->APB1ENR &= ~( 1 << 19 ));}while(0)
#define  UART5_REG_RESET()	do{( RCC->APB1ENR |= (1 << 20))	; ( RCC->APB1ENR &= ~( 1 << 20 ));}while(0)
#define  USART6_REG_RESET()	do{( RCC->APB1ENR |= (1 << 5))	; ( RCC->APB1ENR &= ~( 1 << 5 ));}while(0)




#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_uart.h"
#include "stm32f407xx_rcc_driver.h"
#endif /* INC_STM32F407XX_H_ */
