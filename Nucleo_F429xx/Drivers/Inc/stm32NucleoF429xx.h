/*
 * stm32NucleoF429xx.h
 *
 *  Created on: Jan 15, 2024
 *      Author: Raghavender Dornala
 */

#ifndef DRIVERS_INC_STM32NUCLEOF429XX_H_
#define DRIVERS_INC_STM32NUCLEOF429XX_H_

#include<stdint.h>

#define __vo volatile


/*
 *****************************ARM CORTEX-M4 specific registers********************************************
 */
/*
 * NVIC Interrupt Set Enable Registers up to 0xE000E11F
 * (Total 240 interrupts of ISER0 to ISER7 each having 32 interrupts)
 * offset = 0xE000E100 + 0x04 * x (x = 0,1,2...7)
 */
#define NVIC_ISER_BASEADDR				((__vo uint32_t *)0xE000E100)

/* NVIC Clear Set Enable Registers up to 0xE000E19F */
#define NVIC_ICER_BASEADDR				((__vo uint32_t *)0xE000E180)

/* Total 60 priority Registers field for each interrupt (IPR0 to IPR59)
 * Each Register(32 bit) holds 4(8 bits) priorities fields
 * only 4bits are programmed[7-4] in each field.
 * offset = 0xE000E400 + 0x04 * x (x = 0,1,2...59)
 */
#define NVIC_IPR_BASEADDR				((__vo uint32_t *)0xE000E400)		/* NVIC Interrupt Priority Registers */

#define No_OF_PRI_BITS_IMPLEMENTED		4	/*(Each priority number has 8[7-0] bits and implemented only 4 bits [7-4]) */

/*
 * NVIC Interrupt Set Enable Registers (ISERx)
 */
#define NVIC_ISER0						((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1						((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2						((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3						((__vo uint32_t *)0xE000E10C)
#define NVIC_ISER4						((__vo uint32_t *)0xE000E110)
#define NVIC_ISER5						((__vo uint32_t *)0xE000E114)
#define NVIC_ISER6						((__vo uint32_t *)0xE000E118)
#define NVIC_ISER7						((__vo uint32_t *)0xE000E11C)

/*
 * NVIC Interrupt Clear Enable Registers (ICERx)
 */
#define NVIC_ICER0						((__vo uint32_t *)0xE000E180)
#define NVIC_ICER1						((__vo uint32_t *)0xE000E184)
#define NVIC_ICER2						((__vo uint32_t *)0xE000E188)
#define NVIC_ICER3						((__vo uint32_t *)0xE000E18C)
#define NVIC_ICER4						((__vo uint32_t *)0xE000E190)
#define NVIC_ICER5						((__vo uint32_t *)0xE000E194)
#define NVIC_ICER6						((__vo uint32_t *)0xE000E198)
#define NVIC_ICER7						((__vo uint32_t *)0xE000E19C)


/*******************************************END OF ARM CORTEX-M4 specific registers********************************/



/*
 * Base addresses of Memory and SRAM
 * SRAM = DTCM + SRAM1 + SRAM2 + Reserved = 512KB
 */
#define FLASH_BASEADDR			0x08000000U			// 1MB upto 0x081FFFFF
#define SRAM1_BASEADDR			0x20000000U			// 112KB upto 0x2001BFFF
#define SRAM2_BASEADDR			0x2001C000U			// 16KB upto 0x2001FFFF
#define SRAM3_BASEADDR			0x20020000U			// 64KB upto 0x2002FFFF
#define ROM_BASEADDR			0x1FFF0000U			// 30KB upto 0x1FFF77FF

/*
 * AHBx and APBx peripheral base addresses
 */
#define PERIPH_BASEADDR				0x40000000U
#define APB1_PERIPH_BASEADDR		PERIPH_BASEADDR		//upto 0x40007FFF
#define APB2_PERIPH_BASEADDR		0x40010000U			//upto 0x40016BFF
#define AHB1_PERIPH_BASEADDR		0x40020000U			//upto 0x4007FFFF
#define AHB2_PERIPH_BASEADDR		0x50000000U			//upto 0x50060BFF
#define AHB3_PERIPH_BASEADDR		0x60000000U			//upto 0xDFFFFFFF

#define RCC_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x3800)			//upto 0x40023BFF

#define APB1_IWDG_BASEADDR			0x40003000U			// upto 0x400033FF
#define APB1_WWDG_BASEADDR			0x40002C00U			// upto 0x40002FFF

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * AHB1 Bus Clock speed = 216 MHz
 */
#define GPIOA_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x0000)			//upto 0x400203FF
#define GPIOB_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x0400)			//upto 0x400207FF
#define GPIOC_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x0800)			//upto 0x40020BFF
#define GPIOD_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x0C00)			//upto 0x40020FFF
#define GPIOE_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x1000)			//upto 0x400213FF
#define GPIOF_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x1400)			//upto 0x400217FF
#define GPIOG_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x1800)			//upto 0x40021BFF
#define GPIOH_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x1C00)			//upto 0x40021FFF
#define GPIOI_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x2000)			//upto 0x400223FF
#define GPIOJ_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x2400)			//upto 0x400227FF
#define GPIOK_BASEADDR				(AHB1_PERIPH_BASEADDR + 0x2800)			//upto 0x40022BFF

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * APB1 Bus Clock speed = 54MHz
 */
#define I2C1_BASEADDR				(APB1_PERIPH_BASEADDR + 0x5400)			//upto 0x400057FF
#define I2C2_BASEADDR				(APB1_PERIPH_BASEADDR + 0x5800)			//upto 0x40005BFF
#define I2C3_BASEADDR				(APB1_PERIPH_BASEADDR + 0x5C00)			//upto 0x40005FFF

#define SPI2_BASEADDR				(APB1_PERIPH_BASEADDR + 0x3800)			//upto 0x40003BFF
#define SPI3_BASEADDR				(APB1_PERIPH_BASEADDR + 0x3C00)			//upto 0x40003FFF

#define USART2_BASEADDR				(APB1_PERIPH_BASEADDR + 0x4400)			//upto 0x400047FF
#define USART3_BASEADDR				(APB1_PERIPH_BASEADDR + 0x4800)			//upto 0x40004BFF
#define UART4_BASEADDR				(APB1_PERIPH_BASEADDR + 0x4C00)			//upto 0x40004FFF
#define UART5_BASEADDR				(APB1_PERIPH_BASEADDR + 0x5000)			//upto 0x400053FF
#define UART7_BASEADDR				(APB1_PERIPH_BASEADDR + 0x7800)			//upto 0x40007BFF
#define UART8_BASEADDR				(APB1_PERIPH_BASEADDR + 0x7C00)			//upto 0x40007FFF


/* Independent Watchdog(IWDG)
 * The independent watchdog (IWDG) is clocked by its own dedicated low-speed clock (LSI) and
 * thus stays active even if the main clock fails.
 * The IWDG is best suited to applications which require the WD to run as a totally
 * independent process outside the main application, but have lower timing accuracy constraints.
 *
 * structure for Independent Watchdog registers
 */

typedef struct
{
	__vo uint32_t IWDG_KR;		/* IWDG Key Register register,	 	Offset = 0x00 */
	__vo uint32_t IWDG_PR;		/* IWDG PreScaler register,			Offset = 0x04 */
	__vo uint32_t IWDG_RLR;		/* IWDG ReLoad register,			Offset = 0x08 */
	__vo uint32_t IWDG_SR;		/* IWDG Status register,			Offset = 0x0C */
}IWDG_RegDef_t;

#define IWDG			((IWDG_RegDef_t *)APB1_IWDG_BASEADDR)

/* Window Watchdog(WWDG)
 * The window watchdog (WWDG) clock is prescaled from the APB1 clock and
 * has a configurable time-window that can be programmed to detect abnormally late or early application behavior.
 * The WWDG is best suited to applications which require the watchdog to react within an accurate timing window.
 */

typedef struct
{
	__vo uint32_t WWDG_CR;		/* IWDG Key Register register,	 	Offset = 0x00 */
	__vo uint32_t WWDG_CFR;		/* IWDG PreScaler register,			Offset = 0x04 */
	__vo uint32_t WWDG_SR;		/* IWDG Status register,			Offset = 0x08 */
}WWDG_RegDef_t;

/* Early wake-up interrupt flag -> This bit is set by hardware when the counter has reached the value 0x40.
 * It must be cleared by software by writing ‘0’. This bit is also set if the interrupt is not enabled.
 */
#define WWDG_SR_EWIF_BITPOS		0

#define WWDG			((WWDG_RegDef_t *)APB1_WWDG_BASEADDR)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * APB2 Bus Clock speed = 108MHz
 */
#define EXTI_BASEADDR				(APB2_PERIPH_BASEADDR + 0x3C00)			//upto 0x40013FFF

#define SPI1_BASEADDR				(APB2_PERIPH_BASEADDR + 0x3000)			//upto 0x400133FF
#define SPI4_BASEADDR				(APB2_PERIPH_BASEADDR + 0x3400)			//upto 0x400137FF
#define SPI5_BASEADDR				(APB2_PERIPH_BASEADDR + 0x5000)			//upto 0x400153FF
#define SPI6_BASEADDR				(APB2_PERIPH_BASEADDR + 0x5400)			//upto 0x400157FF

#define SYSCFG_BASEADDR				(APB2_PERIPH_BASEADDR + 0x3800)			//upto 0x40013BFF

#define USART1_BASEADDR				(APB2_PERIPH_BASEADDR + 0x1000)			//upto 0x400113FF
#define USART6_BASEADDR				(APB2_PERIPH_BASEADDR + 0x1400)			//upto 0x400117FF


/*
 * structure for RCC registers
 */
typedef struct
{
	__vo uint32_t CR;				     /* RCC clock control register, 								 Offset = 0x00 */
	__vo uint32_t PLLCFGR;			     /* RCC PLL configuration register,  							 Offset = 0x04 */
	__vo uint32_t CFGR;				     /* RCC clock configuration register, 							 Offset = 0x08 */
	__vo uint32_t CIR;				     /* RCC clock interrupt register, 								 Offset = 0x0C */
	__vo uint32_t AHB1RSTR;			     /* RCC AHB1 peripheral reset register, 						 Offset = 0x10 */
	__vo uint32_t AHB2RSTR;			     /* RCC AHB2 peripheral reset register, 						 Offset = 0x14 */
	__vo uint32_t AHB3RSTR;			     /* RCC AHB3 peripheral reset register, 						 Offset = 0x18*/
		 uint32_t RESERVED0;		     /* Reserved , 													 Offset = 0x1C */
	__vo uint32_t APB1RSTR;			     /* RCC APB1 peripheral reset register, 						 Offset = 0x20 */
	__vo uint32_t APB2RSTR;			     /* RCC APB2 peripheral reset register, 						 Offset = 0x24 */
		 uint32_t RESERVED1[2];	     	 /* Reserved, 													 Offset = 0x28, 0x2C */
	__vo uint32_t AHB1ENR;			     /* RCC AHB1 peripheral clock register, 						 Offset = 0x30 */
	__vo uint32_t AHB2ENR;			     /* RCC AHB2 peripheral clock register, 						 Offset = 0x34 */
	__vo uint32_t AHB3ENR;			     /* RCC AHB3 peripheral clock register, 						 Offset = 0x38 */
		 uint32_t RESERVED2;		     /* Reserved, 													 Offset = 0x3C */
	__vo uint32_t APB1ENR;			     /* RCC APB1 peripheral clock register, 						 Offset = 0x40 */
	__vo uint32_t APB2ENR;			     /* RCC APB2 peripheral clock register, 						 Offset = 0x44 */
		 uint32_t RESERVED3[2];	     	 /* Reserved, 													 Offset = 0x48, 0x4C */
	__vo uint32_t AHB1LPENR;		     /* RCC AHB1 peripheral clock enable in low-power mode register, Offset = 0x50 */
	__vo uint32_t AHB2LPENR;		     /* RCC AHB2 peripheral clock enable in low-power mode register, Offset = 0x54 */
	__vo uint32_t AHB3LPENR;		     /* RCC AHB3 peripheral clock enable in low-power mode register, Offset = 0x58 */
		 uint32_t RESERVED4;		     /* Reserved , 													 Offset = 0x5C */
	__vo uint32_t APB1LPENR;		     /* RCC APB1 peripheral clock enable in low-power mode register, Offset = 0x60 */
	__vo uint32_t APB2LPENR;		     /* RCC APB2 peripheral clock enable in low-power mode register, Offset = 0x64 */
		 uint32_t RESERVED5[2];		     /* Reserved, 													 Offset = 0x68, 0x6C */
	__vo uint32_t BDCR;				     /* RCC backup domain control register, 						 Offset = 0x70 */
	__vo uint32_t CSR;				     /* RCC clock control & status register, 						 Offset = 0x74 */
		 uint32_t RESERVED6[2];		     /* Reserved, 													 Offset = 0x78, 0x7C */
	__vo uint32_t SSCGR;			     /* RCC spread spectrum clock generation register, 				 Offset = 0x80 */
	__vo uint32_t PLLI2SCFGR;		     /* RCC PLLI2S configuration register, 							 Offset = 0x84 */
	__vo uint32_t PLLSAICFGR;		     /* RCC PLLSAI configuration register, 							 Offset = 0x88 */
	__vo uint32_t DCKCFGR1;			     /* RCC dedicated clocks configuration register, 				 Offset = 0x8C */
}RCC_RegDef_t;

/*
 * Structure for EXTI Register
 *
 */
typedef struct
{
	__vo uint32_t EXTI_IMR;				/* Interrupt Mask Register,            offset = 0x00*/
	__vo uint32_t EXTI_EMR;				/* Event Mask Register,                offset = 0x04*/
	__vo uint32_t EXTI_RTSR;			/* Raising Trigger Selection Register, offset = 0x08*/
	__vo uint32_t EXTI_FTSR;			/* Falling Trigger Selection Register, offset = 0x0C*/
	__vo uint32_t EXTI_SWIER;			/* Software Interrupt Event Register,  offset = 0x10*/
	__vo uint32_t EXTI_PR;				/* Pending Register,                   offset = 0x14*/
}EXTI_RegDef_t;

/*
 * Structure for SYSCFG Register
 */
typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;				/* Memory Remap Register,            			offset = 0x00*/
	__vo uint32_t SYSCFG_PMC;					/* Peripheral Mode Configuration Register,      offset = 0x04*/
	__vo uint32_t SYSCFG_EXTICR[4];				/* External peripheral Configuration Registers, offset = 0x08, 0x0C, 0x10, 0x14*/
	uint32_t SYSCFG_Reserved;
	uint32_t SYSCFG_Reserved2;
	__vo uint32_t SYSCFG_CMPCR;					/* Compensation Cell Control Register,          offset = 0x20*/
}SYSCFG_RegDef_t;


#define RCC			((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI        ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG      ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

/*
 * Clock disable for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))
#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))


/********************************************GPIO PERIPHERAL***************************************/
/*
 * structure for GPIO peripheral registers
 */
typedef struct
{
	__vo uint32_t MODER;		/* GPIO port mode register,							Offset = 0x00 */
	__vo uint32_t OTYPER;		/* GPIO port output type register,					Offset = 0x04 */
	__vo uint32_t OSPEEDR;		/* GPIO port output speed register,					Offset = 0x08 */
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register,			Offset = 0x0C */
	__vo uint32_t IDR;			/* GPIO port input data register,					Offset = 0x10 */
	__vo uint32_t ODR;			/* GPIO port output data register,					Offset = 0x14 */
	__vo uint32_t BSRR;			/* GPIO port bit set/reset register,				Offset = 0x18 */
	__vo uint32_t LCKR;			/* GPIO port configuration lock register, 			Offset = 0x1C */
	__vo uint32_t AFRL[2];		/* AFRL[0] = GPIO alternate function low register , Offset = 0x20 and AFRL[1] = GPIO alternate function high register , Offset = 0x24 */
}GPIO_RegDef_t;

/*
 * Assigning base address of each GPIOx to the structure
 */
#define GPIOA		((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t *)GPIOK_BASEADDR)

/*
 * Clock enable for GPIO peripheral
 */
#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()					(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()					(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()					(RCC->AHB1ENR |= (1 << 10))

/*
 * Clock disable for GPIO peripherals
 */
#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 10))

/*
 * Reset GPIOx peripheral
 * this is a technique to execute multiple statements in a single macro
 */
		/*                            1. SET 					    2. RESET*/
#define GPIOA_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);}while(0)
#define GPIOI_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8);}while(0)
#define GPIOJ_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 9); RCC->AHB1RSTR &= ~(1 << 9);}while(0)
#define GPIOK_REG_RESET()			do {RCC->AHB1RSTR |= (1 << 10); RCC->AHB1RSTR &= ~(1 << 10);}while(0)


#define GPIO_BASEADDR_TO_CODE(ADDR)	  ( (ADDR == GPIOA) ? 0 : \
										(ADDR == GPIOB) ? 1 : \
										(ADDR == GPIOC) ? 2 : \
										(ADDR == GPIOD) ? 3 : \
										(ADDR == GPIOE) ? 4 : \
										(ADDR == GPIOF) ? 5 : \
										(ADDR == GPIOG) ? 6 : \
										(ADDR == GPIOH) ? 7 : \
										(ADDR == GPIOI) ? 8 : \
										(ADDR == GPIOJ) ? 9 : \
										(ADDR == GPIOK) ? 10 : 0 )

/*********************************************END OF GPIO PERIPHERAL****************************************/

/******************************************** SPI PERIPHERAL *************************************************/
/*
 * structure for SPI registers
 */
typedef struct
{
	__vo uint32_t SPI_CR1;			/* SPI Control Register 1, 		   offset = 0x00 */
	__vo uint32_t SPI_CR2;			/* SPI Control Register 2,		   offset = 0x04 */
	__vo uint32_t SPI_SR;			/* SPI Status Register,			   offset = 0x08 */
	__vo uint32_t SPI_DR;			/* SPI Data Register, 			   offset = 0x0C */
	__vo uint32_t SPI_CRCPR;		/* SPI CRC Polynomial Register,    offset = 0x10 */
	__vo uint32_t SPI_RxCRCR;		/* SPI RX CRC Register,			   offset = 0x14 */
	__vo uint32_t SPI_TxCRCR;		/* SPI TX CRC Register,			   offset = 0x18 */
	__vo uint32_t SPI_I2SCFGR;		/* SPI I2S Configuration Register, offset = 0x1C */
	__vo uint32_t SPI_I2SPR;		/* SPI I2S prescalar Register,	   offset = 0x20 */
}SPI_RegDef_t;

/*
 * Assigning base address of each SPIx to the structure
 */
#define SPI1			((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t *)SPI4_BASEADDR)
#define SPI5			((SPI_RegDef_t *)SPI5_BASEADDR)
#define SPI6			((SPI_RegDef_t *)SPI6_BASEADDR)

/*
 * Clock enable for SPI peripheral
 */
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()					(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()					(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()					(RCC->APB2ENR |= (1 << 21))

/*
 * Clock Disable for SPI peripheral
 */
#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 21))

/*
 * Reset SPIx peripheral
 * this is a technique to execute multiple statements in a single macro
 */
		/*                            1. SET 					    2. RESET*/
#define SPI1_REG_RESET()			do {RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12);}while(0)
#define SPI2_REG_RESET()			do {RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0)
#define SPI3_REG_RESET()			do {RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15);}while(0)
#define SPI4_REG_RESET()			do {RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13);}while(0)
#define SPI5_REG_RESET()			do {RCC->APB2RSTR |= (1 << 20); RCC->APB2RSTR &= ~(1 << 20);}while(0)
#define SPI6_REG_RESET()			do {RCC->APB2RSTR |= (1 << 21); RCC->APB2RSTR &= ~(1 << 21);}while(0)

/******************************************** END OF SPI PERIPHERAL *************************************************/


/******************************************** I2C PERIPHERAL *************************************************/

/*
 * structure for I2C registers
 */
typedef struct
{
	__vo uint32_t I2C_CR1;			/* I2C Control Register 1, 		   		offset = 0x00 */
	__vo uint32_t I2C_CR2;			/* I2C Control Register 2,		   		offset = 0x04 */
	__vo uint32_t I2C_OAR1;			/* I2C Own Address 1 Register,	   		offset = 0x08 */
	__vo uint32_t I2C_OAR2;			/* I2C Own Address 2 Register, 	   		offset = 0x0C */
	__vo uint32_t I2C_DR;			/* I2C Data Register,            		offset = 0x10 */
	__vo uint32_t I2C_SR1;			/* I2C Status Register 1,		   		offset = 0x14 */
	__vo uint32_t I2C_SR2;			/* I2C Status Register 2,				offset = 0x18 */
	__vo uint32_t I2C_CCR;			/* I2C Clock Control Register, 			offset = 0x1C */
	__vo uint32_t I2C_TRISE;		/* I2C TRISE Register,					offset = 0x20 */
	__vo uint32_t I2C_FLTR;			/* I2C FLTR Register,	   				offset = 0x24 */
}I2C_RegDef_t;

/*
 * Assigning base address of each I2Cx to the structure
 */
#define I2C1			((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t *)I2C3_BASEADDR)

#define I2C1_RST_BIT			21
#define I2C2_RST_BIT			22
#define I2C3_RST_BIT			23

#define I2C1_CLK_ENR_BIT		21
#define I2C2_CLK_ENR_BIT		22
#define I2C3_CLK_ENR_BIT		23

/*
 * Clock enable for I2C peripheral
 */
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << I2C1_CLK_ENR_BIT))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << I2C2_CLK_ENR_BIT))
#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1 << I2C3_CLK_ENR_BIT))

/*
 * Clock Disable for I2C peripheral
 */
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << I2C1_CLK_ENR_BIT))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << I2C2_CLK_ENR_BIT))
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << I2C3_CLK_ENR_BIT))


/*
 * Reset I2Cx peripheral
 * this is a technique to execute multiple statements in a single macro
 */
		/*                            1. SET 					    2. RESET*/
#define I2C1_REG_RESET()			do {RCC->APB1RSTR |= (1 << I2C1_RST_BIT); RCC->APB1RSTR &= ~(1 << I2C1_RST_BIT);}while(0)
#define I2C2_REG_RESET()			do {RCC->APB1RSTR |= (1 << I2C2_RST_BIT); RCC->APB1RSTR &= ~(1 << I2C2_RST_BIT);}while(0)
#define I2C3_REG_RESET()			do {RCC->APB1RSTR |= (1 << I2C3_RST_BIT); RCC->APB1RSTR &= ~(1 << I2C3_RST_BIT);}while(0)

/******************************************** END OF I2C PERIPHERAL *************************************************/

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40


#define IRQ_PR_EXTI0				6
#define IRQ_PR_EXTI1				7
#define IRQ_PR_EXTI2				8
#define IRQ_PR_EXTI3				9
#define IRQ_PR_EXTI4				10
#define IRQ_PR_EXTI9_5				23
#define IRQ_PR_EXTI15_10			40


// General macros

#define ENABLE		1
#define DISABLE		0
#define SET			ENABLE
#define RESET		DISABLE
#define PIN_SET		SET
#define PIN_RESET	RESET



//#include <stm32NucleoF429xxGpio_Driver.h>
//#include <stm32NucleoF429xxSPI_Driver.h>
//#include <stm32NucleoF429xxI2C_Driver.h>


#endif /* DRIVERS_INC_STM32NUCLEOF76XXX_H_ */
