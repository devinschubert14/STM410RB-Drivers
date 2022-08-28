/***********************************************************
 * STM32F410RB.h										   *
 * Created By: Devin Schubert							   *
 * Created on: August 25, 2022							   *
 ***********************************************************/

#ifndef INC_STM32F410RB_H_
#define INC_STM32F410RB_H_

#include <stdint.h>

/***********************************************************
 *														   *
 * 				STM32F410RB Address Macros			       *
 *														   *
 ***********************************************************/

/*Base Addresses of Flash and ROM memories*/

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define ROM 0x1FFF0000U
#define SRAM SRAM1_BASEADDR


/*AHBx and APBx Bus Peripheral base addresses*/

#define PERIPH_BASE 0x40000000U
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000U
#define AHB1PERIPH_BASE 0x40020000U
#define AHB2PERIPH_BASE 0x50000000U


/*Base addresses of AHB1 peripherals*/

#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000U)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400U)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800U)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00U)
#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800U)
/*Base addresses of APB1 peripherals*/

//I2C
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C4_BASEADDR (APB1PERIPH_BASEADDR + 0x6000U)
//SPI
#define SPI2_BASEADDR (APB1PERIPH_BASEADDR + 0x3800U)
//USART
#define USART2_BASEADDR (APB1PERIPH_BASEADDR + 0x4400U)

/*Base addresses of APB2 peripherals*/

//SPI
#define SPI1_BASEADDR (APB2PERIPH_BASEADDR + 0x3000U)
//USART
#define USART1_BASEADDR (APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR (APB2PERIPH_BASEADDR + 0x1400U)
//EXTI
#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00U)
//SYSCFG
#define SYSCFG_BASEADDR (APB2PERIPH_BASEADDR + 0x3800U)



/***********************************************************
 *														   *
 * 			Peripheral Register Definition Structures	   *
 *														   *
 ***********************************************************/


/*
 * Peripheral Register Definition for GPIO
 */
typedef struct{
	volatile uint32_t MODER; 		//GPIO port mode register 													Address Offset: 0x00
	volatile uint32_t OTYPER; 		//GPIO port output type register 											Address Offset: 0x04
	volatile uint32_t OSPEEDR; 		//GPIO port output speed register 											Address Offset: 0x08
	volatile uint32_t PUPDR;		//GPIO port pull-up/pull-down register										Address Offset: 0x0C
	volatile uint32_t IDR;			//GPIO port input data register												Address Offset: 0x10
	volatile uint32_t ODR; 			//GPIO port output data register											Address Offset: 0x14
	volatile uint32_t BSRR; 		//GPIO port bit set/reset register											Address Offset: 0x18
	volatile uint32_t LCKR; 		//GPIO port configuration lock register										Address Offset: 0x1C
	volatile uint32_t AFR[2]; 		//GPIO alternate function high/low register: AFR[0] = low, AFR[1] = high	Address Offset: 0x20-0x24
}GPIO_RegDef_t;

/*
 * Peripheral Register Definition for RCC
 */
typedef struct{
	volatile uint32_t CR; 			//RCC clock control register												Address Offset: 0x00
	volatile uint32_t PLLCFGR;		//RCC PLL configuration register											Address Offset: 0x04
	volatile uint32_t CFGR;			//RCC clock configuration register											Address Offset: 0x08
	volatile uint32_t CIR;			//RCC clock interrupt register												Address Offset: 0x0C
	volatile uint32_t AHB1RSTR;		//RCC AHB1 peripheral reset register										Address Offset: 0x10
	uint32_t RESERVED0[3];			//Reserved 0x14-1C
	volatile uint32_t APB1RSTR;		//RCC APB1 peripheral reset register										Address Offset: 0x20
	volatile uint32_t APB2RSTR;		//RCC APB2 peripheral reset register										Address Offset: 0x24
	uint32_t RESERVED1[2];			//Reserved 0x28-2C
	volatile uint32_t AHB1ENR;		//RCC AHB1 peripheral clock enable register									Address Offset: 0x30
	uint32_t RESERVED2[3];			//Reserved 0x34-3C
	volatile uint32_t APB1ENR;		//RCC APB1 peripheral clock enable register									Address Offset: 0x40
	volatile uint32_t APB2ENR;		//RCC APB2 peripheral clock enable register									Address Offset: 0x44
	uint32_t RESERVED3[2];			//Reserved 0x48-4C
	volatile uint32_t AHB1LPENR;	//RCC AHB1 peripheral clock enable in low power mode register				Address Offset: 0x50
	uint32_t RESERVED4[3];			//Reserved 0x54-5C
	volatile uint32_t APB1LPENR;	//RCC APB1 peripheral clock enabled in low power mode register				Address Offset: 0x60
	volatile uint32_t APB2LPENR;	//RCC APB2 peripheral clock enabled in low power mode register				Address Offset: 0x64
	uint32_t RESERVED5[2];			//Reserved 0x68-6C
	volatile uint32_t BDCR;			//RCC Backup domain control register										Address Offset: 0x70
	volatile uint32_t CSR;			//RCC clock control & status register										Address Offset: 0x74
	uint32_t RESERVED6[2];			//Reserved 0x78-7C
	volatile uint32_t SSCGR;		//RCC spread spectrum clock generation register								Address Offset:	0x80
	uint32_t RESERVED7[2];			//Reserved 0x84-88
	volatile uint32_t DCKCFGR;		//RCC Dedicated Clocks Configuration Register								Address Offset: 0x8C
	volatile uint32_t DCKCFGR2;		//RCC Dedicated Clocks Configuration Register 2								Address Offset: 0x94
}RCC_RegDef_t;


/* @GPIO_BASEADDR
 * GPIO Register Definitions
 */
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

/*
 * RCC Register Definitions
 */
#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)



/***********************************************************
 *														   *
 * 					  Clock Macros						   *
 *														   *
 ***********************************************************/


/*
 * Clock Enable Macros for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN() ((RCC->APB1ENR) |= (1 << 21))
#define I2C2_PCLK_EN() ((RCC->APB1ENR) |= (1 << 22))
#define I2C4_PCLK_EN() ((RCC->APB1ENR) |= (1 << 24))

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN() ((RCC->APB2ENR) |= (1 << 12))
#define SPI2_PCLK_EN() ((RCC->APB1ENR) |= (1 << 14))
#define SPI3_PCLK_EN() ((RCC->APB1ENR) |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() ((RCC->APB2ENR) |= (1 << 4))
#define USART2_PCLK_EN() ((RCC->APB1ENR) |= (1 << 17))
#define USART6_PCLK_EN() ((RCC->APB2ENR) |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() ((RCC->APB2ENR) |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx Peripherals
 */
#define GPIOA_PLCK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PLCK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PLCK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOH_PLCK_DI() (RCC->AHB1ENR &= ~(1 << 7))

/*
 * Clock Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 21))
#define I2C2_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 22))
#define I2C4_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 24))

/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_DI() ((RCC->APB2ENR) &= ~(1 << 12))
#define SPI2_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 14))
#define SPI3_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() ((RCC->APB2ENR) &= ~(1 << 4))
#define USART2_PCLK_DI() ((RCC->APB1ENR) &= ~(1 << 17))
#define USART6_PCLK_DI() ((RCC->APB2ENR) &= ~1 << 5))

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() ((RCC->APB2ENR) &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOH_REG_RESET() do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)




#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE

#include "stm32f410RB_gpio_driver.h"

#endif /* INC_STM32F410RB_H_ */
