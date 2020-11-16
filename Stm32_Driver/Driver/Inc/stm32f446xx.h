/*
 * stm32f446xx.h
 *
 *  Created on: Nov 3, 2020
 *      Author: g1
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

#define _vo volatile

#define FLASH_BASEADDR    0x08000000U     /*Flash memory base address*/
#define SRAM1_BASEADDR    0x20000000U     /*SRAM1 memory base address*/
#define SRAM2_BASEADDR    0x2001C000U     /*SRAM2 memory base address*/
#define ROM_BASEADDR      0x1FFF0000U     /*System memory(ROM)base address*/

/*********************************************************
 *   AHBx and APBx peripheral base address               *
 *********************************************************/
#define PERIPH_BASE		  0x40000000U
#define APB1_BASEADDR     PERIPH_BASE    /*APB1 bus base address*/
#define APB2_BASEADDR     0x40010000U    /*APB2 bus base address*/
#define AHB1_BASEADDR     0x40020000U    /*AHB1 bus base address*/
#define AHB2_BASEADDR     0x50000000U    /*AHB2 bus base address*/

/*********************************************************
 *   Base address of Peripherals hanging on AHB1 BUS     *
 *********************************************************/
#define GPIOA_BASEADDR       (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR       (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR       (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR       (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR       (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR       (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR       (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR       (AHB1_BASEADDR + 0x1C00)
#define CRC_BASRADDR         (AHB1_BASEADDR + 0x3000)
#define RCC_BASRADDR         (AHB1_BASEADDR + 0x3800)

/*********************************************************
 *   Base address of Peripherals hanging on APB1 BUS     *
 *********************************************************/
#define I2C1_BASEADDR        (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR        (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR        (APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR        (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR        (APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR      (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR      (APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR       (APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR       (APB1_BASEADDR + 0x5000)


/*********************************************************
 *   Base address of Peripherals hanging on APB2 BUS     *
 *********************************************************/
#define EXTI_BASEADDR        (APB2_BASEADDR + 0x3C00)

#define SPI1_BASEADDR        (APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR        (APB2_BASEADDR + 0x3400)

#define SYSCFG_BASEADDR      (APB2_BASEADDR + 0x3800)

#define USART1_BASEADDR      (APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR      (APB2_BASEADDR + 0x1400)

/*********************************************************
 *  Peripheral register definition structure for GPIO    *
 *********************************************************/

typedef struct{
	_vo uint32_t MODER;         /*GPIO port mode register,                                  Address Offset: 0x00  */
	_vo uint32_t OTYPER;        /*GPIO port output type register,                           Address Offset: 0x04  */
	_vo uint32_t OSPEEDER;      /*GPIO port output speed register,                          Address Offset: 0x08  */
	_vo uint32_t PUPDR;         /*GPIO port pull-up/pull-down register,                     Address Offset: 0x0C  */
	_vo uint32_t IDR;           /*GPIO port input data register,                            Address Offset: 0x10  */
	_vo uint32_t ODR;           /*GPIO port output data register,                           Address Offset: 0x14  */
	_vo uint32_t BSRR;          /*GPIO port bit set/reset register,                         Address Offset: 0x18  */
	_vo uint32_t LCKR;          /*GPIO port configuration lock register,                    Address Offset: 0x1C  */
	_vo uint32_t AFR[2];        /*AFR[0]/AFR[1] GPIO alternate function low/high register,  Address Offset: 0x20/0x24  */
} GPIO_RegDef_t;


/*********************************************************
 *  Peripheral register definition structure for GPIO    *
 *********************************************************/

typedef struct{
	_vo uint32_t CR;             /*RCC clock control register,                               Address Offset: 0x00  */
	_vo uint32_t PLLCFGR;        /*RCC PLL configuration register,                           Address Offset: 0x04  */
	_vo uint32_t CFGR;           /*RCC clock configuration register,                         Address Offset: 0x08  */
	_vo uint32_t CIR;            /*RCC clock interrupt register,                             Address Offset: 0x0C  */
	_vo uint32_t AHB1RSTR;       /*RCC AHB1 peripheral reset register,                       Address Offset: 0x10  */
	_vo uint32_t AHB2RSTR;       /*RCC AHB2 peripheral reset register,                       Address Offset: 0x14  */
	_vo uint32_t AHB3RSTR;       /*RCC AHB3 peripheral reset register,                       Address Offset: 0x18  */
	uint32_t RESERVED0;          /*Reserved,                                                 Address Offset: 0x1C  */
	_vo uint32_t APB1RSTR;       /*RCC APB1 peripheral reset register,                       Address Offset: 0x20  */
	_vo uint32_t APB2RSTR;       /*RCC APB2 peripheral reset register,                       Address Offset: 0x24  */
	uint32_t RESERVED1[2];       /*Reserved,                                                 Address Offset: 0x28, 0x2C  */
	_vo uint32_t AHB1ENR;        /*RCC AHB1 peripheral clock enable register,                Address Offset: 0x30  */
	_vo uint32_t AHB2ENR;        /*RCC AHB2 peripheral clock enable register,                Address Offset: 0x34  */
	_vo uint32_t AHB3ENR;        /*RCC AHB3 peripheral clock enable register,                Address Offset: 0x38  */
	uint32_t RESERVED3;          /*Reserved,                                                 Address Offset: 0x3C  */
	_vo uint32_t APB1ENR;        /*RCC APB1 peripheral clock enable register,                Address Offset: 0x40  */
	_vo uint32_t APB2ENR;        /*RCC APB2 peripheral clock enable register,                Address Offset: 0x44  */
	uint32_t RESERVED4[2];       /*Reserved,                                                 Address Offset: 0x48, 0x4C  */
	_vo uint32_t AHB1LPENR;      /*RCC AHB1 peripheral clock enable in low power mode register,                                  Address Offset: 0x50  */
	_vo uint32_t AHB2LPENR;      /*RCC AHB2 peripheral clock enable in low power mode register,                                  Address Offset: 0x54  */
	_vo uint32_t AHB3LPENR;      /*RCC AHB3 peripheral clock enable in low power mode register,                                  Address Offset: 0x58  */
	uint32_t RESERVED6;          /*Reserved,                                                 Address Offset: 0x5C  */
	_vo uint32_t APB1LPENR;      /*RCC APB1 peripheral clock enable in low power mode register,                                  Address Offset: 0x60  */
	_vo uint32_t APB2LPENR;      /*RCC APB2 peripheral clock enable in low power mode register,                                  Address Offset: 0x64  */
	uint32_t RESERVED7[2];       /*Reserved,                                                 Address Offset: 0x68, 0x6C  */
	_vo uint32_t BDCR;           /*RCC Backup domain control register,                       Address Offset: 0x70  */
	_vo uint32_t CSR;            /*RCC clock control & status register,                      Address Offset: 0x74  */
	uint32_t RESERVED8[2];       /*GPIO port mode register,                                  Address Offset: 0x78, 0x7C  */
	_vo uint32_t SSCGR;          /*RCC spread spectrum clock generation register,            Address Offset: 0x80  */
	_vo uint32_t PLLI2SCFGR;     /*RCC PLLI2S configuration register,                        Address Offset: 0x84  */
	_vo uint32_t PLLSAICFGR;     /*RCC PLL configuration register,                           Address Offset: 0x88  */
	_vo uint32_t DCKCFGR;        /*RCC Dedicated Clock Configuration register,               Address Offset: 0x8C  */
	_vo uint32_t CKGATENR;       /*RCC clocks gated enable register,                         Address Offset: 0x90  */
	_vo uint32_t DCKCFGR2;       /*RCC dedicated clocks configuration register,              Address Offset: 0x94  */
}RCC_RedDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE  ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG  ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH  ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_RedDef_t*)RCC_BASRADDR)

/************************************************************************************
 *  Clock enable Macros for GPIOx peripherals                                       *
 ************************************************************************************/
#define GPIOA_PCLK_EN()      (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()      (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()      (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()      (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()      (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()      (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()      (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()      (RCC->AHB1ENR |= (1 << 7))

/************************************************************************************
 *  Clock enable Macros for I2Cx peripherals                                        *
 ************************************************************************************/
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23))

/************************************************************************************
 *  Clock enable Macros for SPIx peripherals                                        *
 ************************************************************************************/
#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1<<12))
#define SPI4_PCLK_EN()      (RCC->APB2ENR |= (1<<13))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()      (RCC->APB2ENR |= (1<<15))

/************************************************************************************
 *  Clock enable Macros for USARTx peripherals                                        *
 ************************************************************************************/
#define USART1_PCLK_EN()      (RCC->APB2ENR |= (1<<4))
#define USART6_PCLK_EN()      (RCC->APB2ENR |= (1<<5))
#define USART2_PCLK_EN()      (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()      (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN()       (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN()       (RCC->APB1ENR |= (1<<20))

/************************************************************************************
 *  Clock enable Macros for USARTx peripherals                                        *
 ************************************************************************************/
#define SYSCFG_PCLK_EN()      (RCC->APB2ENR |= (1<<14))
 
/************************************************************************************
 *  Clock disable Macros for GPIOx peripherals                                       *
 ************************************************************************************/
#define GPIOA_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()      (RCC->AHB1ENR &= ~(1 << 7))

/************************************************************************************
 *  Clock disable Macros for I2Cx peripherals                                        *
 ************************************************************************************/
#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 23))

/************************************************************************************
 *  Clock disable Macros for SPIx peripherals                                        *
 ************************************************************************************/
#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1<<12))
#define SPI4_PCLK_DI()      (RCC->APB2ENR &= ~(1<<13))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()      (RCC->APB2ENR &= ~(1<<15))

/************************************************************************************
 *  Clock disable Macros for USARTx peripherals                                        *
 ************************************************************************************/
#define USART1_PCLK_DI()      (RCC->APB2ENR &= ~(1<<4))
#define USART6_PCLK_DI()      (RCC->APB2ENR &= ~(1<<5))
#define USART2_PCLK_DI()      (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()      (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()       (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()       (RCC->APB1ENR &= ~(1<<20))

/************************************************************************************
 *  Clock disable Macros for USARTx peripherals                                        *
 ************************************************************************************/
#define SYSCFG_PCLK_DI()      (RCC->APB2ENR &= ~(1<<14))

/*
 * Generic Macros
 */
#define ENABLE                 1
#define DISABLE                0
#define SET                    ENABLE
#define RESET                  DISABLE
#define GPIO_PIN_SET           SET
#define GPIO_PIN_RESET         RESET

 
#endif /* INC_STM32F446XX_H_ */
