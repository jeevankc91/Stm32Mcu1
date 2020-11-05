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

/************************************************************************************
 *  Peripheral definitions (Peripheral base address typecasted to xxxx_RegDef_t)    *
 ************************************************************************************/

#define GPIOA                ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#endif /* INC_STM32F446XX_H_ */
