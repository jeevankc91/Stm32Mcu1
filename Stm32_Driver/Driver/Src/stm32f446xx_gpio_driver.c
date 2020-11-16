/*
 * stm32f446xx_drive.c
 *
 *  Created on: 10-Nov-2020
 *      Author: g1
 */


#include "stm32f446xx_gpio_driver.h"


/****************************************************************
 * @fn              - GPIO_PeriClockControl
 *
 * @brief           - This function enables or disables peripheral clock for the given GPIO port
 * 
 * @param[in]       - base address of the gpio peripheral
 * @param[in]       - ENABLE or DISABLE macros
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDis)
{
   if(EnorDis == ENABLE)
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

/****************************************************************
 * @fn              - GPIO_Init
 *
 * @brief           - This function initialized GPIO port
 * 
 * @param[in]       - GPIO handler address
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp=0;
	/*1. pin mode configuration*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER = temp;
	}
	else
	{
		/*interrupt mode*/
	}
	temp = 0;

	/*2. pin speed configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER = temp;
	temp = 0;

	/*3. pin pull up/pull down configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR = temp;
	temp = 0;

	/*4. pin output type configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER = temp;

	/*5. pin alternate function mode configuration*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
	{

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	
}

/*
 * Data read and write 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return 0;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return 0;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDis)
{
	
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	
}
