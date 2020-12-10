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
	uint32_t temp=0;
	/*1. pin mode configuration*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}
	else 
	{
		/*interrupt mode*/
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//FTSR configuration
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//RTSR configuration
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			//RTSR and FTSR configuration
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		uint8_t temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);
	}
	temp = 0;

	/*2. pin speed configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;
	temp = 0;

	/*3. pin pull up/pull down configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/*4. pin output type configuration*/
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	/*5. pin alternate function mode configuration*/
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4*temp2));

	}
}

/****************************************************************
 * @fn              - GPIO_DeInit
 *
 * @brief           - This function de-initialize GPIO port
 * 
 * @param[in]       - GPIO port address
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
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

/****************************************************************
 * @fn              - GPIO_ReadFromInputPin
 *
 * @brief           - This function read data from gpio pin
 * 
 * @param[in]       - GPIO handler address
 * @param[in]       - GPIO pin number
 *
 * @return          - 0 or 1
 *
 * @Note            - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;
}

/****************************************************************
 * @fn              - GPIO_ReadFromInputPort
 *
 * @brief           - This function read data from gpio port
 * 
 * @param[in]       - GPIO handler address
 *
 * @return          - uint16_t
 *
 * @Note            - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/****************************************************************
 * @fn              - GPIO_WriteToOutputPin
 *
 * @brief           - This function write data to gpio pin
 * 
 * @param[in]       - GPIO handler address
 * @param[in]       - PinNumber
 * @param[in]       - Value
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
	
}

/****************************************************************
 * @fn              - GPIO_WriteToOutputPort
 *
 * @brief           - This function write data to gpio port
 * 
 * @param[in]       - GPIO handler address
 * @param[in]       - Value
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************
 * @fn              - GPIO_ToggleOutputPin
 *
 * @brief           - This function toggles output pin of gpio port
 * 
 * @param[in]       - GPIO handler address
 * @param[in]       - Value
 *
 * @return          - none
 *
 * @Note            - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis)
{
	if(EnorDis == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if((IRQNumber > 31) && (IRQNumber <=63))
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));
		}
		else if((IRQNumber > 63) && (IRQNumber <96))
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if((IRQNumber > 31) && (IRQNumber <=63))
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));
		}
		else if((IRQNumber > 63) && (IRQNumber <96))
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1<<(IRQNumber % 64));
		}
	}
	
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx =  IRQNumber/4;
	uint8_t iprx_section =  IRQNumber%4;

	uint8_t shift_amount = (8*iprx_section) + (8 - NUM_OF_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority << (shift_amount));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if((EXTI->PR) & (1<<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);
	}
}
