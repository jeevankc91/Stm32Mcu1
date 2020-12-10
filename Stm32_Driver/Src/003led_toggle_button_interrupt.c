/*
 * 003led_toggle_button_interrupt.c
 *
 *  Created on: 05-Dec-2020
 *      Author: g1
 */
#include "stm32f446xx.h"

#define LOW 0x0
#define HIGH 0x1
#define BUTTON_PRESSED HIGH

void delay()
{
	uint16_t index;
	for(index = 0;index<65500;index++);
}

int main()
{
	GPIO_Handle_t gpioled,gpio_user_button;

	gpioled.pGPIOx = GPIOB;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	gpio_user_button.pGPIOx = GPIOB;
	gpio_user_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpio_user_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    gpio_user_button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
    gpio_user_button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

    GPIO_PeriClockControl(GPIOB,ENABLE);
    GPIO_Init(&gpioled);
    GPIO_Init(&gpio_user_button);

    GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_5, GPIO_PIN_RESET);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);

    while(1)
    {
    	/*
    	if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_9) == HIGH)
    	{
    		delay();
    		GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_5);
    	}
    	*/

    }

	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_9);
	GPIO_ToggleOutputPin(GPIOB,GPIO_PIN_NO_5);
}

