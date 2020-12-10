/*
 * 001leg_toggle.c
 *
 *  Created on: 29-Nov-2020
 *      Author: g1
 */
#include "stm32f446xx.h"

void delay()
{
	uint16_t index;
	for(index = 0;index<60000;index++);
}

int main(void)
{
	GPIO_Handle_t gpioled;

	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&gpioled);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
