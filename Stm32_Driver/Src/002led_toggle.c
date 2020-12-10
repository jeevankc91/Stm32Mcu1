/*
 * 002led_toggle.c
 *
 *  Created on: 30-Nov-2020
 *      Author: g1
 */
#include "stm32f446xx.h"

#define LOW 0x0
#define BUTTON_PRESSED LOW

void delay()
{
	uint16_t index;
	for(index = 0;index<65500;index++);
}

int main()
{
	GPIO_Handle_t gpioled,gpio_user_button;
	uint8_t read_pin;

	gpioled.pGPIOx = GPIOA;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	gpio_user_button.pGPIOx = GPIOC;
	gpio_user_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpio_user_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    gpio_user_button.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
    gpio_user_button.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PU_PD;
    gpio_user_button.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;

    GPIO_PeriClockControl(GPIOA,ENABLE);
    GPIO_PeriClockControl(GPIOC,ENABLE);
    GPIO_Init(&gpioled);
    GPIO_Init(&gpio_user_button);

    while(1)
    {
    	read_pin = GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13);
    	if(read_pin == BUTTON_PRESSED)
    	{
    		delay();
    		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
    	}

    }
	return 0;
}

