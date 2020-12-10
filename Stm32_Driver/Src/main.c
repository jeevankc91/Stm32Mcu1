/*
 * main.c
 *
 *  Created on: 04-Dec-2020
 *      Author: g1
 */
#include "stm32f446xx.h"

int main(void)
{
	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	//interrupt handler
	GPIO_IRQHandling(0);
}
