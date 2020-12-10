/*
 * stm32f446xx_drive.h
 *
 *  Created on: 10-Nov-2020
 *      Author: g1
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
* This is a configuration structure for gpio pin
*/
typedef struct
{
	uint8_t GPIO_PinNumber;           /*possible values from @GPIO_PIN_NUMBER*/
	uint8_t GPIO_PinMode;             /*possible values from @GPIO_PIN_MODES*/ 
	uint8_t GPIO_PinSpeed;            /*possible values from @GPIO_OP_SPEEDS*/
	uint8_t GPIO_PinPuPdControl;      /*possible values from @GPIO_PUPD_STATE*/
	uint8_t GPIO_PinOPType;           /*possible values from @GPIO_OP_TYPES*/
	uint8_t GPIO_PinAltFunMode;       /*possible values from @GPIO_PIN_NUMBER*/
}GPIO_PinConfig_t;

/*
* This is a handle structure for gpio pin
*/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*************************************************************************************
*                         APIs supported by this driver
* For more information about APIs refer each function definition
**************************************************************************************/
/*
 * Peripheral clock setup 
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDis);

/*
 * Init and Deint
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write 
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

/*
 * @GPIO_PIN_MODES
 * GPIO possible modes
 */
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_FRT   6

/*
 * @GPIO_OP_TYPES
 * GPIO possible output types
 */
#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1

/*
 * @GPIO_PIN_NUMBER
 * GPIO available pins
 */
#define GPIO_PIN_NO_0   0 
#define GPIO_PIN_NO_1   1 
#define GPIO_PIN_NO_2   2 
#define GPIO_PIN_NO_3   3 
#define GPIO_PIN_NO_4   4 
#define GPIO_PIN_NO_5   5 
#define GPIO_PIN_NO_6   6 
#define GPIO_PIN_NO_7   7 
#define GPIO_PIN_NO_8   8 
#define GPIO_PIN_NO_9   9 
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15
 
/*
 * @GPIO_OP_SPEEDS
 * GPIO possible output speeds
 */
#define GPIO_SPEED_LOW   0 
#define GPIO_SPEED_MED   1 
#define GPIO_SPEED_FAST  2 
#define GPIO_SPEED_HIGH  3 
 
/*
 * @GPIO_PUPD_STATE
 * GPIO macro for pull up and pull down configure states
 */
#define GPIO_NO_PU_PD   0
#define GPIO_PIN_PU     1
#define GPIO_PIN_PD     2

 
#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
