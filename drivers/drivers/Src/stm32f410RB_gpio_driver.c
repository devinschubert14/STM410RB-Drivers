/***********************************************************
 * STM32F410RB_GPIO_driver.c
 * Created By: Devin Schubert
 * Created on: August 25, 2022
 ***********************************************************/
#include "stm32f410RB_gpio_driver.h"


/***********************************************************
 *
 * 				Clock Control Functions
 *
 ***********************************************************/


/* Function Name: GPIO_PeriClockControl
 * Desc: Enables or Disables the peripheral clock for the specified GPIO port.
 * Params:
 * 	- *pGPIOx: GPIO port from @GPIO_BASEADDR
 * 	- EnOrDi: Enable or disable port by using ENABLE/DISBALE or 0/1.
 * Return: None	
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}
	else if(EnOrDi == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}


/***********************************************************
 *
 * 				Init and De-init Functions
 *
 ***********************************************************/

/* Function Name: GPIO_Init
 * Desc: Initialize a GPIO port
 * Params: *pGPIOHandle: A handler for the configuration settings and base address of GPIO port.
 * Return: None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; //temp reg

	//Enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//2. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	//2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Configure the PUPD settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//4. Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//5. Configure the alt function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/* Function Name: GPIO_DeInit
 * Desc: De-initialize a GPIO port
 * Params: *pGPIOHandle: GPIO port from @GPIO_BASEADDR.
 * Return: None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_REG_RESET();
		}
}


/***********************************************************
 *
 * 				Data read and write functions
 *
 ***********************************************************/

/* Function Name: GPIO_ReadFromInputPin
 * Desc: Read from pin number specified by GPIO port given.
 * Params:
 * 	-*pGPIOx: GPIO port from @GPIO_BASEADDR.
 * 	-pin_number: Pin number from @GPIO_PIN_NUMBERS
 * Return: uint8_t: value read from pin stored in LSB
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number){
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pin_number) & 0x00000001);
	return value;
}

/* Function Name: GPIO_ReadFromInputPort
 * Desc: Read from GPIO port.
 * Params:
 * 	-*pGPIOx: GPIO port from @GPIO_BASEADDR.
 * Return: uint16_t: Value read from port
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/* Function Name: GPIO_WriteToOutputPin
 * Desc: Write to GPIO port and pin number specified.
 * Params:
 * 	-*pGPIOx: GPIO port from @GPIO_BASEADDR.
 * 	-pin_number: Pin number from @GPIO_PIN_NUMBERS
 * 	-value: SET/RESET or 1/0 will be output to pin
 * Return: None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number, uint8_t value){
	if(value == SET){
		//write 0
		pGPIOx->ODR |= (1 << pin_number);
	}
	else{
		//write 0
		pGPIOx->ODR &= ~(1 << pin_number);
	}
}

/* Function Name: GPIO_WriteToOutputPort
 * Desc: Write given value to output port.
 * Params:
 * 	-*pGPIOx: GPIO port from @GPIO_BASEADDR.
 * 	-value: 8-bit value that will be output to port.
 * Return: None 
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value){
	pGPIOx->ODR = value;
}

/* Function Name: GPIO_ToggleOutputPin
 * Desc: Toggle pin number and GPIO port specified.
 * Params:
 * 	-*pGPIOx: GPIO port from @GPIO_BASEADDR.
 * 	-pin_number: Pin number from @GPIO_PIN_NUMBERS
 * Return: None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pin_number){
	pGPIOx->ODR ^= (1 << pin_number);
}


/***********************************************************
 * 
 * 			IRQ Configuration and ISR handling
 * 
 ***********************************************************/

/* Function Name: GPIO_IRQInterruptConfig
 * Desc: Enable/Disable IRQ number on NVIC
 * Params:
 * 	- IRQNumber: Numbered pin 1-96 to enable/disable in NVIC.
 * 	- EnOrDI: ENABLE/DISABLE or 0/1
 * Return: None */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(IRQNumber <= 31){
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64){
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			//program ISER2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber <= 31){
				//program ICER0 register
				*NVIC_ICER0 |= (1 << IRQNumber);
			}else if(IRQNumber > 31 && IRQNumber < 64){
				//program ICER1 register
				*NVIC_ICER1 |= (1 << (IRQNumber % 32));
			}else if(IRQNumber >= 64 && IRQNumber < 96){
				//program ICER2 register
				*NVIC_ICER3 |= (1 << (IRQNumber % 64));
			}
	}
}

/* Function Name: GPIO_IRQPriorityConfig
 * Desc: Configure priority level of IRQ number provided.
 * Params:
 * 	- IRQNumber: Numbered pin 1-96 to set priority of interrupt in NVIC
 * 	- IRQPriority: Priority number for interrupt
 * Return: None 
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. Calculate ipr register and section
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amt = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amt);
}

/* Function Name: GPIO_IRQHandling
 * Desc: Clears the pending register in EXTI for provided pin number.
 * Params:
 * 	-pin_number: Pin number from @GPIO_PIN_NUMBERS
 * Return: None 
 */
void GPIO_IRQHandling(uint8_t pin_number){
	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << pin_number)){
		//clear
		EXTI->PR |= (1 << pin_number);
	}
}
