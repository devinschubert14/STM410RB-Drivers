/*
 * stm43410RB_i2c_driver.c
 *
 *  Created on: Jan 28, 2023
 *      Author: devin
 */
#include "stm32f410RB_i2c_driver.h"


/***********************************************************
 *
 * 				I2C Interrupt Helper Functions
 *
 ***********************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

/* Function Name: I2C_GenerateStartCondition
 * Desc: Generates a start condition on I2C port.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * Return: None
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/* Function Name: I2C_GenerateStopCondition
 * Desc: Generates a stop condition on I2C port.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * Return: None
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/* Function Name: I2C_ExecuteAddressPhaseWrite
 * Desc: Sends address on I2C port with write bit.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * 	- slaveAddr: Address of slave
 * Return: None
 */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1); //slave address + r/nw bit=0
	pI2Cx->DR = slaveAddr;
}

/* Function Name: I2C_ExecuteAddressPhaseRead
 * Desc: Sends address on I2C port with read bit.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * 	- slaveAddr: Address of slave
 * Return: None
 */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr){
	slaveAddr = slaveAddr << 1;
	slaveAddr |= 1; //slave address + r/nw bit=1
	pI2Cx->DR = slaveAddr;
}

/* Function Name: I2C_ManageAcking
 * Desc: Enable or Disabe ACKing on I2C port.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * 	- EnOrDi: I2C_ACK_ENABLE/I2C_ACK_DISABLE or 1/0 respectively.
 * Return: None
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){
	if(EnOrDi == I2C_ACK_ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
/* Function Name: I2C_ClearADDRFlag
 * Desc: Clear ADDR flag in I2C port registers.
 * Params:
 * 	- *pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummyRead;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		//Master Mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				//disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
	}
	else{
		//Slave Mode
	}
}
/***********************************************************
 *
 * 				Clock and Peripheral Control Functions
 *
 ***********************************************************/

/* Function Name: I2C_PeriClockControl
 * Desc: Enables or Disables the peripheral clock for the specified I2C port.
 * Params:
 * 	- *pI2Cx: SPI port from @I2C_BASEADDR
 * 	- EnOrDi: Enable or disable port by using ENABLE/DISBALE or 0/1.
 * Return: None
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C4){
			I2C4_PCLK_EN();
		}
	}
	else if (EnOrDi == DISABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}

		else if(pI2Cx == I2C4){
			I2C4_PCLK_EN();
		}
	}
}
/* Function Name: I2C_PeripheralControl
 * Desc: Enables or Disables the peripheral in control register for the specified I2C port.
 * Params:
 * 	- *pI2Cx: I2C port from @I2C_BASEADDR
 * 	- EnOrDi: Enable or disable port by using ENABLE/DISBALE or 0/1.
 * Return: None
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/***********************************************************
 *
 * 				Init and De-init Functions
 *
 ***********************************************************/

/* Function Name: I2C_Init
 * Desc: Initialize a I2C port
 * Params: - *pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	//enable clock for i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//Set FREQ in CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//Set devices address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//Standard Mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}else {
		//Fast Mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			ccr_value = (RCC_GetPCLK1Value()/(3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		else
			ccr_value = (RCC_GetPCLK1Value()/(25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//Standard Mode
		tempreg = (RCC_GetPCLK1Value()/1000000U) + 1;
	}
	else{
		//Fast Mode
		tempreg = ((RCC_GetPCLK1Value() * 300)/1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	//disable clock for i2cx peripheral
	I2C_PeriClockControl(pI2Cx,DISABLE);
}


/***********************************************************
 *
 * 				Data Send and Receive
 *
 ***********************************************************/

/******* MASTER *********/

/* Function Name: I2C_MasterSendData
 * Desc: Write to I2C port with provided slave address and data in TxBuffer.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * 	- pTxBuffer: Pointer to a buffer array to read from
 * 	- len: Total bytes to send from pTxBuffer
 * 	- slaveAddr: 7 or 10 bit addresss of slave to send to
 * 	- sr: boolean for if this should be a repeated start
 * Return: None
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr){
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm start generation completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to 1
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

	//4. Confirm that address phase is complete
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//5. Clear the ADDR flag
	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len is 0
	while(len){
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	//7. Wait for TXE=1 and BTF=1 before generating STOP condition
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF));

	//8. Generate STOP condition
	if(sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

/* Function Name: I2C_MasterReceiveData
 * Desc: Read from I2C port with provided slave address and to RxBuffer.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * 	- pRxBuffer: Pointer to a buffer array to write to
 * 	- len: Total bytes to write to RxBuffer
 * 	- slaveAddr: 7 or 10 bit addresss of slave to send to
 * 	- sr: boolean for if this should be a repeated start
 * Return: None
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr){
	//1. Generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm START generation completed
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB));

	//3. Send the address
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, slaveAddr);

	//4. Confirm address phase is complete
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR));

	//Read one byte
	if(len == 1){
		//Disable ACK
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Wait until RXNE completes
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE));

		//Generate STOP condition
		if(sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//Read data
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(len > 1){
		//Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//Read data
		while(len){
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );
			if(len == 2){
				//Disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//Generate STOP
				if(sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read data to buffer
			//Read data
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
			len--;
		}
	}

	//Enable ACKing

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}

/******* SLAVE *********/

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data){
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return pI2Cx->DR;
}


/***********************************************************
 *
 * 				SPI Interrupt Functions
 *
 ***********************************************************/

/* Function Name: I2C_MasterSendDataIT
 * Desc: Write to I2C port with provided slave address and data in TxBuffer in nonblocking mode.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * 	- pTxBuffer: Pointer to a buffer array to read from
 * 	- len: Total bytes to send from pTxBuffer
 * 	- slaveAddr: 7 or 10 bit addresss of slave to send to
 * 	- sr: boolean for if this should be a repeated start
 * Return: State of the I2C from @I2C_AppStates
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr){
	uint8_t busystate  = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = sr;

		//Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Enable ITEVTEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/* Function Name: I2C_MasterReceiveDataIT
 * Desc: Read from I2C port with provided slave address and to RxBuffer in nonblocking mode.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * 	- pRxBuffer: Pointer to a buffer array to write to
 * 	- len: Total bytes to write to RxBuffer
 * 	- slaveAddr: 7 or 10 bit addresss of slave to send to
 * 	- sr: boolean for if this should be a repeated start
 * Return: State of the I2C from @I2C_AppStates
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr){
	uint8_t busystate = pI2CHandle->TxRxState;

		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = slaveAddr;
			pI2CHandle->Sr = sr;
			//Generate START Condition
				I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			//Enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

			//Enable ITEVTEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

			//Enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

		}

		return busystate;
}

/***********************************************************
 *
 * 				IRQ Config and Priority
 *
 ***********************************************************/

/* Function Name: I2C_IRQInterruptConfig
 * Desc: Enable/Disable interrupts on NVIC.
 * Params:
 * 	- IRQNumber: Numbered pin 1-96 to enable/disable in NVIC
 * 	- EnOrDI: ENABLE/DISABLE or 0/1
 * Return: None */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/* Function Name: I2C_IRQPriorityConfig
 * Desc: Configure priority level of IRQ number provided.
 * Params:
 * 	- IRQNumber: Numbered pin 1-96 to set priority of interrupt in NVIC
 * 	- IRQPriority: Priority number for interrupt
 * Return: None
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Calculate ipr register and section
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/***********************************************************
 *
 * 				I2C Interrupt Helper Functions
 *
 ***********************************************************/

/* Function Name: I2C_MasterHandleTXEInterrupt
 * Desc: Interrupt handler for writing from TxBuffer to data register.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxLen > 0){
		//Load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//decrement the len
		pI2CHandle->TxLen--;
		//increment buffer address
		pI2CHandle->pTxBuffer++;
	}
}

/* Function Name: I2C_MasterHandleRXNEInterrupt
 * Desc: Interrupt handler for reading from data register to RX buffer.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
		if(pI2CHandle->RxSize == 1){
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->RxLen--;
		}


		if(pI2CHandle->RxSize > 1){

			//Disable ACK
			if(pI2CHandle->RxLen == 2){
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
			}

			//read DR
			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
		}

		if(pI2CHandle->RxLen == 0){
			//close the I2C data reception and notify application

			//1. Generate STOP condition
			if(pI2CHandle->Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			//2. Close the I2C rx
			I2C_CloseReceiveData(pI2CHandle);

			//3. Notify the application
			I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
		}
	}
}

/* Function Name: I2C_EV_IRQHandling
 * Desc: Calls interrupt handler for I2C port events.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){
	//Interrupt handling for both master and slave mode


	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);

	//1. Handle the interrupt generated by SB event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB);

	if(temp1 && temp3){
		//SB flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handle interrupt for address event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR);

	if(temp1 && temp3){
		//ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	//3. Handle the interrupt for BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF);

	if(temp1 && temp3){
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0){
					//1. generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
					//2. reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			;
		}
	}

	//4. Handle interrupt for STOPF event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF);

	if(temp1 && temp3){
		//STOPF flag is set
		//Clear the STOPF
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	//5. Handle interrupt for TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE);

	if(temp1 && temp2 && temp3){
		//TXE flag is set
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else{
			//Slave mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}

	//6. Handle interrupt for RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE);

	if(temp1 && temp2 && temp3){
		//RXNE flag is set
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//Master Mode
			I2C_MasterHandleRXNEInterrupt(pI2CHandle);
		}
		else{
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_RCV);
			}
		}
	}
}

/* Function Name: I2C_ER_IRQHandling
 * Desc: Calls interrupt handler for I2C port errors.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){
	uint32_t statusITERREN = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);

	//Bus Error
	uint32_t statusBERR = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(statusITERREN && statusBERR){
		//Clear error flag
		(pI2CHandle->pI2Cx->SR1) &= ~(1 << I2C_SR1_BERR);

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}


	//Arbitration error
	uint32_t statusARLO = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(statusARLO  && statusITERREN)
	{
		//Clear error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_ARLO );

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	//ACK failure
	uint32_t statusAF = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(statusAF  && statusITERREN)
	{
		//Clear error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_AF );

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	//Overrun/underrun error
	uint32_t statusOVR = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(statusOVR  && statusITERREN)
	{
		//Clear error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_OVR );

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	//Overrun/underrun error
	uint32_t statusTO= (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(statusTO  && statusITERREN)
	{
		//Clear error flag
		(pI2CHandle->pI2Cx->SR1) &= ~( 1 << I2C_SR1_TIMEOUT );

		//Notify application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/***********************************************************
 *
 * 				SPI Flag Functions
 *
 ***********************************************************/

/* Function Name: I2C_GetFlagStatus
 * Desc: Returns flag status of provided @I2C_Flags.
 * Params:
 * 	-*pI2Cx: SPI port from @I2C_BASEADDR.
 * 	-flagName: Flag macro from @I2C_Flags
 * Return: None
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName)
{
	if (pI2Cx->SR1 & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/***********************************************************
 *
 * 				Helper Functions
 *
 ***********************************************************/

/* Function Name:  I2C_CloseReceiveData
 * Desc: Resets I2C Handle receiving information to default and enables ACKing.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){
	//Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/* Function Name:  I2C_CloseReceiveData
 * Desc: Resets I2C Handle transmitting information to default.
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * Return: None */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){
	//Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
}

/* Function Name:  I2C_SlaveEnableDisableCallbackEvents
 * Desc: Enables or disables callback events when in slave mode.
 * Params:
 * 	-*pI2Cx: I2C port from @I2C_BASEADDR.
 * 	- EnOrDi: ENABLE/DISABLE or 1/0 respectively.
 * Return: None */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else{
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}

/* Function Name:  I2C_ApplicationEventCallback
 * Desc: Weak prototype to be overwritten by programmer and will be called
 * whenever an applicaton event stated in @I2C_AppEvents occurs
 * Params:
 * 	-*pI2CHandle: A handler for the configuration settings and base address of I2C port.
 * 	- AppEv: Interrupt that was called from I2C port in handle. Events are provided in @I2C_AppEvents.
 * Return: None */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	// Application implemented
}
