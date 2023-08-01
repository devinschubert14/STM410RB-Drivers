/*
 * stm32f410RB_usart_driver.h
 *
 *  Created on: Jul 8, 2023
 *      Author: devin
 */

#include "stm32f410RB_usart_driver.h"


/***********************************************************
 *
 * 				Clock and Peripheral Control Functions
 *
 ***********************************************************/

/* Function Name: USART_PeriClockControl
 * Desc: Enables or Disables the peripheral clock for the specified I2C port.
 * Params:
 * 	- *pUSARTx: USART port from @USART_BASEADDR
 * 	- EnOrDi: Enable or disable port by using ENABLE/DISBALE or 0/1.
 * Return: None
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){
			USART2_PCLK_EN();
		}else if(pUSARTx == USART6){
			USART6_PCLK_EN();
		}
	}else{
		if(pUSARTx == USART1){
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){
			USART2_PCLK_DI();
		}else if(pUSARTx == USART6){
			USART6_PCLK_DI();
		}
	}
}

/* Function Name: USART_PeripheralControl
 * Desc: Enables or Disables the peripheral in control register for the specified SPI port.
 * Params:
 * 	- *pUSARTx: USART port from @USART_BASEADDR
 * 	- EnOrDi: Enable or disable port by using ENABLE/DISBALE or 0/1.
 * Return: None
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pUSARTx->CR1 |= (1 << 13);
	}else{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}


/***********************************************************
 *
 * 				Init and De-init Functions
 *
 ***********************************************************/

/* Function Name: USART_Init
 * Desc: Initialize a USART port with provided configuration
 * Params: - *pUSARTHandle: A handler for the configuration settings and base address of USART port.
 * Return: None
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;

    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( ( 1 << USART_CR3_RTSE) | ( 1 << USART_CR3_CTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	//Set Baud Rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void USART_DeInit(USART_RegDef_t *pUSARTx){
	return;
}


/***********************************************************
 *
 * 				Data Send and Receive
 *
 ***********************************************************/

/* Function Name: USART_SendData
 * Desc: Write to USART port with provided data in TxBuffer.
 * Params:
 * 	-*pUSARTx: USART port from @USART_BASEADDR.
 * 	-pTxBuffer: Pointer to a buffer array to read from
 * 	-len: Total bytes to send from pTxBuffer
 * Return: None
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			pTxBuffer++;
		}
	}

	//Wait until TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/* Function Name: USART_ReceiveData
 * Desc: Read from USART port to RxBuffer provided.
 * Params:
 * 	-*pUSARTx: USART port from @USART_BASEADDR.
 * 	-pRxBuffer: Pointer to a buffer array to write to
 * 	-len: Total bytes to write to pRxBuffer
 * Return: None
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		//Wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE))

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x1FF);

				//Now increment the pRxBuffer two times
				pRxBuffer+=2;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/* Function Name: USART_SendDataIT
 * Desc: Write to USART port with provided data in TxBuffer in nonblocking mode.
 * Params:
 * 	-*pUSARTHandle: A handler for the configuration settings and base address of USART port.
 * 	-pTxBuffer: Pointer to a buffer array to read from
 * 	-len: Total bytes to send from pTxBuffer
 * Return: State of USART from @USART_AppStates
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);


		//Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);\
	}
	return txstate;
}


/* Function Name: USART_ReceiveDataIT
 * Desc: Read from SPI port to RxBuffer provided in nonblocking mode.
 * Params:
 * 	-*pUSARTHandle: A handler for the configuration settings and base address of USART port.
 * 	-pRxBuffer: Pointer to a buffer array to write to
 * 	-len: Total bytes to write to pRxBuffer
 * Return: State of USART from @USART_AppStates
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		//Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxstate;

}


/***********************************************************
 *
 * 				IRQ Config and Priority
 *
 ***********************************************************/\

 /* Function Name: USART_IRQInterruptConfig
  * Desc: Enable/Disable interrupts on NVIC
  * Params:
  * 	- IRQNumber: Numbered pin 1-96 to enable/disable in NVIC
  * 	- EnOrDI: ENABLE/DISABLE or 0/1
  * Return: None */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE)
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

 /* Function Name: USART_IRQPriorityConfig
  * Desc: Configure priority level of IRQ number provided.
  * Params:
  * 	- IRQNumber: Numbered pin 1-96 to set priority of interrupt in NVIC
  * 	- IRQPriority: Priority number for interrupt
  * Return: None */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/* Function Name: USART_IRQHandling
 * Desc: Calls the corresponding interrupt handlers depending on USART flags set.
 * Params:
 * 	-*pUSARTHandle: A handler for the configuration settings and base address of USART port.
 * Return: None */
void USART_IRQHandling(USART_Handle_t *pHandle){
	uint32_t temp1, temp2;

	uint16_t* data;

	//Transmit Data Register Empty Flag
	temp1 = pHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if(temp1 && temp2){

		if(pHandle->TxBusyState == USART_BUSY_IN_TX){
			if(pHandle->TxLen > 0){
				//9 bit transfer
				if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					data = (uint16_t*) pHandle->pTxBuffer;
					pHandle->pUSARTx->DR = (*data & (uint16_t)0x01FF);

					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						pHandle->pTxBuffer+=2;
						pHandle->TxLen-=2;
					}
					else{
						pHandle->pTxBuffer++;
						pHandle->TxLen--;
					}
				}
				//8 bit transfer
				else{
					pHandle->pUSARTx->DR = (*pHandle->pTxBuffer & (uint8_t)0xFF);

					pHandle->pTxBuffer++;
					pHandle->TxLen--;
				}
			}
			if(!pHandle->TxLen){
				pHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}
	//Transmit Complete Flag
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_TC);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2){
		if(pHandle->TxBusyState == USART_BUSY_IN_TX){
			if(!pHandle->TxLen){
				pHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Reset App State
				pHandle->TxBusyState = USART_READY;

				//Reset buffer address
				pHandle->pTxBuffer = NULL;

				//Reset len to 0
				pHandle->TxLen = 0;

				//Call the application call back
				USART_ApplicationEventCallback(pHandle, USART_EVENT_TX_CMPLT);
			}
		}

	}

	//Receive Data Register Empty Flag
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2){
		if(pHandle->RxBusyState == USART_BUSY_IN_RX){
			if(pHandle->RxLen > 0){

				//9 Bit Transfer
				if(pHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						*((uint16_t*) pHandle->pRxBuffer) = (pHandle->pUSARTx->DR & (uint16_t)0x01FF);

						pHandle->pRxBuffer+=2;
						pHandle->RxLen-=2;
					}
					//8 Bit Transfer
					else{
						*pHandle->pRxBuffer = (pHandle->pUSARTx->DR & (uint8_t)0xFF);
						pHandle->pRxBuffer++;
						pHandle->RxLen--;
					}
				}
				else{
					if(pHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
						//8 bits of data
						*pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR & (uint8_t)0xFF);
					}
					else{
						//7 bits of data
						*pHandle->pRxBuffer = (uint8_t) (pHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					pHandle->pRxBuffer++;
					pHandle->RxLen--;
				}
			}

			if(!pHandle->RxLen){
				//Disable RXNE
				pHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	//CTS Flag
	temp1 = pHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	if(temp1 && temp2){
		pHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pHandle, USART_EVENT_CTS);
	}

	//Idle Flag
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);

	if(temp1 && temp2){
		temp1 = pHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pHandle,USART_EVENT_IDLE);
	}

	//Overrun Flag
	temp1 = pHandle->pUSARTx->SR & ( 1 << USART_SR_ORE);
	temp2 = pHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		USART_ApplicationEventCallback(pHandle,USART_ERR_ORE);
	}


	//Noise Flag, Overrun Flag, and Framing Error Flag (MultiBuffer Communication)
	temp2 =  pHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE);

	if(temp2){
		temp1 = pHandle->pUSARTx->SR;

		//Framing Error
		if(temp1 & (1 << USART_SR_FE)){
			USART_ApplicationEventCallback(pHandle, USART_ERR_FE);
		}

		//Noise Error
		if(temp1 & (1 << USART_SR_NE)){
			USART_ApplicationEventCallback(pHandle, USART_ERR_NE);
		}

		//Overrun error
		if(temp1 & ( 1 << USART_SR_ORE) ){
			USART_ApplicationEventCallback(pHandle,USART_ERR_ORE);
		}
	}
}

/***********************************************************
 *
 * 			Other Peripheral Control Functions
 *
 ***********************************************************/

/* Function Name: USART_GetFlagStatus
 * Desc: Returns flag status of provided @USART_Flags.
 * Params:
 * 	-*pUSARTx: USART port from @USART_BASEADDR.
 * 	-flagName: Flag macro from @USART_Flags
 * Return: None
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t flagName){
	if(pUSARTx->SR & flagName){
		return  SET;
	}
	return RESET;
}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t statusFlagName);


/* Function Name: USART_SetBaudRate
 * Desc: Set the baudrate of provided USART port.
 * Params:
 * 	-*pUSARTx: USART port from @USART_BASEADDR
 * 	-BaudRate: Baud rate from @USART_Baud
 * Return: State of USART from @USART_AppStates
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);
   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);
   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

/*
 * Application callback
 */
__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{
	// Application implemented
}
