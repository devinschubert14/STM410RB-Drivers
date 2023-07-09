#include <stdio.h>
#include "stm32f410RB_spi_driver.h"

static void SPI_txne_ir_handle(SPI_Handle_t *pSPIHandle);
static void SPI_rxne_ir_handle(SPI_Handle_t *pSPIHandle);
static void SPI_ovr_err_ir_handle(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		/*else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}*/
	}
	else if (EnOrDi == DISABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		/*
		else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}*/
	}
}

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	/**** Configure SPI_CR1 Register ****/
	uint32_t tempreg = 0;

	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI should be cleared
		// RXONLY bit should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag_name)
{
	if (pSPIx->SR & flag_name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET)
			;

		// 2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit
			// 1. load the data in to the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			len--;
			len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			// 8-bit
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
			;

		// 2. check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit
			// 1. load the data in to the DR
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			len--;
			len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			// 8-bit
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->tx_state;

	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->tx_len = len;
		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same spi peripheral until transmission is over
		pSPIHandle->tx_state = SPI_BUSY_IN_TX;
		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
		// 4. Data transmission will be handled by the ISR code
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->rx_state;

	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rx_len = len;
		// 2. Mark the SPI state as busy in transmission so that
		// no other code can take over same spi peripheral until transmission is over
		pSPIHandle->rx_state = SPI_BUSY_IN_RX;
		// 3. Enable the RXEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
		// 4. Data transmission will be handled by the ISR code
	}
	return state;
}

/*
 * IRQ Configuration and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	// Check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// Handle TXE
		SPI_txne_ir_handle(pHandle);
	}

	// Check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		// Handle RXE
		SPI_rxne_ir_handle(pHandle);
	}

	// Check for OVR error
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
		// Handle OVR
		SPI_ovr_err_ir_handle(pHandle);
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

// Helper Functions
static void SPI_txne_ir_handle(SPI_Handle_t *pSPIHandle)
{
	// 2. check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16-bit
		// 1. load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->tx_len--;
		pSPIHandle->tx_len--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		// 8-bit
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->tx_len--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->tx_len)
	{
		// TxLen is zero, close SPI transmission and inform application

		// Prevent interrupts from setting TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void SPI_rxne_ir_handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16-bit
		// 1. load the data in to the DR
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rx_len-=2;
		(uint16_t *)pSPIHandle->pRxBuffer++;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else
	{
		// 8-bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rx_len--;
		pSPIHandle->pRxBuffer++;
	}
	if (!pSPIHandle->rx_len)
	{
		// TxLen is zero, close SPI transmission and inform application
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_ovr_err_ir_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. clear the ovr flag
	if (pSPIHandle->tx_state != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	(void)temp;
	// 2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->tx_len = 0;
	pSPIHandle->tx_state = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->rx_len = 0;
	pSPIHandle->rx_state = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// Application implemented
}
