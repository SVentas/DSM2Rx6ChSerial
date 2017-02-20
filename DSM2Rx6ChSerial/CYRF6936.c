//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "CYRF6936.h"	// CYRF6936 register definitions and macros

/* GLOBAL DEFINITIONS */
#define REG_DIR_FLAG    (0x80)
#define REG_INC_FLAG    (0x40)
#define SPI_DUMMY_BYTE  (0x00)

/* MACROS */
#define SS_SET_LOW      (SS_Data_ADDR &= ~SS_MASK)
#define SS_SET_HIGH     (SS_Data_ADDR |=  SS_MASK)

/**
 *
 */
void cyrf6936Write (BYTE regAddr, BYTE regData)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr | REG_DIR_FLAG);
	/* This loop waits until data register becomes empty */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
	/* Send register data */
	SPIM_SendTxData (regData);
	/* This loop waits until master done transmitting a byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_SPI_COMPLETE)){};
	SS_SET_HIGH;
}

/**
 *
 */
BYTE cyrf6936Read (BYTE regAddr)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr);
	/* This loop waits until data register becomes empty */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
	/* Send register data */
	SPIM_SendTxData (SPI_DUMMY_BYTE);
	/* This loop waits until master receives one byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
	SS_SET_HIGH;

	return SPIM_bReadRxData ();
}

/**
 *
 */
void cyrf6936WriteMulti (BYTE regAddr, BYTE *pData, BYTE dataLen)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr | REG_DIR_FLAG);
	while (dataLen) {
		/* This loop waits until data register becomes empty */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
		/* Send register data */
		SPIM_SendTxData (*pData);
		pData++;
		dataLen--;
	}
	/* This loop waits until master done transmitting a byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_SPI_COMPLETE)){};
	SS_SET_HIGH;
}

/**
 *
 */
void cyrf6936CWriteMulti (BYTE regAddr, const BYTE *pData, BYTE dataLen)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr | REG_DIR_FLAG);
	while (dataLen) {
		/* This loop waits until data register becomes empty */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
		/* Send register data */
		SPIM_SendTxData (*pData);
		pData++;
		dataLen--;
	}
	/* This loop waits until master done transmitting a byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_SPI_COMPLETE)){};
	SS_SET_HIGH;
}

/**
 *
 */
void cyrf6936WriteBlock (BYTE regAddr, BYTE *pData, BYTE dataLen)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr | REG_DIR_FLAG | REG_INC_FLAG);
	while (dataLen) {
		/* This loop waits until data register becomes empty */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
		/* Send register data */
		SPIM_SendTxData (*pData);
		pData++;
		dataLen--;
	}
	/* This loop waits until master done transmitting a byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_SPI_COMPLETE)){};
	SS_SET_HIGH;
}

/**
 *
 */
void cyrf6936ReadMulti (BYTE regAddr, BYTE *pData, BYTE dataLen)
{
	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (regAddr);
	/* This loop waits until data register becomes empty */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
	while (dataLen) {
		/* Send dummy data */
		SPIM_SendTxData (SPI_DUMMY_BYTE);
		/* This loop waits until master receives one byte */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
		*pData = SPIM_bReadRxData ();
		pData++;
		dataLen--;
	}
	SS_SET_HIGH;
}

/**
 *
 */
BYTE cyrf6936RxIrqStatusGet (void)
{
	BYTE irqStatus;

	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (RX_IRQ_STATUS_ADR);
	/* This loop waits until data register becomes empty */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
	/* Send dummy data to run SPI clock */
	SPIM_SendTxData (SPI_DUMMY_BYTE);
	/* This loop waits until master receives one byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
	irqStatus = SPIM_bReadRxData ();
	if ((irqStatus & (RXC_IRQ | RXE_IRQ)) != (RXC_IRQ | RXE_IRQ)) {
		/* Send dummy data again to run SPI clock */
		SPIM_SendTxData (SPI_DUMMY_BYTE);
		/* This loop waits until master receives one byte */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
		irqStatus |= SPIM_bReadRxData ();
	}
	SS_SET_HIGH;
	return irqStatus;
}

/**
 *
 */
BYTE cyrf6936TxIrqStatusGet (void)
{
	BYTE irqStatus;

	SS_SET_LOW;
	/* Send register address */
	SPIM_SendTxData (TX_IRQ_STATUS_ADR);
	/* This loop waits until data register becomes empty */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_TX_BUFFER_EMPTY)){};
	/* Send dummy data to run SPI clock */
	SPIM_SendTxData (SPI_DUMMY_BYTE);
	/* This loop waits until master receives one byte */
	while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
	irqStatus = SPIM_bReadRxData ();
	if ((irqStatus & (TXC_IRQ | TXE_IRQ)) != (TXC_IRQ | TXE_IRQ)) {
		/* Send dummy data again to run SPI clock */
		SPIM_SendTxData (SPI_DUMMY_BYTE);
		/* This loop waits until master receives one byte */
		while (!(SPIM_bReadStatus () & SPIM_SPIM_RX_BUFFER_FULL)){};
		irqStatus |= SPIM_bReadRxData ();
	}
	SS_SET_HIGH;
	return irqStatus;
}
