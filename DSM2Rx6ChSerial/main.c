//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "CYRF6936.h"	// CYRF6936 register definitions and macros

/**
 * PSoC1 Configuration:
 *
 *   MISO - P0[0]; (CYRF6936);
 *   MOSI - P0[5]; (CYRF6936);
 *   SS   - P0[7]; (CYRF6936);
 *   SCLK - P0[3]; (CYRF6936);
 *
 *   RST  - P0[4]; (CYRF6936);
 *   IRQ  - P0[6]; (CYRF6936);
 *
 *   LED  - P3[0];
 *
 *   RX   - P0[1]; (Satellite RX);
 *
 *   BIND - P3[2];
 *
 *   CH1  - P1[7];
 *   CH2  - P1[1]; (SCLK);
 *   CH3  - P1[5];
 *   CH4  - P1[0]; (SDATA);
 *   CH5  - P1[4]; (PPM);
 *   CH6  - P1[6]; (TX);
 */

/* Uncomment the next line to eneble Channels Hold feature. */
#define USE_CHN_HOLD
/* Uncomment the next line to enable DBG PIN toggling. */
#define DEBUG_MODE

/* GLOBAL DEFINITIONS */
#define FLASH_BLOCK_SIZE        (0x0040)
#define MODEL_ID_LENGTH         (0x0004)
#define MODEL_ID_ADDR_OFFSET    (0x0000)
#define E2PROM_EMPTY_BYTE       (0x30)
#define ROOM_TEMPERATURE        (0x19)

#define PN_CODE_SIZE            (0x08)

#define CHECKSUM_BIAS           (0x0170)

#define RX_NUM_CHANNELS         (0x50)
#define RX_RESYNC_DELAY         (0x30)

#define SPEKTRUM_FRAME_SIZE     (0x10)
#define TRANSMIT_BUFFER_SIZE    (0x0A)
#define TRANSMIT_PACKET_NUM     (600)

#define TICKS_PER_1MS           (5)
#define TICKS_PER_0MS5          (3)

#define BIND_ERROR_THOLD        (0x02)

/* MACROS */
#define RST_SET_HIGH            (RST_Data_ADDR |=  RST_MASK)
#define RST_SET_LOW             (RST_Data_ADDR &= ~RST_MASK)

#define LED_ON                  (LED_Data_ADDR &= ~LED_MASK)
#define LED_OFF                 (LED_Data_ADDR |=  LED_MASK)
#define LED_TOGGLE              (LED_Data_ADDR ^=  LED_MASK)

#define IRQ_DETECTED            (IRQ_Data_ADDR  &  IRQ_MASK)

#define BIND_NOT_DETECTED       (BIND_Data_ADDR &  BIND_MASK)
#ifdef DEBUG_MODE
#define DBG_PIN_TOGGLE          (DBG_Data_ADDR ^=  DBG_MASK)
#endif
#define BIND_CHANNEL_GET_NEXT(chn)  {(chn)+=0x02; if((chn)>=RX_NUM_CHANNELS)(chn)=0x00;}
#define RECV_CHANNEL_GET_NEXT(chn)  {(chn)+=0x02; if((chn)>=(RX_NUM_CHANNELS-0x01))(chn)%=(RX_NUM_CHANNELS-0x01);}
#define CHANNEL_UPDATE_SYNC(snc)    {if((snc))(snc)--;}

/* GLOBAL VARIABLES */
const BYTE pnCodes[43][PN_CODE_SIZE] =
{
/* 00 */{0x83, 0xf7, 0xa8, 0x2d, 0x7a, 0x44, 0x64, 0xd3},
/* 01 */{0x3f, 0x2c, 0x4e, 0xaa, 0x71, 0x48, 0x7a, 0xc9},
/* 02 */{0x17, 0xff, 0x9e, 0x21, 0x36, 0x90, 0xc7, 0x82},
/* 03 */{0xbc, 0x5d, 0x9a, 0x5b, 0xee, 0x7f, 0x42, 0xeb},
/* 04 */{0x24, 0xf5, 0xdd, 0xf8, 0x7a, 0x77, 0x74, 0xe7},
/* 05 */{0x3d, 0x70, 0x7c, 0x94, 0xdc, 0x84, 0xad, 0x95},
/* 06 */{0x1e, 0x6a, 0xf0, 0x37, 0x52, 0x7b, 0x11, 0xd4},
/* 07 */{0x62, 0xf5, 0x2b, 0xaa, 0xfc, 0x33, 0xbf, 0xaf},

/* 08 */{0x40, 0x56, 0x32, 0xd9, 0x0f, 0xd9, 0x5d, 0x97},
/* 09 */{0x8e, 0x4a, 0xd0, 0xa9, 0xa7, 0xff, 0x20, 0xca},
/* 10 */{0x4c, 0x97, 0x9d, 0xbf, 0xb8, 0x3d, 0xb5, 0xbe},
/* 11 */{0x0c, 0x5d, 0x24, 0x30, 0x9f, 0xca, 0x6d, 0xbd},
/* 12 */{0x50, 0x14, 0x33, 0xde, 0xf1, 0x78, 0x95, 0xad},
/* 13 */{0x0c, 0x3c, 0xfa, 0xf9, 0xf0, 0xf2, 0x10, 0xc9},
/* 14 */{0xf4, 0xda, 0x06, 0xdb, 0xbf, 0x4e, 0x6f, 0xb3},
/* 15 */{0x9e, 0x08, 0xd1, 0xae, 0x59, 0x5e, 0xe8, 0xf0},

/* 16 */{0xc0, 0x90, 0x8f, 0xbb, 0x7c, 0x8e, 0x2b, 0x8e},
/* 17 */{0x80, 0x69, 0x26, 0x80, 0x08, 0xf8, 0x49, 0xe7},
/* 18 */{0x7d, 0x2d, 0x49, 0x54, 0xd0, 0x80, 0x40, 0xc1},
/* 19 */{0xb6, 0xf2, 0xe6, 0x1b, 0x80, 0x5a, 0x36, 0xb4},
/* 20 */{0x42, 0xae, 0x9c, 0x1c, 0xda, 0x67, 0x05, 0xf6},
/* 21 */{0x9b, 0x75, 0xf7, 0xe0, 0x14, 0x8d, 0xb5, 0x80},
/* 22 */{0xbf, 0x54, 0x98, 0xb9, 0xb7, 0x30, 0x5a, 0x88},
/* 23 */{0x35, 0xd1, 0xfc, 0x97, 0x23, 0xd4, 0xc9, 0x88},

/* 24 */{0xe1, 0xd6, 0x31, 0x26, 0x5f, 0xbd, 0x40, 0x93},
/* 25 */{0xdc, 0x68, 0x08, 0x99, 0x97, 0xae, 0xaf, 0x8c},
/* 26 */{0xc3, 0x0e, 0x01, 0x16, 0x0e, 0x32, 0x06, 0xba},
/* 27 */{0xe0, 0x83, 0x01, 0xfa, 0xab, 0x3e, 0x8f, 0xac},
/* 28 */{0x5c, 0xd5, 0x9c, 0xb8, 0x46, 0x9c, 0x7d, 0x84},
/* 29 */{0xf1, 0xc6, 0xfe, 0x5c, 0x9d, 0xa5, 0x4f, 0xb7},
/* 30 */{0x58, 0xb5, 0xb3, 0xdd, 0x0e, 0x28, 0xf1, 0xb0},
/* 31 */{0x5f, 0x30, 0x3b, 0x56, 0x96, 0x45, 0xf4, 0xa1},

/* 32 */{0x03, 0xbc, 0x6e, 0x8a, 0xef, 0xbd, 0xfe, 0xf8},
/* 33 */{0x88, 0x17, 0x13, 0x3b, 0x2d, 0xbf, 0x06, 0xd6},
/* 34 */{0xf1, 0x94, 0x30, 0x21, 0xa1, 0x1c, 0x88, 0xa9},
/* 35 */{0xd0, 0xd2, 0x8e, 0xbc, 0x82, 0x2f, 0xe3, 0xb4},
/* 36 */{0x8c, 0xfa, 0x47, 0x9b, 0x83, 0xa5, 0x66, 0xd0},
/* 37 */{0x07, 0xbd, 0x9f, 0x26, 0xc8, 0x31, 0x0f, 0xb8},
/* 38 */{0xef, 0x03, 0x95, 0x89, 0xb4, 0x71, 0x61, 0x9d},
/* 39 */{0x40, 0xba, 0x97, 0xd5, 0x86, 0x4f, 0xcc, 0xd1},

/* 40 */{0xd7, 0xa1, 0x54, 0xb1, 0x5e, 0x89, 0xae, 0x86},
/* 41 */{0x98, 0x88, 0x1b, 0xe4, 0x30, 0x79, 0x03, 0x84},
/* 42 */{0x06, 0x0c, 0x12, 0x18, 0x1e, 0x24, 0xc9, 0x2c}
};

BYTE spektrumFrame[SPEKTRUM_FRAME_SIZE];
BYTE cyrf6936Buf[SPEKTRUM_FRAME_SIZE];
BYTE modelID[MODEL_ID_LENGTH];

WORD crcSeed;
BOOL fCRCSeedInv = FALSE;

BYTE idxSOP;
BYTE idxDATA;

BYTE chnANum;
BYTE chnBNum;
BYTE chnASync = 0x00;
BYTE chnBSync = 0x00;
BYTE chnACntr;
BYTE chnBCntr;

BOOL fSyncLocked = FALSE;

BYTE rssiA = 0x00;
BYTE rssiB = 0x00;

BOOL fBinding = FALSE;
BOOL fTransmitting = FALSE;

#ifdef USE_CHN_HOLD
BOOL fChnHold = FALSE;
#endif

/**
 *
 */
BYTE cyrf6936RxStartBinding (BYTE channel)
{
	cyrf6936Write (CHANNEL_ADR, channel + 0x01);
	cyrf6936Write (RX_CTRL_ADR, RX_GO | RXC_IRQEN | RXE_IRQEN); // 0x83
	return cyrf6936Read (RSSI_ADR);
}

/**
 *
 */
void cyrf6936TxStartTransmitting (void)
{
	cyrf6936Write (TX_LENGTH_ADR, TRANSMIT_BUFFER_SIZE);
	cyrf6936Write (TX_CTRL_ADR, TX_GO | TX_CLR | TXC_IRQEN | TXE_IRQEN); // 0xC3
	cyrf6936WriteMulti (TX_BUFFER_ADR, cyrf6936Buf, TRANSMIT_BUFFER_SIZE);
}

/**
 *
 */
BYTE cyrf6936RxStartReceiving (BYTE channel)
{
	BYTE idxRow;

	cyrf6936WriteBlock (CRC_SEED_LSB_ADR, (BYTE *) &crcSeed, sizeof (crcSeed));
	/* Calculate row index of the pnCodes array. */
	idxRow  = channel % 0x05;
	idxRow *= 0x08;
	cyrf6936CWriteMulti (SOP_CODE_ADR,  pnCodes[idxRow + idxSOP], PN_CODE_SIZE);
	cyrf6936CWriteMulti (DATA_CODE_ADR, pnCodes[idxRow + idxDATA], PN_CODE_SIZE * 0x02);
	cyrf6936Write (CHANNEL_ADR, channel + 0x01);
	cyrf6936Write (RX_CTRL_ADR, RX_GO | RXC_IRQEN | RXE_IRQEN); // 0x83
	return cyrf6936Read (RSSI_ADR);
}

/**
 *
 */
void cyrf6936RxAbort (void)
{
	BYTE rssi;
	BYTE tmpVal;

	do {
		cyrf6936Write (RX_ABORT_ADR, ABORT_EN); // Receive Abort Enable. 0x20
		rssi = cyrf6936Read (RSSI_ADR);
		if ((rssi & SOP_RSSI) || IRQ_DETECTED) {
			/* Get RX IRQ Status and RX Status. */
			tmpVal = cyrf6936RxIrqStatusGet ();

			/* Get received packet size. */
			tmpVal = cyrf6936Read (RX_COUNT_ADR);

			/* Read data. */
			if (tmpVal) {
				cyrf6936ReadMulti (RX_BUFFER_ADR, cyrf6936Buf, tmpVal);
			}
#ifdef DEBUG_MODE
			DBG_PIN_TOGGLE;
#endif
		}
		cyrf6936Write (XACT_CFG_ADR, FRC_END | END_STATE_RXSYNTH); // Force to Synth Mode (RX) 0x2C
		while (cyrf6936Read (XACT_CFG_ADR) & FRC_END) {}
		cyrf6936Write (RX_ABORT_ADR, 0x00);
	} while (IRQ_DETECTED);
}

/**
 *
 */
void cyrf6936Init (void)
{
	BYTE tmp;

	/* Toggle RST Pin */
	RST_SET_HIGH;
	for (tmp = 0xFF; tmp; tmp--) { asm("nop"); }
	RST_SET_LOW;
	for (tmp = 0xFF; tmp; tmp--) { asm("nop"); }

	cyrf6936Write (MODE_OVERRIDE_ADR, RST);         // Reset register content.
	for (tmp = 0xFF; tmp; tmp--) { asm("nop"); }

	cyrf6936Write (CLK_EN_ADR, RXF);                // Force Receive Clock Enable.
	cyrf6936Write (AUTO_CAL_TIME_ADR, 0x3C);        // This is a MUST value.
	cyrf6936Write (AUTO_CAL_OFFSET_ADR, 0x14);      // This is a MUST value.
	cyrf6936Write (IO_CFG_ADR, IRQ_POL);            // Set IRQ polarity to be active HIGH.
	cyrf6936Write (TX_OFFSET_LSB_ADR, 0x55);        // Set the 1MHz offset and make TX and RX
	cyrf6936Write (TX_OFFSET_MSB_ADR, 0x05);        // frequencies to be the same.
	cyrf6936Write (XACT_CFG_ADR, FRC_END | END_STATE_RXSYNTH); // Force to Synth Mode (RX)
	while (cyrf6936Read (XACT_CFG_ADR) & FRC_END) {}         
	cyrf6936Write (RX_CFG_ADR, LNA | FAST_TURN_EN); // LNA Manual Control Enable. Fast Turn Mode Enable.
	cyrf6936Write (DATA64_THOLD_ADR, 0x07);         // This is a recommended value.
	cyrf6936Write (XTAL_CTRL_ADR, FNC_RAD_STREAM);  // Radio data serial bit stream.
}

/**
 *
 */
void cyrf6936RxStart (BOOL fBinding)
{
	if (fBinding) {
		cyrf6936Write (TX_CFG_ADR, DATA_CODE_LENGTH | MODE_SDR | PA_4_DBM); // 64 chip codes. SDR Mode. PA +4dBm.
		cyrf6936Write (FRAMING_CFG_ADR, SOP_LEN | 0x0E);        // SOP PN Code Length is 64 chips. SOP Correlator Threshold = 0x0E.
		cyrf6936Write (RX_OVERRIDE_ADR, FRC_RXDR | DIS_RXCRC);  // Force Receive Data Rate. Disable CRC16 checker.
		cyrf6936Write (TX_OVERRIDE_ADR, DIS_TXCRC); // Disable Transmit CRC16.
		cyrf6936Write (EOP_CTRL_ADR, EOP1);         // EOP Symbol Count = 2.

		cyrf6936CWriteMulti (DATA_CODE_ADR, pnCodes[40], PN_CODE_SIZE * 0x02);

		chnANum = 0x00;
		chnBNum = 0x01;

		chnACntr = 12 * TICKS_PER_1MS;
		chnBCntr = 0xFF;
	} else {
		cyrf6936Write (TX_CFG_ADR, DATA_CODE_LENGTH | MODE_8DR | PA_4_DBM); // 64 chip codes. 8DR Mode. PA +4dBm.
		cyrf6936Write (FRAMING_CFG_ADR, SOP_EN | SOP_LEN | LEN_EN | 0x0E);  // SOP Enable.Packet Length enable. SOP PN Code Length is 64 chips. SOP Correlator Threshold = 0x0E.
		cyrf6936Write (RX_OVERRIDE_ADR, 0x00);
		cyrf6936Write (TX_OVERRIDE_ADR, 0x00);

		idxSOP  = 0x00;
		idxSOP -= modelID[0];
		idxSOP -= modelID[1];
		idxSOP -= modelID[2];
		idxSOP -= 0x01;
		idxSOP &= 0x07;

		idxDATA = 0x07 - idxSOP;

		crcSeed = (modelID[1] << 0x08) | (modelID[0]);

		chnANum = 0x00;
		chnBNum = RX_NUM_CHANNELS / 0x02;

		chnACntr = 23 * TICKS_PER_1MS;
		chnBCntr = 46 * TICKS_PER_1MS;
	}
}

/**
 *
 */
void cyrf6936TxStart (void)
{
	WORD checksum = 0x0000;

	cyrf6936CWriteMulti (DATA_CODE_ADR, pnCodes[41], PN_CODE_SIZE * 0x02);

	cyrf6936Buf[4] = spektrumFrame[10];
	cyrf6936Buf[5] = spektrumFrame[11];
	cyrf6936Buf[6] = spektrumFrame[12];
	cyrf6936Buf[7] = spektrumFrame[13];

	checksum  = cyrf6936Buf[0];
	checksum += cyrf6936Buf[1];
	checksum += cyrf6936Buf[2];
	checksum += cyrf6936Buf[3];
	checksum += cyrf6936Buf[4];
	checksum += cyrf6936Buf[5];
	checksum += cyrf6936Buf[6];
	checksum += cyrf6936Buf[7];
	checksum += CHECKSUM_BIAS;

	cyrf6936Buf[8] = checksum >> 8;
	cyrf6936Buf[9] = checksum & 0x00FF;
}

/**
 *
 */
void channelUpdate (void)
{
	if (fCRCSeedInv) {
		CHANNEL_UPDATE_SYNC (chnBSync);
		if (fSyncLocked) {
			if ((chnASync == 0x00) && (chnBSync == 0x00)) {
				fSyncLocked = FALSE;
#ifdef USE_CHN_HOLD
				if (!fChnHold)
#endif
				RECV_CHANNEL_GET_NEXT (chnBNum);
 
			}
		} else if (chnBSync == 0x00) {
#ifdef USE_CHN_HOLD
			if (!fChnHold)
#endif
			RECV_CHANNEL_GET_NEXT (chnBNum);
		}

		if (chnANum == chnBNum) {
			RECV_CHANNEL_GET_NEXT (chnBNum);
		}
	} else {
		CHANNEL_UPDATE_SYNC (chnASync);
		if (fSyncLocked) {
			if ((chnASync == 0x00) && (chnBSync == 0x00)) {
				fSyncLocked = FALSE;
#ifdef USE_CHN_HOLD
				if (!fChnHold)
#endif
				RECV_CHANNEL_GET_NEXT (chnANum);
			}
		} else if (chnASync == 0x00) {
			if (fBinding) {
				BIND_CHANNEL_GET_NEXT (chnANum);
			} else {
#ifdef USE_CHN_HOLD
				if (!fChnHold)
#endif
				RECV_CHANNEL_GET_NEXT (chnANum);
			}
		}

		if (chnANum == chnBNum) {
			RECV_CHANNEL_GET_NEXT (chnANum);
		}
	}
}

/**
 *
 */
void spektrumFrameUpdate (void)
{
	BYTE i;

	for (i = 0x00; i < SPEKTRUM_FRAME_SIZE; i++) {
		spektrumFrame[i] = cyrf6936Buf[i];
	}
}

/**
 *
 */
BYTE bindPacketVerify (void)
{
	WORD checksum;

	checksum  = cyrf6936Buf[0];
	checksum += cyrf6936Buf[1];
	checksum += cyrf6936Buf[2];
	checksum += cyrf6936Buf[3];
	checksum *= 0x02;
	checksum += CHECKSUM_BIAS;

	if (((checksum >> 8) == cyrf6936Buf[8]) && ((checksum & 0x00FF) == cyrf6936Buf[9])) {
		checksum += cyrf6936Buf[ 8];
		checksum += cyrf6936Buf[ 9];
		checksum += cyrf6936Buf[10];
		checksum += cyrf6936Buf[11];
		checksum += cyrf6936Buf[12];
		checksum += cyrf6936Buf[13];

		if (((checksum >> 8) == cyrf6936Buf[14]) && ((checksum & 0x00FF) == cyrf6936Buf[15])) {
			if ((cyrf6936Buf[10] == 0x01) && (cyrf6936Buf[11] == 0x06) &&
				(cyrf6936Buf[12] == 0x01) && (cyrf6936Buf[13] == 0x00)) {
				return 0x01;
			}
		}
	}
	return 0x00;
}

/**
 *
 */
void init (void )
{
	LED_ON; /* I'm alive pulse start! */

	SPIM_DisableInt ();
	SPIM_Start (SPIM_SPIM_MODE_0 | SPIM_SPIM_MSB_FIRST);

	TX8_DisableInt ();
	TX8_Start (TX8_PARITY_NONE);

	Counter8_EnableInt ();

	cyrf6936Init ();

	if (BIND_NOT_DETECTED) {
		E2PROM_Start ();
		E2PROM_E2Read (MODEL_ID_ADDR_OFFSET, modelID, MODEL_ID_LENGTH);
		E2PROM_Stop ();

		if ((modelID[0] == E2PROM_EMPTY_BYTE) && (modelID[1] == E2PROM_EMPTY_BYTE) &&
			(modelID[2] == E2PROM_EMPTY_BYTE) && (modelID[3] == E2PROM_EMPTY_BYTE)) {
			modelID[0] = 0x3C; //0xC3 inv;
			modelID[1] = 0x4E; //0xB1 inv;
			modelID[2] = 0x3A; //0xC5 inv;
			modelID[3] = 0x38; //0xC7 inv;
		}
		fBinding = FALSE;
	} else {
		fBinding = TRUE;
	}

	cyrf6936RxStart (fBinding);

	LED_OFF; /* End of the I'm alive pulse! */
}

/**
 *
 */
void main(void)
{
	BYTE tmpVal;

	init ();

	/* Clear any pending GPIO interrupts. */
	INT_CLR0 &= ~INT_MSK0_GPIO;

	/* Enable GPIO an Sleep_Timer interrupts. */
	M8C_EnableIntMask (INT_MSK0, INT_MSK0_GPIO | INT_MSK0_SLEEP);

	/* Enable Global Interrupts. */
	M8C_EnableGInt;

	/* Clear Watchdog. */
	M8C_ClearWDT;

	Counter8_Start ();
	if (fBinding) {
		rssiA = cyrf6936RxStartBinding (chnANum) & RSSI_MASK;
	} else {
		rssiA = cyrf6936RxStartReceiving (chnANum) & RSSI_MASK;
	}

	while (1) {
		/* Process ChannelA timeout. */
		if (chnACntr == 0x0000) {
			if (fTransmitting) {
				chnACntr = 38; /* 7ms6 */
				cyrf6936TxStartTransmitting ();
			} else if (fBinding && chnASync) {
				cyrf6936RxAbort ();
				cyrf6936TxStart ();

				E2PROM_Start ();
				tmpVal = E2PROM_bE2Write (MODEL_ID_ADDR_OFFSET, spektrumFrame, FLASH_BLOCK_SIZE, ROOM_TEMPERATURE);
				E2PROM_Stop ();

				crcSeed = 0x0000; /* In TRANSMIT Mode crcSeed is a packet counter. */
				fBinding = FALSE;
				fTransmitting = TRUE;

				chnACntr = 38; /* 7ms6 */
				cyrf6936TxStartTransmitting ();
			} else {
				/* Disable GPIO interrupt. */
				M8C_DisableIntMask (INT_MSK0, INT_MSK0_GPIO);

				channelUpdate ();
				if (fBinding) {
					chnACntr = 12 * TICKS_PER_1MS;
				} else if (chnASync || chnBSync) {
					chnACntr = 22 * TICKS_PER_1MS;
				} else {
					M8C_DisableGInt;
					chnACntr = 46 * TICKS_PER_1MS;
					chnBCntr = 23 * TICKS_PER_1MS;
					M8C_EnableGInt;
				}

				cyrf6936RxAbort ();

				/* Clear any pending GPIO interrupts. */
				INT_CLR0 &= ~INT_MSK0_GPIO;
				/* Enable GPIO interrupt. */
				M8C_EnableIntMask (INT_MSK0, INT_MSK0_GPIO);

				if (fBinding) {
					rssiA = cyrf6936RxStartBinding (chnANum) & RSSI_MASK;
				} else {
					crcSeed = ~crcSeed;
					fCRCSeedInv = !fCRCSeedInv;
					rssiB = cyrf6936RxStartReceiving (chnBNum) & RSSI_MASK;
				}
			}
		}

		/* Process ChannelB timeout. */
		/* ChannelB is not used while in BIND or TRANSMIT modes. */
		if (!fBinding && !fTransmitting && (chnBCntr == 0x0000)) {
			/* Disable GPIO interrupt. */
			M8C_DisableIntMask (INT_MSK0, INT_MSK0_GPIO);

			channelUpdate ();
			if (chnASync || chnBSync) {
				chnBCntr = 22 * TICKS_PER_1MS;
			} else {
				M8C_DisableGInt;
				chnBCntr = 46 * TICKS_PER_1MS;
				chnACntr = 23 * TICKS_PER_1MS;
				M8C_EnableGInt;
			}

			cyrf6936RxAbort ();

			/* Clear any pending GPIO interrupts. */
			INT_CLR0 &= ~INT_MSK0_GPIO;
			/* Enable GPIO interrupt. */
			M8C_EnableIntMask (INT_MSK0, INT_MSK0_GPIO);

			crcSeed = ~crcSeed;
			fCRCSeedInv = !fCRCSeedInv;
			rssiA = cyrf6936RxStartReceiving (chnANum) & RSSI_MASK;

			if (fSyncLocked) {
				tmpVal  = rssiA;
				tmpVal += rssiB;
				tmpVal *= 4;
				spektrumFrame[SPEKTRUM_FRAME_SIZE - 0x01] |= tmpVal;
				TX8_Write (spektrumFrame, SPEKTRUM_FRAME_SIZE);
			}
		}
		/* Clear Watchdog. */
		M8C_ClearWDT;
	}
}

/**
 * GPIO Interrupt Service Routine.
 */
#pragma interrupt_handler CYRF6936_IRQ_ISR
void CYRF6936_IRQ_ISR (void)
{
	BYTE tmpVal;
	BYTE irqStatus;

	/**
	 * T R A N S M I T   M o d e
	 */
	if (fTransmitting) {
		/* Get TX IRQ Status. */
		irqStatus = cyrf6936TxIrqStatusGet ();

		/* In TRANSMIT Mode crcSeed is a packet counter. */
		if (crcSeed >= TRANSMIT_PACKET_NUM) {
			Counter8_Stop ();

			chnASync = 0x00;
			chnBSync = 0x00;
			fSyncLocked = FALSE;
			fTransmitting = FALSE;

			modelID[0] = spektrumFrame[0];
			modelID[1] = spektrumFrame[1];
			modelID[2] = spektrumFrame[2];
			modelID[3] = spektrumFrame[3];

			cyrf6936RxStart (fBinding);

			Counter8_Start ();

			rssiA = cyrf6936RxStartReceiving (chnANum) & RSSI_MASK;
		} else {
			/* In TRANSMIT Mode crcSeed is a packet counter. */
			crcSeed++;
		}
		return;
	}

	/**
	 * B I N D   a n d   R E C E I V E   M o d e s
	 */

	/* Make CYRF6936_IRQ_ISR interrupt a nested interrupt. */
	M8C_EnableGInt;

	/* Get RX IRQ Status. */
	irqStatus = cyrf6936RxIrqStatusGet ();

	if (fBinding) {
		chnACntr = 12 * TICKS_PER_1MS;
	} else {
		if (fCRCSeedInv) {
			if ((irqStatus & RXE_IRQ) && (chnBCntr > (2 * TICKS_PER_1MS)) && (chnASync || chnBSync)) {
				chnBCntr += 22 * TICKS_PER_1MS;
#ifdef DEBUG_MODE
				DBG_PIN_TOGGLE;
#endif
			} else {
				chnBCntr = 23 * TICKS_PER_1MS + TICKS_PER_0MS5;
				chnACntr = 19 * TICKS_PER_1MS + TICKS_PER_0MS5;
			}
		} else {
			if ((irqStatus & RXE_IRQ) && (chnACntr > (2 * TICKS_PER_1MS)) && (chnASync || chnBSync)) {
				chnACntr += 22 * TICKS_PER_1MS;
#ifdef DEBUG_MODE
				DBG_PIN_TOGGLE;
#endif
			} else {
				chnACntr = 23 * TICKS_PER_1MS + TICKS_PER_0MS5;
				chnBCntr =  5 * TICKS_PER_1MS + TICKS_PER_0MS5;
			}
		}
	}

	/* Get received packet size. */
	tmpVal = cyrf6936Read (RX_COUNT_ADR);

	/* Read data. */
	if (tmpVal) {
		cyrf6936ReadMulti (RX_BUFFER_ADR, cyrf6936Buf, tmpVal);
	}

	/* End reception. */
	cyrf6936Write (XACT_CFG_ADR, FRC_END | END_STATE_RXSYNTH); // 0x2C
	while (cyrf6936Read (XACT_CFG_ADR) & FRC_END) {}

	if (fBinding) {
		/* Check for errors. */
		if (irqStatus & RXE_IRQ) {
			chnBSync++; /* In BIND Mode only ChannelA is used. chnBSync is a bind error counter. */
			if (chnBSync > BIND_ERROR_THOLD) {
				channelUpdate ();
				chnBSync = 0x00;
			}
		} else if (bindPacketVerify ()) {
			spektrumFrameUpdate ();

			chnASync = RX_RESYNC_DELAY;
			chnBSync = 0x00;
		} else {
			chnBSync++; /* In BIND Mode only ChannelA is used. chnBSync is a bind error counter. */
			if (chnBSync > BIND_ERROR_THOLD) {
				channelUpdate ();
				chnBSync = 0x00;
			}
		}
		rssiA = cyrf6936RxStartBinding (chnANum) & RSSI_MASK;
	} else {
		/* Check for errors. */
		if (irqStatus & RXE_IRQ) {
			channelUpdate ();
		} else if ((cyrf6936Buf[0] != modelID[2]) || (cyrf6936Buf[1] != modelID[3])) {
			channelUpdate ();
		} else {
			if (fCRCSeedInv) {
				chnBSync = RX_RESYNC_DELAY;
			} else {
				chnASync = RX_RESYNC_DELAY;
			}
		}

		/* Update Sync Lock first. */
		if (chnASync && chnBSync) {
			fSyncLocked = TRUE;
#ifdef USE_CHN_HOLD
			fChnHold = TRUE;
#endif
		}

		if (fSyncLocked) {
			spektrumFrameUpdate ();
		}

		fCRCSeedInv = !fCRCSeedInv;
		crcSeed = ~crcSeed;

		if (fCRCSeedInv) {
			rssiB = cyrf6936RxStartReceiving (chnBNum) & RSSI_MASK;
		} else {
			rssiA = cyrf6936RxStartReceiving (chnANum) & RSSI_MASK;
			if (fSyncLocked) {
				tmpVal  = rssiA;
				tmpVal += rssiB;
				tmpVal *= 4;
				spektrumFrame[SPEKTRUM_FRAME_SIZE - 0x01] |= tmpVal;
				TX8_Write (spektrumFrame, SPEKTRUM_FRAME_SIZE);
			}
		}
	}
}

/**
 * Counter8 Interrupt Service Routine.
 */
#pragma interrupt_handler Counter8_ISR
void Counter8_ISR (void)
{
	chnACntr--;
	chnBCntr--;
}

/**
 * Sleep_Timer Interrupt Service Routine.
 */
#define BIND_CLK_DIV        (0x04)
#define TRANSMIT_CLK_DIV    (0x10)
#pragma interrupt_handler Sleep_Timer_ISR
void Sleep_Timer_ISR (void)
{
	static BYTE clkDiv = 0x00;

	clkDiv++;
	if (fBinding) {
		if (clkDiv >= BIND_CLK_DIV) {
			clkDiv = 0x00;
			LED_TOGGLE;
		}
	} else if (fTransmitting) {
		if (clkDiv >= TRANSMIT_CLK_DIV) {
			clkDiv = 0x00;
			LED_TOGGLE;
		}
	} else if (fSyncLocked) {
		LED_ON;
	} else {
		LED_OFF;
	}
}
