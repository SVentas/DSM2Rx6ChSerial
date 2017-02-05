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

/* GLOBAL DEFINITIONS */
#define FLASH_BLOCK_SIZE        (0x0040)
#define MODEL_ID_LENGTH         (0x0004)
#define MODEL_ID_ADDR_OFFSET    (0x0000)
#define E2PROM_EMPTY_BYTE       (0x30)
#define ROOM_TEMPERATURE        (0x19)

#define PN_CODE_NUM_ROWS        (0x05)
#define PN_CODE_NUM_COLS        (0x09)
#define PN_CODE_SIZE            (0x08)

#define CHECKSUM_BIAS           (0x0170)

#define RX_NUM_CHANNELS         (0x50)
#define RX_RESYNC_DELAY         (0x30)

#define SPEKTRUM_FRAME_SIZE     (0x10)
#define TRANSMIT_BUFFER_SIZE    (0x0A)
#define TRANSMIT_PACKET_NUM     (600)

#define TICKS_PER_MS            (10)

#define IRQ_WAIT_TIMEOUT_48MS   (48) /* 48ms window */
#define IRQ_WAIT_TIMEOUT_24MS   (24) /* 24ms window */
#define IRQ_WAIT_TIMEOUT_22MS   (22) /* 22ms window */
#define IRQ_WAIT_TIMEOUT_19MS   (19) /* 19ms window */
#define IRQ_WAIT_TIMEOUT_12MS   (12) /* 12ms window */
#define IRQ_WAIT_TIMEOUT_06MS   ( 6) /*  6ms window */
#define TX_TRANSMIT_PERIOD_X10  (76) /* 7ms6 x 10 period */

#define BIND_ERROR_THOLD        (0x02)
#define RX_STATUS_ERROR         (PKT_ERR | EOP_ERR | BAD_CRC)

/* MACROS */
#define RST_SET_HIGH        (RST_Data_ADDR |=  RST_MASK)
#define RST_SET_LOW         (RST_Data_ADDR &= ~RST_MASK)

#define LED_ON              (LED_Data_ADDR &= ~LED_MASK)
#define LED_OFF             (LED_Data_ADDR |=  LED_MASK)
#define LED_TOGGLE          (LED_Data_ADDR ^=  LED_MASK)

#define BIND_NOT_DETECTED   (BIND_Data_ADDR & BIND_MASK)

#define RX_CHANNEL_GET_NEXT_BIND(chn)   {(chn)+=0x02; if((chn)>=RX_NUM_CHANNELS)(chn)=0x00;}
#define RX_CHANNEL_GET_NEXT_RECV(chn)   {(chn)+=0x02; if((chn)>=(RX_NUM_CHANNELS-0x01))(chn)%=(RX_NUM_CHANNELS-0x01);}
#define RX_CHANNEL_UPDATE_SYNC(snc)     {if((snc))(snc)--;}

/* GLOBAL VARIABLES */
const BYTE pnCodes[PN_CODE_NUM_ROWS][PN_CODE_NUM_COLS][PN_CODE_SIZE] = {
/* Note these are in order transmitted (LSB 1st) */
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93}
  /* Col 8    {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40} */
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},
};

const BYTE pnBind[3][PN_CODE_SIZE] = {
  /* Col 0 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86},
  /* Col 1 */ {0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84},
  /* Col 2 */ {0x06, 0x0C, 0x12, 0x18, 0x1E, 0x24, 0xC9, 0x2C}
};

BYTE spektrumFrame[SPEKTRUM_FRAME_SIZE];
BYTE cyrf6936Buf[SPEKTRUM_FRAME_SIZE];
BYTE modelID[MODEL_ID_LENGTH];

WORD crcSeed;
BOOL fCRCSeedInv = FALSE;

BYTE colIdxSOP;
BYTE colIdxDATA;

BYTE chnANum;
BYTE chnBNum;
BYTE chnASync = 0x00;
BYTE chnBSync = 0x00;
WORD chnACntr;
WORD chnBCntr;

BOOL fSyncLocked = FALSE;

BYTE rssiA = 0x00;
BYTE rssiB = 0x00;

BOOL fBinding = FALSE;
BOOL fTransmitting = FALSE;

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
	BYTE rowIdx;

	cyrf6936WriteBlock (CRC_SEED_LSB_ADR, (BYTE *) &crcSeed, sizeof (crcSeed));
	/* Calculate row index of the pnCodes array. */
	rowIdx = (channel + 0x01) % PN_CODE_NUM_ROWS;
	cyrf6936CWriteMulti (SOP_CODE_ADR,  pnCodes[rowIdx][colIdxSOP], PN_CODE_SIZE);
	cyrf6936CWriteMulti (DATA_CODE_ADR, pnCodes[rowIdx][colIdxDATA], PN_CODE_SIZE * 0x02);
	cyrf6936Write (CHANNEL_ADR, channel + 0x01);
	cyrf6936Write (RX_CTRL_ADR, RX_GO | RXC_IRQEN | RXE_IRQEN); // 0x83
	return cyrf6936Read (RSSI_ADR);
}

/**
 *
 */
void cyrf6936RxAbort (void)
{
	cyrf6936Write (RX_ABORT_ADR, ABORT_EN); // Receive Abort Enable. 0x20
	(void) cyrf6936Read (RSSI_ADR);
	cyrf6936Write (XACT_CFG_ADR, FRC_END | END_STATE_RXSYNTH); // Force to Synth Mode (RX) 0x2C
	while (cyrf6936Read (XACT_CFG_ADR) & FRC_END) {}
	cyrf6936Write (RX_ABORT_ADR, 0x00);
}

/**
 *
 */
void cyrf6936Init (void)
{
	BYTE tmp;

	/* Toggle RST Pin */
	RST_SET_HIGH;
	for (tmp = 0x00; tmp < 0xFF; tmp++) { asm("nop"); }
	RST_SET_LOW;
	for (tmp = 0x00; tmp < 0xFF; tmp++) { asm("nop"); }

	cyrf6936Write (MODE_OVERRIDE_ADR, RST);         // Reset register content.
	for (tmp = 0x00; tmp < 0xFF; tmp++) { asm("nop"); }

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

		cyrf6936CWriteMulti (DATA_CODE_ADR, pnBind[0], PN_CODE_SIZE * 0x02);

		chnANum = 0x00;
		chnBNum = 0x00;

		chnACntr = (IRQ_WAIT_TIMEOUT_12MS * TICKS_PER_MS) - 1;
		chnBCntr = 0xFFFF;
	} else {
		cyrf6936Write (TX_CFG_ADR, DATA_CODE_LENGTH | MODE_8DR | PA_4_DBM); // 64 chip codes. 8DR Mode. PA +4dBm.
		cyrf6936Write (FRAMING_CFG_ADR, SOP_EN | SOP_LEN | LEN_EN | 0x0E);  // SOP Enable.Packet Length enable. SOP PN Code Length is 64 chips. SOP Correlator Threshold = 0x0E.
		cyrf6936Write (RX_OVERRIDE_ADR, 0x00);
		cyrf6936Write (TX_OVERRIDE_ADR, 0x00);

		colIdxSOP  = ~modelID[0];
		colIdxSOP += ~modelID[1];
		colIdxSOP += ~modelID[2];
		colIdxSOP +=  0x02;
		colIdxSOP &=  0x07;

		colIdxDATA = 0x07 - colIdxSOP;

		crcSeed = (modelID[1] << 0x08) | (modelID[0]);

		chnANum = 0x00;
		chnBNum = RX_NUM_CHANNELS / 0x02;

		chnACntr = (IRQ_WAIT_TIMEOUT_24MS * TICKS_PER_MS) - 1;
		chnBCntr = (IRQ_WAIT_TIMEOUT_48MS * TICKS_PER_MS) - 1;
	}
}

/**
 *
 */
void cyrf6936TxStart (void)
{
	WORD checksum = 0x0000;

	cyrf6936CWriteMulti (DATA_CODE_ADR, pnBind[1], PN_CODE_SIZE * 0x02);

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
		RX_CHANNEL_UPDATE_SYNC (chnBSync);
		if (fSyncLocked) {
			if ((chnASync == 0x00) && (chnBSync == 0x00)) {
				fSyncLocked = FALSE;
				RX_CHANNEL_GET_NEXT_RECV (chnBNum);
			}
		} else if (chnBSync == 0x00) {
			RX_CHANNEL_GET_NEXT_RECV (chnBNum);
		}

		if (chnANum == chnBNum) {
			RX_CHANNEL_GET_NEXT_RECV (chnBNum);
		}
	} else {
		RX_CHANNEL_UPDATE_SYNC (chnASync);
		if (fSyncLocked) {
			if ((chnASync == 0x00) && (chnBSync == 0x00)) {
				fSyncLocked = FALSE;
				RX_CHANNEL_GET_NEXT_RECV (chnANum);
			}
		} else if (chnASync == 0x00) {
			if (fBinding) {
				RX_CHANNEL_GET_NEXT_BIND (chnANum);
			} else {
				RX_CHANNEL_GET_NEXT_RECV (chnANum);
			}
		}

		if ((chnANum == chnBNum) && !fBinding) {
			RX_CHANNEL_GET_NEXT_RECV (chnANum);
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
			modelID[0] = 0x3C; //0xC3;
			modelID[1] = 0x4E; //0xB1;
			modelID[2] = 0x3A; //0xC5;
			modelID[3] = 0x38; //0xC7;
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

	/* Enable GPIO interrupts */
	M8C_EnableIntMask (INT_MSK0, INT_MSK0_GPIO | INT_MSK0_SLEEP);

	/* Enable Global Interrupts */
	M8C_EnableGInt;

	Counter8_Start ();
	if (fBinding) {
		rssiA = cyrf6936RxStartBinding (chnANum) & 0x1F;
	} else {
		rssiA = cyrf6936RxStartReceiving (chnANum) & 0x1F;
	}

	while (1) {
		/* Process ChannelA timeout. */
		if (chnACntr == 0x0000) {
			if (fTransmitting) {
				M8C_DisableGInt;
				chnACntr = ((TX_TRANSMIT_PERIOD_X10 * TICKS_PER_MS) / 10) - 1;
				M8C_EnableGInt;

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

				M8C_DisableGInt;
				chnACntr = ((TX_TRANSMIT_PERIOD_X10 * TICKS_PER_MS) / 10) - 1;
				M8C_EnableGInt;

				cyrf6936TxStartTransmitting ();
			} else {
				channelUpdate ();
				if (fBinding) {
					M8C_DisableGInt;
					chnACntr = (IRQ_WAIT_TIMEOUT_12MS * TICKS_PER_MS) - 1;
					M8C_EnableGInt;
				} else if (chnASync || chnBSync) {
					M8C_DisableGInt;
					chnACntr = (IRQ_WAIT_TIMEOUT_22MS * TICKS_PER_MS) - 1;
					M8C_EnableGInt;
				} else {
					M8C_DisableGInt;
					chnACntr = (IRQ_WAIT_TIMEOUT_48MS * TICKS_PER_MS) - 1;
					chnBCntr = (IRQ_WAIT_TIMEOUT_24MS * TICKS_PER_MS) - 1;
					M8C_EnableGInt;
				}

				cyrf6936RxAbort ();
				if (fBinding) {
					rssiA = cyrf6936RxStartBinding (chnANum) & 0x1F;
				} else {
					crcSeed = ~crcSeed;
					fCRCSeedInv = !fCRCSeedInv;
					rssiB = cyrf6936RxStartReceiving (chnBNum) & 0x1F;
				}
			}
		}

		/* Process ChannelB timeout. */
		/* ChannelB is not used while in BIND or TRANSMIT modes. */
		if (!fBinding && !fTransmitting && (chnBCntr == 0x0000)) {
			channelUpdate ();
			if (chnASync || chnBSync) {
				M8C_DisableGInt;
				chnBCntr = (IRQ_WAIT_TIMEOUT_22MS * TICKS_PER_MS) - 1;
				M8C_EnableGInt;
			} else {
				M8C_DisableGInt;
				chnBCntr = (IRQ_WAIT_TIMEOUT_48MS * TICKS_PER_MS) - 1;
				chnACntr = (IRQ_WAIT_TIMEOUT_24MS * TICKS_PER_MS) - 1;
				M8C_EnableGInt;
			}

			cyrf6936RxAbort ();
			crcSeed = ~crcSeed;
			fCRCSeedInv = !fCRCSeedInv;
			rssiA = cyrf6936RxStartReceiving (chnANum) & 0x1F;

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
 * GPIO Interrupt Service Routine.
 */
#pragma interrupt_handler CYRF6936_IRQ_ISR
void CYRF6936_IRQ_ISR (void)
{
	BYTE tmpVal;
	BYTE rxStatus;

	/**
	 * T R A N S M I T   M o d e
	 */
	if (fTransmitting) {
		/* Get TX IRQ Status. */
		tmpVal = cyrf6936TxIrqStatusGet ();

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
			rssiA = cyrf6936RxStartReceiving (chnANum) & 0x1F;
		} else {
			/* In TRANSMIT Mode crcSeed is a packet counter. */
			crcSeed++;
		}

		/* Clear any pending GPIO interrupts. */
		INT_CLR0 &= ~INT_MSK0_GPIO;
		return;
	}

	/**
	 * B I N D   a n d   R E C E I V E   M o d e s
	 */
	if (fBinding) {
		chnACntr = (IRQ_WAIT_TIMEOUT_12MS * TICKS_PER_MS) - 1;
	} else {
		if (fCRCSeedInv) {
			chnBCntr = (IRQ_WAIT_TIMEOUT_24MS * TICKS_PER_MS) - 1;
			chnACntr = (IRQ_WAIT_TIMEOUT_19MS * TICKS_PER_MS) - 1;
		} else {
			chnACntr = (IRQ_WAIT_TIMEOUT_24MS * TICKS_PER_MS) - 1;
			chnBCntr = (IRQ_WAIT_TIMEOUT_06MS * TICKS_PER_MS) - 1;
		}
	}

	/* Make CYRF6936_IRQ_ISR interrupt a nested interrupt. */
	M8C_EnableGInt;

	/* Get RX IRQ Status and RX Status. */
	tmpVal = cyrf6936RxIrqStatusGet ();
	rxStatus = cyrf6936Read (RX_STATUS_ADR);

	/* Get received packet size. */
	tmpVal = cyrf6936Read (RX_COUNT_ADR);

	/* Read data. */
	cyrf6936ReadMulti (RX_BUFFER_ADR, cyrf6936Buf, tmpVal);

	/* End reception. */
	cyrf6936Write (XACT_CFG_ADR, FRC_END | END_STATE_RXSYNTH); // 0x2C
	while (cyrf6936Read (XACT_CFG_ADR) & FRC_END) {}

	if (fBinding) {
		/* Check for errors. */
		if (rxStatus & RX_STATUS_ERROR) {
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
		rssiA = cyrf6936RxStartBinding (chnANum) & 0x1F;
	} else {
		/* Check for errors. */
		if (rxStatus & RX_STATUS_ERROR) {
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
		}

		if (fSyncLocked) {
			spektrumFrameUpdate ();
		}

		fCRCSeedInv = !fCRCSeedInv;
		crcSeed = ~crcSeed;

		if (fCRCSeedInv) {
			rssiB = cyrf6936RxStartReceiving (chnBNum) & 0x1F;
		} else {
			rssiA = cyrf6936RxStartReceiving (chnANum) & 0x1F;
			if (fSyncLocked) {
				tmpVal  = rssiA;
				tmpVal += rssiB;
				tmpVal *= 4;
				spektrumFrame[SPEKTRUM_FRAME_SIZE - 0x01] |= tmpVal;
				TX8_Write (spektrumFrame, SPEKTRUM_FRAME_SIZE);
			}
		}
	}

	/* Clear any pending GPIO interrupts. */
	INT_CLR0 &= ~INT_MSK0_GPIO;
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
