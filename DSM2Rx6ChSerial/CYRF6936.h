#ifndef __CYRF6936_H
#define __CYRF6936_H

/**
 * CYRF6936 Register Map
 */
#define CHANNEL_ADR         (0x00)
#define TX_LENGTH_ADR       (0x01)
#define TX_CTRL_ADR         (0x02)
#define TX_CFG_ADR          (0x03)
#define TX_IRQ_STATUS_ADR   (0x04)
#define RX_CTRL_ADR         (0x05)
#define RX_CFG_ADR          (0x06)
#define RX_IRQ_STATUS_ADR   (0x07)
#define RX_STATUS_ADR       (0x08)
#define RX_COUNT_ADR        (0x09)
#define RX_LENGTH_ADR       (0x0A)
#define PWR_CTRL_ADR        (0x0B)
#define XTAL_CTRL_ADR       (0x0C)
#define IO_CFG_ADR          (0x0D)
#define GPIO_CTRL_ADR       (0x0E)
#define XACT_CFG_ADR        (0x0F)
#define FRAMING_CFG_ADR     (0x10)
#define DATA32_THOLD_ADR    (0x11)
#define DATA64_THOLD_ADR    (0x12)
#define RSSI_ADR            (0x13)
#define EOP_CTRL_ADR        (0x14)
#define CRC_SEED_LSB_ADR    (0x15)
#define CRC_SEED_MSB_ADR    (0x16)
#define TX_CRC_LSB_ADR      (0x17)
#define TX_CRC_MSB_ADR      (0x18)
#define RX_CRC_LSB_ADR      (0x19)
#define RX_CRC_MSB_ADR      (0x1A)
#define TX_OFFSET_LSB_ADR   (0x1B)
#define TX_OFFSET_MSB_ADR   (0x1C)
#define MODE_OVERRIDE_ADR   (0x1D)
#define RX_OVERRIDE_ADR     (0x1E)
#define TX_OVERRIDE_ADR     (0x1F)
#define XTAL_CFG_ADR        (0x26)
#define CLK_OFFSET_ADR      (0x27)
#define CLK_EN_ADR          (0x28)
#define RX_ABORT_ADR        (0x29)
#define AUTO_CAL_TIME_ADR   (0x32)
#define AUTO_CAL_OFFSET_ADR (0x35)
#define ANALOG_CTRL_ADR     (0x39)
#define TX_BUFFER_ADR       (0x20)
#define RX_BUFFER_ADR       (0x21)
#define SOP_CODE_ADR        (0x22)
#define DATA_CODE_ADR       (0x23)
#define PREAMBLE_ADR        (0x24)
#define MFG_ID_ADR          (0x25)

/**
 * Bit definitions
 */

/* TX_CTRL_ADR: 0x02h */
#define TXE_IRQEN           (0x01)
#define TXC_IRQEN           (0x02)
#define TXBERR_IRQEN        (0x04)
#define TXB0_IRQEN          (0x08)
#define TXB8_IRQEN          (0x10)
#define TXB15_IRQEN         (0x20)
#define TX_CLR              (0x40)
#define TX_GO               (0x80)

/* TX_CFG_ADR: 0x03h */
#define PA_SETTING0         (0x01)
#define PA_SETTING1         (0x02)
#define PA_SETTING2         (0x04)
#define DATA_MODE0          (0x08)
#define DATA_MODE1          (0x10)
#define DATA_CODE_LENGTH    (0x20)

#define PA_N30_DBM          (0x00)
#define PA_N25_DBM          (0x01)
#define PA_N20_DBM          (0x02)
#define PA_N15_DBM          (0x03)
#define PA_N10_DBM          (0x04)
#define PA_N5_DBM           (0x05)
#define PA_0_DBM            (0x06)
#define PA_4_DBM            (0x07)

#define MODE_1MBPS          (0x00)
#define MODE_8DR            (0x08)
#define MODE_DDR            (0x10)
#define MODE_SDR            (0x18)

/* TX_IRQ_STATUS_ADR: 0x04h */
#define TXE_IRQ             (0x01)
#define TXC_IRQ             (0x02)
#define TXBERR_IRQ          (0x04)
#define TXB0_IRQ            (0x08)
#define TXB8_IRQ            (0x10)
#define TXB15_IRQ           (0x20)
#define LV_IRQ              (0x40)
#define OS_IRQ              (0x80)

/* RX_CTRL_ADR: 0x05h */
#define RXE_IRQEN           (0x01)
#define RXC_IRQEN           (0x02)
#define RXBERR_IRQEN        (0x04)
#define RXB1_IRQEN          (0x08)
#define RXB8_IRQEN          (0x10)
#define RXB16_IRQEN         (0x20)
#define RX_GO               (0x80)

/* RX_CFG_ADR: 0x06h */
#define VLD_EN              (0x01)
#define RXOW_EN             (0x02)
#define FAST_TURN_EN        (0x08)
#define HILO                (0x10)
#define ATT                 (0x20)
#define LNA                 (0x40)
#define EGC_EN              (0x80)

/* RX_IRQ_STATUS_ADR: 0x07h */
#define RXE_IRQ             (0x01)
#define RXC_IRQ             (0x02)
#define RXBERR_IRQ          (0x04)
#define RXB1_IRQ            (0x08)
#define RXB8_IRQ            (0x10)
#define RXB16_IRQ           (0x20)
#define SOPDET_IRQ          (0x40)
#define RXOW_IRQ            (0x80)

/* RX_STATUS_ADR: 0x08h */
#define RX_DATA_MODE0       (0x01)
#define RX_DATA_MODE1       (0x02)
#define RX_CODE             (0x04)
#define BAD_CRC             (0x08)
#define CRC0                (0x10)
#define EOP_ERR             (0x20)
#define PKT_ERR             (0x40)
#define RX_ACK              (0x80)

/* XTAL_CTRL_ADR: 0x0Ch */
#define FRQ0                (0x01)
#define FRQ1                (0x02)
#define FRQ2                (0x04)
#define XSIRQ_EN            (0x20)
#define XOUT_FN0            (0x40)
#define XOUT_FN1            (0x80)

#define FREQ_12MHZ          (0x00)
#define FREQ_6MHZ           (0x01)
#define FREQ_3MHZ           (0x02)
#define FREQ_1P5MHZ         (0x03)
#define FREQ_P75MHZ         (0x04)

#define FNC_XOUT_FREQ       (0x00)
#define FNC_PA_N            (0x40)
#define FNC_RAD_STREAM      (0x80)
#define FNC_GPIO            (0xC0)

/* IO_CFG_ADR: 0x0Dh */
#define IRQ_GPIO            (0x01)
#define SPI_3PIN            (0x02)
#define PACTL_GPIO          (0x04)
#define PACTL_OD            (0x08)
#define XOUT_OD             (0x10)
#define MISO_OD             (0x20)
#define IRQ_POL             (0x40)
#define IRQ_OD              (0x80)

/* XACT_CFG_ADR: 0x0Fh */
#define ACK_TO0             (0x01)
#define ACK_TO1             (0x02)
#define END_STATE0          (0x04)
#define END_STATE1          (0x08)
#define END_STATE2          (0x10)
#define FRC_END             (0x20)
#define ACK_EN              (0x80)

#define END_STATE_SLEEP     (0x00)
#define END_STATE_IDLE      (0x04)
#define END_STATE_TXSYNTH   (0x08)
#define END_STATE_RXSYNTH   (0x0C)
#define END_STATE_RX        (0x10)

#define ACK_TO_4X           (0x00)
#define ACK_TO_8X           (0x01)
#define ACK_TO_12X          (0x02)
#define ACK_TO_15X          (0x03)

/* FRAMING_CFG_ADR: 0x10h */
#define SOP_TH0             (0x01)
#define SOP_TH1             (0x02)
#define SOP_TH2             (0x04)
#define SOP_TH3             (0x08)
#define SOP_TH4             (0x10)
#define LEN_EN              (0x20)
#define SOP_LEN             (0x40)
#define SOP_EN              (0x80)

/* EOP_CTRL_ADR: 0x14h */
#define EOP0                (0x01)
#define EOP1                (0x02)
#define EOP2                (0x04)
#define EOP3                (0x08)
#define HINT0               (0x10)
#define HINT1               (0x20)
#define HINT2               (0x40)
#define HEN                 (0x80)

/* MODE_OVERRIDE_ADR 0x1Dh */
#define RST                 (0x01)
#define FRC_AWAKE0          (0x08)
#define FRC_AWAKE1          (0x10)
#define FRC_SEN             (0x20)

/* RX_OVERRIDE_ADR 0x1Eh */
#define ACE                 (0x02)
#define DIS_RXCRC           (0x04)
#define DIS_CRC0            (0x08)
#define FRC_RXDR            (0x10)
#define MAN_RXACK           (0x20)
#define RXTX_DLY            (0x40)
#define ACK_RX              (0x80)

/* TX_OVERRIDE_ADR 0x1Fh */
#define TX_INV              (0x01)
#define DIS_TXCRC           (0x04)
#define OVRD_ACK            (0x08)
#define MAN_TXACK           (0x10)
#define FRC_PRE             (0x40)
#define ACK_TX              (0x80)

/* CLK_EN_ADR: 0x28h */
#define RXF                 (0x02)

/* RX_ABORT_ADR: 0x29h */
#define ABORT_EN            (0x20)

void cyrf6936Write (BYTE regAddr, BYTE regData);
BYTE cyrf6936Read (BYTE regAddr);
void cyrf6936WriteMulti (BYTE regAddr, BYTE *pData, BYTE dataLen);
void cyrf6936CWriteMulti (BYTE regAddr, const BYTE *pData, BYTE dataLen);
void cyrf6936WriteBlock (BYTE regAddr, BYTE *pData, BYTE dataLen);
void cyrf6936ReadMulti (BYTE regAddr, BYTE *pData, BYTE dataLen);
BYTE cyrf6936RxIrqStatusGet (void);
BYTE cyrf6936TxIrqStatusGet (void);

#endif /* __CYRF6936_H */
