/*
 * File        : tpduart.h
 * Autor       : Stuart Menefy
 * Description : DUART definitions
 * Header      : $Id: tpduart.h,v 1.1 1994/01/10 14:09:46 stuart Exp $
 *
 * Details of the Signetics SCN2681 Dual Asynchronous Receiver/Transmitter
 * which is used to control the serial ports on all INMOS boards, including
 * the B001, B002, B006 and B016.
 *
 * History:
 *   27/06/93  SIM   Created
 */

/* Structure describing the register layout of a single channel. */
typedef union
{
    struct {
        int mode;
        int status;
        int _reserved;
        int hold;
    } read;
    struct {
        int mode;
        int clock;
        int command;
        int hold;
    } write;
    int _padding[8];
} duart_chan_s;

/* Description of the layout of the DUART as a whole. */
typedef union {
    duart_chan_s chan[2];
    union {
        struct {
            int _padding1[4];
            int input_change;
            int int_status;
            int counter[2];
            int _padding2[5];
            int input_port;
            int start_counter;
            int stop_counter;
        } read;
        struct {
            int _padding1[4];
            int aux_control;
            int int_mask;
            int counter[2];
            int _padding2[5];
            int output_conf;
            int set_output;
            int reset_output;
        } write;
    } general;
} duart_s;

#if B002
#  define DUART            ((duart_s*)0x80200000)
#  define EVENT_ACK        ((int*)0x80300000)
#  define EVENT_ACK_VALUE  0
#elif B016
#  define DUART            ((duart_s*)0x7fd80000)
#  define EVENT_ACK        ((int*)0x7fd98004)
#  define EVENT_ACK_VALUE  0x100
#else
#  error No suitable board defined
#endif

/* Bits within registers */
#define SR_RxRDY 1  /* Staus register : RxRdy */
#define SR_TxRDY 4  /* Staus register : TxRdy */
#define SR_TxEMT 8  /* Staus register : TxEmt */
#define INVERT !
#if INVERT 0
/* Not inverted */
#  define IM_RxReadyB 0x02 /* Chan A Receiver (of FIFO) full */
#  define IM_RxReadyA 0x20 /* Chan B Receiver (of FIFO) full */
#  define IM_TxReadyB 0x01 /* Chan A Tranmitter reday */
#  define IM_TxReadyA 0x10 /* Chan B Tranmitter reday */
#else
#  define IM_RxReadyA 0x02 /* Chan A Receiver (of FIFO) full */
#  define IM_RxReadyB 0x20 /* Chan B Receiver (of FIFO) full */
#  define IM_TxReadyA 0x01 /* Chan A Tranmitter reday */
#  define IM_TxReadyB 0x10 /* Chan B Tranmitter reday */
#endif

/* Command register commands */
#define CR_NONE           0
#define CR_RESET_MR    0x10
#define CR_RESET_Rx    0x20
#define CR_RESET_Tx    0x30
#define CR_RESET_ERROR 0x40
#define CR_RESET_BCI   0x50 /* Reset break change interrupt */
#define CR_START_BREAK 0x60
#define CR_STOP_BREAK  0x70

/* Command register other bits */
#define CR_ENABLE_Rx  1
#define CR_DISABLE_Rx 2
#define CR_ENABLE_Tx  4
#define CR_DISABLE_Tx 8

/* Mode register 1 */
#define MR1_NO_PARITY   0x10
#define MR1_8BPC           3

/* Mode register 2 */
#define MR2_STOP_BITS_1 7

/* Clock select register */
#define CS_9600 0xb

/* Aux control register */
#define ACR_BRGS2 0x80

/* The following defines are those taken from rs232.c, and may not
 * remain unchanged for long.
 */

/* Line control bits. */
#define LC_NO_PARITY            0x10
#define LC_ODD_PARITY           0x04
#define LC_EVEN_PARITY          0x00
#define LC_DATA_BITS            3

/* Modem status bits. */
#define MS_CTS               0x10

/* Line Status bits */
#define LS_OVERRUN_ERR          2
#define LS_PARITY_ERR           4
#define LS_FRAMING_ERR          8
#define LS_BREAK_INTERRUPT   0x10
#define LS_TRANSMITTER_READY 0x20

#define DEF_BAUD             9600	/* default baud rate */
#define DATA_BITS_SHIFT         8	/* amount data bits shifted in mode */

