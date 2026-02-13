/*{{{Comments*/
/*
 * File        : termio.c
 * Author      : Stuart Menefy
 * Description : Serial terminal I/O functions.
 * Header      : $Id: termio.c,v 1.1 1994/02/08 11:57:56 stuart Exp $
 *
 * This module provides the terminl I/O functions which are used by
 * csecond.c and cmonitor.c.
 * It is used with a B016 when talking to a memory mapped SCN2681.
 *
 * History:
 *   06/02/94  SIM  Created.
 */
/*}}}*/

/*{{{#include*/
#include <minix/config.h>

#if ! (B002 || B016)
#error Board type not defined
#endif

#include "tpduart.h"
#include "io.h"
/*}}}*/
/*{{{externs and globals*/
extern void* bootLinkIn;
extern void* bootLinkOut;

#define CHAN 1  /* RS232 channel 0=A, 1=B */
/*}}}*/

/*{{{term_init*/
void term_init(void)
{
    /* Initialise various line control parameters for RS232 port 0. */

    /* Initialise the registers */
    DUART->chan[CHAN].write.command = CR_RESET_Rx | CR_DISABLE_Rx | CR_DISABLE_Tx;
    DUART->chan[CHAN].write.command = CR_RESET_Tx;
    DUART->chan[CHAN].write.command = CR_RESET_ERROR;
    DUART->chan[CHAN].write.command = CR_RESET_BCI;

    DUART->general.write.aux_control = ACR_BRGS2;
    DUART->general.write.output_conf = 0;    /* Output port from OPR */

    /* Reset mode register pointer to 0 */
    DUART->chan[CHAN].write.command = CR_RESET_MR;

    /* First access after reset, so MR1 */
    DUART->chan[CHAN].write.mode = MR1_NO_PARITY | MR1_8BPC;

    /* Second access after reset, so MR2 */
    DUART->chan[CHAN].write.mode = MR2_STOP_BITS_1;

    /* Set send and receive baud rates */
    DUART->chan[CHAN].write.clock = (CS_9600 << 4) | CS_9600;

    /* Reenable Rx and Tx */
    DUART->chan[CHAN].write.command = CR_ENABLE_Rx | CR_ENABLE_Tx;
}
/*}}}*/
/*{{{raw_putc*/
static void raw_putc(const char c)
{
    while (! (DUART->chan[CHAN].read.status & SR_TxRDY))
        ;

    DUART->chan[CHAN].write.hold = c;
}
/*}}}*/
/*{{{putchar*/
void putchar(const char c)
{
    raw_putc(c);
    if (c == '\n')
        raw_putc('\r');
}
/*}}}*/
/*{{{printk*/
void printk(const char* msg)
{
    for ( ; *msg != '\0'; msg++) {
        putchar(*msg);
    }
}
/*}}}*/
/*{{{getchar*/
char getchar(void)
{
    while (! (DUART->chan[CHAN].read.status & SR_RxRDY))
        ;

    return DUART->chan[CHAN].read.hold;
}
/*}}}*/
