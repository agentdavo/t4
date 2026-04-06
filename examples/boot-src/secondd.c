/*{{{Comments*/
/*
 * File        : secondd.c
 * Author      : Stuart Menefy
 * Description : Secondary bootstrap disk reading functions.
 * Header      : $Id: secondd.c,v 1.4 1994/05/16 18:18:36 stuart Exp $
 *
 * Routines for the secondary bootstrap when booting from a disk.
 * Provides disk specific commands for the secondary bootstrap.
 *
 * History:
 *   06/02/94  SIM  Created.
 */
/*}}}*/

/*{{{#include*/
#include "io.h"
/*}}}*/
/*{{{#defines*/
#define BLOCK_SIZE 256
/*}}}*/

/*{{{load*/
void load(int memory, void **code, int **data)
{
    /* Remember with diskio block size is 256 */
    char* ptr;
    int count;

    r_open("monitorl");
    ptr = (char*)0x80100000;
    for (count = 0; count < 50; count++) {
        (void)r_read(ptr);
        ptr += BLOCK_SIZE;
    }
    r_close();

    *code = (void*)0x80100020;
    *data = (int*)0x80080000;
}
/*}}}*/

