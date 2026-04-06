/*{{{Comments*/
/*
 * File        : linkio.c
 * Author      : Stuart Menefy
 * Description : IServer file reading and terminal I/O routines.
 * Header      : $Id: linkio.c,v 1.8 1994/10/11 10:23:27 stuart Exp stuart $
 *   
 * This file supplies the IO functions to support cmonitor.c and csecond.c.
 * As such, it contains only the routines common to both.
 * This is used when booting from link and using the hosts's file system
 * via an IServer.
 *
 * History:
 *   04/07/93  SIM  Created.
 */
/*}}}*/

/*{{{#include*/
#include <string.h>
#include <transputer/channel.h>
#include <transputer/iserver.h>
#include "io.h"

#undef BLOCK_SIZE
#define BLOCK_SIZE 507
/*}}}*/
/*{{{externs and globals*/
void print_int(const int i);

extern channel_t* bootLinkIn;
extern channel_t* bootLinkOut;

int file_inode; /* Make global for r_readline */

#define B002 0
#define B016 1
/*}}}*/

#define OLD 1

/*{{{disk_init*/
int disk_init(void)
{
    return 1;   /* Attempt to open minix.cf */
}
/*}}}*/
#if OLD
/*{{{r_open*/
void r_open(const char* filename)
{
    int len = strlen(filename);
    char hdr[3] = {10,              /* Fopen */
                   0,0};            /* Length (patched) */
    char trailer[2] = {1,           /* Binary */
                       1};          /* Open for input */                   
    int pkt_len = sizeof(hdr) + sizeof(trailer) + len;
    int pad = pkt_len & 1;

    hdr[1] = len % 256;
    hdr[2] = len / 256;

    pkt_len += pad;

    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, sizeof(hdr));
    ChanOut(bootLinkOut, filename, len);
    ChanOut(bootLinkOut, trailer, sizeof(trailer));
    if (pad)
        ChanOut(bootLinkOut, hdr, 1);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, &trailer, 1);    /* Result */
    pkt_len--;
    if (trailer[0] == 0) {
        ChanIn(bootLinkIn, &file_inode, sizeof(file_inode));
        pkt_len -= sizeof(file_inode);
    }
    if (pkt_len > 0)
        ChanIn(bootLinkIn, hdr, pkt_len);

    if (trailer[0] != 0) {
        printk("Unable to find file \"");
        printk(filename);
        printk("\" (reason ");
        print_int(trailer[0]);
        printk(")\n");
        file_inode = -1;
    }
}
/*}}}*/
/*{{{r_read*/
int r_read(char* address)
{
    char hdr[7] = {12,                  /* Fread */
                   0,0,0,0,             /* Handle (patched) */
                   BLOCK_SIZE & 0xff,   /* Length (lsb) */
                   BLOCK_SIZE >> 8};    /* Length (msb) */
    int pkt_len = 8;    /* 7 + padding */
    int len =0;         /* Clear msb */

    memcpy(&hdr[1], &file_inode, 4);
    
    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, pkt_len);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, &hdr, 1);
    ChanIn(bootLinkIn, &len, 2);
    if (len > 0) {
        ChanIn(bootLinkIn, address, len);
    }

    pkt_len -= (3 + len);
    if (pkt_len > 0)
        ChanIn(bootLinkIn, hdr, pkt_len);
    
    return len;
}
/*}}}*/
#else
/* is_message version */
/*{{{r_open*/
void r_open(const char* filename)
{
    int result;
    
    result = iserver_open(filename, 1, 1, &file_inode);
    if (result != ER_SUCCESS) {
        printk("Unable to find file \"");
        printk(filename);
        printk("\" (reason ");
        print_int(result);
        printk(")\n");
        file_inode = -1;
    }
}

    /*}}}*/
/*{{{r_read*/
int r_read(char* address)
{
    return iserver_read(file_inode, address, BLOCK_SIZE);    
}
/*}}}*/
/*{{{iserver_message*/
void iserver_message(
    iovec_t      *iov,          /* Vector of requests */
    unsigned int  nr_writes,    /* Number of writes in vector */
    unsigned int  nr_reads)     /* Number of reads in vector */
{
  /*
   * A task has issused a scatterd IO request to the host link.
   * Having got information about the host link if required,
   * we output the writes and then read back as many reads as possible.
   * Any surplus is read and discarded.
   */

  int length;           /* Total length of input or output message. */
  int partlen;          /* Length of this part of the message. */
  int padding;          /* Length of padding. */
  int i;

  /* Find the length of the outgoing message. */
  length = 0;
  for (i = 0; i < nr_writes; i++) {
        length += (int)iov[i].iov_size;
  }

  /* Pad to minimum IServer requirements. */
  if (length < 6) {
        padding = 6 - length;
  } else {
        padding = length & 1;
  }
    
  /* Send the request. */
  ChanOutInt16(bootLinkOut, length + padding);
  for ( ; nr_writes > 0; nr_writes--) {
        ChanOut(bootLinkOut, (void*)iov->iov_addr, (int)iov->iov_size);
        iov++;
  }

  /* Send any padding required. */
  if (padding > 0)
        ChanOut(bootLinkOut, (void*)MININT, padding);

  /* Now get the result. */
  length = ChanInInt16(bootLinkIn);
  for ( ; (nr_reads > 0) && (length >0); nr_reads--) {
        partlen = (int)iov->iov_size;
        if (partlen > length)
                partlen = length;
                
        ChanIn(bootLinkIn, (void*)iov->iov_addr, partlen);
        length -= partlen;
        iov++;
  }

  /* Read in any padding. */
  for ( ; length > 0; length--)
        (void)ChanInByte(bootLinkIn);
}


/*}}}*/
#endif
/*{{{r_close*/
void r_close()
{
    ;
}
/*}}}*/

/*{{{term_init*/
void term_init(void)
{
    ;
}
/*}}}*/
/*{{{putchar*/
void putchar(const char c)
{
    char string[2];

    string[0] = c;
    string[1] = '\0';
    printk(string);
}

/*}}}*/
#if OLD
/*{{{printk*/
void printk(const char* msg)
{
    int len = strlen(msg);
    char hdr[7] = {13,              /* Fwrite */
                   1,0,0,0,         /* Stream = 1 = stdout */
                   0,0};            /* Length (patched) */
    int pkt_len = 7 + len;
    int pad = pkt_len & 1;

    if (len == 0)
        return;
    
    hdr[5] = len % 256;
    hdr[6] = len / 256;

    pkt_len += pad;
    
    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, 7);
    ChanOut(bootLinkOut, msg, len);
    if (pad)
        ChanOut(bootLinkOut, hdr, 1);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, hdr, pkt_len);
}
/*}}}*/
/*{{{getchar*/
char getchar(void)
{
    char hdr[6] = {30,              /* GetKey */
                   0,0,0,0,0};      /* padding */
    int pkt_len = 6;
    char result[2];    
    
    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, pkt_len);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, result, pkt_len);

    return result[1];
}
/*}}}*/
#else
/*{{{printk*/
void printk(const char* msg)
{
    iserver_write(1, msg, strlen(msg));
}
/*}}}*/
#endif
