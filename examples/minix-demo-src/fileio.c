/*{{{Comments*/
/*
 * File        : fileio.c
 * Author      : Stuart Menefy
 * Description : Minix file system resing routines
 * Header      : $Id: fileio.c,v 1.6 1994/08/05 11:10:35 stuart Exp stuart $
 *   
 * This file supplies the file input routines to support cmonitor.c.
 * This is used when running on a B016 and using a MINIX filesystem via
 * an M212.
 * As a result it also uses rawfs.c, and so provides readblock.
 *
 * History:
 *   06/02/94  SIM  Created.
 */
/*}}}*/

/*{{{#include*/
#include <stddef.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <minix/config.h>
#include <minix/const.h>
#include <minix/diskparm.h>
#include <minix/partition.h>
#include "rawfs.h"
#include "io.h"
/*}}}*/
/*{{{externs and globals*/
extern void* bootLinkIn;
extern void* bootLinkOut;

extern int errno;

int file_inode;
static int offset;
ino_t dir_inode = 0;
static struct stat file_stat;
static int block_size;
static int partition_start;
/*}}}*/
/*{{{Channel communications macros*/
#define ChanOut(link, data, size) \
  __asm{ ldabc size, link, data; out; }
#define ChanIn(link, data, size) \
  __asm{ ldabc size, link, data; in; }
/*}}}*/

/*{{{select_block*/
static void select_block(int block)
{
    static char data[] = {
        3, 0x4, 0,      /* WriteParam(LogicalSector0, x) */
        3, 0x5, 0,      /* WriteParam(LogicalSector1, x) */
        3, 0x6, 0       /* WriteParam(LogicalSector2, x) */
    };

    data[2] = (block      ) & 0xff;
    data[5] = (block >>  8) & 0xff;
    data[8] = (block >> 16) & 0xff;
    ChanOut(bootLinkOut, data, sizeof(data));
}
/*}}}*/
/*{{{select_buffer*/
static void select_buffer(int buffer)
{
    static char data[] = {
        3, 0x1d, 0      /* WriteParam(DesiredSectorBuffer, n) */
    };

    data[2] = buffer;
    ChanOut(bootLinkOut, data, sizeof(data));
}
/*}}}*/
/*{{{check_error*/
static void check_error(int block)
{
    static char data2[] = {
        2, 0x20         /* ReadParam(Error) */
    };
    char error;

    ChanOut(bootLinkOut, data2, sizeof(data2));
    ChanIn(bootLinkIn, &error, 1);
    if (error != 0) {
        printk("Unable to read sector ");
        print_int(block);
        printk(" (reason ");
        print_int(error);
        printk(")\n");
    }
}
/*}}}*/
/*{{{readblock*/
void readblock(off_t block, char* address)
{
    static char data1[] = {
        6               /* ReadSector() */
    };
    static char data2[] = {
        4               /* ReadBuffer() */
    };
    int length;

    /* Calculate logical sector number from block number */
    block += partition_start;
    if (block_size == 512)
        block *= 2;

    /* Read data from disk into M2 buffer */
    select_block(block);
    select_buffer(0);
    for (length = BLOCK_SIZE; length > 0; length -= block_size) {
        ChanOut(bootLinkOut, data1, sizeof(data1));
    }

    /* Read data from M2 buffer into user space */
    select_buffer(0);
    for (length = BLOCK_SIZE; length > 0; length -= block_size) {
        ChanOut(bootLinkOut, data2, sizeof(data2));
        ChanIn(bootLinkIn, address, block_size);
        address += block_size;
    }

    check_error(block);
}
/*}}}*/
/*{{{dump - not used*/
#if 0
void dump(char* address)
{
    char data[4];
    int line, row;
    char val;

    data[2] = ' ';
    data[3] = '\0';
    
    for (line = 0; line < BLOCK_SIZE; line += 16) {
        print_int(line);
        printk("  ");
        for (row = 0; row < 16; row++) {
            val = (address[line + row] & 0xf0) >> 4;
            data[0] = (val > 9) ? val + 'a' - 10 : val + '0';
            val = (address[line + row] & 0x0f);
            data[1] = (val > 9) ? val + 'a' - 10 : val + '0';
            printk(data);
        }
        printk("\n");
    }
}
#endif
/*}}}*/

/*{{{disk_init*/
int disk_init(void)
{
    return 0;   /* Do not attempt to open minix.cf */
}
/*}}}*/
/*{{{drive_select*/
void drive_select(int drive, int partition)
{
    static char select_drive[] = {
        3, 0x1e, 1,     /* WriteParam(DesiredDrive, x(overwritten)) */
        1, 1,           /* Initialise(0 = floppy, 1 = winny) */
        3, 0x09, 10,    /* WriteParam(sectorSizeLg2, 10 = 1024) */
        3, 0x0a,  9,    /* WriteParam(NumberOfSectors, 9) */
        3, 0x7,   7,    /* WriteParam(Addressing, Logical & Inc both) */
        8               /* Restore() */
    };
    static char write_param[] = {
        3, 0, 0         /* WriteParam(x, y) */
    };
    static size_t fd_offsets[4] = {
        offsetof(struct block0_s, spt[0]),      /* A: NumberOfSectors */
        offsetof(struct block0_s, nsides[0]),   /* B: NumberOfHeads */
        0,                                      /* C: NumberOfCylinders0 */
        1                                       /* D: NumberOfCylinders1 */
    };
    static size_t hd_offsets[4] = {
        offsetof(struct hard_info, sectors),    /* A: NumberOfSectors */
        offsetof(struct hard_info, heads),      /* B: NumberOfHeads */
        offsetof(struct hard_info, cylinders),  /* C: NumberOfCylinders0 */
        offsetof(struct hard_info, cylinders)+1 /* D: NumberOfCylinders1 */
    };

    union {
        struct hard_info hd;
        struct block0_s  fd;
        char             raw[BLOCK_SIZE];
    } buffer;
    int   param;
    off_t disk_size;
    size_t (*offsets)[4];

    printk("About to init disk\n");
    select_drive[2] = drive;
    if (drive > 2) {
        select_drive[4] = 0; /* Floppy */
        select_drive[7] = 9; /* 512 bytes */
        block_size = 512;
    } else {
        block_size = 1024;
    }

    partition_start = 0;
    ChanOut(bootLinkOut, select_drive, sizeof(select_drive));
    printk("Init disk OK\n");

    printk("About to read root block\n");
    readblock(0, &buffer.raw[0]);
    if ((buffer.hd.magic[0] == PART_MAGIC0) &&
        (buffer.hd.magic[1] == PART_MAGIC1))
    {
        /* Assume this is a hard disk with a partition table */
        int part;
        printk("Ahh, this is a hard disk\n");
        for (part=0; part<4; part++) {
            printk("Partition ");
            print_int(part);
            printk(buffer.hd.parts[part].bootind ? " active " : " not active ");
            print_int(buffer.hd.parts[part].sysind);
            printk(" start sector ");
            print_int(buffer.hd.parts[part].lowsec);
            printk(" size ");
            print_int(buffer.hd.parts[part].size);
            printk("\n");
        }
        partition_start = buffer.hd.parts[partition].lowsec;
        offsets = &hd_offsets;
    } else {
        /* Assume this is a floppy */
        int sectors = buffer.fd.nsects[0] + (buffer.fd.nsects[1] << 8);
        int sectors_per_cylinder = buffer.fd.spt[0] * buffer.fd.nsides[0];

        printk("Ahh, this is a floppy disk\n");
        print_int(sectors);
        printk(" sectors\n");

        /* Fudge the number of cylinders value for initialising the M2 */
        *((int*)(&buffer)) = sectors / sectors_per_cylinder;
        offsets = &fd_offsets;
    }

    /* Initialise the M2 disk geometry registers */
    for (param=0; param < 4; param++) {
        write_param[1] = 0xa + param;
        write_param[2] = buffer.raw[(*offsets)[param]];
        ChanOut(bootLinkOut, write_param, sizeof(write_param));
        printk("Write param "); print_int(write_param[1]);
        printk(" value "); print_int(write_param[2]);
        printk("\n");
    }
    
    disk_size = r_super();
    printk("size of disk ");
    print_int(disk_size);
    printk("\n");

    dir_inode = ROOT_INO;
}
/*}}}*/
/*{{{r_open*/
void r_open(const char* filename)
{
    if (dir_inode == 0) {
        printk("Unable to open file, drive not initialised yet\n");
        return;
    }

    file_inode = r_lookup(dir_inode, filename);
    if (file_inode == 0) {
        printk("Unable to find file \"");
        printk(filename);
        printk("\" (reason ");
        print_int(errno);
        printk(")\n");
        return;
    }

    offset = 0;

    /* Need to stat the file to set 'curfil' for future reads. */
    r_stat(file_inode, &file_stat);
}
/*}}}*/
/*{{{r_read*/
int r_read(char* address)
{
    int block;

    if (file_stat.st_size == 0)
        return 0;    

    block = r_vir2abs(offset);
    readblock(block, address);
    offset++;

    if (file_stat.st_size > BLOCK_SIZE) {
        block = BLOCK_SIZE;
    } else {
        block = file_stat.st_size;
    }

    file_stat.st_size -= block;
    return block;
}
/*}}}*/
/*{{{r_readline*/
int r_readline(char* buffer, int max_len)
{
    static char block[BLOCK_SIZE];
    static int remaining = 0;   /* Remaining chars in block */
    static char *ptr;           /* Ptr into block */

    while (1) {
        while (remaining-- > 0) {
            if ((*buffer = *(ptr++)) == '\n') {
                *buffer = '\0';
                return 1;
            }
            buffer++;
        }

        if ((remaining = r_read(block)) == 0) {
            *buffer = '\0';
            return 0;
        }

        ptr = block;
    }
}
/*}}}*/
/*{{{r_close*/
void r_close(void)
{
    ;
}
/*}}}*/
