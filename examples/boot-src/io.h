/*
 * File        : io.h
 * Author      : Stuart Menefy
 * Description : Interface to IO routines
 * Header      : $Id: io.h,v 1.4 1994/08/05 10:26:31 stuart Exp $
 *
 * This file defines the interface which is required by cmonitor.c to
 * access the file system and terminal.
 *
 * History:
 *   04/07/93  SIM  Created
 *   06/02/94  SIM  Divided terminal and disk/file operations
 */

/* Terminal I/O functions (termio.c or linkio.c). */
void term_init(void);
void putchar(const char c);
void printk(const char* msg);
void print_int(int num);
char getchar(void);

/* Higher level terminal I/O functions (monitorl.c or fileio.c). */
int r_readline(char* buffer, int max_len);

/* Disk/file I/O functions (diskio.c or linkio.c). */
int disk_init(void);
void r_open(const char* filename);
int r_read(char* buffer);
void r_close(void);
void disk_finish(void);
