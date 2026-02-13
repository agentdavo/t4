/*
 * File        : monitor.h
 * Author      : Stuart Menefy
 * Description : Boot monitor definitions.
 * Header      : $Id: monitor.h,v 1.6 1994/08/05 11:10:00 stuart Exp stuart $
 *
 * This file defines the types and functions which are shared between
 * the common and the disk or link specific modules of the boot monitor.
 *
 * History:
 *   07/02/94  SIM  Created
 *   10/02/94  SIM  Added new elements to command table.
 */

/* Commands list */
typedef struct command_t {
    char* cmd;
    int   min_args, max_args;
    struct command_t *(*fn)(int argc, char* argv[]);
    char* args;
    char* help;
} command_t;

extern command_t commands[];

/* Common functions */
command_t *boot(int argc, char* argv[]);
command_t *help(int argc, char* argv[]);
command_t *module(int argc, char* argv[]);
command_t *more(int argc, char* argv[]);
command_t *ramsize(int argc, char* argv[]);
command_t *rboot(int argc, char* argv[]);
command_t *rootdev(int argc, char* argv[]);
command_t *source(int argc, char* argv[]);

