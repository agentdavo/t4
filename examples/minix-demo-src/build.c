/*{{{Comment*/
/*
 * File        : build.c
 * Author      : Stuart Menefy
 * Description : Build a transputer Minix boot file or disk.
 * Header      : $Id: build.c,v 1.3 1994/02/10 13:44:26 stuart Exp $
 *
 * The code will combine a small bootstrap program with the boot
 * monitor to generate a bootable file or disk.
 *
 * History:
 *   19/02/92  SIM  Created
 *   29/04/92  SIM  Added code for writing to a MINIX device
 *   04/07/93  SIM  Overhaul for boot monitor version.
 *   30/01/94  SIM  Major changes to support multi-stage bootstrap.
 *   10/02/94  SIM  New style command line parsing (tags etc).
 */

/*}}}*/
/*{{{Check #defines*/
/*
 * This program is able to produce bootables in three ways, depending
 * on the compilation option used:
 */

#ifdef HOST
/*
 * Run on a host (running UNIX or MINIX), generating boot from
 * link files.
 * Host 'long' must be 32 bits.
 * Must define BIG_ENDIAN or LITTLE_ENDIAN as appropiate for the host.
 */
#if ! (defined(BIG_ENDIAN) || defined(LITTLE_ENDIAN))
#error Must define BIG_ENDIAN or LITTLE_ENDIAN for hosted version
#endif
#else
#ifdef MINIX
/*
 * To be run under MINIX generating a bootable disk.
 */
#define LITTLE_ENDIAN
#else
#ifdef M2
/*
 * To be compiled with the INMOS toolset and run on a transputer talking
 * directly to an M2 to generate a disk.
 */
#define LITTLE_ENDIAN
#else
#error Compilation method not defined
#endif
#endif
#endif

/*}}}*/

/*{{{#includes*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#ifdef MINIX
#include <errno.h>
#include <a.out.h>
#include <minix/config.h>
#include <minix/diskparm.h>
#include <sgtty.h>
#else
/* These need to be the MINIX versions. */
#include "include/ansi.h"
#include "include/a.out.h"
#endif

#ifndef SEEK_SET
#define SEEK_SET     0
#define SEEK_CURRENT 1
#define SEEK_END     2
#endif
/*}}}*/
/*{{{#defines*/
#define NAME "build"
#define VERSION "0.06"

#ifdef HOST
#define DEF_OUTPUT_NAME "minix.btl"
#endif
#ifdef MINIX
#define DEF_OUTPUT_NAME "/dev/rfd0"
#endif

#define MAX_BOOTSTRAP_SIZE 256
#define TEXT_PATCH 0x12345678

#if defined(M2) || defined(MINIX)
#define sectorSize       256  /* 512 */
#define sectorSizeLg2      8  /*   9 */
#define sectorsPerTrack 0x10  /*   9 */
#endif
/*}}}*/

/*{{{Global variables*/
/* Command line arguments */
int           info   = 0;
#ifndef HOST
int           format = 0;
#endif
#ifndef M2
int           out_fd;
char*         out_filename = NULL;
#endif

typedef struct {
    /* These are filled in by parse_cl */
    int   tag;
    char* filename;

    /* While these are filled in by read_sizes (if appropiate) */
    FILE* file;
    long  text_len, bss_len;
    long  file_len;
} module_t;

#define TAG_BOOTSTRAP   1
#define TAG_INCLUDE     2
#define TAG_PATCH       4
#define TAG_STRIP       8
#define TAG_SET        16

#define MAX_MODULES 10
module_t modules[MAX_MODULES];
int num_modules;

/*}}}*/

/*{{{M2 stuff*/
#ifdef M2
/*{{{#defines*/
#define M2cmd_Initialise     1
#define M2cmd_ReadParameter  2
#define M2cmd_WriteParameter 3
#define M2cmd_ReadBuffer     4
#define M2cmd_WriteBuffer    5
#define M2cmd_ReadSector     6
#define M2cmd_WriteSector    7
#define M2cmd_Restore        8
#define M2cmd_SelectDrive    0xb
#define M2cmd_FormatTrack    0xd
#define M2param_ControllerAccess 0x7f

#define M2param_LogicalSector0     4
#define M2param_LogicalSector1     5
#define M2param_LogicalSector2     6
#define M2param_SectorSizeLg2      9
#define M2param_NumberOfSectors 0x0a
#define M2param_DesiredDrive    0x1e
#define M2param_Error           0x20
#define M2param_Reason          0x21

#define M2error_AllOk          0

#define M2hardcmd_ExternalWriteClock 7

#define CHAN_TO_M2   LINK2OUT
#define CHAN_FROM_M2 LINK2IN
/*}}}*/

/*{{{write_param*/
int write_param(int param, int value)
{
    ChanOutChar(CHAN_TO_M2, M2cmd_WriteParameter);
    ChanOutChar(CHAN_TO_M2, param);
    ChanOutChar(CHAN_TO_M2, value & 0xff);
}
/*}}}*/
/*{{{read_param*/
int read_param(int param)
{
    ChanOutChar(CHAN_TO_M2, M2cmd_ReadParameter);
    ChanOutChar(CHAN_TO_M2, param);

    return ChanInChar(CHAN_FROM_M2);
}
/*}}}*/
/*{{{write_sector*/
int write_sector(int sector, unsigned char* buffer)
{
    int error;

    write_param(M2param_LogicalSector0, (sector     ) & 0xff);
    write_param(M2param_LogicalSector1, (sector >> 8) & 0xff);
    write_param(M2param_LogicalSector2, (sector >>16) & 0xff);

    ChanOutChar(CHAN_TO_M2, M2cmd_WriteBuffer);
    ChanOut    (CHAN_TO_M2, buffer, sectorSize);
    ChanOutChar(CHAN_TO_M2, M2cmd_WriteSector);

    error = read_param(M2param_Error);
    if (error != M2error_AllOk)
    {
        fprintf(stderr, "Error while writing sector %04x : %02x\n",
                sector, error);
    }
}
/*}}}*/
#endif
/*}}}*/
/*{{{htotl & ttohl*/
#ifdef LITTLE_ENDIAN
#  define htotl(x) (x)
#  define ttohl(x) (x)
#else
/* Big endian, need to do word swapping. */
#  define htotl(x) _bit_rev(x)
#  define ttohl(x) _bit_rev(x)
unsigned long _bit_rev(const unsigned long value)
{
    unsigned long result;
    unsigned char* d, *s;

    s = (unsigned char*)&value;
    d = ((unsigned char*)&result)+3;

    *(d--) = *(s++);
    *(d--) = *(s++);
    *(d--) = *(s++);
    * d    = * s   ;

    return result;
}
#endif
/*}}}*/
/*{{{print_version*/
static void print_version(void)
{
    fprintf(stderr, "%s : Version %s (built %s %s)\n\n", NAME, VERSION,
            __DATE__, __TIME__);
}
/*}}}*/

/*{{{parse_cl*/
void parse_cl(int argc, char* argv[])
{
    char* arg;
    int tag;

    if (argc == 1)
    {
        print_version();

        fprintf(stderr, "%s [options] tag module [module ...] ...\n", NAME);
        fprintf(stderr, "Options:\n");
        fprintf(stderr, "  -v     Print information\n");
#ifndef HOST
        fprintf(stderr, "  -f     Format disk\n");
#endif
#ifndef M2
        fprintf(stderr, "  -o out Specify output filename (default %s)\n",
                DEF_OUTPUT_NAME);
#endif
        fprintf(stderr, "Tags:\n");
        fprintf(stderr, "  -b     Bootstrap (preceed by length byte)\n");
        fprintf(stderr, "  -i     Include (in output file)\n");
        fprintf(stderr, "  -p     Patch (with next text length)\n");
        fprintf(stderr, "  -s     Strip (a.out header)\n");
        exit(1);
    }

    out_filename = DEF_OUTPUT_NAME;
    num_modules = 0;

    argc--;
    while (argc > 0)
    {
        arg = *(++argv);
        argc--;

        if (arg[0] == '-') {
            tag = 0;
            for (arg++ ; *arg != '\0'; arg++) {
                switch (*arg)
                {
                    case 'v':
                        info++;
                        break;

#ifndef HOST
                    case 'f':
                        format++;
                        break;
#endif

#ifndef M2
                    case 'o':
                        if (arg[1] != '\0') {
                            fprintf(stderr, "o must the last option");
                            exit(1);
                        }
                        out_filename = *(++argv);
                        argc--;
                        if (out_filename == NULL)
                        {
                            fprintf(stderr, "No output filename given\n");
                            exit(1);
                        }
                        break;
#endif

                    case 'b':
                        tag |= (TAG_SET | TAG_BOOTSTRAP);
                        break;

                    case 'p':
                        tag |= (TAG_SET | TAG_PATCH);
                        break;

                    case 'i':
                        tag |= (TAG_SET | TAG_INCLUDE);
                        break;

                    case 's':
                        tag |= (TAG_SET | TAG_STRIP);
                        break;

                    default:
                        fprintf(stderr, "Unrecognised option \"%c\"\n", *arg);
                        exit(1);
                }
            }
        } else {
            /* Option does not start with a -, must be a filename. */

            if (num_modules == MAX_MODULES) {
                fprintf(stderr, "Number of modules exceeded\n");
                exit(1);
            }

            if (! (tag & TAG_SET)) {
                fprintf(stderr, "No tags set for module \"%s\"\n", arg);
                exit(1);
            }

            if ((tag & (TAG_STRIP | TAG_PATCH | TAG_BOOTSTRAP)) &&
                ! (tag & TAG_INCLUDE)) {
                fprintf(stderr, "Cannot strip, patch or mark as bootstrap module \"%s\"\n", arg);
                fprintf(stderr, "without including it\n");
                exit(1);
            }

            if ((tag & TAG_BOOTSTRAP) && !(tag & TAG_STRIP)) {
                fprintf(stderr, "Module \"%s\" marked as bootstrap must be stripped\n", arg);
                exit(1);
            }
            
            modules[num_modules].tag = tag;
            modules[num_modules].filename = arg;
            num_modules++;
        }
    }

    /* Some quick sanity checks. */
    if (num_modules == 0) {
        fprintf(stderr, "Must specify at least module\n");
        exit(1);
    }

    if (modules[num_modules-1].tag & TAG_PATCH) {
        fprintf(stderr, "Final module cannot take the patch directive\n");
        exit(1);
    }
}
/*}}}*/
/*{{{init*/
/* Three different versions of init */
#ifdef HOST
/*{{{init : HOST version*/
void init(void)
{
    if (info)
    {
        fprintf(stderr, "Writing to file \"%s\"\n", out_filename);
    }

    out_fd = open(out_filename, O_WRONLY | O_TRUNC | O_CREAT, 0666);
    if (out_fd == -1)
    {
        fprintf(stderr, "Unable to open output file \"%s\"\n", out_filename);
        exit(1);
    }
}

/*}}}*/
#endif
#ifdef MINIX
/*{{{init : MINIX version*/
void init(void)
{
    struct stat        sbuf;
    struct disk_char_s disk_char;
    int                track, param;

    if (info)
    {
        fprintf(stderr, "Writing to file \"%s\"\n", out_filename);
    }

    out_fd = open(out_filename, O_RDWR | O_NONBLOCK, 0666);
    if (out_fd == -1)
    {
        fprintf(stderr, "Unable to open output file \"%s\"\n",out_filename);
        exit(1);
    }

    /* Check if file is a special file, and if so init geometry */
    if (fstat(out_fd, &sbuf) == -1)
    {
        fprintf(stderr, "Unable to stat \"%s\"\n", out_filename);
        exit(1);
    }

    if (! S_ISCHR(sbuf.st_mode))
    {
        fprintf(stderr, "Output file \"%s\" must be a character special file\n",
                out_filename);
        exit(1);
    }

    /* Using a character special file, so init geometry */
    if (ioctl(out_fd, DK_GETPARAM, (struct sgttyb*)&disk_char) != 0)
    {
        perror("Failed to read drive parameters");
        exit(1);
    }

    disk_char.sector_size_lg2   = sectorSizeLg2;
    disk_char.number_of_sectors = sectorsPerTrack;

    if (ioctl(out_fd, DK_SETPARAM, (struct sgttyb*)&disk_char) != 0)
    {
        perror("Failed to write drive parameters");
        exit(1);
    }

    /* Format drive if requested */
    if (format)
    {
        fprintf(stderr, "Formatting disk...\n");
    
        param = 0;
        for (track=0; track < 80*2; track++)
        {
            if (ioctl(out_fd, DK_FORMAT, (struct sgttyb*)&param) != 0)
            {
                perror("Format error");
                exit(1);
            }
            fprintf(stderr, ".");
            fflush(stderr);
            param += sectorsPerTrack;
        }
        fprintf(stderr, "\ndone\n");
    }
}
/*}}}*/
#endif
#ifdef M2
/*{{{init : M2 version*/
void init(void)
{
    int error, track;

    if (info)
    {
        fprintf(stderr, "Writing to disk (%s)\n",
                format ? "Formatting first" : "Not formatting");
    }

    /* Initialise drive */
    write_param(M2param_ControllerAccess, M2hardcmd_ExternalWriteClock);
    write_param(M2param_DesiredDrive, 3);   /* Floppy drive */
    ChanOutChar(CHAN_TO_M2, M2cmd_SelectDrive);
    ChanOutChar(CHAN_TO_M2, M2cmd_Initialise);
    ChanOutChar(CHAN_TO_M2, 0);
    ChanOutChar(CHAN_TO_M2, M2cmd_SelectDrive);
    ChanOutChar(CHAN_TO_M2, M2cmd_Restore);
    write_param(M2param_SectorSizeLg2, sectorSizeLg2);
    write_param(M2param_NumberOfSectors, sectorsPerTrack);

    error = read_param(M2param_Error);
    if (error == M2error_AllOk)
    {
        if (info)
            fprintf(stderr, "Initialised drive ok\n");
    }
    else
    {
        fprintf(stderr, "Unable to initialise drive because %02x\n", error);
        exit(1);
    }
    
    /* Format drive if requested */
    if (format)
    {
        fprintf(stderr, "Formatting disk...\n");
    
        for (track=0; track < 80*2; track++)
        {
            ChanOutChar(CHAN_TO_M2, M2cmd_FormatTrack);
            fprintf(stderr, ".");
            fflush(stderr);
        }

        error = read_param(M2param_Error);
        if (error == M2error_AllOk)
        {
            fprintf(stderr, "done\n");
        }
        else
        {
            fprintf(stderr, "Error while formatting %02x\n", error);
            exit(1);
        }

        ChanOutChar(CHAN_TO_M2, M2cmd_Restore);
    }
}
/*}}}*/
#endif
/*}}}*/
/*{{{read_sizes*/
void read_sizes(module_t *module)
{
    char* filename = module->filename;
    FILE* file;
    struct exec header;
    struct stat sbuf;

    file = fopen(filename, "rb");
    if (file == NULL)
    {
        fprintf(stderr, "Unable to open file \"%s\"\n", filename);
        exit(1);
    }

    if (info)
        fprintf(stderr, "Reading header for \"%s\"\n", filename);
    
    if (fread(&header, A_MINHDR, 1, file) != 1)
    {
        fprintf(stderr, "Unable to read header for \"%s\"\n", filename);
        exit(1);
    }
    
    if (BADMAG(header))
    {
        fprintf(stderr, "\"%s\" is not an executable file\n", filename);
        exit(1);
    }

    if (fstat(fileno(file), &sbuf)) {
        fprintf(stderr, "Unable tp stat \"%s\"\n", filename);
        exit(1);
    }
    
    module->file     = file;
    module->text_len = ttohl(header.a_text);
    module->bss_len  = ttohl(header.a_bss);
    module->file_len = (int)sbuf.st_size;

    if (info)
    {
        fprintf(stderr, "  Text length #%04x\n", module->text_len);
        fprintf(stderr, "  BSS  length #%04x\n", module->bss_len);
    }

    if (module->tag & TAG_STRIP)
        fseek(file, header.a_hdrlen, SEEK_SET);
    else
        fseek(file, 0, SEEK_SET);
}
/*}}}*/
/*{{{output*/
void output(module_t *module, module_t* next_module)
{
    char*           filename = module->filename;
    int             module_len;     /* Length of module outputting */
    unsigned char   buffer[1024];   /* Must be a multiple of sectorSize */
                                    /* and at least MAX_BOOTSTRAP+1 */
    int             found;          /* If patch found */
    int             part_len;       /* Length of this part */
    int             length_read;    /* Result of read() */
    unsigned char  *ptr;            /* Offset into buffer */
    long            tmp;            /* Temporary storage for patch word */
#ifdef M2
    unsigned char  *data;           /* Pointer into buffer */
#endif

    if (! (module->tag & TAG_INCLUDE))
        return;
    
    if (info)
        fprintf(stderr, "Copying \"%s\"...\n", filename);

    if (module->tag & TAG_STRIP)
        module_len = module->text_len;
    else
        module_len = module->file_len;

    /* Special processing for primary bootstrap. */
    if (module->tag & TAG_BOOTSTRAP) {
        if (module_len > MAX_BOOTSTRAP_SIZE) {
            fprintf(stderr, "Bootstrap code size too big (%d bytes)\n",
                    module_len);
            exit(1);
        }
    }

    found = 0;

    while (module_len > 0)
    {
        part_len = (int)((module_len > sizeof(buffer)) ? sizeof(buffer) :
                                                            module_len);
        ptr = &buffer[0];
        if (module->tag & TAG_BOOTSTRAP) {
            *ptr = (unsigned char)module_len;
            ptr++;
        }
        
        length_read = fread(ptr, 1, part_len, module->file);
        if (length_read < 0) {
            perror("Error reading from input file");
            exit(1);
	}
        if (length_read != part_len) {
            fprintf(stderr, "Error reading from input file: short file (read %d)\n",
                    length_read);
            exit(1);
        }

        if (module->tag & TAG_BOOTSTRAP)
            part_len++;
        
        /* Patch in the next length. */
        if (module->tag & TAG_PATCH) {
            for (ptr = &buffer[0]; ptr < &buffer[part_len-3]; ptr++) {
                memcpy(&tmp, ptr, sizeof(long));
                if (ttohl(tmp) == TEXT_PATCH) {
                    if (info)
                        fprintf(stderr, "(Patch at %04x)", ptr - buffer);

                    if (found) {
                        fprintf(stderr, "Found multiple patch tags in \"%s\"\n",
                                filename);
                        exit(1);
                    }

                    found = 1;
#ifdef HOST
                    /* Add the length in BYTES */
                    tmp = htotl(next_module->text_len);
#else
                    /* Add the length in BLOCKS */
                    tmp = htotl((next_module->text_len + (sectorSize-1))/sectorSize);
#endif
                    memcpy(ptr, &tmp, sizeof(long));
                }
            }
        }

        /* Write out the buffer. */
#ifdef M2
        for (data = buffer; part_len > 0; data += sectorSize)
        {
            write_sector(sector++, data);
            part_len -= sectorSize,             
        }
#else
#ifndef HOST
        /* Round upto sector size when writing to disk */
        part_len = (int)((part_len + (sectorSize-1)) / sectorSize) * sectorSize;
#endif
        if (write(out_fd, buffer, part_len) != part_len)
        {
            perror("Unable to write to boot disk");
            exit(1);
        }

        module_len -= part_len;
#endif
        if (info)
        {
            fprintf(stderr, ".");
            fflush(stderr);
        }
    }

    if (info)
        fprintf(stderr, "\n");

    if ((module->tag & TAG_PATCH) && (! found))
    {
        fprintf(stderr, "Unable to find patch tag in \"%s\"\n", filename);
        exit(1);
    }

    fclose(module->file);
}
/*}}}*/
/*{{{finish*/
void finish(void)
{
#ifdef M2
    /* Deselect the disk */    
    write_param(M2param_DesiredDrive, 0);
    ChanOutChar(CHAN_TO_M2, M2cmd_SelectDrive);
#else
    close(out_fd);
#endif
}
/*}}}*/

/*{{{main*/
int main(int argc, char* argv[])
{
    module_t *cur_module, *next_module;

    parse_cl(argc, argv);
    if (info)
        print_version();
    init();

    cur_module  = &modules[0];
    next_module = &modules[1];
    read_sizes(cur_module);
    while (--num_modules > 0) {
        read_sizes(next_module);
        output(cur_module, next_module);
        cur_module = next_module;
        next_module++;
    }
    output(cur_module, NULL);

    finish();
    return 0;
}
/*}}}*/
