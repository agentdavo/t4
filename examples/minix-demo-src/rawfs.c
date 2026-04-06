/*	rawfs.c - Raw Minix file system support.	Author: Kees J. Bot
 *					     Based on readfs by Paul Polderman
 * Transputer Minix ID: $Id: rawfs.c,v 1.1 1994/01/10 14:07:14 stuart Exp $
 */
#define nil 0
#define _POSIX_SOURCE	1
#define _MINIX		1
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <minix/config.h>
#include <minix/const.h>
#include <minix/type.h>
#include <fs/const.h>
#include <fs/type.h>
#include <fs/buf.h>
#include <fs/super.h>
#include <fs/inode.h>
#include "rawfs.h"

void readblock(off_t block, char* address);

/* The following code handles two file system types: Version 1 with small inodes
 * and 16-bit disk addresses and Version 2 with big inodes and 32-bit disk
 * addresses.
#ifdef FLEX
 * To make matters worse, my own hacked file system knows about the normal Unix
 * Version 7 directories and directories with flexible entries.
#endif
#ifndef SUPER_V2
 * And of course it must still compile under Minix 1.5.
#endif
 *
 * All of these problems have been solved in the most tasteful way with only
 * a few #ifdef's here and there.  (They have been specially arranged so that
 * ifdef(1) can remove them flawlessly.)
 */

#ifdef SUPER_V2		/* Test for V2 file system include files. */
static unsigned NR_DZONES;	/* Fill these in after reading superblock. */
static unsigned NR_TZONES;
static unsigned NR_INDIRECTS;
static unsigned INODES_PER_BLOCK;
#include <sys/dir.h>
typedef struct direct	direct;		/* Can't use the 'struct' name. */
#ifdef FLEX
#include <dirent.h>
typedef struct _fl_direct  _fl_direct;
#endif
#else
/* Map old 1.5 names to 2.0 names. */
#define b_v1_ind	b_ind
#define b_v1_ino	b_inode
#define block_t		block_nr
#define zone_t		zone_nr
#define NR_DZONES	NR_DZONE_NUM
#define V1_NR_TZONES	NR_ZONE_NUMS
#define d1_inode	d_inode
#define d1_mode		i_mode
#define d1_uid		i_uid
#define d1_size		i_size
#define d1_mtime	i_mtime
#define d1_gid		i_gid
#define d1_nlinks	i_nlinks
#define d1_zone		i_zone
#define direct		dir_struct
#define d_ino		d_inum
#ifdef FLEX
#define _fl_direct	fl_dir_struct
#endif
#endif

static struct super_block super;	/* Superblock of file system */
static struct inode curfil;		/* Inode of file under examination */
static char indir[BLOCK_SIZE];		/* Single indirect block. */
static char dindir[BLOCK_SIZE];		/* Double indirect block. */
static char dirbuf[BLOCK_SIZE];		/* Scratch/Directory block. */
#define scratch dirbuf

static block_t a_indir, a_dindir;	/* Addresses of the indirects. */
static off_t dirpos;			/* Reading pos in a dir. */

#define fsbuf(b)	(* (struct buf *) (b))

#define	zone_shift	(super.s_log_zone_size)	/* zone to block ratio */

off_t r_super()
/* Initialize variables, return size of file system in blocks,
 * (zero on error).
 */
{
	/* Read superblock. */
	readblock((off_t) SUPER_BLOCK, scratch);

	memcpy((void *) &super, (void *) scratch, sizeof(super));

	/* Is it really a MINIX file system ? */
#ifdef SUPER_V2
	if (super.s_magic == SUPER_V2) {
		NR_DZONES= V2_NR_DZONES;
		NR_TZONES= V2_NR_TZONES;
		NR_INDIRECTS= V2_INDIRECTS;
		INODES_PER_BLOCK= V2_INODES_PER_BLOCK;
		return (off_t) super.s_zones << zone_shift;
	} else
#endif
	if (super.s_magic == SUPER_MAGIC) {
#ifdef SUPER_V2
		NR_DZONES= V1_NR_DZONES;
		NR_TZONES= V1_NR_TZONES;
		NR_INDIRECTS= V1_INDIRECTS;
		INODES_PER_BLOCK= V1_INODES_PER_BLOCK;
#endif
		return (off_t) super.s_nzones << zone_shift;
	} else {
		/* Filesystem not recognized as Minix. */
		return 0;
	}
}

void r_stat(inum, stp) ino_t inum; struct stat *stp;
/* Return information about a file like stat(2) and remember it. */
{
	block_t block;
	block_t ino_block;
	ino_t ino_offset;

	/* Calculate start of i-list */
	block = SUPER_BLOCK + 1 + super.s_imap_blocks + super.s_zmap_blocks;

	/* Calculate block with inode inum */
	ino_block = ((inum - 1) / INODES_PER_BLOCK);
	ino_offset = ((inum - 1) % INODES_PER_BLOCK);
	block += ino_block;

	/* Fetch the block */
	readblock((off_t) block, scratch);

#ifdef SUPER_V2
	if (super.s_magic == SUPER_V2) {
		d2_inode *dip;
		int i;

		dip= &fsbuf(scratch).b_v2_ino[ino_offset];

		curfil.i_mode= dip->d2_mode;
		curfil.i_nlinks= dip->d2_nlinks;
		curfil.i_uid= dip->d2_uid;
		curfil.i_gid= dip->d2_gid;
		curfil.i_size= dip->d2_size;
		curfil.i_atime= dip->d2_atime;
		curfil.i_mtime= dip->d2_mtime;
		curfil.i_ctime= dip->d2_ctime;
		for (i= 0; i < V2_NR_TZONES; i++)
			curfil.i_zone[i]= dip->d2_zone[i];
	} else
#endif
	{
		d1_inode *dip;
		int i;

		dip= &fsbuf(scratch).b_v1_ino[ino_offset];

		curfil.i_mode= dip->d1_mode;
		curfil.i_nlinks= dip->d1_nlinks;
		curfil.i_uid= dip->d1_uid;
		curfil.i_gid= dip->d1_gid;
		curfil.i_size= dip->d1_size;
		curfil.i_atime= dip->d1_mtime;
		curfil.i_mtime= dip->d1_mtime;
		curfil.i_ctime= dip->d1_mtime;
		for (i= 0; i < V1_NR_TZONES; i++)
			curfil.i_zone[i]= dip->d1_zone[i];
	}
	curfil.i_dev= -1;	/* Can't fill this in alas. */
	curfil.i_num= inum;

	stp->st_dev= curfil.i_dev;
	stp->st_ino= curfil.i_num;
	stp->st_mode= curfil.i_mode;
	stp->st_nlink= curfil.i_nlinks;
	stp->st_uid= curfil.i_uid;
	stp->st_gid= curfil.i_gid;
	stp->st_rdev= (dev_t) curfil.i_zone[0];
	stp->st_size= curfil.i_size;
	stp->st_atime= curfil.i_atime;
	stp->st_mtime= curfil.i_mtime;
	stp->st_ctime= curfil.i_ctime;

	a_indir= a_dindir= 0;
	dirpos= 0;
}

ino_t r_readdir(name) char *name;
/* Read next directory entry at "dirpos" from file "curfil". */
{
	ino_t inum= 0;
	int blkpos;
	direct *dp;

	if (!S_ISDIR(curfil.i_mode)) { errno= ENOTDIR; return -1; }

	while (inum == 0 && dirpos < curfil.i_size) {
		if ((blkpos= (int) (dirpos % BLOCK_SIZE)) == 0) {
			/* Need to fetch a new directory block. */

			readblock(r_vir2abs(dirpos / BLOCK_SIZE), dirbuf);
		}
#ifdef FLEX
		if (super.s_flags & S_FLEX) {
			_fl_direct *dp= (_fl_direct *) (dirbuf + blkpos);

			if ((inum= dp->d_ino) != 0) strcpy(name, dp->d_name);

			dirpos+= (1 + dp->d_extent) * FL_DIR_ENTRY_SIZE;
			continue;
		}
#endif
		/* Let dp point to the next entry. */
		dp= (direct *) (dirbuf + blkpos);

		if ((inum= dp->d_ino) != 0) {
			/* This entry is occupied, return name. */
			strncpy(name, dp->d_name, sizeof(dp->d_name));
			name[sizeof(dp->d_name)]= 0;
		}
		dirpos+= DIR_ENTRY_SIZE;
	}
	return inum;
}

off_t r_vir2abs(virblk) off_t virblk;
/* Translate a block number in a file to an absolute disk block number.
 * Returns 0 for a hole or if block is past end of file.
 */
{
	block_t b= virblk;
	zone_t zone, ind_zone;
	block_t z, zone_index;

	/* Check if virblk within file. */
	if (virblk * BLOCK_SIZE >= curfil.i_size) return 0;

	/* Calculate zone in which the datablock number is contained */
	zone = (zone_t) (b >> zone_shift);

	/* Calculate index of the block number in the zone */
	zone_index = b - ((block_t) zone << zone_shift);

	/* Go get the zone */
	if (zone < (zone_t) NR_DZONES) {	/* direct block */
		zone = curfil.i_zone[zone];
		z = ((block_t) zone << zone_shift) + zone_index;
		return z;
	}

	/* The zone is not a direct one */
	zone -= (zone_t) NR_DZONES;

	/* Is it single indirect ? */
	if (zone < (zone_t) NR_INDIRECTS) {	/* single indirect block */
		ind_zone = curfil.i_zone[NR_DZONES];
	} else {			/* double indirect block */
		/* Fetch the double indirect block */
		if ((ind_zone = curfil.i_zone[NR_DZONES + 1]) == 0) return 0;

		z = (block_t) ind_zone << zone_shift;
		if (a_dindir != z) {
			readblock((off_t) z, dindir);
			a_dindir= z;
		}
		/* Extract the indirect zone number from it */
		zone -= (zone_t) NR_INDIRECTS;

		ind_zone =
#ifdef SUPER_V2
		    super.s_magic == SUPER_V2 ?
			fsbuf(dindir).b_v2_ind[zone / (zone_t) NR_INDIRECTS] :
#endif
			fsbuf(dindir).b_v1_ind[zone / (zone_t) NR_INDIRECTS];
		zone %= (zone_t) NR_INDIRECTS;
	}
	if (ind_zone == 0) return 0;

	/* Extract the datablock number from the indirect zone */
	z = (block_t) ind_zone << zone_shift;
	if (a_indir != z) {
		readblock((off_t) z, indir);
		a_indir= z;
	}
	zone =
#ifdef SUPER_V2
		super.s_magic == SUPER_V2 ?
			fsbuf(indir).b_v2_ind[zone] :
#endif
			fsbuf(indir).b_v1_ind[zone];

	/* Calculate absolute datablock number */
	z = ((block_t) zone << zone_shift) + zone_index;
	return z;
}

ino_t r_lookup(cwd, path) ino_t cwd; char *path;
/* Translates a pathname to an inode number.  This is just a nice utility
 * function, it only needs r_stat and r_readdir.
 */
{
	char name[NAME_MAX+1], r_name[NAME_MAX+1];
	char *n;
	struct stat st;
	ino_t ino;

	ino= path[0] == '/' ? ROOT_INO : cwd;

	for (;;) {
		if (ino == 0) {
			errno= ENOENT;
			return 0;
		}

		while (*path == '/') path++;

		if (*path == 0) return ino;

		r_stat(ino, &st);

		if (!S_ISDIR(st.st_mode)) {
			errno= ENOTDIR;
			return 0;
		}

		n= name;
		while (*path != 0 && *path != '/')
			if (n < name + NAME_MAX) *n++ = *path++;
		*n= 0;

		while ((ino= r_readdir(r_name)) != 0
					&& strcmp(name, r_name) != 0) {}
	}
}
/* Kees J. Bot 23-12-91. */
