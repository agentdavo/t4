# File        : Makefile
# Author      : Stuart Menefy
# Description : Makefile for boot monitor and build program
# Header      : $Id: Makefile,v 1.10 1994/10/11 11:19:22 stuart Exp $
#
# The will build either the link or disk bootable system, depending on
# the target specified on the command line.
#
# History:
#   19/02/92  SIM  Created
#   04/07/93  SIM  Overhaul for boot monitor version.
#   07/01/94  SIM  Altered to use new features of CC
#   11/04/94  SIM  Shortened filenames for MS-DOS.
#   09/05/94  SIM  Update hosted version to use environment vars properly

include ../Configfile

CFLAGS=		-Wall $(_CPPFLAGS) $(_CFLAGS) -I$(SRCDIR)
LIBC=		$(LIBDIR)/libc.a
LIBSYS=		$(LIBDIR)/libsys.a

all:
		@echo "Need to specify make target:"
		@echo "  disk - boot from disk version (FORMAT=-f if required)"
		@echo "  link - boot from link version"

disk:		buildd bootd secondd monitord
		./buildd $(FORMAT) -vo /dev/rfd0 \
			-ibps bootd -is secondd -i monitord

link:		buildl bootl secondl monitorl
		./buildl -vo boot.btl -ibps bootl -is secondl

###

buildd:	build.c
		$(CC) $(CFLAGS) build.c -o buildd -DMINIX

buildl:	build.c
		$(HOSTCC) $(HOSTCFLAGS) build.c -o buildl -DHOST -I$(HOSTINC)

###

bootd:	bootstrap.S
		$(CC) -DDISK -nostdlib -o bootd -e bootstrap bootstrap.S

bootl:	bootstrap.S
		$(CC) -DLINK -nostdlib -o bootl -e bootstrap bootstrap.S

###

secondd:	second.o csecond.o secondd.o diskio.o termio.o
		$(LD) -o secondd \
		    second.o csecond.o secondd.o diskio.o termio.o $(LIBC)


secondl:	second.o csecond.o secondl.o linkio.o
		$(LD) -o secondl \
		    second.o csecond.o secondl.o linkio.o $(LIBC)

second.o:	second.S
csecond.o:	csecond.c io.h
secondl.o:	secondl.c io.h
secondd.o:	secondd.c io.h

###

monitord:	monitor.o cmonitor.o monitord.o rawfs.o fileio.o termio.o
		$(LD) monitor.o cmonitor.o monitord.o rawfs.o fileio.o termio.o \
			-e monitor $(LIBC) $(LIBSYS) -o monitord

monitorl:	monitor.o cmonitor.o monitorl.o linkio.o
		$(LD) monitor.o cmonitor.o monitorl.o linkio.o \
			-e monitor $(LIBC) $(LIBSYS) -o monitorl

###

monitor.o:	monitor.S
cmonitor.o:	cmonitor.c io.h monitor.h
monitord.o:	monitord.c rawfs.h io.h monitor.h
monitorl.o:	monitorl.c io.h monitor.h
rawfs.o:	rawfs.c rawfs.h
diskio.o:	diskio.c io.h
linkio.o:	linkio.c io.h
fileio.o:	fileio.c rawfs.h io.h
termio.o:	termio.c tpduart.h io.h

#########

clean:
		rm -f *.o

clobber:	clean
		rm -f buildd   buildl
		rm -f bootd    bootl
		rm -f secondd  secondl
		rm -f monitord monitorl
		rm -f boot.btl
