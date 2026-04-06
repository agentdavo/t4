# Choose a C compiler. (Must be ANSI C capable).
CC = cc -flto
RM = rm -f
#
# 64bit emulator on Linux-x64, 32bit emulator on  Linux-armhf
# CC = gcc -flto -ffloat-store -frounding-math
#
# 32bit emulator on Linux-x86
# CC = gcc -m32 -mfpmath=sse -msse2 -ffloat-store -frounding-math
#
# 32bit/64bit emulator on Linux-POWER
# CC = gcc [-m64] -ffloat-store -frounding-math -std=c99
#

# For a well-optimised version compiled with gcc on a Sun use:
# CFLAGS = -O2 -DSUN -finline -finline-functions -fexpensive-optimizations
# For a reasonably optimised version for NetBSD.
# Define
#       -DT4CACHEBITS=N         Icache size is 2^N (default 14, 16K)
#       -DT4NANOMSG=1           use the nanomsg library for links
#       -DT4SHLINKS=1           use shared-memory links
#       -DT4COMBINATIONS=1      to replace common instruction sequences
#       -DT4RELEASE=1           produce fast emulator (no profiling,tracing)
#
# Release build: -O3, no debug overhead
CFLAGS = -I. -O3 -march=native -fomit-frame-pointer -Wall \
	-DCURTERM=1 -DT4SHLINKS=1 -DT4COMBINATIONS=1 -DT4RELEASE=1 -DNDEBUG \
	-DT4_SDL_FB $(shell pkg-config --cflags sdl2) $(shell pkg-config --cflags libpng)

# Debug build flags (use with: make t4-debug)
CFLAGS_DEBUG = -I. -O2 -fomit-frame-pointer -Wall -g \
	-DCURTERM=1 -DT4SHLINKS=1 -DT4COMBINATIONS=1 -DEMUDEBUG \
	-DT4_SDL_FB $(shell pkg-config --cflags sdl2) $(shell pkg-config --cflags libpng)

# Put any additional libraries here.
LFLAGS	= -lm $(shell pkg-config --libs sdl2) $(shell pkg-config --libs libpng)

#
# https://github.com/JuliaMath/openlibm
#
FDMSRC  = k_standard.c s_scalbn.c s_scalbnf.c s_ldexp.c s_ldexpf.c \
	  e_fmod.c e_remainder.c e_sqrt.c w_remainder.c w_sqrt.c
FDMOBJ	= k_standard.o s_scalbn.o s_scalbnf.o s_ldexp.o s_ldexpf.o \
	  e_fmod.o e_remainder.o e_sqrt.o w_remainder.o w_sqrt.o

SRC	= $(FDMSRC) curterm.c arithmetic.c fparithmetic.c netcfg.c shlink.c server.c display_backend_sdl.c p.c main.c
OBJ	= $(FDMOBJ) curterm.o arithmetic.o fparithmetic.o netcfg.o shlink.o server.o display_backend_sdl.o p.o main.o

t4 : $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(LFLAGS) -o t4
	strip t4
	ls -l t4

t4-debug : $(SRC)
	$(CC) $(CFLAGS_DEBUG) $(SRC) $(LFLAGS) -o t4
	ls -l t4

clean	:
	$(RM) t4 $(OBJ)
