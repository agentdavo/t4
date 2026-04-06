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
# Release build: -O3, no debug overhead. Use T4_OPT=-Ofast only for explicit
# experiments; the emulator has strict floating-point and old-C aliasing code.
T4_OPT ?= -O3
CFLAGS = -I. $(T4_OPT) -march=native -fomit-frame-pointer -Wall \
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

SRC	= $(FDMSRC) curterm.c arithmetic.c fparithmetic.c netcfg.c shlink.c server.c display_backend_sdl.c timer_integration.c mmio.c debug_config.c t9000_protect.c t9000_pmi.c p.c main.c
OBJ	= $(FDMOBJ) curterm.o arithmetic.o fparithmetic.o netcfg.o shlink.o server.o display_backend_sdl.o timer_integration.o t9000_protect.o t9000_pmi.o p.o main.o

# Headers that the single-shot build must also depend on, otherwise editing
# them silently leaves a stale t4 binary behind.
HDR	= emu.h processor.h opcodes.h arithmetic.h fparithmetic.h fpiface.h \
	  tvx.h tvx_vec.h tvx_iq.h tvx_gguf_tables.h tvx_gguf_decode.h \
	  t9000_group.h t9000_protect.h t9000_pmi.h display_backend.h \
	  mmio.h debug.h t4debug.h decode_cache.h

t4 : $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(SRC) $(LFLAGS) -o t4
	strip t4
	ls -l t4

t4-debug : $(SRC) $(HDR)
	$(CC) $(CFLAGS_DEBUG) $(SRC) $(LFLAGS) -o t4
	ls -l t4

# Optimized emulator with symbols, for sampling production-path stalls under
# gdb without changing instruction dispatch or enabling EMUDEBUG behavior.
t4-gdb : $(SRC)
	$(CC) $(CFLAGS) -g $(SRC) $(LFLAGS) -o t4
	ls -l t4

# Host unit tests for protection translate (no full T4 guest)
test-protect: t9000_protect.c t9000_protect.h
	$(CC) -I. -O2 -Wall -DT9K_PROTECT_SELFTEST -o tests/test_t9000_protect \
		tests/test_t9000_protect.c t9000_protect.c
	./tests/test_t9000_protect

clean	:
	$(RM) t4 $(OBJ)

.PHONY: smoke benchmark coremark
smoke: t4
	./run-btl-suite.sh

benchmark: t4
	SUITE=benchmark ./run-btl-suite.sh

coremark: t4
	SUITE=coremark ./run-btl-suite.sh
