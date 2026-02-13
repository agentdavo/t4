--{{{  Comment
/*
 * File        : second.S
 * Author      : Stuart Menefy
 * Description : Transputer Minix secondary bootstrap.
 * Header      : $Id: second.S,v 1.3 1994/02/10 13:50:02 stuart Exp $
 *
 * This is the second block of code which is executed on the 32 bit transputer.
 * As such it will be loaded directly above the stack of the primary bootstrap
 * and so has very limited stack.
 *
 * It's purpose is initialise enough of the I/O system to be able to print
 * some messages while testing the size of memory, and load in the remainder
 * of the boot monitor.
 *
 * History:
 *   30/01/94  SIM  Created.
 */
--}}}

--{{{  Descriptor
bootstrap:
global bootstrap
descriptor bootstrap "" occam_harness 0 0 ""

--}}}
--{{{  Workspace layout
/* #defines for workspace usage */
#define codeStart     11	/* Where code is actually loaded */
#define codePtr       10        /* First free word, used as GSB */
#define storedData1    9
#define storedData0    8
#define oldIptr        7
#define oldWdesc       6
#define memStart       5
#define linkInAddress  4
#define linkOutAddress 3
#define loopEnd        2        /* These two MUST remain together       */
#define loopValue      1        /* to allow lend instruction to be used */
/* Don't use workspace 0 to allow for outbyte instructons */
--}}}

--{{{  Initialise the static segment
        -- Call the static initialisation code
        ldl     10      -- gsb
        extern  init.static
        patch   3 call codefix init.static 0
        byte    #20, #20, #20

        -- Copy the GSB into slot 1, as normal
        ldl     10
        stl     1

        -- Jump into the main program, having set up the GSB.
        extern main
        patch   3 j codefix main  0
        byte    #20, #20, #20
--}}}

--{{{  Check for B016
-- We know that the current board has at least 4M, so check if it is a B016.
-- To do this, see if a write to the MAP RAM at the 1M position causes the
-- memory to be corrupted.


check_B016:
        global check_B016

        -- Precompute address
        ldc     #80100000
        stl     1       -- addr

    	-- Set 'next cycle to map ram' flag
        ldc	1
	ldc	#7FDC0048
	stnl	0

	-- Write enable sequence to map ram
        ldc	0
	ldl     1       -- addr
	stnl	0

        -- Read back the memory location
        ldl     1       -- addr
        ldnl    0
        cj      zero - here
here:
        -- Location did not contain 0, so assume that this is a B016.
        ldc     1
        ret

zero:
        -- Location contains 0, so this is not a B016.
        ldl     1       -- addr
        ldl     1       -- addr
        stnl    0

        ldc     0
        ret
--}}}
--{{{  _ldtimer
_ldtimer:
        global _ldtimer

        ldtimer
        ret

--}}}
--{{{  start
-- Jump into the kernel at the given locations:
-- 3 stack
-- 2 code
-- 1 GSB        (not used)
-- 0 Ret Iptr   (not used)
$start:
        global  $start
        ldl     3       -- stack
        gajw
        ldnl    2       -- code
        gcall
--}}}
--{{{  top_code
-- Return the first address which is not in use
-- 1 GSB        (not used)
-- 0 Ret Iptr   (not used)
top_code:
        global  top_code
        ldl     1
        patch   2 ldnlp limit
        byte    #20, #20
        ret
--}}}

