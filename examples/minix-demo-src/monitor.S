/*
 * File        : monitor.S
 * Author      : Stuart Menefy
 * Description : Initial portion of boot monitor.
 * Header      : $Id: monitor.S,v 1.4 1994/02/21 11:22:45 stuart Exp $
 *
 * This is the startup code of the monitor. It performs the static
 * initialisation for the main part of the monitor, and if necessary
 * initialises the B016, before jumping into the C code.
 * 
 * History:
 *   04/07/93  SIM  Created.
 */

/* #include <minix/config.h> */
#define B016 0
 
/*
 * The stack on entry looks like:
 *
 * 11 codePtr   - gsb
 * 10 storedData1
 *  9 storedData0
 *  8 oldIptr
 *  7 oldWdesc
 *  6 memSize
 *  5 memStart
 *  4 linkInAddress
 *  3 linkOutAddress
 *  2 loopEnd
 *  1 loopValue
 *  0 lsb
 */

monitor:
global monitor
descriptor monitor "" occam_harness 0 0 ""

#if B016
        -- B016 initialisation
        ldc     (b016_setup - locallabel65)
        ldpi
locallabel65:
	ldl	5	-- memStart
	ldc	(b016_setup_end - b016_setup)
	move
	ldl	5	-- memStart
        gcall
#endif

        -- In theory there are two parameters to init.static, but we dont
        -- know swiChannelPtr yet, so we pass in a dummy value and patch it
        -- later.
        ldc     0       -- dummy value for swiChannelPtr
        ldl     11      -- gsb
        extern init.static
        patch 6 call codefix init.static 0
        byte #20, #20, #20, #20, #20, #20

        -- Initialise the static from the boot information
        ldl     11       -- gsb
        patch   3 ldnl modnumber
        byte    #20, #20, #20
        stl     0       -- lsb

        ldl     4       -- linkInAddress
        ldl     0       -- lsb
        stnl    0       -- bootLinkIn
        ldl     3       -- linkOutAddress
        ldl     0       -- lsb
        stnl    1       -- bootLinkOut
        ldl     6       -- memSize
        ldl     0       -- lsb
        stnl    6       -- memSize

        ldlp    7       -- oldWdesc     : C = src
        ldl     0       -- lsb
        ldnlp   2       -- savedDetails : B = dest
        ldc     4*4     -- 4 words      : A = len
        move
        
        -- Jump into the C monitor. This call will never return, so
        -- move the stack up as high as possible.
        ldlp    12      -- codeStart
        gajw
        ldnl    11      -- gsb
        extern c_monitor
        patch 6 call codefix c_monitor 0
        byte #20, #20, #20, #20, #20, #20


#if B016
b016_setup:
        global b016_setup
	-- Setup the mam RAM so we can access the VIC

	-- Set 'next cycle to map ram' flag
        ldc	1
	ldc	#7FDC0048
	stnl	0

	-- Write enable sequence to map ram
        ldc	#1800
	ldc	#7FD90000
	stnl	0

	gcall
b016_setup_end:
#endif

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

-- This is this modules static data.
-- This is the data which is made available from the booting process to the
-- main C code.
        global  bootLinkIn
        global  bootLinkOut
        global  savedDetails
        global  memSize
        data    bootLinkIn   1
        data    bootLinkOut  1
        data    savedDetails 4
        data    memSize      1

-- Make this module part of the static chain
        align
        init
        byte    #20, #20, #20, #20

-- The code to perform the initialisation
        ajw     -1

        -- Find  the address of the lsb
        ldl     2       -- gsb
        patch   6 ldnlp staticfix
        byte    #20, #20, #20, #20, #20, #20
        stl     0       -- lsb

        -- Store the address of the lsb in the table at the start
        ldl     0       -- lsb
        ldl     2       -- gsb
        patch   3 stnl modnumber
        byte    #20, #20, #20
        
        ajw     1
        ret
