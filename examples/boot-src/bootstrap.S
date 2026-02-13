--{{{  Comment
/*
 * File        : bootstrap.S
 * Author      : Stuart Menefy
 * Description : Transputer Minix bootstrap.
 * Header      : $Id: bootstrap.S,v 1.4 1994/08/05 10:18:46 stuart Exp $
 *
 * This is the first code which is executed on the 32 bit transputer.
 * As such it will be loaded as memstart, with the workspace immediatly
 * above it.
 *
 * It's purpose is set up the processor and load in the secondary
 * bootstrap program. This is loaded directly above the bootstrap stack,
 * and executed with the same stack.
 *
 * History:
 *             SIM  Created.
 *   30/01/94  SIM  Changes assuming secondary bootstrap.
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
#define codePtr       10
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

--{{{  Init processor
codeBase:
	-- Leave room for negative workspace (min 3) plus some
	-- additional workspace for the secondary bootstrap.
	ajw	40

        -- Store info in registers
        stl     oldIptr
        stl     oldWdesc
        stl     linkInAddress

        -- Store the debugging info saved in low memory.
        mint
        ldnl    #1A
        stl     storedData0
        mint
        ldnl    #1B
        stl     storedData1
      
        -- This will read memStart.
        ldc	codeBase - l1
        ldpi
l1:     stl     memStart

        -- Derive linkOutAddress
        ldl     linkInAddress
        ldnlp   -4
        stl     linkOutAddress
      
        -- Init process queue pointers
        mint
        sthf
        mint
        stlf

        -- Init links and timer queues
        ldc     0
        stl     loopValue
        ldc     11
        stl     loopEnd

istart: mint
        ldl     loopValue
        mint
        wsub
        stnl    0
        ldlp    loopValue
        ldc     iend - istart
        lend
iend:

        -- Start timer
        ldc     0
        sttimer

        -- Clear error flag and halt on error
        testerr
        clrhalterr
--}}}
--{{{  Load monitor
#ifdef LINK 
--{{{  Read monitor from link
	-- Read in the code.
	-- The number of bytes is stored at numberBlocks.
	ldlp	codeStart
        ldl     linkInAddress 	
	ldc	numberBlocks - ll3
	ldpi
ll3:	ldnl	0
        in

	-- Compute the first free word after the end of the code.
	ldc	numberBlocks - ll4
	ldpi
ll4:	ldnl	0
	ldlp	codeStart
        bsub
	adc	3
	ldc	#FFFFFFFC
	and
        stl     codePtr
--}}}
#elif defined(DISK)
--{{{  Read monitor from disk
#define M2cmd_WriteParam 3
#define M2cmd_ReadBuffer 4
#define M2cmd_ReadSector 6
#define M2cmd_SelectDrve 11
#define M2param_DesiredDrive 30
#define sectorSize       256

	-- Read in the code.
	-- Keep things simple, read in the rest of the code from
	-- successive blocks. 
	-- The number of blocks is stored at numberBlocks.
	
        ldc     0
        stl     loopValue
	ldc	numberBlocks - ll3
	ldpi
ll3:	ldnl	0
        stl     loopEnd
	ldlp	codeStart
	stl	codePtr

        -- Read in one sector
bootl:  ldl	linkOutAddress
        ldc     M2cmd_ReadSector
        outbyte
        
        ldl     linkOutAddress
        ldc     M2cmd_ReadBuffer
        outbyte
        
        ldl     codePtr
        ldl     linkInAddress 
        ldc     sectorSize
        in                 
        
        -- Add on length of code just read
        ldl     codePtr
        adc     sectorSize
        stl     codePtr
        
        -- Continue loop      
        ldlp    loopValue
        ldc     bootle - bootl
        lend          
bootle:
      
        -- De-select the drive
        ldc     desel - ll4
        ldpi
ll4:    ldl     linkOutAddress
        ldc     4
        out
--}}}
#else
#error Boot method not defined
#endif
--}}}
--{{{  Execute it
        -- Execute it
        ldlp    codeStart
        gcall
--}}}
--{{{  Data
#ifdef DISK
desel:  byte    M2cmd_WriteParam, M2param_DesiredDrive, 0
        byte    M2cmd_SelectDrve 
#endif

-- The size of the monitor is patched in by the build program.
-- This is the number of BYTES when booting from link, the number of
-- BLOCKS when booting from disk.

        align
numberBlocks:
        -- To be patched in later
        word #12345678


--}}}
