/*
 * start.S - entry to the ray tracer image, at MemStart.
 *
 * The chain bootstrap (mkbtl.py) has loaded the image, zeroed its BSS
 * and set Wptr to the top of the main process's workspace.  Clear the
 * error flags, empty the scheduling queues, start the clocks, then run
 * node_main, which starts the node's processes and returns; this process
 * then stops and leaves the transputer to them.
 */
.section .text.start,"ax"
.global _start

_start:
    testerr
    clrhalterr
    fptesterr
    mint
    sthf
    mint
    stlf
    ldc 0
    sttimer
    call node_main
    stopp
