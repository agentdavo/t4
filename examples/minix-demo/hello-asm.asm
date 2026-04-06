; Simple IServer hello world - pure assembly
; Uses IServer protocol to output "Hello!\n"

; Constants
LINK0_OUT    EQU 0x80000000
SP_PUTS      EQU 0x0F

start:
    ; Send SP_PUTS command
    ldc SP_PUTS
    ldc LINK0_OUT
    outbyte

    ; Send stream ID (1 = stdout)
    ldc 1
    ldc LINK0_OUT
    outbyte

    ; Send length (7 bytes for "Hello!\n") - little endian
    ldc 7
    ldc LINK0_OUT
    outbyte

    ldc 0        ; high byte of length
    ldc LINK0_OUT
    outbyte

    ; Send "Hello!\n"
    ldc 'H'
    ldc LINK0_OUT
    outbyte

    ldc 'e'
    ldc LINK0_OUT
    outbyte

    ldc 'l'
    ldc LINK0_OUT
    outbyte

    ldc 'l'
    ldc LINK0_OUT
    outbyte

    ldc 'o'
    ldc LINK0_OUT
    outbyte

    ldc '!'
    ldc LINK0_OUT
    outbyte

    ldc 10       ; '\n'
    ldc LINK0_OUT
    outbyte

    ; Halt
    stopp
