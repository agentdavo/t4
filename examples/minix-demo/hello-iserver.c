/*
 * Minimal IServer hello world test
 * Uses IServer protocol to send "Hello from Transputer!\n" to Link 0
 */

#define LINK0_OUT 0x80000000  /* Link 0 output channel */

/* IServer protocol commands */
#define SP_OPEN     0x0A
#define SP_CLOSE    0x0B
#define SP_READ     0x0C
#define SP_WRITE    0x0D
#define SP_GETS     0x0E
#define SP_PUTS     0x0F
#define SP_GETKEY   0x10
#define SP_POLLKEY  0x11
#define SP_GETENV   0x12
#define SP_TIME     0x13
#define SP_SYSTEM   0x14
#define SP_EXIT     0x15

void _start(void) {
    unsigned char *msg = (unsigned char *)"Hello from Transputer!\n";
    int len = 24;
    int i;
    volatile unsigned int *link0_out = (volatile unsigned int *)LINK0_OUT;

    /* Send SP_PUTS command */
    *link0_out = SP_PUTS;

    /* Send stream ID (1 = stdout) */
    *link0_out = 1;

    /* Send length (little-endian short) */
    *link0_out = len & 0xFF;
    *link0_out = (len >> 8) & 0xFF;

    /* Send message bytes */
    for (i = 0; i < len; i++) {
        *link0_out = msg[i];
    }

    /* Halt - use while loop instead of inline asm */
    while(1);
}
