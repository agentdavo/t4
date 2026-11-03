/*
 * node.c - what occ/raytrace.cfs configured: which processes run on which
 * transputer, and which channels join them.
 *
 *   PROCESSOR framebuf.p      (System[0], PRI HIGH)
 *     frameBuffer (frame.in, frame.out, graphics, image, processors)
 *   PROCESSOR cntlsys.p       (System[0], PRI HIGH)
 *     controlSystem (forward[0], back[0], graphics, image, tracers)
 *   PROCESSOR raytrace.p[i]   (System[i])
 *     rayTrace (forward[i], back[i], back[i+1], forward[i+1], true, i)
 *   last one:
 *     rayTrace (forward[i], back[i], soft, soft, false, i)
 *
 * Every node runs this one image.  The chain bootstrap (mkbtl.py) that
 * loaded it leaves the node's place in the chain in BOOTINFO: its index,
 * the number of nodes, the link the chain goes on by, and the input
 * channel it was booted through, which is the link back towards node 0.
 * So the network is whatever the map file wired up: the 16-node map runs
 * the chain on links 1 -> 0 and the others on 2 -> 1, and the image does
 * not change.
 *
 * On the last node `soft` joins throughPut's toNext to pixelBypass's
 * fromNext, so the c.stop that throughPut passes on comes straight back.
 */
#include "raytrace.h"

/* mkbtl.py's B_WS: the bootstrap's workspace, its first words the header
 * it was sent, then the boot link */
#define BOOTINFO ((volatile int *)0x801FF100)
#define bi_index   0
#define bi_nodes   1
#define bi_down    2
#define bi_bootin  5

static Channel forward0 = NOPROCESS;    /* cntlsys -> rayTrace[0] */
static Channel back0 = NOPROCESS;       /* rayTrace[0] -> cntlsys */
static Channel graphics = NOPROCESS;    /* cntlsys -> frameBuffer */
static Channel image = NOPROCESS;       /* frameBuffer -> cntlsys: model */
static Channel soft = NOPROCESS;

void startProcess(void (*func)(void), void *ws, int wsize, int pri)
{
    PRun(PSetup(ws, wsize, func) | pri);
}

void node_main(void)
{
    int me = BOOTINFO[bi_index];
    int processors = BOOTINFO[bi_nodes];
    int down = BOOTINFO[bi_down];
    Channel *bootin = (Channel *)BOOTINFO[bi_bootin];
    int last = me == processors - 1;
    Channel *fromNext = last ? &soft : LINKIN(down);
    Channel *toNext = last ? &soft : LINKOUT(down);

    if (me == 0) {
        frameBuffer(&graphics, &image, processors);
        controlSystem(&forward0, &back0, &graphics, &image, processors);
        rayTrace(&forward0, &back0, fromNext, toNext, !last, me);
    } else
        /* a link's output channel is 16 bytes below its input */
        rayTrace(bootin, bootin - 4, fromNext, toNext, !last, me);
    /* the processes run on; this one is done (start.S stops it) */
}
