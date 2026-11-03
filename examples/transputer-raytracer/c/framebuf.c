/*
 * framebuf.c - frameBuffer (occ/framebuf.occ): collects the patches into
 * the picture, prints the workers' messages, times the run and writes
 * ray.ppm through the host server.
 *
 * As in the occam, two processes: a relay that takes each command off the
 * control system's `graphics` channel as soon as it is offered, and the
 * main process, which does the host I/O.  Both run at high priority
 * (raytrace.cfs: MAP framebuf.p ... PRI HIGH), so TIMER ticks are 1us.
 *
 * The occam kept a [256][256] INT screenRAM of 10-bit-per-channel colours
 * and converted it to bytes in FileImage.  At RES = 2048 that would be
 * 16 MB, so the C converts each patch as it arrives and keeps the picture
 * as the file's own bytes, rows already flipped: 3*RES*RES bytes, 12 MB at
 * 2048, which lives above the image at FRAME_ADDR (node 0 then needs
 * t4 -sm 24).  A small picture stays in BSS, so RES=256 runs in t4's
 * default 2 MB like the occam did.
 */
#include "raytrace.h"

#define FRAME_BYTES (3 * RES * RES)
#if FRAME_BYTES <= 256 * 1024
static unsigned char screenRAM[FRAME_BYTES];
#else
#define FRAME_ADDR 0x80200000   /* node.c keeps everything else below it */
#define screenRAM ((unsigned char *)FRAME_ADDR)
#endif

static Channel *in, *image;
static int processors;
static Channel fromRenderer = NOPROCESS;

/* the relay's copy of the message it holds */
static int relayHdr[3];
static int relayPatch[patchSize * patchSize];
static unsigned char relayChars[256];

/* the main process's */
static int mainHdr[3];
static int mainPatch[patchSize * patchSize];
static unsigned char mainChars[257];

static int relayWs[512], mainWs[1024];

/* the occam's col conversion: the top 8 of each 10-bit channel */
static void plot(int x, int y, unsigned col)
{
    unsigned char *p = screenRAM + (RES - 1 - y) * (3 * RES) + 3 * x;

    p[0] = col >> 2;
    p[1] = col >> 12;
    p[2] = col >> 22;
}

static void FileImage(int *Status)
{
    int OutFileId, Result;

    Result = so_open("ray.ppm", spt_binary, spm_output, &OutFileId);
    if (Result == spr_ok) {
#define STR_(x) #x
#define STR(x) STR_(x)
        so_puts(OutFileId, "P6 " STR(RES) " " STR(RES) " 255");
        so_write(OutFileId, screenRAM, FRAME_BYTES);
        so_write_string_nl(" OK, all done!!");
        so_close(OutFileId);
        *Status = sps_success;
    } else {
        so_write_string(" so.open failed for ");
        so_write_string("ray.ppm");
        so_write_string(" with ");
        so_write_int(Result, 4);
        so_write_nl();
        *Status = sps_failure;
    }
}

static void MenuSelection(Channel *image)
{
    int InKey, Selected = 0;

    so_write_string_nl("                                                           ");
    so_write_string_nl("               RayTracer Image Selection                   ");
    so_write_string_nl("               =========================                   ");
    so_write_string_nl("                                                           ");
    so_write_string_nl("               <1>  10 Sphere Image                        ");
    so_write_string_nl("               <2>  14 Sphere Image                        ");
    so_write_string_nl("               <3>  Cone, Ellipsoid Image                  ");
    so_write_string_nl("               <4>  Cylinder, Sphere Image                 ");
    so_write_string_nl("                                                           ");
    so_write_string(" Your Selection ");

    while (!Selected)
        if (so_getkey(&InKey) == spr_ok && InKey >= '1' && InKey <= '4') {
            ChanOutInt(image, InKey - '0');
            so_write_int(InKey - '0', 1);
            so_write_nl();
            Selected = 1;
        }
}

/* WHILE TRUE: in ? command -> fromRenderer.  The occam relay loops for
 * ever; one picture is all frameBuffer ever takes, so this one ends with
 * the c.stop it passes on. */
static void relay(void)
{
    int command, size;

    for (;;) {
        command = ChanInInt(in);
        if (command == c_patch) {
            ChanInWords(in, relayHdr, 3);
            size = relayHdr[2];
            ChanInWords(in, relayPatch, size * size);
            ChanOutInt(&fromRenderer, command);
            ChanOutWords(&fromRenderer, relayHdr, 3);
            ChanOutWords(&fromRenderer, relayPatch, size * size);
        } else if (command == c_message) {
            size = ChanInInt(in);
            if (size)
                ChanIn(in, relayChars, size);
            ChanOutInt(&fromRenderer, command);
            ChanOutInt(&fromRenderer, size);
            if (size)
                ChanOut(&fromRenderer, relayChars, size);
        } else if (command == c_stop) {
            ChanOutInt(&fromRenderer, command);
            return;
        } else {
            /* an occam IF with no branch that fits is STOP */
            __stopp();
        }
    }
}

/* The occam took `end MINUS start` of the 1us clock, which wraps every 71
 * minutes, and a 2048 x 2048 picture on one node takes longer than that.
 * So the time is summed a message at a time, and only a gap of half an
 * hour between two messages could go wrong. */
static int secs, usecs;
static unsigned lastTick;

static void clock_(void)
{
    unsigned now = __ldtimer();

    usecs += (int)(now - lastTick);
    lastTick = now;
    while (usecs >= 1000000) {
        usecs -= 1000000;
        secs++;
    }
}

static void fbMain(void)
{
    int command, Status, i, j, x, y, size;
    float elapsed;

    /* clear screenRAM */
    for (i = 0; i < FRAME_BYTES / 4; i++)
        ((int *)screenRAM)[i] = 0;

    MenuSelection(image);
    so_write_string_nl(" The inmos ray tracer is GO");
    lastTick = __ldtimer();
    command = ChanInInt(&fromRenderer);
    while (command != c_stop) {
        clock_();
        if (command == c_patch) {
            ChanInWords(&fromRenderer, mainHdr, 3);
            x = mainHdr[0] - 128;
            y = mainHdr[1] - 128;
            size = mainHdr[2];
            ChanInWords(&fromRenderer, mainPatch, size * size);
            for (j = 0; j < size; j++)
                for (i = 0; i < size; i++)
                    plot(x + i, y + j, mainPatch[j * size + i]);
        } else if (command == c_message) {
            size = ChanInInt(&fromRenderer);
            if (size)
                ChanIn(&fromRenderer, mainChars, size);
            mainChars[size] = 0;
            so_write(spid_stdout, mainChars, size);
        } else {
            so_write_string_nl(" GARBAGE COMMAND");
            __stopp();
        }
        command = ChanInInt(&fromRenderer);
    }
    clock_();
    elapsed = (float)secs + (float)usecs * 0.000001f;
    so_write_nl();
    so_write_string(" DONE, took ");
    so_write_real32(elapsed, 5, 2);
    so_write_string(" seconds on ");
    so_write_int(processors, 2);
    so_write_string_nl(" processors.");

    FileImage(&Status);
    so_exit(Status);
    for (;;)
        __stopp();
}

void frameBuffer(Channel *in_, Channel *image_, int processors_)
{
    in = in_;
    image = image_;
    processors = processors_;
    startProcess(relay, relayWs, sizeof relayWs, PRI_HIGH);
    startProcess(fbMain, mainWs, sizeof mainWs, PRI_HIGH);
}
