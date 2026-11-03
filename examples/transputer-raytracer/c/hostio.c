/*
 * hostio.c - the part of the INMOS hostio library framebuf.occ uses, over
 * link 0 to the host file server (t4's iserver, server.c).
 *
 * An SP packet is a 16-bit little-endian length and that many bytes; the
 * length is even and between 6 and 510.  Every request gets one reply
 * packet.  Only frameBuffer's main process calls these, so the one buffer
 * needs no lock.
 */
#include "raytrace.h"

#define SP_OPEN    10
#define SP_CLOSE   11
#define SP_WRITE   13
#define SP_PUTS    15
#define SP_GETKEY  30
#define SP_EXIT    35

#define SP_MAXDATA 500          /* bytes of data in one write packet */

static unsigned char sp[2 + 512];

static void put32(unsigned char *p, int v)
{
    p[0] = v;
    p[1] = v >> 8;
    p[2] = v >> 16;
    p[3] = v >> 24;
}

static int get32(const unsigned char *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

/* sp[2..2+len) holds the request; on return sp[2..] holds the reply and
 * sp[2] is its result byte. */
static int sp_call(int len)
{
    if (len < 6)
        len = 6;
    len = (len + 1) & ~1;
    sp[0] = len;
    sp[1] = len >> 8;
    ChanOut(LINK0OUT, sp, len + 2);
    ChanIn(LINK0IN, sp, 2);
    len = sp[0] | (sp[1] << 8);
    ChanIn(LINK0IN, sp + 2, len);
    return sp[2];
}

int so_open(const char *name, int type, int mode, int *id)
{
    int n = 0;

    while (name[n])
        n++;
    sp[2] = SP_OPEN;
    sp[3] = n;
    sp[4] = n >> 8;
    memcpy(sp + 5, name, n);
    sp[5 + n] = type;
    sp[6 + n] = mode;
    if (sp_call(5 + n) != spr_ok)
        return sp[2];
    *id = get32(sp + 3);
    return spr_ok;
}

int so_close(int id)
{
    sp[2] = SP_CLOSE;
    put32(sp + 3, id);
    return sp_call(5);
}

/* so.write: as many packets as it takes */
int so_write(int id, const void *buf, int n)
{
    const unsigned char *p = buf;
    int len, done = 0;

    while (n > 0) {
        len = n > SP_MAXDATA ? SP_MAXDATA : n;
        sp[2] = SP_WRITE;
        put32(sp + 3, id);
        sp[7] = len;
        sp[8] = len >> 8;
        memcpy(sp + 9, p, len);
        if (sp_call(7 + len) != spr_ok)
            break;
        done += sp[3] | (sp[4] << 8);
        p += len;
        n -= len;
    }
    return done;
}

/* so.puts: the string and a newline */
int so_puts(int id, const char *s)
{
    int n = 0;

    while (s[n])
        n++;
    sp[2] = SP_PUTS;
    put32(sp + 3, id);
    sp[7] = n;
    sp[8] = n >> 8;
    memcpy(sp + 9, s, n);
    return sp_call(7 + n);
}

int so_getkey(int *key)
{
    sp[2] = SP_GETKEY;
    if (sp_call(1) != spr_ok)
        return sp[2];
    *key = sp[3];
    return spr_ok;
}

void so_exit(int status)
{
    sp[2] = SP_EXIT;
    put32(sp + 3, status);
    sp_call(5);
}

void so_write_string(const char *s)
{
    int n = 0;

    while (s[n])
        n++;
    so_write(spid_stdout, s, n);
}

void so_write_nl(void)
{
    so_puts(spid_stdout, "");
}

void so_write_string_nl(const char *s)
{
    so_write_string(s);
    so_write_nl();
}

/* so.write.int: decimal, right justified in `width` */
void so_write_int(int n, int width)
{
    char buf[16];
    unsigned u = n < 0 ? -(unsigned)n : (unsigned)n;
    int i = sizeof buf;

    buf[--i] = 0;
    do {
        buf[--i] = '0' + u % 10;
        u /= 10;
    } while (u);
    if (n < 0)
        buf[--i] = '-';
    while ((int)sizeof buf - 1 - i < width)
        buf[--i] = ' ';
    so_write_string(buf + i);
}

/* so.write.real32 for the one use it has, a positive time: `ip` places
 * before the point (sign included, as REAL32TOSTRING counts them), `dp`
 * after. */
void so_write_real32(float x, int ip, int dp)
{
    char buf[32];
    int i, scale = 1, whole, frac;

    for (i = 0; i < dp; i++)
        scale *= 10;
    frac = (int)(x * (float)scale + 0.5f);
    whole = frac / scale;
    frac %= scale;
    so_write_int(whole, ip + 1);
    buf[0] = '.';
    for (i = dp; i > 0; i--) {
        buf[i] = '0' + frac % 10;
        frac /= 10;
    }
    buf[dp + 1] = 0;
    so_write_string(buf);
}
