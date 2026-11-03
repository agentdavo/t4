/*
 * raytrace.c - rayTrace (occ/raytrace.occ), which every node runs:
 *
 *   PRI PAR
 *     PAR
 *       throughPut  -- jobs and scene data from the previous node: keep a
 *                      job if the renderer is idle or has none waiting,
 *                      else pass it on to the next node
 *       pixelBypass -- this node's and the later nodes' results, back
 *                      towards node 0
 *     renderer      -- the ray tracer proper
 *
 * The routers run at high priority, the renderer at low.  The renderer's
 * arithmetic follows the occam's REAL32 expressions operation for
 * operation, brackets and all, so a picture comes out bit for bit the
 * same.  Comments in the occam are kept where they explain something.
 */
#include "raytrace.h"

static Channel *fromPrev, *toPrev, *fromNext, *toNext;
static int propogate, me;

static Channel pixelFeed = NOPROCESS;    /* throughPut -> renderer */
static Channel pixelTrigger = NOPROCESS; /* renderer -> throughPut: done */
static Channel pixelOut = NOPROCESS;     /* renderer -> pixelBypass */

static int throughWs[512], bypassWs[512], rendererWs[4096];

/* ------------------------------------------------------------------ */
/* throughPut                                                          */
/* ------------------------------------------------------------------ */

#define closeDown  0
#define active     1
#define busy       2
#define buffered   4

static int work[256];

static void feedJob(int x, int y)
{
    int xy[2];

    xy[0] = x;
    xy[1] = y;
    ChanOutInt(&pixelFeed, c_render);
    ChanOutWords(&pixelFeed, xy, 2);
}

static void throughPut(void)
{
    Channel *chans[2];
    int state, tag, x = 0, y = 0, size, ab[2];

    chans[0] = &pixelTrigger;
    chans[1] = fromPrev;
    state = active;
    while (state != closeDown) {
        if (ChanAlt(chans, 2) == 0) {
            ChanInInt(&pixelTrigger);   /* should be guarded by busy .. */
            if (state & buffered) {
                feedJob(x, y);
                state = active | busy;
            } else
                state = active;         /* i.e not busy and not buffered */
            continue;
        }
        tag = ChanInInt(fromPrev);
        if (tag == c_render) {
            ChanInWords(fromPrev, ab, 2);
            if ((state & busy) == 0) {
                feedJob(ab[0], ab[1]);
                state = active | busy;
            } else if (state & buffered) {
                ChanOutInt(toNext, tag);
                ChanOutWords(toNext, ab, 2);
            } else {
                x = ab[0];
                y = ab[1];
                state = active | (busy | buffered);
            }
        } else if (tag == c_stop) {
            if (state & busy)
                ChanInInt(&pixelTrigger);
            if (state & buffered) {
                feedJob(x, y);
                ChanInInt(&pixelTrigger);
            }
            ChanOutInt(&pixelFeed, tag);
            ChanOutInt(toNext, tag);
            state = closeDown;
        } else {
            size = ChanInInt(fromPrev);
            ChanInWords(fromPrev, work, size);
            if (propogate) {
                ChanOutInt(toNext, tag);
                ChanOutInt(toNext, size);
                ChanOutWords(toNext, work, size);
            }
            ChanOutInt(&pixelFeed, tag);
            ChanOutInt(&pixelFeed, size);
            ChanOutWords(&pixelFeed, work, size);
        }
    }
}

/* ------------------------------------------------------------------ */
/* pixelBypass                                                         */
/* ------------------------------------------------------------------ */

#define lubThis   1
#define lubOther  2

static int pixels[256];

/* one command from `in`, the tag already read; returns 1 on c.stop */
static int bypass(Channel *in, int command)
{
    int size;

    if (command == c_message) {
        size = ChanInInt(in);
        if (size)
            ChanIn(in, pixels, size);
        ChanOutInt(toPrev, command);
        ChanOutInt(toPrev, size);
        if (size)
            ChanOut(toPrev, pixels, size);
    } else if (command == c_patch) {
        int hdr[3];

        ChanInWords(in, hdr, 3);
        ChanOutInt(toPrev, command);
        ChanOutWords(toPrev, hdr, 3);
        size = hdr[2];
        ChanInWords(in, pixels, size * size);
        ChanOutWords(toPrev, pixels, size * size);
    } else if (command == c_stop)
        return 1;
    else
        __stopp();
    return 0;
}

static void pixelBypass(void)
{
    Channel *chans[2];
    int state;

    chans[0] = &pixelOut;               /* fromThis */
    chans[1] = fromNext;
    state = lubThis | lubOther;
    while (state != closeDown) {
        if (ChanAlt(chans, 2) == 0) {
            if (bypass(&pixelOut, ChanInInt(&pixelOut)))
                state ^= lubThis;
        } else {
            if (bypass(fromNext, ChanInInt(fromNext)))
                state ^= lubOther;
        }
    }
    ChanOutInt(toPrev, c_stop);
}

/* ------------------------------------------------------------------ */
/* renderer: the big vectors, outside the procedures as in the occam   */
/* ------------------------------------------------------------------ */

static W grunData[runDataSize];

#define ambient(i)    grunData[0 + (i)].f
#define renderingMode grunData[3].i
#define runPatchSize  grunData[4].i
#define screenOrg(i)  grunData[6 + (i)].f
#define screendX(i)   grunData[9 + (i)].f
#define screendY(i)   grunData[12 + (i)].f
#define pinhole(i)    grunData[15 + (i)].f

/* The occam's [maxNodes][nodeSize] cleanTree holds 132 nodes.  A tree of
 * maxDepth levels can have 2^(maxDepth+1) - 1 = 127 rays, and shading
 * claims two more nodes (shadow, phong) for every ray, so a worst-case
 * pixel needs three times that.  The occam would stop with a range error
 * there; the reference pictures never get near it, but a fine sampling
 * might, so the C has room for the worst case.  Nothing else changes. */
#define treeNodes  (3 * (2 << maxDepth) + 4)
static W tree[treeNodes * nodeSize];

static int freeNode;
static W worldModel[sizeWorld];
static int world_p;
static W lights[sizeLights];
static int light_p;
static int samples[gridSize][gridSize];
static int colours[patchSize][patchSize];
static int nrays;

static unsigned char message[256];
static int mess_p;

/* occam's SQRT is inline: fpusqrtfirst, fpusqrtstep, ... */
#define sqrtf_(x) __fpsqrt(x)

static void normalize(float *vector, float *oldHyp)
{
    float t;

    t = (vector[0] * vector[0]) +
        ((vector[1] * vector[1]) +
         (vector[2] * vector[2]));
    *oldHyp = sqrtf_(t);
    vector[0] = vector[0] / *oldHyp;
    vector[1] = vector[1] / *oldHyp;
    vector[2] = vector[2] / *oldHyp;
}

static float dotProduct(const float *a, const float *b)
{
    return (a[0] * b[0]) + ((a[1] * b[1]) + (a[2] * b[2]));
}

static void wrch(int ch)
{
    if (ch == -7) {
        ChanOutInt(&pixelOut, c_message);
        ChanOutInt(&pixelOut, mess_p);
        if (mess_p)
            ChanOut(&pixelOut, message, mess_p);
        mess_p = 0;
    } else if (mess_p == 256)
        ;
    else
        message[mess_p++] = ch;
}

static void writen(int num)
{
    int stack[20], sp = 0, n;

    if (num < 0) {
        n = -num;
        wrch('-');
    } else
        n = num;
    stack[sp++] = 0xDEAD;               /* nil */
    while (n > 9) {
        stack[sp++] = n % 10;
        n = n / 10;
    }
    while (n != 0xDEAD) {
        wrch(n + '0');
        n = stack[--sp];
    }
}

/* writef: %I prints an integer argument.  The occam also has %F through a
 * 400-line REAL32 formatter (writeReal); nothing prints a REAL32, so that
 * part is not ported. */
static void writef(const char *str, int a, int b, int c, int d)
{
    int args[4], ptr = 0, ch;

    args[0] = a;
    args[1] = b;
    args[2] = c;
    args[3] = d;
    for (; *str; str++) {
        ch = *str;
        if (ch == '%') {
            ch = *++str;
            if (ch == 'I' || ch == 'i') {
                writen(args[ptr]);
                if (ptr != 3)
                    ptr++;
            }
            if (!ch)
                break;
        } else
            wrch(ch);
    }
    wrch(-7);
}

/* one word, then block copies of what is there, doubling each time - the
 * occam's [vec FROM dest FOR xfer] := [vec FROM 0 FOR xfer], a `move` */
static void initWORDvec(int *vec, int pattern, int words)
{
    int dest, xfer;

    vec[0] = pattern;
    xfer = 1;
    dest = xfer;
    while (dest < words) {
        if ((dest + xfer) >= words)
            xfer = words - dest;
        memcpy(&vec[dest], &vec[0], xfer * 4);
        dest = dest + xfer;
        xfer = xfer + xfer;
    }
}

/*
 *   The classic ray tracing paper is
 *      'An improved illumination model for shaded display'
 *      Turner Whitted CACM June 1980
 *
 *   V' = V / |V.N|,  R = V' + 2N,  P = kf(N + V') - N
 *   where kf = (kn^2 |V'|^2 - |V' + N|^2) ^ -1/2, kn = index of refraction
 *
 *   NOTE that if V.N < 0 we are INSIDE the object - be careful about
 *   the refractive index
 */

static void reflectRay(int reflected, int incident,
                       float *Vprime, float *Vvec, int *flip)
{
    /*
     * this code assumes a ray normalized on entry.
     *
     * normalization is not maintained after a reflection, but from the
     * geometry of the system the ratio of V' to V is the ratio of R' to R:
     * V is 1, R is 1, so R = R'/V' - 3 divides, not 3 muls, 2 adds and a
     * root.
     */
    W *node = &tree[incident];
    W *spec = &tree[reflected];
    float V_N, twoN;

    V_N = dotProduct(&node[n_dx].f, &node[n_normx].f);
    if (V_N > 0.0f) {
        node[n_normx].i ^= mint;
        node[n_normy].i ^= mint;
        node[n_normz].i ^= mint;
        *Vprime = 1.0f / V_N;
        *flip = 1;
    } else {
        *Vprime = (-1.0f) / V_N;
        *flip = 0;
    }
    /*
     *  V = 1, V' = -1/(N.V), so dxV' = dxV*Vprime
     *  so Rx = Nx + Nx + dxV*Vprime ...
     */
    twoN = node[n_normx].f * 2.0f;
    Vvec[0] = node[n_dx].f * *Vprime;
    spec[n_dx].f = Vvec[0] + twoN;

    twoN = node[n_normy].f * 2.0f;
    Vvec[1] = node[n_dy].f * *Vprime;
    spec[n_dy].f = Vvec[1] + twoN;

    twoN = node[n_normz].f * 2.0f;
    Vvec[2] = node[n_dz].f * *Vprime;
    spec[n_dz].f = Vvec[2] + twoN;

    spec[n_dx].f = spec[n_dx].f / *Vprime;
    spec[n_dy].f = spec[n_dy].f / *Vprime;
    spec[n_dz].f = spec[n_dz].f / *Vprime;

    spec[n_startx].f = node[n_sectx].f + node[n_normx].f;
    spec[n_starty].f = node[n_secty].f + node[n_normy].f;
    spec[n_startz].f = node[n_sectz].f + node[n_normz].f;
}

static void refractRay(int refracted, int incident, float Vprime,
                       const float *Vvec, int flip, int *totalInternal)
{
    W *node = &tree[incident];
    W *frac = &tree[refracted];
    int objptr = node[n_objptr].i;                      /* size word */
    float kn = worldModel[objptr + (o_refix + 1)].f;    /* skip it */
    float *norm = &node[n_normx].f;
    float kn_Vprime, Vprime_plus_N, t[3], kf2, kf;

    if (flip)
        kn_Vprime = (Vprime * Vprime) / kn;
    else
        kn_Vprime = (Vprime * Vprime) * kn;

    t[0] = norm[0] + Vvec[0];
    t[1] = norm[1] + Vvec[1];
    t[2] = norm[2] + Vvec[2];

    Vprime_plus_N = (t[0] * t[0]) +
                    ((t[1] * t[1]) +
                     (t[2] * t[2]));

    if (Vprime_plus_N > kn_Vprime)
        *totalInternal = 1;
    else {
        *totalInternal = 0;

        kf2 = kn_Vprime - Vprime_plus_N;
        kf = sqrtf_(kf2);

        t[0] = t[0] / kf;               /* get kf (N + V') */
        t[1] = t[1] / kf;
        t[2] = t[2] / kf;

        frac[n_dx].f = t[0] - norm[0];
        frac[n_dy].f = t[1] - norm[1];
        frac[n_dz].f = t[2] - norm[2];

        normalize(&frac[n_dx].f, &kf);

        frac[n_startx].f = node[n_sectx].f - node[n_normx].f;
        frac[n_starty].f = node[n_secty].f - node[n_normy].f;
        frac[n_startz].f = node[n_sectz].f - node[n_normz].f;
    }
}

/*
 *  the leaf intersector processes act on nodes in the shade tree,
 *  and either set the t record to 0 if no intersection, or
 *  set up t, sectxyz, normxyz if intersection occurs
 */
#define minSect 0.001f

#define a_11 0
#define a_12 1
#define a_13 2
#define a_21 3
#define a_22 4
#define a_23 5
#define a_31 6
#define a_32 7
#define a_33 8

#define p_xx 0
#define p_yy 1
#define p_zz 2

static void TransLocal(const float *matrix, float *point)
{
    float newpoint[3];
    int i;

    newpoint[p_xx] = ((matrix[a_11] * point[p_xx]) +
                      ((matrix[a_12] * point[p_yy]) +
                       (matrix[a_13] * point[p_zz])));
    newpoint[p_yy] = ((matrix[a_21] * point[p_xx]) +
                      ((matrix[a_22] * point[p_yy]) +
                       (matrix[a_23] * point[p_zz])));
    newpoint[p_zz] = ((matrix[a_31] * point[p_xx]) +
                      ((matrix[a_32] * point[p_yy]) +
                       (matrix[a_33] * point[p_zz])));
    for (i = 0; i < 3; i++)
        point[i] = newpoint[i];
}

static void TransGlobal(const float *matrix, float *point)
{
    float invers[9];

    invers[a_11] = matrix[a_11];
    invers[a_12] = matrix[a_21];
    invers[a_13] = matrix[a_31];
    invers[a_21] = matrix[a_12];
    invers[a_22] = matrix[a_22];
    invers[a_23] = matrix[a_32];
    invers[a_31] = matrix[a_13];
    invers[a_32] = matrix[a_23];
    invers[a_33] = matrix[a_33];
    TransLocal(invers, point);
}

static void sphereSect(W *node, const W *sphere)
{
    float xoff, yoff, zoff, xcomp, ycomp, zcomp, minusb, rootb2m4ac, b2, c;
    float t1, t2, t = 0.0f;
    int proceed, t1ok, t2ok;

    xoff = node[n_startx].f - sphere[s_x].f;
    yoff = node[n_starty].f - sphere[s_y].f;
    zoff = node[n_startz].f - sphere[s_z].f;

    xcomp = xoff * node[n_dx].f;
    ycomp = yoff * node[n_dy].f;
    zcomp = zoff * node[n_dz].f;

    minusb = -(xcomp + (ycomp + zcomp));
    b2 = minusb * minusb;

    xcomp = xoff * xoff;
    ycomp = yoff * yoff;
    zcomp = zoff * zoff;

    c = (xcomp + (ycomp + zcomp)) - (sphere[s_rad].f * sphere[s_rad].f);
    if (b2 > c) {
        rootb2m4ac = sqrtf_(b2 - c);
        t1 = minusb - rootb2m4ac;
        t2 = minusb + rootb2m4ac;
        t1ok = t1 > minSect;
        t2ok = t2 > minSect;
        if (t1ok && t2ok) {
            proceed = 1;
            if (t2 > t1)
                t = t1;
            else
                t = t2;
        } else if (t1ok) {
            t = t1;
            proceed = 1;
        } else if (t2ok) {
            t = t2;
            proceed = 1;
        } else
            proceed = 0;
        if (proceed) {
            node[n_t].f = t;
            node[n_sectx].f = (node[n_dx].f * t) + node[n_startx].f;
            node[n_secty].f = (node[n_dy].f * t) + node[n_starty].f;
            node[n_sectz].f = (node[n_dz].f * t) + node[n_startz].f;

            node[n_normx].f = (node[n_sectx].f - sphere[s_x].f) / sphere[s_rad].f;
            node[n_normy].f = (node[n_secty].f - sphere[s_y].f) / sphere[s_rad].f;
            node[n_normz].f = (node[n_sectz].f - sphere[s_z].f) / sphere[s_rad].f;
        } else
            node[n_t].i = 0;            /* no intersection */
    } else
        node[n_t].i = 0;
}

static void ellipsoidSect(W *node, const W *ellipsoid)
{
    float aa, bb, cc, b2mac, rootb2mac, length;
    float point[3], normal[3], matrix[9];
    float t1, t2, t = 0.0f;
    int proceed, t1ok, t2ok, i;

    for (i = 0; i < 3; i++)
        point[i] = node[i + n_startx].f - ellipsoid[i + e_x].f;
    for (i = 0; i < 3; i++)
        normal[i] = node[i + n_dx].f;
    for (i = 0; i < 9; i++)
        matrix[i] = ellipsoid[i + e_a11].f;
    TransLocal(matrix, point);
    for (i = 0; i < 3; i++)
        point[i] = point[i] / ellipsoid[e_radx + i].f;
    TransLocal(matrix, normal);
    for (i = 0; i < 3; i++)
        normal[i] = normal[i] / ellipsoid[e_radx + i].f;
    aa = dotProduct(normal, normal);
    bb = dotProduct(normal, point);
    cc = dotProduct(point, point);
    cc = (cc - 1.0f);
    b2mac = (bb * bb) - (aa * cc);
    if (b2mac > 0.0f) {
        rootb2mac = sqrtf_(b2mac);
        t1 = -((rootb2mac + bb) / aa);
        t2 = ((rootb2mac - bb) / aa);
        t1ok = t1 > minSect;
        t2ok = t2 > minSect;
        if (t1ok && t2ok) {
            proceed = 1;
            if (t2 > t1)
                t = t1;
            else
                t = t2;
        } else if (t1ok) {
            t = t1;
            proceed = 1;
        } else if (t2ok) {
            t = t2;
            proceed = 1;
        } else
            proceed = 0;
        if (proceed) {
            node[n_t].f = t;
            for (i = 0; i < 3; i++)
                point[i] = ((normal[i] * t) + point[i]);
            for (i = 0; i < 3; i++)
                normal[i] = point[i] / ellipsoid[e_radx + i].f;
            for (i = 0; i < 3; i++)
                point[i] = point[i] * ellipsoid[e_radx + i].f;
            TransGlobal(matrix, point);
            TransGlobal(matrix, normal);
            node[n_sectx].f = (ellipsoid[e_x].f + point[p_xx]);
            node[n_secty].f = (ellipsoid[e_y].f + point[p_yy]);
            node[n_sectz].f = (ellipsoid[e_z].f + point[p_zz]);
            normalize(normal, &length);
            node[n_normx].f = normal[p_xx];
            node[n_normy].f = normal[p_yy];
            node[n_normz].f = normal[p_zz];
        } else
            node[n_t].i = 0;            /* no intersection */
    } else
        node[n_t].i = 0;
}

static void cylinderSect(W *node, const W *cylinder)
{
    float t1, t2, t3, t4, t12 = 0.0f, t34 = 0.0f, t;
    int t1ok, t2ok, t3ok, t4ok, proceed12, proceed34, i;
    float zaxis = 0.0f;
    float aa, bb, cc, b2mac, rootb2mac, length;
    float point[3], normal[3], matrix[9];

    for (i = 0; i < 3; i++)
        point[i] = node[i + n_startx].f - cylinder[i + c_x].f;
    for (i = 0; i < 3; i++)
        normal[i] = node[i + n_dx].f;
    for (i = 0; i < 9; i++)
        matrix[i] = cylinder[i + c_a11].f;
    TransLocal(matrix, point);
    point[0] = point[0] / cylinder[c_rad].f;
    point[1] = point[1] / cylinder[c_rad].f;
    point[2] = point[2] / cylinder[c_len].f;
    TransLocal(matrix, normal);
    normal[0] = normal[0] / cylinder[c_rad].f;
    normal[1] = normal[1] / cylinder[c_rad].f;
    normal[2] = normal[2] / cylinder[c_len].f;
    aa = (normal[0] * normal[0]) + (normal[1] * normal[1]);
    bb = (normal[0] * point[0]) + (normal[1] * point[1]);
    cc = (point[0] * point[0]) + (point[1] * point[1]);
    cc = (cc - 1.0f);
    b2mac = (bb * bb) - (aa * cc);
    if (b2mac > 0.0f) {
        rootb2mac = sqrtf_(b2mac);
        t1 = -((rootb2mac + bb) / aa);
        t2 = ((rootb2mac - bb) / aa);
        zaxis = point[2] + (t1 * normal[2]);
        if (zaxis < 0.0f)
            t1ok = 0;
        else if (zaxis > 1.0f)
            t1ok = 0;
        else
            t1ok = t1 > minSect;
        zaxis = point[2] + (t2 * normal[2]);
        if (zaxis < 0.0f)
            t2ok = 0;
        else if (zaxis > 1.0f)
            t2ok = 0;
        else
            t2ok = t2 > minSect;
        if (t1ok && t2ok) {
            proceed12 = 1;
            if (t2 > t1)
                t12 = t1;
            else
                t12 = t2;
        } else if (t1ok) {
            t12 = t1;
            proceed12 = 1;
        } else if (t2ok) {
            t12 = t2;
            proceed12 = 1;
        } else
            proceed12 = 0;
    } else {
        proceed12 = 0;
        t1ok = 0;
        t2ok = 0;
    }
    if (normal[2] == 0.0f) {
        proceed34 = 0;
        t3ok = 0;
        t4ok = 0;
    } else {
        t3 = ((-point[2]) / normal[2]);
        t4 = ((1.0f - point[2]) / normal[2]);

        if (t3 < minSect)
            t3ok = 0;
        else
            t3ok = ((aa * (t3 * t3)) + (bb * (2.0f * t3))) < (-cc);
        if (t4 < minSect)
            t4ok = 0;
        else
            t4ok = ((aa * (t4 * t4)) + (bb * (2.0f * t4))) < (-cc);
        if (t3ok && t4ok) {
            proceed34 = 1;
            if (t4 > t3) {
                t34 = t3;
                zaxis = 0.0f;
            } else {
                t34 = t4;
                zaxis = cylinder[c_len].f;
            }
        } else if (t3ok) {
            t34 = t3;
            zaxis = 0.0f;
            proceed34 = 1;
        } else if (t4ok) {
            t34 = t4;
            zaxis = cylinder[c_len].f;
            proceed34 = 1;
        } else
            proceed34 = 0;
    }
    if ((proceed12 && proceed34 && t34 > t12) || (proceed12 && !proceed34)) {
        t = t12;
        node[n_t].f = t;
        point[0] = ((normal[0] * t) + point[0]) * cylinder[c_rad].f;
        point[1] = ((normal[1] * t) + point[1]) * cylinder[c_rad].f;
        point[2] = ((normal[2] * t) + point[2]) * cylinder[c_len].f;
        normal[0] = point[0];
        normal[1] = point[1];
        normal[2] = 0.0f;
        TransGlobal(matrix, point);
        node[n_sectx].f = (cylinder[c_x].f + point[p_xx]);
        node[n_secty].f = (cylinder[c_y].f + point[p_yy]);
        node[n_sectz].f = (cylinder[c_z].f + point[p_zz]);
        TransGlobal(matrix, normal);
        normalize(normal, &length);
        node[n_normx].f = normal[p_xx];
        node[n_normy].f = normal[p_yy];
        node[n_normz].f = normal[p_zz];
    } else if (proceed34) {
        t = t34;
        node[n_t].f = t;
        point[0] = ((normal[0] * t) + point[0]) * cylinder[c_rad].f;
        point[1] = ((normal[1] * t) + point[1]) * cylinder[c_rad].f;
        point[2] = zaxis;
        normal[0] = 0.0f;
        normal[1] = 0.0f;
        normal[2] = 1.0f;
        TransGlobal(matrix, point);
        node[n_sectx].f = (cylinder[c_x].f + point[p_xx]);
        node[n_secty].f = (cylinder[c_y].f + point[p_yy]);
        node[n_sectz].f = (cylinder[c_z].f + point[p_zz]);
        TransGlobal(matrix, normal);
        node[n_normx].f = normal[p_xx];
        node[n_normy].f = normal[p_yy];
        node[n_normz].f = normal[p_zz];
    } else
        node[n_t].i = 0;                /* no intersection */
}

static void coneSect(W *node, const W *cone)
{
    float t1, t2, t3, t4, t12 = 0.0f, t34 = 0.0f, t;
    int t1ok, t2ok, t3ok, t4ok, proceed12, proceed34, i;
    float zaxis;
    float aa, bb, cc, b2mac, rootb2mac, length;
    float delta, delta_2, bma;
    float point[3], normal[3], matrix[9];
    float sinus, cosinus, hyperbole;

    bma = cone[co_radb].f - cone[co_rada].f;
    delta = (cone[co_radb].f / cone[co_rada].f) - 1.0f;
    delta_2 = delta * delta;
    for (i = 0; i < 3; i++)
        point[i] = node[i + n_startx].f - cone[i + co_x].f;
    for (i = 0; i < 3; i++)
        normal[i] = node[i + n_dx].f;
    for (i = 0; i < 9; i++)
        matrix[i] = cone[i + co_a11].f;
    TransLocal(matrix, point);
    point[0] = point[0] / cone[co_rada].f;
    point[1] = point[1] / cone[co_rada].f;
    point[2] = point[2] / cone[co_len].f;
    TransLocal(matrix, normal);
    normal[0] = normal[0] / cone[co_rada].f;
    normal[1] = normal[1] / cone[co_rada].f;
    normal[2] = normal[2] / cone[co_len].f;
    aa = (((normal[0] * normal[0]) + (normal[1] * normal[1])) -
          ((normal[2] * normal[2]) * delta_2));
    bb = (((normal[0] * point[0]) + (normal[1] * point[1])) -
          (((normal[2] * point[2]) * delta_2) + (delta * normal[2])));
    cc = (((point[0] * point[0]) +
           (point[1] * point[1])) -
          (((point[2] * point[2]) * delta_2) +
           ((delta * point[2]) * 2.0f)));
    cc = (cc - 1.0f);
    b2mac = (bb * bb) - (aa * cc);
    if (b2mac > 0.0f) {
        rootb2mac = sqrtf_(b2mac);
        t1 = -((rootb2mac + bb) / aa);
        t2 = ((rootb2mac - bb) / aa);
        zaxis = point[2] + (normal[2] * t1);
        if (zaxis < 0.0f)
            t1ok = 0;
        else if (zaxis > 1.0f)
            t1ok = 0;
        else
            t1ok = t1 > minSect;
        zaxis = point[2] + (normal[2] * t2);
        if (zaxis < 0.0f)
            t2ok = 0;
        else if (zaxis > 1.0f)
            t2ok = 0;
        else
            t2ok = t2 > minSect;
        if (t1ok && t2ok) {
            proceed12 = 1;
            if (t2 > t1)
                t12 = t1;
            else
                /* The occam assigns t, not t12, here.  Both roots are on
                 * the cone's one nappe only when aa > 0, and then t2 > t1,
                 * so this branch is never taken; it is kept as written. */
                node[n_t].f = t2;
        } else if (t1ok) {
            t12 = t1;
            proceed12 = 1;
        } else if (t2ok) {
            t12 = t2;
            proceed12 = 1;
        } else
            proceed12 = 0;
    } else {
        proceed12 = 0;
        t1ok = 0;
        t2ok = 0;
    }
    if (normal[2] == 0.0f) {
        proceed34 = 0;
        t3ok = 0;
        t4ok = 0;
    } else {
        t3 = ((-point[2]) / normal[2]);
        t4 = ((1.0f - point[2]) / normal[2]);
        if (t3 < minSect)
            t3ok = 0;
        else
            t3ok = ((aa * (t3 * t3)) + (bb * (2.0f * t3))) < (-cc);
        if (t4 < minSect)
            t4ok = 0;
        else
            t4ok = ((aa * (t4 * t4)) + (bb * (2.0f * t4))) < (-cc);
        if (t3ok && t4ok) {
            proceed34 = 1;
            if (t4 > t3)
                t34 = t3;
            else
                t34 = t4;
        } else if (t3ok) {
            t34 = t3;
            proceed34 = 1;
        } else if (t4ok) {
            t34 = t4;
            proceed34 = 1;
        } else
            proceed34 = 0;
    }
    if ((proceed12 && proceed34 && t34 > t12) || (proceed12 && !proceed34)) {
        hyperbole = sqrtf_(((bma * bma) + (cone[co_len].f * cone[co_len].f)));
        cosinus = cone[co_len].f / hyperbole;
        sinus = bma / hyperbole;
        t = t12;
        node[n_t].f = t;
        point[0] = ((normal[0] * t) + point[0]) * cone[co_rada].f;
        point[1] = ((normal[1] * t) + point[1]) * cone[co_rada].f;
        point[2] = ((normal[2] * t) + point[2]) * cone[co_len].f;
        normal[0] = point[0];
        normal[1] = point[1];
        normal[2] = 0.0f;
        normalize(normal, &length);
        normal[0] = normal[0] * cosinus;
        normal[1] = normal[1] * cosinus;
        normal[2] = -sinus;
        TransGlobal(matrix, point);
        node[n_sectx].f = (cone[co_x].f + point[p_xx]);
        node[n_secty].f = (cone[co_y].f + point[p_yy]);
        node[n_sectz].f = (cone[co_z].f + point[p_zz]);
        TransGlobal(matrix, normal);
        node[n_normx].f = normal[p_xx];
        node[n_normy].f = normal[p_yy];
        node[n_normz].f = normal[p_zz];
    } else if (proceed34) {
        t = t34;
        node[n_t].f = t;
        point[0] = ((normal[0] * t) + point[0]) * cone[co_rada].f;
        point[1] = ((normal[1] * t) + point[1]) * cone[co_rada].f;
        point[2] = ((normal[2] * t) + point[2]);
        normal[0] = 0.0f;
        normal[1] = 0.0f;
        normal[2] = 1.0f;
        point[2] = point[2] * cone[co_len].f;
        TransGlobal(matrix, point);
        node[n_sectx].f = (cone[co_x].f + point[p_xx]);
        node[n_secty].f = (cone[co_y].f + point[p_yy]);
        node[n_sectz].f = (cone[co_z].f + point[p_zz]);
        TransGlobal(matrix, normal);
        node[n_normx].f = normal[p_xx];
        node[n_normy].f = normal[p_yy];
        node[n_normz].f = normal[p_zz];
    } else
        node[n_t].i = 0;                /* no intersection */
}

static void planeSect(W *node, const W *plane)
{
    float pw, rw, ou, ov, t, q[3];
    int proceed, i;

    rw = ((plane[p_wx].f * node[n_dx].f) +
          (plane[p_wy].f * node[n_dy].f)) +
         (plane[p_wz].f * node[n_dz].f);
    if (rw != 0.0f) {
        pw = ((plane[p_wx].f * (node[n_startx].f - plane[p_x].f)) +
              (plane[p_wy].f * (node[n_starty].f - plane[p_y].f))) +
             (plane[p_wz].f * (node[n_startz].f - plane[p_z].f));
        t = -(pw / rw);
        node[n_t].f = t;
        if (t > 0.0f) {
            for (i = 0; i < 3; i++)
                q[i] = node[n_startx + i].f + (node[n_dx + i].f * t);
            ou = ((plane[p_ux].f * (q[0] - plane[p_x].f)) +
                  (plane[p_uy].f * (q[1] - plane[p_y].f))) +
                 (plane[p_uz].f * (q[2] - plane[p_z].f));
            if (ou < 0.0f)
                proceed = 0;
            else if (ou > plane[p_sizeu].f)
                proceed = 0;
            else
                proceed = 1;
            if (proceed) {              /* e.g. value on u axis valid. */
                ov = ((plane[p_vx].f * (q[0] - plane[p_x].f)) +
                      (plane[p_vy].f * (q[1] - plane[p_y].f))) +
                     (plane[p_vz].f * (q[2] - plane[p_z].f));
                if (ov < 0.0f)
                    proceed = 0;
                else if (ov > plane[p_sizev].f)
                    proceed = 0;
                if (proceed)            /* e.g. value on u and v valid */
                    for (i = 0; i < 3; i++) {
                        node[n_sectx + i].f = q[i];
                        if (rw < 0.0f)
                            node[n_normx + i].f = plane[p_wx + i].f;
                        else
                            node[n_normx + i].f = -plane[p_wx + i].f;
                    }
                else
                    node[n_t].i = 0;
            } else
                node[n_t].i = 0;
        } else
            node[n_t].i = 0;
    } else
        node[n_t].i = 0;                /* no intersection */
}

static void xyPlaneSect(W *node, const W *plane)
{
    float aa, bb, t;
    int proceed;

    if (node[n_dz].f != 0.0f) {
        t = (plane[pxy_z].f - node[n_startz].f) / node[n_dz].f;
        node[n_t].f = t;
        if (t > 0.0f) {
            aa = node[n_startx].f + (node[n_dx].f * t);
            if (aa < plane[pxy_x].f)
                proceed = 0;
            else if (aa > (plane[pxy_x].f + plane[pxy_sizex].f))
                proceed = 0;
            else
                proceed = 1;
            if (proceed) {              /* e.g. value on x axis valid */
                bb = node[n_starty].f + (node[n_dy].f * t);
                if (bb < plane[pxy_y].f)
                    proceed = 0;
                else if (bb > (plane[pxy_y].f + plane[pxy_sizey].f))
                    proceed = 0;
                if (proceed) {          /* e.g. value on y axis valid */
                    node[n_sectx].f = aa;
                    node[n_secty].f = bb;
                    node[n_sectz].f = plane[pxy_z].f;
                    node[n_normx].f = 0.0f;
                    node[n_normy].f = 0.0f;
                    if (node[n_dz].f > 0.0f)
                        node[n_normz].f = -1.0f;
                    else
                        node[n_normz].f = 1.0f;
                } else
                    node[n_t].i = 0;
            } else
                node[n_t].i = 0;
        } else
            node[n_t].i = 0;
    } else
        node[n_t].i = 0;                /* no intersection */
}

static void yzPlaneSect(W *node, const W *plane)
{
    float aa, bb, t;
    int proceed;

    if (node[n_dx].f != 0.0f) {
        t = (plane[pyz_x].f - node[n_startx].f) / node[n_dx].f;
        node[n_t].f = t;
        if (t > 0.0f) {
            aa = node[n_startz].f + (node[n_dz].f * t);
            if (aa < plane[pyz_z].f)
                proceed = 0;
            else if (aa > (plane[pyz_z].f + plane[pyz_sizez].f))
                proceed = 0;
            else
                proceed = 1;
            if (proceed) {              /* e.g. value on z axis valid */
                bb = node[n_starty].f + (node[n_dy].f * t);
                if (bb < plane[pyz_y].f)
                    proceed = 0;
                else if (bb > (plane[pyz_y].f + plane[pyz_sizey].f))
                    proceed = 0;
                if (proceed) {          /* e.g. value on y axis valid */
                    node[n_sectx].f = plane[pyz_x].f;
                    node[n_secty].f = bb;
                    node[n_sectz].f = aa;
                    if (node[n_dx].f > 0.0f)
                        node[n_normx].f = -1.0f;
                    else
                        node[n_normx].f = 1.0f;
                    node[n_normy].f = 0.0f;
                    node[n_normz].f = 0.0f;
                } else
                    node[n_t].i = 0;
            } else
                node[n_t].i = 0;
        } else
            node[n_t].i = 0;
    } else
        node[n_t].i = 0;                /* no intersection */
}

static void xzPlaneSect(W *node, const W *plane)
{
    float aa, bb, t;
    int proceed;

    if (node[n_dy].f != 0.0f) {
        t = (plane[pxz_y].f - node[n_starty].f) / node[n_dy].f;
        node[n_t].f = t;
        if (t > 0.0f) {
            aa = node[n_startx].f + (node[n_dx].f * t);
            if (aa < plane[pxz_x].f)
                proceed = 0;
            else if (aa > (plane[pxz_x].f + plane[pxz_sizex].f))
                proceed = 0;
            else
                proceed = 1;
            if (proceed) {              /* e.g. value on x axis valid */
                bb = node[n_startz].f + (node[n_dz].f * t);
                if (bb < plane[pxz_z].f)
                    proceed = 0;
                else if (bb > (plane[pxz_z].f + plane[pxz_sizez].f))
                    proceed = 0;
                if (proceed) {          /* e.g. value on z axis valid */
                    node[n_sectx].f = aa;
                    node[n_secty].f = plane[pxz_y].f;
                    node[n_sectz].f = bb;
                    node[n_normx].f = 0.0f;
                    if (node[n_dy].f > 0.0f)
                        node[n_normy].f = -1.0f;
                    else
                        node[n_normy].f = 1.0f;
                    node[n_normz].f = 0.0f;
                } else
                    node[n_t].i = 0;
            } else
                node[n_t].i = 0;
        } else
            node[n_t].i = 0;
    } else
        node[n_t].i = 0;                /* no intersection */
}

/* an occam array assignment, `closest := iNode`: a block move */
#define copyNode(dst, src) memcpy((dst), (src), nodeSize * sizeof(W))

static void sceneSect(int nodePtr, int shadowRay)
{
    W *iNode = &tree[nodePtr];
    W closest[nodeSize];
    int ptr, obj_p, proceed, size, type, garbage;
    W *object;

    nrays = nrays + 1;
    copyNode(closest, iNode);           /* copies ptrs, dx, dy etc. */
    proceed = 1;    /* a quick 'get out' clause for shadow checking */
    obj_p = nil;
    ptr = 0;
    while (worldModel[ptr].i != nil && proceed) {
        size = worldModel[ptr].i;
        object = &worldModel[ptr + 1];
        type = object[o_type].i;
        garbage = 0;
        if (type == o_sphere)
            sphereSect(iNode, object);
        else if (type == o_ellipsoid)
            ellipsoidSect(iNode, object);
        else if (type == o_cylinder)
            cylinderSect(iNode, object);
        else if (type == o_cone)
            coneSect(iNode, object);
        else if (type == o_plane)
            planeSect(iNode, object);
        else if (type == o_xyplane)
            xyPlaneSect(iNode, object);
        else if (type == o_yzplane)
            yzPlaneSect(iNode, object);
        else if (type == o_xzplane)
            xzPlaneSect(iNode, object);
        else
            garbage = 1;
        if (garbage) {
            writef("\n\rGarbage object %I in list at offset %I",
                   type, ptr, 0, 0);
            __stopp();
        } else if (iNode[n_t].i == 0)
            ;
        else if (shadowRay) {
            proceed = 0;
            iNode[n_objptr].i = nil;
        } else if (obj_p == nil) {
            obj_p = ptr;        /* pointer to type slot in world model */
            iNode[n_objptr].i = obj_p;
            copyNode(closest, iNode);
        } else if (iNode[n_t].f < closest[n_t].f) {
            obj_p = ptr;        /* pointer to size slot in world model */
            iNode[n_objptr].i = obj_p;
            copyNode(closest, iNode);
        }
        ptr = ptr + (size + 1);
    }
    if (!shadowRay)
        copyNode(iNode, closest);
}

static int claim(int type)
{
    W *node = &tree[freeNode];
    int nodePtr = freeNode;

    node[n_reflect].i = nil;
    node[n_refract].i = nil;
    node[n_next].i = nil;
    node[n_objptr].i = nil;
    node[n_type].i = type;
    freeNode = freeNode + nodeSize;
    return nodePtr;
}

static void shadeNode(int nodePtr)
{
    /*
     * Total intensity at node is sum of ambient, diffuse, glossy,
     * specular and transmitted intensities.
     *
     *   I = (Ia * kd) + Ig + kd * SUM(j=1..ls) (N.Lj)
     *
     * Here we compute ambient diffuse and glossy terms, specular and
     * transmitted components are added at mix time.  Ig is given by Phong:
     *
     *   Ig = SUM(j=1..ls) (N.Lj')^n
     */
    W *node = &tree[nodePtr];
    int objPtr = node[n_objptr].i;
    float *colour = &node[n_red].f;

    if (objPtr == nil) {
        colour[0] = ambient(0);
        colour[1] = ambient(1);
        colour[2] = ambient(2);
    } else {
        W *object = &worldModel[objPtr + 1];
        float spec = object[o_kg].f;    /* gloss coefficient */
        int attr = object[o_attr].i;
        int shadow, phong, lightPtr, size;
        W *shadowNode, *phongNode, *light;

        if ((attr & a_spec) == 0) {
            colour[0] = ambient(0) * object[o_kdR].f;
            colour[1] = ambient(1) * object[o_kdG].f;
            colour[2] = ambient(2) * object[o_kdB].f;
        } else {
            colour[0] = 0.0f;
            colour[1] = 0.0f;
            colour[2] = 0.0f;
        }
        shadow = claim(rt_root);
        phong = claim(rt_root);
        shadowNode = &tree[shadow];
        phongNode = &tree[phong];

        lightPtr = 0;
        shadowNode[n_startx].f = node[n_sectx].f + node[n_normx].f;
        shadowNode[n_starty].f = node[n_secty].f + node[n_normy].f;
        shadowNode[n_startz].f = node[n_sectz].f + node[n_normz].f;
        while (lights[lightPtr].i != nil) {
            size = lights[lightPtr].i;
            light = &lights[lightPtr + 1];
            /* set up shadow ray direction cosines */
            shadowNode[n_dx] = light[l_dx];
            shadowNode[n_dy] = light[l_dy];
            shadowNode[n_dz] = light[l_dz];
            sceneSect(shadow, 1);
            if (shadowNode[n_t].i == 0) {
                W lambert;

                lambert.f = dotProduct(&light[l_dx].f, &node[n_normx].f);
                if ((lambert.i & mint) == 0) {  /* not -ve */
                    colour[0] = colour[0] +
                        (lambert.f * (object[o_kdR].f * light[l_ir].f));
                    colour[1] = colour[1] +
                        (lambert.f * (object[o_kdG].f * light[l_ig].f));
                    colour[2] = colour[2] +
                        (lambert.f * (object[o_kdB].f * light[l_ib].f));
                }
                /*
                 * the phong shader is a mite messy at the moment, with
                 * lots of sign inversions all over the place.
                 */
                if ((attr & a_spec) != 0) {
                    W cosPhong, flipNode[3];
                    float Vprime, Vvec[3];      /* to keep reflect ray happy */
                    int signFlip;

                    shadowNode[n_normx] = node[n_normx];
                    shadowNode[n_normy] = node[n_normy];
                    shadowNode[n_normz] = node[n_normz];
                    shadowNode[n_sectx] = node[n_sectx];
                    shadowNode[n_secty] = node[n_secty];
                    shadowNode[n_sectz] = node[n_sectz];
                    /* a nasty sign inversion due to the light direction
                     * being optimized for shadow spotting */
                    shadowNode[n_dx].i ^= mint;
                    shadowNode[n_dy].i ^= mint;
                    shadowNode[n_dz].i ^= mint;

                    reflectRay(phong, shadow, &Vprime, Vvec, &signFlip);
                    /* again, a nasty sign inversion is required here -
                     * see diagram newmann / sproull, p. 391 */
                    flipNode[0] = node[n_dx];
                    flipNode[1] = node[n_dy];
                    flipNode[2] = node[n_dz];
                    flipNode[0].i ^= mint;
                    flipNode[1].i ^= mint;
                    flipNode[2].i ^= mint;
                    cosPhong.f = dotProduct(&flipNode[0].f, &phongNode[n_dx].f);
                    if ((cosPhong.i & mint) != 0)
                        cosPhong.i ^= mint;
                    else {
                        cosPhong.f = (cosPhong.f * cosPhong.f);  /* power 2 */
                        cosPhong.f = (cosPhong.f * cosPhong.f);  /* power 4 */
                        cosPhong.f = (cosPhong.f * cosPhong.f);  /* power 8 */
                        colour[0] = colour[0] +
                            (spec * (cosPhong.f * light[l_ir].f));
                        colour[1] = colour[1] +
                            (spec * (cosPhong.f * light[l_ig].f));
                        colour[2] = colour[2] +
                            (spec * (cosPhong.f * light[l_ib].f));
                    }
                }
            }
            lightPtr = lightPtr + (size + 1);
        }
    }
}

static void mixChildren(int nodePtr)
{
    /*
     * Here we attenuate specular and transmitted components from the
     * children, and add into the parent node.
     */
    W *node = &tree[nodePtr];
    int objPtr = node[n_objptr].i;
    W *object = &worldModel[objPtr + 1];
    int specPtr = node[n_reflect].i;
    int fracPtr = node[n_refract].i;
    float xmit[3], spec[3];

    if (specPtr == nil) {
        spec[0] = 0.0f;
        spec[1] = 0.0f;
        spec[2] = 0.0f;
    } else {
        /* specular contribution is independent of colour of surface,
         * but here we should attenuate according to 't' */
        spec[0] = tree[specPtr + n_red].f;
        spec[1] = tree[specPtr + n_green].f;
        spec[2] = tree[specPtr + n_blue].f;
        spec[0] = spec[0] * object[o_ks].f;
        spec[1] = spec[1] * object[o_ks].f;
        spec[2] = spec[2] * object[o_ks].f;
    }
    if (fracPtr == nil) {
        xmit[0] = 0.0f;
        xmit[1] = 0.0f;
        xmit[2] = 0.0f;
    } else {
        xmit[0] = tree[fracPtr + n_red].f;
        xmit[1] = tree[fracPtr + n_green].f;
        xmit[2] = tree[fracPtr + n_blue].f;
        xmit[0] = xmit[0] * object[o_xmitR].f;  /* colour the light */
        xmit[1] = xmit[1] * object[o_xmitG].f;
        xmit[2] = xmit[2] * object[o_xmitB].f;
    }
    node[n_red].f = node[n_red].f + (spec[0] + xmit[0]);
    node[n_green].f = node[n_green].f + (spec[1] + xmit[1]);
    node[n_blue].f = node[n_blue].f + (spec[2] + xmit[2]);
}

static void shade(int rootNode)
{
    /*
     * Enter with a pointer to the root node of the tree, leave with three
     * 10-bit integer values in the root node's r g b fields.  Bottom-up
     * shading of the ray intersection tree, equivalent to
     *
     *   PROC shade (VAL INT node)
     *     shade (reflect child); shade (refract child)
     *     shadeNode (node); mixNode (node)
     *
     * with an explicit stack of (node, action) pairs.
     */
#define a_stop    0
#define a_refract 1
#define a_mix     2
#define a_reflect 3
    int stack[(maxDepth + 2) * 2 * 2];
    int sp = 0, nodePtr, action;
    W *node, *root;
    const float maxC = 1022.99f;        /* just under 10 bits */
    float red, green, blue, maxP, fudge = 0.0f;
    int desaturate;

#define push2(a, b) (stack[sp] = (a), stack[sp + 1] = (b), sp += 2)
#define pop2(a, b)  (sp -= 2, (a) = stack[sp], (b) = stack[sp + 1])

    push2(nil, a_stop);         /* pre load stack with 'terminate' action */
    nodePtr = rootNode;
    action = a_reflect;
    while (action != a_stop) {
        node = &tree[nodePtr];
        if (action == a_reflect) {
            if (node[n_reflect].i == nil)   /* no reflected ray */
                action = a_refract;
            else if (node[n_refract].i == nil) {
                push2(nodePtr, a_mix);
                nodePtr = node[n_reflect].i;
            } else {
                push2(nodePtr, a_refract);  /* direction to go on return */
                nodePtr = node[n_reflect].i;
            }
        } else if (node[n_refract].i == nil) {
            shadeNode(nodePtr);             /* shade leaf node */
            pop2(nodePtr, action);
            while (action == a_mix) {       /* all these nodes have had */
                shadeNode(nodePtr);         /* their children shaded */
                mixChildren(nodePtr);
                pop2(nodePtr, action);
            }
        } else {
            push2(nodePtr, a_mix);
            nodePtr = node[n_refract].i;
            action = a_reflect;
        }
    }
#undef push2
#undef pop2

    root = &tree[rootNode];
    red = root[n_red].f;
    green = root[n_green].f;
    blue = root[n_blue].f;

    desaturate = 1;
    if (red > green)
        maxP = red;
    else
        maxP = green;
    if (maxP > blue) {
        if (maxP > maxC)
            fudge = maxC / maxP;
        else
            desaturate = 0;
    } else if (blue > maxC)
        fudge = maxC / blue;
    else
        desaturate = 0;
    if (desaturate) {
        root[n_red].i = (int)(red * fudge);
        root[n_green].i = (int)(green * fudge);
        root[n_blue].i = (int)(blue * fudge);
    } else {
        root[n_red].i = (int)red;
        root[n_green].i = (int)green;
        root[n_blue].i = (int)blue;
    }
}

/*
 * buildShadeTree is the recursive guts of the ray tracer, performing
 * reflection / refraction where necessary.  The tree is built breadth
 * first, keeping it balanced, by walking the linked list of leaf nodes at
 * level N while building the list for level N+1.
 *
 * evolveNode's intoNode is a location in the tree (or `head`), and is
 * assigned; outofNode is a variable holding an index into the tree.  On
 * exit outofNode is the location to pass as intoNode next time.
 */
static int head;

static void evolveNode(int *intoNode, int *outofNode, int nodePtr, int *spawn)
{
    W *node;
    int objptr, attr, spec, frac, flect, signFlip, tIR;
    float Vprime, Vvec[3];

    sceneSect(nodePtr, 0);              /* NOT casting shadow rays */
    node = &tree[nodePtr];
    if (node[n_objptr].i == nil) {
        *spawn = 0;
        return;
    }
    objptr = node[n_objptr].i;
    attr = worldModel[objptr + (o_attr + 1)].i;    /* skip size */
    if ((attr & a_spec) != 0) {
        spec = claim(rt_spec);
        node[n_reflect].i = spec;
        *intoNode = spec;               /* chain me into list of leaves */
        reflectRay(spec, nodePtr, &Vprime, Vvec, &signFlip);
        flect = 1;
    } else
        flect = 0;
    if (flect && (attr & a_frac) != 0) {
        frac = claim(rt_frac);
        refractRay(frac, nodePtr, Vprime, Vvec, signFlip, &tIR);
        if (tIR) {                      /* total internal reflection */
            *outofNode = spec + n_next; /* return pointer into tree */
            *spawn = 1;
        } else {
            node[n_refract].i = frac;
            tree[spec + n_next].i = frac;
            *spawn = 2;
            *outofNode = frac + n_next;
        }
    } else if (flect) {
        *outofNode = spec + n_next;     /* return pointer into tree */
        *spawn = 1;
    } else
        *spawn = 0;
}

static void evolveTree(int *nodesAdded)
{
    int spawn, node, next, prev;

    node = head;
    next = node + n_next;
    evolveNode(&head, &next, node, nodesAdded);
    while (tree[node + n_next].i != nil) {
        prev = next;
        next = node + n_next;
        node = tree[next].i;
        evolveNode(&tree[prev].i, &next, node, &spawn);
        *nodesAdded = *nodesAdded + spawn;
    }
    tree[next].i = nil;
}

/* takes an (x, y) in screen space and turns it into an (x,y,z) (dx,dy,dz)
 * ray */
static void createRay(int rootRay, float x, float y)
{
    W *node = &tree[rootRay];
    float scrx, scry, hyp;

    scrx = 512.0f - x;
    scry = 512.0f - y;

    node[n_startx].f = screenOrg(0) +
                       ((scrx * screendX(0)) +
                        (scry * screendY(0)));
    node[n_starty].f = screenOrg(1) +
                       ((scrx * screendX(1)) +
                        (scry * screendY(1)));
    node[n_startz].f = screenOrg(2) +
                       ((scrx * screendX(2)) +
                        (scry * screendY(2)));

    node[n_dx].f = pinhole(0) - node[n_startx].f;
    node[n_dy].f = pinhole(1) - node[n_starty].f;
    node[n_dz].f = pinhole(2) - node[n_startz].f;
    normalize(&node[n_dx].f, &hyp);
}

static int buildShadeTree(float x, float y)
{
    int rootNode, newNodes, nodes, depth;

    freeNode = 0;                       /* initialize tree */
    nodes = 0;
    depth = 1;
    rootNode = claim(rt_root);          /* get root ray */
    createRay(rootNode, x, y);          /* build direction cosines etc. */
    head = rootNode;                    /* init linked list of leaves */
    tree[rootNode + n_next].i = nil;    /* close linked list */
    evolveTree(&newNodes);
    while ((depth < maxDepth) && (newNodes != 0)) {
        nodes = nodes + newNodes;
        depth = depth + 1;
        evolveTree(&newNodes);
    }
    return rootNode;
}

/* A grid position (patch origin in pixels, sub-pixel offset in quarter
 * pixels) to the occam's 256 x 256 screen coordinates.  At RES = 256 this
 * is the occam's own (REAL32 TRUNC t) / 4.0; above it the steps are
 * 256/RES as long, a power of two, so every step is exact. */
static float screenCoord(int t)
{
    float w = (float)t / (float)descendPower;
#if RES != 256
    w = ((w - 128.0f) * (256.0f / (float)RES)) + 128.0f;
#endif
    return w;
}

static int renderPatchx, renderPatchy;

static int pointSample(int x, int y)
{
    int *patch_p = &samples[y][x];
    int tree_p, colour;
    W *node;

    if (*patch_p == notRendered) {
        tree_p = buildShadeTree(screenCoord((renderPatchx << maxDescend) + x),
                                screenCoord((renderPatchy << maxDescend) + y));
        shade(tree_p);
        node = &tree[tree_p];
        colour = node[n_red].i |
                 ((node[n_green].i << colourBits) |
                  (node[n_blue].i << (colourBits + colourBits)));
        *patch_p = colour;
    } else
        colour = *patch_p;
    return colour;
}

static int findRange(int a, int b, int c, int d)
{
    int max, min;

    if (a > b) {
        max = a;
        min = b;
    } else {
        max = b;
        min = a;
    }
    if (c > max)
        max = c;
    else if (c < min)
        min = c;
    if (d > max)
        max = d;
    else if (d < min)
        min = d;
    return max - min;
}

/* the four colours' average, channel by channel (the blue sum can carry
 * into bit 32: the occam adds it with PLUS) */
static unsigned average(unsigned a, unsigned b, unsigned c, unsigned d)
{
    unsigned R, G, B, m;

    m = rMask;
    R = ((((a & m) + (b & m)) + ((c & m) + (d & m))) >> 2) & m;
    m = gMask;
    G = ((((a & m) + (b & m)) + ((c & m) + (d & m))) >> 2) & m;
    m = bMask;
    B = ((((a & m) + (b & m)) + ((c & m) + (d & m))) >> 2) & m;
    return R | (G | B);
}

static int renderPixels(int patchx, int patchy, int x0, int y0,
                        int patchSize_, int renderingMode_)
{
#define a_render 0
#define a_shade  1
#define a_stopr  2
    int stack[maxDescend * (4 * 17)];   /* 4 * (render, x, y, hop); shade */
    unsigned colstack[(maxDescend + 1) * 4];
    int sp, cp, x, y, hop, action;
    unsigned colour = 0;

    renderPatchx = patchx;
    renderPatchy = patchy;
    if (renderingMode_ == m_adaptive) {
        cp = 0;                         /* empty colour stack */
        action = a_render;              /* init action */

        stack[0] = a_stopr;
        stack[1] = descendPower;        /* grid hop: gross pixel level */
        stack[2] = y0 << maxDescend;
        stack[3] = x0 << maxDescend;    /* locations within this patch */
        sp = 4;             /* pre load stack with render x y hop; stop */
        while (action != a_stopr) {
            if (action == a_render) {
                unsigned a, b, c, d;
                int rRange, gRange, bRange;
                const int sg = colourBits, sb = colourBits + colourBits;
                int *record;

                x = stack[sp - 1];
                y = stack[sp - 2];
                hop = stack[sp - 3];
                sp = sp - 3;

                a = pointSample(x, y);
                b = pointSample(x + hop, y);
                c = pointSample(x, y + hop);
                d = pointSample(x + hop, y + hop);

                rRange = findRange(a & rMask, b & rMask, c & rMask, d & rMask);
                gRange = findRange((a & gMask) >> sg, (b & gMask) >> sg,
                                   (c & gMask) >> sg, (d & gMask) >> sg);
                bRange = findRange(a >> sb, b >> sb, c >> sb, d >> sb);

                if ((hop != 1) &&
                    ((rRange > threshold) ||
                     ((gRange > threshold) || (bRange > threshold)))) {
                    record = &stack[sp];
                    hop = hop >> 1;
                    record[0] = a_shade;
                    record[1 + 0] = hop;
                    record[1 + 1] = y;
                    record[1 + 2] = x;
                    record[1 + 3] = a_render;
                    record[5 + 0] = hop;
                    record[5 + 1] = y;
                    record[5 + 2] = x + hop;
                    record[5 + 3] = a_render;
                    record[9 + 0] = hop;
                    record[9 + 1] = y + hop;
                    record[9 + 2] = x;
                    record[9 + 3] = a_render;
                    record[13 + 0] = hop;
                    record[13 + 1] = y + hop;
                    record[13 + 2] = x + hop;
                    record[13 + 3] = a_render;
                    sp = sp + 17;
                } else
                    colstack[cp++] = average(a, b, c, d);
                sp = sp - 1;
                action = stack[sp];
            } else if (action == a_shade) {
                unsigned *record = &colstack[cp - 4];

                record[0] = average(record[3], record[2], record[1], record[0]);
                cp = cp - 3;
                sp = sp - 1;
                action = stack[sp];
            }
        }
        colour = colstack[cp - 1];
    } else if (renderingMode_ == m_stochastic)
        ;
    else if (renderingMode_ == m_dumb) {
        int tree_p;
        W *node;

        tree_p = buildShadeTree(screenCoord((patchx + x0) << maxDescend),
                                screenCoord((patchy + y0) << maxDescend));
        shade(tree_p);
        node = &tree[tree_p];
        colour = node[n_red].i |
                 ((node[n_green].i << colourBits) |
                  (node[n_blue].i << (colourBits + colourBits)));
    } else if (renderingMode_ == m_test)
        colour = (x0 + (y0 * patchSize_));
    return colour;
}

static void renderer(void)
{
    static int packed[patchSize * patchSize];
    int command, nobj, nlight, nrender, x0y0[3], x, y, size;
    float hyp;

    nrays = 0;
    nobj = 0;
    nlight = 0;
    nrender = 0;
    world_p = 0;                        /* empty world model */
    light_p = 0;                        /* and lights list */
    mess_p = 0;                         /* clear debug message */
    command = ChanInInt(&pixelFeed);
    while (command != c_stop) {
        if (command == c_render) {
            ChanInWords(&pixelFeed, x0y0, 2);
            if (renderingMode == m_adaptive)
                initWORDvec(&samples[0][0], notRendered, gridSize * gridSize);
            for (x = 0; x < runPatchSize; x++)
                for (y = 0; y < runPatchSize; y++)
                    colours[y][x] = renderPixels(x0y0[0], x0y0[1], x, y,
                                                 runPatchSize, renderingMode);
            ChanOutInt(&pixelTrigger, 42);  /* done, BEFORE pixelsOut */
            x0y0[2] = runPatchSize;
            for (y = 0; y < runPatchSize; y++)
                for (x = 0; x < runPatchSize; x++)
                    packed[y * runPatchSize + x] = colours[y][x];
            ChanOutInt(&pixelOut, c_patch);
            ChanOutWords(&pixelOut, x0y0, 3);
            ChanOutWords(&pixelOut, packed, runPatchSize * runPatchSize);
            nrender = nrender + 1;
        } else if (command == c_object) {
            nobj = nobj + 1;
            size = ChanInInt(&pixelFeed);
            worldModel[world_p].i = size;
            ChanInWords(&pixelFeed, &worldModel[world_p + 1], size);
            world_p = world_p + (size + 1);
        } else if (command == c_light) {
            nlight = nlight + 1;
            size = ChanInInt(&pixelFeed);
            lights[light_p].i = size;
            ChanInWords(&pixelFeed, &lights[light_p + 1], size);
            normalize(&lights[light_p + (l_dx + 1)].f, &hyp);
            light_p = light_p + (size + 1);
        } else if (command == c_runData) {
            size = ChanInInt(&pixelFeed);
            ChanInWords(&pixelFeed, grunData, size);
            lights[light_p].i = nil;
            worldModel[world_p].i = nil;
        } else {
            writef("\n\r received garbage command %I", command, 0, 0, 0);
            __stopp();
        }
        command = ChanInInt(&pixelFeed);
    }
    writef("\n\r Worker %I DONE, %I rays, %I render jobs",
           me, nrays, nrender, 0);
    if (me == 0)
        writef("\n\r Scene of %I objects and %I lights", nobj, nlight, 0, 0);
    ChanOutInt(&pixelOut, command);
}

void rayTrace(Channel *fromPrev_, Channel *toPrev_, Channel *fromNext_,
              Channel *toNext_, int propogate_, int me_)
{
    fromPrev = fromPrev_;
    toPrev = toPrev_;
    fromNext = fromNext_;
    toNext = toNext_;
    propogate = propogate_;
    me = me_;
    startProcess(throughPut, throughWs, sizeof throughWs, PRI_HIGH);
    startProcess(pixelBypass, bypassWs, sizeof bypassWs, PRI_HIGH);
    startProcess(renderer, rendererWs, sizeof rendererWs, PRI_LOW);
}
