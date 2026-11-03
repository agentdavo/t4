/*
 * cntlsys.c - controlSystem (occ/cntlsys.occ): sends the world model, the
 * lights and the run data down the chain, then farms the picture out a
 * patch at a time and passes the patches that come back to frameBuffer.
 *
 * One change from the occam, to the farm.  cntlsys.occ ran
 *
 *      PAR
 *        sendPatches  (a, patchEdge)
 *        outputBuffer (b, out)            -- a one-place buffer
 *        returnBuffer (back, c, buffer [1])
 *        loadBalance  (a, b, c, graphics, worker, buffer [2])
 *
 * and that can deadlock: loadBalance blocks handing a job to outputBuffer,
 * which is blocked offering its job to node 0's throughPut, which is
 * blocked passing a job on to node 1's throughPut, which is blocked
 * feeding its renderer, which is still sending the pixels of its last job
 * - and those come back through returnBuffer to loadBalance, which is not
 * listening.  bin/testall.sh describes it: real links make the window
 * tiny, emulated ones wide.  Here loadBalance queues the jobs itself and
 * hands one to jobSender only when jobSender asks for it, so it is never
 * blocked while pixels are waiting.  A job is still sent only against one
 * of the 2 * worker credits, so the queue never holds more than that.
 */
#include "raytrace.h"

/*
 * run data is a vector of
 *
 *  runData [0] ->  ambient R
 *           1      ambient G
 *           2      ambient B
 *           3      rendering mode  -- m.adaptive / m.dumb / m.stochastic
 *           4      patch size
 *           5      scale factor    -- 1 2 or 4
 *           6      screenOrg x
 *           7                y
 *           8                z
 *           9      screen  dxX
 *           10             dyX
 *           11             dzX
 *           12     screen  dxY
 *           13             dyY
 *           14             dzY
 *           15     pinhole  x
 *           16     pinhole  y
 *           17     pinhole  z
 */
static W runData[runDataSize];

#define ambient(i)    runData[0 + (i)].f
#define renderingMode runData[3].i
#define runPatchSize  runData[4].i
#define scaleFactor   runData[5].i
#define screenOrg(i)  runData[6 + (i)].f
#define screendX(i)   runData[9 + (i)].f
#define screendY(i)   runData[12 + (i)].f
#define pinhole(i)    runData[15 + (i)].f

#define patchEdge PATCH

#define MAXWORKERS 1024
#define QMAX (2 * MAXWORKERS + 1)

static Channel *out, *back, *graphics, *image;
static int worker;

static Channel a = NOPROCESS;           /* sendPatches  -> loadBalance */
static Channel c = NOPROCESS;           /* returnBuffer -> loadBalance */
static Channel ready = NOPROCESS;       /* jobSender: "give me a job" */
static Channel job = NOPROCESS;         /* loadBalance -> jobSender */

static int sendWs[256], senderWs[256], returnWs[512];
static int returnHdr[3], returnPatch[patchSize * patchSize];
static unsigned char returnChars[256];
static int balanceHdr[3], balancePatch[patchSize * patchSize];
static unsigned char balanceChars[256];

static void writes(const char *s)
{
    int n = 0;

    while (s[n])
        n++;
    ChanOutInt(graphics, c_message);
    ChanOutInt(graphics, n);
    ChanOut(graphics, s, n);
}

static void sendObject(W *temp, int size)
{
    ChanOutInt(out, c_object);
    ChanOutInt(out, size);
    ChanOutWords(out, temp, size);
}

static void sphere(W *t, int attr, float kdR, float kdG, float kdB, float ks,
                   float rad, float x, float y, float z)
{
    t[o_type].i = o_sphere;
    t[o_attr].i = attr;
    t[o_kdR].f = kdR;
    t[o_kdG].f = kdG;
    t[o_kdB].f = kdB;
    t[o_ks].f = ks;
    t[s_rad].f = rad;
    t[s_x].f = x;
    t[s_y].f = y;
    t[s_z].f = z;
}

static void matrix(W *t, int first, float a11, float a12, float a13,
                   float a21, float a22, float a23,
                   float a31, float a32, float a33)
{
    t[first + 0].f = a11;
    t[first + 1].f = a12;
    t[first + 2].f = a13;
    t[first + 3].f = a21;
    t[first + 4].f = a22;
    t[first + 5].f = a23;
    t[first + 6].f = a31;
    t[first + 7].f = a32;
    t[first + 8].f = a33;
}

static void pumpWorldModels(void)
{
    /* Like the occam's [30] INT temp, one record reused for every object:
     * a field an object does not set keeps the last object's value. */
    static W temp[30];
    W *ftemp = temp;
    const float root_2 = 1.414213562f;
    int model;

    model = ChanInInt(image);
    if (model == 1) {                           /* 10 sphere selection */
        {
            const float rad = 102.0f, y = -210.0f, z = 3910.0f;
            const float x0 = -210.0f, x1 = 3.0f, x2 = 210.0f;

            sphere(temp, 0, 0.7f, 0.1f, 0.1f, 0.9f, rad, x0, y, z);
            sendObject(temp, s_size);
            sphere(temp, a_spec, 0.2f, 0.3f, 0.4f, 0.9f, rad, x1, y, z);
            ftemp[o_kg].f = 0.95f;
            sendObject(temp, s_size);
            sphere(temp, 0, 0.1f, 0.9f, 0.1f, 0.9f, rad, x2, y, z);
            sendObject(temp, s_size);
        }
        writes("\n\r sent 3 spheres");
        {
            const float rad = 102.0f, y = 5.0f, z = 3910.0f;
            const float x0 = -210.0f, x1 = 3.0f, x2 = 210.0f;

            sphere(temp, a_spec, 0.1f, 0.4f, 0.1f, 0.9f, rad, x0, y, z);
            ftemp[o_kg].f = 0.9f;
            sendObject(temp, s_size);
            sphere(temp, a_spec, 0.3f, 0.1f, 0.1f, 0.95f, rad, x1, y, z);
            ftemp[o_kg].f = 0.95f;
            sendObject(temp, s_size);
            sphere(temp, a_spec, 0.3f, 0.1f, 0.3f, 0.9f, rad, x2, y, z);
            ftemp[o_kg].f = 0.95f;
            sendObject(temp, s_size);
        }
        writes("\n\r sent 3 spheres");
        {
            const float rad = 102.0f, y = 210.0f, z = 3910.0f;
            const float x0 = -210.0f, x1 = 3.0f, x2 = 210.0f;

            sphere(temp, 0, 0.7f, 0.7f, 0.1f, 0.9f, rad, x0, y, z);
            sendObject(temp, s_size);
            sphere(temp, a_spec, 0.4f, 0.1f, 0.4f, 0.9f, rad, x1, y, z);
            ftemp[o_kg].f = 0.9f;
            sendObject(temp, s_size);
            sphere(temp, 0, 0.1f, 0.1f, 0.7f, 0.9f, rad, x2, y, z);
            sendObject(temp, s_size);
        }
        writes("\n\r sent 3 spheres");
        sphere(temp, a_spec | a_frac, 0.0f, 0.0f, 0.0f, 0.3f,
               100.0f, -90.0f, -80.0f, 3100.0f);
        ftemp[o_kg].f = 0.8f;
        ftemp[o_xmitR].f = 0.7f;
        ftemp[o_xmitG].f = 0.7f;
        ftemp[o_xmitB].f = 0.7f;
        ftemp[o_refix].f = 1.02f;
        temp[o_power].i = 10;
        sendObject(temp, s_size);
        writes("\n\r sent glass sphere");
    } else if (model == 2) {                    /* 14 sphere selection */
        const float r = 300.0f;
        const float r_2 = 2.0f * r;
        const float r_2_root_2 = 2.828427f * r;
        const float r_root_2 = 1.414213f * r;
        const float floor = r_2 - 1000.0f;
        const float xc = 0.0f, zc = 12000.0f;
        const float bdull[2] = { 0.5f, 0.04f };
        const float offs[2] = { -r, r };
        int ix, jx, i, j, m;

        for (ix = 0; ix < 3; ix++)
            for (jx = 0; jx < 3; jx++) {
                i = ix - 1;
                j = jx - 1;
                sphere(temp, a_spec, bdull[jx & 1], bdull[ix & 1],
                       bdull[ix & 4], 0.7f, r,
                       xc + (r_2 * (float)i), floor, zc + (r_2 * (float)j));
                ftemp[o_kg].f = 0.7f;
                sendObject(temp, s_size);
            }
        for (i = 0; i < 2; i++)
            for (j = 0; j < 2; j++) {
                static const float kd[4][3] = {
                    { 0.5f, 0.04f, 0.03f }, { 0.1f, 0.7f, 0.1f },
                    { 0.1f, 0.1f, 0.7f },   { 0.2f, 0.5f, 0.1f },
                };
                m = (i * 2) + j;
                sphere(temp, a_spec, kd[m][0], kd[m][1], kd[m][2], 0.7f, r,
                       xc + offs[i], floor + r_root_2, zc + offs[j]);
                ftemp[o_kg].f = 0.7f;
                sendObject(temp, s_size);
            }
        temp[o_type].i = o_ellipsoid;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.04f;
        ftemp[o_kdG].f = 0.04f;
        ftemp[o_kdB].f = 0.3f;
        ftemp[o_ks].f = 0.7f;
        ftemp[o_kg].f = 0.7f;
        ftemp[e_x].f = xc;
        ftemp[e_y].f = (floor + r_2_root_2) + 100.0f;
        ftemp[e_z].f = zc;
        matrix(temp, e_a11, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
               0.0f, 0.0f, 1.0f);
        ftemp[e_radx].f = r;
        ftemp[e_rady].f = r + 200.0f;
        ftemp[e_radz].f = r;
        sendObject(temp, e_size);
        writes("\n\r sent 14 spheres");
    } else if (model == 3) {                    /* cone and ellipsoid image */
        temp[o_type].i = o_cone;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.04f;
        ftemp[o_kdG].f = 0.04f;
        ftemp[o_kdB].f = 0.4f;
        ftemp[o_ks].f = 0.9f;
        ftemp[o_kg].f = 0.95f;
        ftemp[co_x].f = 300.0f;
        ftemp[co_y].f = 0.0f;
        ftemp[co_z].f = 12000.0f;
        matrix(temp, co_a11, 0.3f * root_2, 0.4f * root_2, 0.5f * root_2,
               -0.8f, 0.6f, 0.0f,
               (-0.3f) * root_2, (-0.4f) * root_2, 0.5f * root_2);
        ftemp[co_rada].f = 200.0f;
        ftemp[co_radb].f = 300.0f;
        ftemp[co_len].f = 800.0f;
        sendObject(temp, co_size);

        temp[o_type].i = o_ellipsoid;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.7f;
        ftemp[o_kdG].f = 0.03f;
        ftemp[o_kdB].f = 0.03f;
        ftemp[o_ks].f = 0.9f;
        ftemp[o_kg].f = 0.95f;
        ftemp[e_x].f = -200.0f;
        ftemp[e_y].f = 500.0f;
        ftemp[e_z].f = 13000.0f;
        matrix(temp, e_a11, 0.6f, 0.8f, 0.0f, -0.8f, 0.6f, 0.0f,
               0.0f, 0.0f, 1.0f);
        ftemp[e_radx].f = 100.0f;
        ftemp[e_rady].f = 500.0f;
        ftemp[e_radz].f = 100.0f;
        sendObject(temp, e_size);

        temp[o_type].i = o_plane;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.03f;
        ftemp[o_kdG].f = 0.4f;
        ftemp[o_kdB].f = 0.03f;
        ftemp[o_ks].f = 0.7f;
        ftemp[o_kg].f = 0.2f;
        ftemp[p_x].f = -700.0f;
        ftemp[p_y].f = -500.0f;
        ftemp[p_z].f = 13000.0f;
        ftemp[p_ux].f = -0.6470f;       /* all the three vectors */
        ftemp[p_uy].f = 0.7548f;        /* must be normalized. */
        ftemp[p_uz].f = 0.1078f;
        ftemp[p_vx].f = 0.6470f;
        ftemp[p_vy].f = 0.1078f;
        ftemp[p_vz].f = 0.7548f;
        ftemp[p_wx].f = 0.5774f;
        ftemp[p_wy].f = 0.5773f;
        ftemp[p_wz].f = -0.5773f;
        ftemp[p_sizeu].f = 1500.0f;
        ftemp[p_sizev].f = 1500.0f;
        sendObject(temp, p_size);
        writes("\n\r sent a Cone and an Ellipsoid");
    } else if (model == 4) {                    /* cylinder and sphere image */
        temp[o_type].i = o_cylinder;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.3f;
        ftemp[o_kdG].f = 0.3f;
        ftemp[o_kdB].f = 0.03f;
        ftemp[o_ks].f = 0.9f;
        ftemp[o_kg].f = 0.95f;
        ftemp[c_x].f = 0.0f;
        ftemp[c_y].f = -500.0f;
        ftemp[c_z].f = 10900.0f;
        matrix(temp, c_a11, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 2.5f,
               0.0f, 1.0f, -0.6f);
        ftemp[c_rad].f = 350.0f;
        ftemp[c_len].f = 800.0f;
        sendObject(temp, c_size);

        sphere(temp, 0, 0.7f, 0.1f, 0.1f, 0.9f,
               80.0f, -310.0f, 690.0f, 11300.0f);
        sendObject(temp, s_size);

        temp[o_type].i = o_xyplane;
        temp[o_attr].i = a_spec;
        ftemp[o_kdR].f = 0.5f;
        ftemp[o_kdG].f = 0.0f;
        ftemp[o_kdB].f = 0.5f;
        ftemp[o_ks].f = 0.9f;
        ftemp[o_kg].f = 0.95f;
        ftemp[pxy_x].f = -1000.0f;
        ftemp[pxy_y].f = -1000.0f;
        ftemp[pxy_z].f = 14000.0f;
        ftemp[pxy_sizex].f = 2000.0f;
        ftemp[pxy_sizey].f = 1000.0f;
        sendObject(temp, pxy_size);
        writes("\n\r sent a Cylinder and a Sphere");
    }
    if (model != 1) {
        temp[o_type].i = o_xzplane;
        temp[o_attr].i = (a_spec | a_bound0) | a_bound1;   /* mirrored plane */
        ftemp[o_kdR].f = 0.05f;
        ftemp[o_kdG].f = 0.4f;
        ftemp[o_kdB].f = 0.05f;
        ftemp[o_ks].f = 0.7f;
        ftemp[o_kg].f = 0.1f;
        ftemp[pxz_x].f = -2000.0f;
        ftemp[pxz_y].f = -690.0f;
        ftemp[pxz_z].f = 10000.0f;
        ftemp[pxz_sizex].f = 4000.0f;
        ftemp[pxz_sizez].f = 3000.0f;
        sendObject(temp, pxz_size);
        writes("\n\r a plane");
    }

    ftemp[l_ir].f = 780.8f;             /* a yellow sun */
    ftemp[l_ig].f = 780.8f;
    ftemp[l_ib].f = 710.7f;
    ftemp[l_dx].f = 1.7f;
    ftemp[l_dy].f = 1.6f;
    ftemp[l_dz].f = -0.9f;
    ChanOutInt(out, c_light);
    ChanOutInt(out, l_size);
    ChanOutWords(out, temp, l_size);
    writes("\n\r a light");
    ftemp[l_ir].f = 780.8f;             /* a yellow sun */
    ftemp[l_ig].f = 780.8f;
    ftemp[l_ib].f = 710.7f;
    ftemp[l_dx].f = -1.1f;
    ftemp[l_dy].f = 1.8f;
    ftemp[l_dz].f = -0.3f;
    ChanOutInt(out, c_light);
    ChanOutInt(out, l_size);
    ChanOutWords(out, temp, l_size);
    writes("\n\r a light");

    {
        const float offX = 0.0f, offY = 0.0f, offZ = 0.0f;

        renderingMode = m_adaptive;
        runPatchSize = patchEdge;
        scaleFactor = 1;

        ambient(0) = 266.0f;
        ambient(1) = 266.0f;
        ambient(2) = 280.0f;

        screenOrg(0) = offX - 256.0f;
        screenOrg(1) = offY - 256.0f;
        screenOrg(2) = offZ + 0.0f;

        screendX(0) = 1.0f;
        screendX(1) = 0.0f;
        screendX(2) = 0.0f;

        screendY(0) = 0.0f;
        screendY(1) = 1.0f;             /* cosTheta */
        screendY(2) = 0.0f;             /* sinTheta */

        pinhole(0) = offX + 0.0f;
        pinhole(1) = offY + 0.0f;
        pinhole(2) = offZ + 1024.0f;

        ChanOutInt(out, c_runData);
        ChanOutInt(out, runDataSize);
        ChanOutWords(out, runData, runDataSize);
    }
    writes("\n\r run data");
}

static void sendPatches(void)
{
    const int patchesWide = RES / patchEdge;
    int x, y, xy[2];

    for (x = 0; x < patchesWide; x++)
        for (y = 0; y < patchesWide; y++) {
            xy[0] = (x * patchEdge) + 128;
            xy[1] = (y * patchEdge) + 128;
            ChanOutInt(&a, c_render);
            ChanOutWords(&a, xy, 2);
        }
    ChanOutInt(&a, c_stop);
}

/* takes over outputBuffer's half: asks for a job, sends it down */
static void jobSender(void)
{
    int j[3];

    do {
        ChanOutInt(&ready, 0);
        ChanInWords(&job, j, 3);
        ChanOutInt(out, j[0]);
        if (j[0] == c_render)
            ChanOutWords(out, j + 1, 2);
    } while (j[0] != c_stop);
}

static void returnBuffer(void)
{
    int command, size;

    command = ChanInInt(back);
    while (command != c_stop) {
        if (command == c_patch) {
            ChanInWords(back, returnHdr, 3);
            size = returnHdr[2];
            ChanInWords(back, returnPatch, size * size);
            ChanOutInt(&c, command);
            ChanOutWords(&c, returnHdr, 3);
            ChanOutWords(&c, returnPatch, size * size);
        } else if (command == c_message) {
            size = ChanInInt(back);
            if (size)
                ChanIn(back, returnChars, size);
            ChanOutInt(&c, command);
            ChanOutInt(&c, size);
            if (size)
                ChanOut(&c, returnChars, size);
        } else
            __stopp();
        command = ChanInInt(back);
    }
    ChanOutInt(&c, command);
}

static void loadBalance(void)
{
    static int q[QMAX][3];
    int qhead = 0, qcount = 0;
    int passActive = 1, farmActive = 1;
    int free = worker * 2;
    int command, size, xy[2];
    Channel *chans[3];
    int guards[3];

    chans[0] = &a;
    chans[1] = &ready;
    chans[2] = &c;
    while (passActive || farmActive) {
        guards[0] = passActive && free > 0;
        guards[1] = qcount > 0;
        guards[2] = 1;
        switch (ChanAltGuarded(chans, guards, 3)) {
        case 0:
            command = ChanInInt(&a);
            if (command == c_render) {
#if CHATTY
                writes("\n\r S c.render to system");
#endif
                ChanInWords(&a, xy, 2);
                q[(qhead + qcount) % QMAX][0] = command;
                q[(qhead + qcount) % QMAX][1] = xy[0];
                q[(qhead + qcount) % QMAX][2] = xy[1];
                qcount++;
                free--;
            } else if (command == c_stop) {
                writes("\n\r R c.stop from pumper");
                q[(qhead + qcount) % QMAX][0] = c_stop;
                qcount++;
                passActive = 0;
            }
            break;
        case 1:
            ChanInInt(&ready);
            ChanOutWords(&job, q[qhead], 3);
            qhead = (qhead + 1) % QMAX;
            qcount--;
            break;
        case 2:
            command = ChanInInt(&c);
            if (command == c_patch) {
#if CHATTY
                writes("\n\r R patch");
#endif
                ChanInWords(&c, balanceHdr, 3);
                size = balanceHdr[2];
                ChanInWords(&c, balancePatch, size * size);
                ChanOutInt(graphics, command);
                ChanOutWords(graphics, balanceHdr, 3);
                ChanOutWords(graphics, balancePatch, size * size);
                free++;
            } else if (command == c_message) {
                size = ChanInInt(&c);
                if (size)
                    ChanIn(&c, balanceChars, size);
                ChanOutInt(graphics, command);
                ChanOutInt(graphics, size);
                if (size)
                    ChanOut(graphics, balanceChars, size);
            } else if (command == c_stop) {
                farmActive = 0;
                writes("\n\r received c.stop");
            }
            break;
        }
    }
    ChanOutInt(graphics, c_stop);
}

/* The occam runs WHILE TRUE, but frameBuffer exits after one picture, so
 * a second pass would only ever wait on `image`. */
static void control(void)
{
    pumpWorldModels();
    startProcess(sendPatches, sendWs, sizeof sendWs, PRI_HIGH);
    startProcess(jobSender, senderWs, sizeof senderWs, PRI_HIGH);
    startProcess(returnBuffer, returnWs, sizeof returnWs, PRI_HIGH);
    loadBalance();
}

static int controlWs[1024];

void controlSystem(Channel *out_, Channel *back_, Channel *graphics_,
                   Channel *image_, int worker_)
{
    out = out_;
    back = back_;
    graphics = graphics_;
    image = image_;
    worker = worker_;
    startProcess(control, controlWs, sizeof controlWs, PRI_HIGH);
}
