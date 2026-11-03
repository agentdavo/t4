/*
 * raytrace.h - the INMOS ray tracer in C: what occ/raytrace.inc held,
 * plus the build options and the pieces the three "processes" share.
 *
 * The occam names are kept, with '.' spelt '_' (n.dx -> n_dx), so the C can
 * be read against occ/*.occ line by line.  Where the occam RETYPES an INT
 * vector as REAL32 the C uses the union W and picks .i or .f.
 */
#ifndef RAYTRACE_H
#define RAYTRACE_H

#include <conc.h>

/* ------------------------------------------------------------------ */
/* Build options                                                       */
/* ------------------------------------------------------------------ */

/* Picture size: RES x RES.  The scene and the camera are those of the
 * occam original whatever RES is - the 256 x 256 screen is sampled RES/256
 * times as finely - so RES=256 renders the original picture, bit for bit
 * (pic/rayM.ppm). */
#ifndef RES
#define RES 2048
#endif

/* Pixels per job edge (cntlsys.occ's patchEdge).  It changes how the work
 * is cut up, never the picture: a sample depends only on where it is. */
#ifndef PATCH
#if RES >= 1024
#define PATCH 16
#else
#define PATCH 8
#endif
#endif

/* 1: print what the occam printed for every job ("S c.render to system",
 * "R patch"); two lines a patch is 131072 lines at 2048 x 2048. */
#ifndef CHATTY
#define CHATTY 0
#endif

#if (RES % PATCH) != 0 || PATCH > 16 || (RES % 256) != 0
#error "RES must be a multiple of 256 and of PATCH, and PATCH at most 16"
#endif

/* ------------------------------------------------------------------ */
/* Commands protocol                                                   */
/* ------------------------------------------------------------------ */

/* Every message is a tag word followed by its fields, each an output of
 * its own, and the receiver inputs them the same way - which a soft
 * channel requires and a link does not care about:
 *
 *   c_stop
 *   c_render   {x0, y0}
 *   c_object   size  [size words]
 *   c_light    size  [size words]
 *   c_patch    {x, y, size}  [size*size words]   (occam: size rows)
 *   c_runData  size  [size words]
 *   c_message  size  [size bytes]                (occam: size words)
 */
#define c_stop     0
#define c_render   1
#define c_object   2
#define c_light    3
#define c_patch    4
#define c_runData  5
#define c_message  6

/* rendering modes */
#define m_adaptive    0
#define m_stochastic  1
#define m_dumb        2
#define m_test        3

/* ------------------------------------------------------------------ */
/* Shade tree                                                          */
/* ------------------------------------------------------------------ */

#define nil  (-1)

#define n_reflect  0        /* point to children */
#define n_refract  1
#define n_next     2        /* maintain linked list of leaf nodes */
#define n_type     3        /* reflected, refracted or primary ray */

#define rt_root  0
#define rt_spec  1
#define rt_frac  2

#define n_objptr   4        /* pointer into world model for intersect */
#define n_sectx    5        /* point of intersection */
#define n_secty    6
#define n_sectz    7
#define n_normx    8        /* surface normal at point of intersection */
#define n_normy    9
#define n_normz   10
#define n_t       11        /* distance parameter 't' of intersection */
#define n_startx  12        /* ray origin */
#define n_starty  13
#define n_startz  14
#define n_dx      15        /* ray direction */
#define n_dy      16
#define n_dz      17
#define n_red     18
#define n_green   19
#define n_blue    20

#define nodeSize  (n_blue + 1)

#define maxDepth  6
#define maxNodes  (4 + (2 << maxDepth))
#define treeSize  (nodeSize * maxNodes)

/* ------------------------------------------------------------------ */
/* Objects                                                             */
/* ------------------------------------------------------------------ */

#define o_sphere     0
#define o_xyplane    1
#define o_xzplane    2
#define o_yzplane    3
#define o_plane      4
#define o_cone       5
#define o_cylinder   6
#define o_ellipsoid  7
#define o_quadric    8

#define a_spec    0x01      /* use lambert / use phong and reflect */
#define a_frac    0x02      /* refract on surface */
#define a_bound0  0x04
#define a_bound1  0x08
#define a_tex     0x10
#define a_bump    0x20

#define o_type      0
#define o_attr      1
#define o_kdR       2       /* coeffs of diffuse reflection R G B */
#define o_kdG       3
#define o_kdB       4
#define o_ks        5       /* coeff of specular reflection */
#define o_kg        6       /* coeff of gloss for phong shading */
#define o_xmitR     7       /* coeffs of transmission R G B */
#define o_xmitG     8
#define o_xmitB     9
#define o_power    10
#define o_refix    11       /* refractive index (relative) */
#define o_specific 12

/* sphere */
#define s_rad   (o_specific + 0)
#define s_x     (o_specific + 1)
#define s_y     (o_specific + 2)
#define s_z     (o_specific + 3)
#define s_size  (o_specific + 7)

/* ellipsoid: centre, global->local matrix, radii */
#define e_x     (o_specific + 0)
#define e_y     (o_specific + 1)
#define e_z     (o_specific + 2)
#define e_a11   (o_specific + 3)
#define e_radx  (o_specific + 12)
#define e_rady  (o_specific + 13)
#define e_radz  (o_specific + 14)
#define e_size  (o_specific + 15)

/* cone: centre of the first circle, matrix, the two radii, length */
#define co_x     (o_specific + 0)
#define co_y     (o_specific + 1)
#define co_z     (o_specific + 2)
#define co_a11   (o_specific + 3)
#define co_rada  (o_specific + 12)
#define co_radb  (o_specific + 13)
#define co_len   (o_specific + 14)
#define co_size  (o_specific + 15)

/* cylinder */
#define c_x      (o_specific + 0)
#define c_y      (o_specific + 1)
#define c_z      (o_specific + 2)
#define c_a11    (o_specific + 3)
#define c_rad    (o_specific + 12)
#define c_len    (o_specific + 13)
#define c_size   (o_specific + 14)

/* general plane: origin, u v w axes, extent on u and v (the second
 * definition of p.* in raytrace.inc, which is the one in scope) */
#define p_x      (o_specific + 1)
#define p_y      (o_specific + 2)
#define p_z      (o_specific + 3)
#define p_ux     (o_specific + 4)
#define p_uy     (o_specific + 5)
#define p_uz     (o_specific + 6)
#define p_vx     (o_specific + 7)
#define p_vy     (o_specific + 8)
#define p_vz     (o_specific + 9)
#define p_wx     (o_specific + 10)
#define p_wy     (o_specific + 11)
#define p_wz     (o_specific + 12)
#define p_sizeu  (o_specific + 13)
#define p_sizev  (o_specific + 14)
#define p_size   (o_specific + 18)

/* axis-aligned planes */
#define pxy_x      (o_specific + 0)
#define pxy_y      (o_specific + 1)
#define pxy_z      (o_specific + 2)
#define pxy_sizex  (o_specific + 3)
#define pxy_sizey  (o_specific + 4)
#define pxy_size   (o_specific + 8)

#define pxz_x      (o_specific + 0)
#define pxz_y      (o_specific + 1)
#define pxz_z      (o_specific + 2)
#define pxz_sizex  (o_specific + 3)
#define pxz_sizez  (o_specific + 4)
#define pxz_size   (o_specific + 8)

#define pyz_x      (o_specific + 0)
#define pyz_y      (o_specific + 1)
#define pyz_z      (o_specific + 2)
#define pyz_sizey  (o_specific + 3)
#define pyz_sizez  (o_specific + 4)
#define pyz_size   (o_specific + 8)

#define sizeWorld  1800     /* a sphere = 17 words + nextPtr: 100 spheres */

/* lights: |nextPtr|r|g|b|dx|dy|dz */
#define l_ir   0
#define l_ig   1
#define l_ib   2
#define l_dx   3
#define l_dy   4
#define l_dz   5
#define l_size 6

#define sizeLight   7
#define maxLights  10
#define sizeLights (sizeLight * maxLights)

/* ------------------------------------------------------------------ */
/* Sub-pixel grid and colours                                          */
/* ------------------------------------------------------------------ */

#define maxDescend    2     /* space limit on the sub-pixel grid */
#define descendPower  4     /* 2 ^^ maxDescend */

#define patchSize  16
#define gridSize   (((patchSize + 1) * descendPower) + 1)

#define mint  ((int)0x80000000)

/* colours are nr | 10 blue bits | 10 green bits | 10 red bits | */
#define threshold   (10 << 2)   /* 10 on a scale of 256, in 10 bits */
#define colourBits  10
#define rMask       ((1 << colourBits) - 1)
#define gMask       (rMask << colourBits)
#define bMask       (gMask << colourBits)
#define notRendered mint        /* top bit set = not rendered */

/* runData[]: ambient RGB, mode, patch size, scale, screen origin, screen
 * dX, screen dY, pinhole - see cntlsys.c */
#define runDataSize 20

/* An INT that the occam RETYPES as a REAL32 when it suits it. */
typedef union { int i; float f; } W;

/* ------------------------------------------------------------------ */
/* The three parts of the program (raytrace.cfs's PROCESSORs)          */
/* ------------------------------------------------------------------ */

/* framebuf.c - node 0 only: talks to the host over link 0 */
void frameBuffer(Channel *in, Channel *image, int processors);

/* cntlsys.c - node 0 only: world model, job farm */
void controlSystem(Channel *out, Channel *back, Channel *graphics,
                   Channel *image, int worker);

/* raytrace.c - every node */
void rayTrace(Channel *fromPrev, Channel *toPrev, Channel *fromNext,
              Channel *toNext, int propogate, int me);

/* node.c: start func as a process at priority pri (PRI_HIGH/PRI_LOW) on
 * a workspace of wsize bytes */
void startProcess(void (*func)(void), void *ws, int wsize, int pri);

/* hostio.c - the INMOS server protocol over link 0 */
#define spid_stdout  1
#define spt_binary   1
#define spt_text     2
#define spm_output   2
#define spr_ok       0
#define sps_success  999999999
#define sps_failure  (-999999999)

int  so_open(const char *name, int type, int mode, int *id);
int  so_close(int id);
int  so_write(int id, const void *buf, int n);
int  so_puts(int id, const char *s);
int  so_getkey(int *key);
void so_exit(int status);
void so_write_string(const char *s);
void so_write_string_nl(const char *s);
void so_write_nl(void);
void so_write_int(int n, int width);
void so_write_real32(float x, int ip, int dp);

/* inlined by the compiler as `move`; there is no libc */
void *memcpy(void *dst, const void *src, unsigned n);

/* small helpers the processes share */
static inline void ChanOutWords(Channel *c, const void *w, int n)
{
    ChanOut(c, w, n * 4);
}

static inline void ChanInWords(Channel *c, void *w, int n)
{
    ChanIn(c, w, n * 4);
}

#endif /* RAYTRACE_H */
