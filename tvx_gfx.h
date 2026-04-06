/*
 * tvx_gfx.h - TVX-GFX: Floating-point Transform & Project group, OPR 0x1E2..0x1E6.
 *
 * Fixed-function-era geometry acceleration for the T9000, in the spirit of
 * the SH-4's FTRV/FSCA/FSRRA but reshaped for a 3-deep stack machine: all
 * operands are memory pointers carried in A/B/C (the TVX convention), state
 * that does not fit on the stack is latched by a set-instruction (the
 * VSETLEN/QSETFMT tradition), and clip results come back as a
 * Cohen-Sutherland outcode in AReg so that `cj` consumes them directly.
 *
 *   0x1E2 FTSETVP   latch viewport: A -> {sx, sy, ox, oy, guard} (5 x f32)
 *   0x1E3 FTRANSP   A=dst record, B=src xyz, C=4x4 matrix; AReg <- outcode
 *   0x1E4 FTRANSPB  A=dst records, B=src xyz array, C=4x4 matrix; the
 *                   vertex count comes from VSETLEN (the stack cannot
 *                   carry dst+src+mtx+count, same reason the TVX vector
 *                   kernels use it).  AReg <- AND of all outcodes
 *                   (nonzero = whole batch is outside one frustum plane,
 *                   i.e. trivially rejectable; an empty batch gives 0x3F)
 *   0x1E5 FSINCOS   A=dst {sin, cos} (2 x f32), B=16-bit BAMS angle; pop 2
 *   0x1E6 FRSQRT    AReg <- bits(1/sqrt(float(AReg)))  (bit domain, like
 *                   the TVX scalar conversions)
 *   0x1E7 FRECIP    AReg <- bits(1/float(AReg)), bit domain.  Runs on the
 *                   dedicated seed+NR reciprocal unit, i.e. NOT the FPU
 *                   divider - which is what lets FTRANSPB overlap vertex
 *                   n's perspective divide with vertex n+1's matrix MACs
 *   0x1E8 FSINCOSB  A=dst {sin, cos} pair array, B=u16 BAMS angle array,
 *                   VSETLEN angles; pop 2.  Amortises the flat dispatch
 *                   cost the scalar form cannot escape
 *   0x1E9 VSETSTRIDE latch: A = element stride in bytes (0 = dense),
 *                   B = stream mask (bit0 applies it to the A stream,
 *                   bit1 to the B stream); pop 2.  Sticky like VSETLEN.
 *                   Lets the block ops walk array-of-struct layouts
 *                   (padded vertex records, plane arrays) without a
 *                   pack/copy pass - the central lesson of scalable-
 *                   vector ports of AoS geometry code.  Honoured by the
 *                   TVX-GFX block ops (FTRANSPB, FSINCOSB, FTCULL, TSKIN);
 *                   the TVX quant ops are block-format and stay dense
 *   0x1EA FTCULL    A=dst codes, B=src xyz array (stride latch), C=param
 *                   {nplanes, mode, radius, xmask, planes[nplanes][4]};
 *                   VSETLEN verts.  mode 0: byte outcode, bit k =
 *                   signbit(dist to plane k), XOR xmask.  mode 1: dual
 *                   +-radius code (bit k / bit k+4), 4 planes max.
 *                   mode 2: f32 distances to plane 0.  AReg <- OR of all
 *                   emitted codes (0 = every vertex fully inside).
 *                   This is FTRANSP with divide/viewport gated off
 *   0x1EC VSETFMT   latch: A = source position format for the B stream
 *                   of FTRANSPB/FTCULL/FLITB (and the activation stream
 *                   of QGEMM): 0 = f32 (12-byte xyz), 1 = bf16, 2 = IEEE
 *                   f16 (both 6-byte xyz); pop 1.  Sticky like VSETLEN.
 *                   Runs the TVX stream converters in front of the MAC
 *                   array, halving vertex fetch traffic - the read-format
 *                   unification of the quant and geometry sides
 *   0x1ED FTSETLIT  latch directional light: A -> {Lx, Ly, Lz, ambient,
 *                   intensity} (5 x f32); pop 1.  L is used as given
 *                   (normalize it once per frame)
 *   0x1EE FLITB     A=dst records (writes word 5 = intensity I), B=src
 *                   normal array (VSETFMT/VSETSTRIDE apply), C=3x3
 *                   row-major normal matrix; VSETLEN vertices.
 *                   I = clamp(ambient + intensity * max(0, N'.L / |N'|))
 *                   with N' = M3*N; the renormalisation runs on the
 *                   FRSQRT unit and overlaps the next vertex's MACs.
 *                   AReg <- 1 iff every vertex came out ambient-only
 *                   (all dark), so `flitb; cj shade` skips dark meshes
 *   0x1EF QGEMM     whole-matrix quantised GEMV: A=dst f32[rows], B=
 *                   param {word0 = activation ptr, word1 = weight ptr},
 *                   C=blocks per row; rows from VSETLEN, weight format
 *                   from QSETFMT (Q8_0/MXFP4/IQx...), activation format
 *                   from VSETFMT (f32 or bf16).  Row r of the weights
 *                   starts at weights + r*nblocks*blockbytes; the
 *                   activations are re-streamed per row.  Per-block
 *                   partial-sum order matches QDOT exactly, so dst[r]
 *                   is bit-identical to a QDOT loop.  One dispatch per
 *                   matrix instead of one per row.  AReg <- rows
 *   0x1F0 VBLEND8   ARGB8888 alpha blend: A=dst pixels, B=src pixels,
 *                   C=control (0..255 = global alpha, >255 = use each
 *                   src pixel's own alpha byte); VSETLEN pixels.
 *                   Per channel: (s*a + d*(255-a)) with exact /255
 *                   rounding.  The classic 2D compositing op; sprite
 *                   copies with transparency stay on MOVE2DNONZERO
 *   0x1F1 VFILL     pattern fill: A=dst, B=32-bit pattern, C=byte count.
 *                   The byte written at address a is pattern byte (a&3),
 *                   so word-aligned colour fills reproduce the pattern
 *                   exactly and a replicated-byte pattern is memset.
 *                   `move` cannot substitute (overlap is undefined)
 *   0x1F2 VFILL2D   2D fill, fifth member of the move2d family: rows
 *                   from BMReg0 and dst stride from BMReg1 as latched by
 *                   move2dinit; A=dst, B=pattern, C=row width in bytes
 *   0x1F3 CSUM      16-bit ones-complement checksum (Internet sum):
 *                   A=src, B=initial 32-bit sum, C=byte count;
 *                   AReg <- folded 16-bit result.  Little-endian
 *                   halfword accumulation, odd trailing byte added as
 *                   the low byte, end-around-carry folded to 16 bits
 *   0x1F4 VBLEND2D  2D alpha blend on the move2d latches: rows from
 *                   BMReg0, dst/src strides from BMReg1/BMReg2, VSETLEN
 *                   pixels per row; A=dst, B=src, C=control as VBLEND8.
 *                   One dispatch per window instead of one per row; its
 *                   per-pixel alpha also subsumes move2dnonzero for
 *                   ARGB (whose transparency test is per BYTE and
 *                   part-copies pixels with one zero channel)
 *   0x1F6 CSUMCPY   fused copy + Internet checksum (the kernel's
 *                   csum_and_copy shape): A=dst, B=src, C=byte count;
 *                   copies the stream and returns the folded 16-bit sum
 *                   in AReg.  Combine partial sums by adding folded
 *                   values and folding again (ones-complement sums are
 *                   fold-invariant).  Reads and writes coalesce like
 *                   move; same cost as CSUM plus the write stream
 *   0x1F5 STVXSTATE store the sticky latch set to A (15 words: VSETLEN,
 *                   QSETFMT, VSETSTRIDE stride+mask, VSETFMT, viewport
 *                   5f, light 5f); pop 1.  The stmove2dinit precedent
 *                   (ISM 11.12.3): a trapping or timeslicing kernel
 *                   captures TVX state with this and restores it by
 *                   replaying the ordinary latch instructions on the
 *                   saved words (the layout matches their descriptors)
 *
 * Memory semantics of the store-side block ops (VFILL/VFILL2D/VBLEND8/
 * VBLEND2D writing a framebuffer): devmove rules (ISM 7.14) - accesses
 * are program-ordered and monotonically ascending, and the write stream
 * does NOT allocate cache lines (write-around), because a 1.9 MB frame
 * store must not churn a 16 KB cache.  All block ops also inherit vin's
 * contract: they never write outside the declared destination extent.
 *
 * Element granularity vs the 64-bit paths: the stream engines coalesce
 * every element size - 4-bit MXFP4 nibbles, 8-bit fill/blend bytes,
 * 16-bit bf16/f16/CSUM halfwords, 32-bit words, 64-bit record pairs -
 * into aligned 64-bit read/write transactions, using the cache's
 * per-byte write enables at the ragged edges (HRM 9.1: part-word write
 * hits carry no read-modify-write penalty).  Peak bandwidth is
 * therefore independent of element width; only total bytes matter,
 * which is what the cycle model charges.
 *   0x1EB TSKIN     A=dst xyz (stride latch), B=joint matrix base (3x4
 *                   row-major floats each), C=param {mode, weights ptr
 *                   (vec4 stream), indexpair ptr (byte-offset, endflag
 *                   words), minmax[6] inout}; VSETLEN verts.  Skeletal
 *                   skinning: each vertex consumes >= 1 weighted joint
 *                   (endflag != 0 marks its last weight).  mode bit0
 *                   fuses a bounds min/max pass over the results.
 *                   AReg <- total weights consumed
 *
 * Destination vertex record is architecturally 8 words (power of two, so
 * record indexing is `shl 3; wsub` - a stage-2 address calc - instead of a
 * stage-4 multiply):
 *
 *   word 0  Xs      screen x            = (Xc/Wc)*sx + ox
 *   word 1  Ys      screen y            = (Yc/Wc)*sy + oy
 *   word 2  Zn      NDC depth           = Zc/Wc
 *   word 3  invW    1/Wc                (perspective-correct interpolation)
 *   word 4  outcode guard-band Cohen-Sutherland bits (integer)
 *   word 5..7       reserved for the lighting group (I, u, v)
 *
 * 64-bit store pairing: the record layout is chosen so results retire
 * through a 64-bit write path - {Xs,Ys} and {Zn,1/Wc} are two paired
 * 64-bit stores and the outcode a 32-bit store (3 transactions/vertex
 * instead of 5), and FSINCOSB's {sin,cos} pair is one 64-bit store.
 * Destination bases must therefore be 8-byte aligned; the 8-word record
 * stride preserves alignment across the batch (VSETSTRIDE users take on
 * the alignment obligation themselves - an unaligned or odd-word stride
 * may degrade pairs to single-word writes but must not change results).
 *
 * Interruptibility: FTRANSPB runs for an unbounded, VSETLEN-controlled
 * time, so like `move`/`move2d`/`fprem` (ISM Table 8.5) it is
 * architecturally interruptible and restartable: progress state is the
 * current vertex index alongside the latched count, the integer stack is
 * undefined after execution, and the latched FTSETVP/VSETLEN state is
 * undefined across descheduling (re-latch per batch, the move2dinit
 * tradition).  The emulator currently executes the loop atomically, the
 * same simplification the TVX GEMV loop makes.
 *
 * Like tvx.h, everything here is a pure header-only function with no
 * dependencies beyond stdint.  These kernels ARE the architectural spec:
 * the emulator, tccgen constant folding and any future silicon must agree
 * bit-exactly, which is why evaluation order is fixed and the two
 * approximate operations (FSINCOS, FRSQRT) are defined as an exact
 * algorithm (Cephes-style minimax polynomials; magic-seed Newton-Raphson)
 * rather than "an approximation of" the true function.
 */

#ifndef TVX_GFX_H
#define TVX_GFX_H

#include <stdint.h>

/* ---- outcode bits ----------------------------------------------------- */
#define TVX_CLIP_LEFT   0x01u   /* Xc < -g*Wc */
#define TVX_CLIP_RIGHT  0x02u   /* Xc >  g*Wc */
#define TVX_CLIP_BOTTOM 0x04u   /* Yc < -g*Wc */
#define TVX_CLIP_TOP    0x08u   /* Yc >  g*Wc */
#define TVX_CLIP_NEAR   0x10u   /* Zc < -g*Wc, or Wc too small to project */
#define TVX_CLIP_FAR    0x20u   /* Zc >  g*Wc */

#define TVX_RECORD_WORDS 8      /* architectural destination stride       */
#define TVX_W_EPS 1e-6f         /* Wc below this cannot be projected      */

typedef struct {
    float sx, sy;               /* viewport scale                         */
    float ox, oy;               /* viewport offset                        */
    float guard;                /* guard band factor, >= 1.0              */
} tvx_viewport;

/* FTSETVP semantic: load the five descriptor floats.  A guard band that is
 * not usable (<= 1.0, NaN) is clamped to exactly 1.0 = the true frustum. */
static inline void tvx_viewport_set(tvx_viewport *vp, const float d[5])
{
    vp->sx = d[0];
    vp->sy = d[1];
    vp->ox = d[2];
    vp->oy = d[3];
    vp->guard = (d[4] > 1.0f) ? d[4] : 1.0f;
}

/* FTRANSP semantic: transform one vertex (x, y, z, implicit w=1) by the
 * row-major 4x4 matrix m, clip-test in clip space (before the divide, so
 * a negative Wc cannot flip the comparisons), then project and viewport-
 * scale.  Writes dst[0..3] = {Xs, Ys, Zn, invW}; returns the outcode.
 * When Wc is too small to project, the screen words are forced to zero
 * and TVX_CLIP_NEAR is set, so the result is total and deterministic. */
static inline uint32_t tvx_transp(float dst[4], const float src[3],
                                  const float m[16], const tvx_viewport *vp)
{
    float x = src[0], y = src[1], z = src[2];
    float xc = m[0]*x  + m[1]*y  + m[2]*z  + m[3];
    float yc = m[4]*x  + m[5]*y  + m[6]*z  + m[7];
    float zc = m[8]*x  + m[9]*y  + m[10]*z + m[11];
    float wc = m[12]*x + m[13]*y + m[14]*z + m[15];
    float gw = vp->guard * wc;
    uint32_t code = 0;

    if (xc < -gw) code |= TVX_CLIP_LEFT;
    if (xc >  gw) code |= TVX_CLIP_RIGHT;
    if (yc < -gw) code |= TVX_CLIP_BOTTOM;
    if (yc >  gw) code |= TVX_CLIP_TOP;
    if (zc < -gw) code |= TVX_CLIP_NEAR;
    if (zc >  gw) code |= TVX_CLIP_FAR;

    if (wc >= TVX_W_EPS) {
        float invw = 1.0f / wc;         /* IEEE divide: the FPU has one   */
        dst[0] = (xc * invw) * vp->sx + vp->ox;
        dst[1] = (yc * invw) * vp->sy + vp->oy;
        dst[2] = zc * invw;
        dst[3] = invw;
    } else {
        code |= TVX_CLIP_NEAR;
        dst[0] = 0.0f;
        dst[1] = 0.0f;
        dst[2] = 0.0f;
        dst[3] = 0.0f;
    }
    return code;
}

/* ---- FSINCOS ---------------------------------------------------------- */
/* The angle is 16-bit BAMS fixed point: 0 -> 0 rad, 0x10000 -> 2*pi, so
 * the argument wraps for free (the range-reduction that dominates a
 * software sin) and the octant/quadrant split is a shift and a mask.
 * Bits above 15 are ignored.  Polynomials are the Cephes sinf/cosf
 * minimax pairs on [0, pi/4]; absolute error is < 1.2e-7, comfortably
 * inside the SH-4 FSCA's 2^-21 spec. */

#define TVX_BAMS_TO_RAD 9.58737992e-05f       /* pi/32768               */

static inline float tvx_sinpoly_(float x)
{
    float z = x * x;
    return x * (1.0f + z * (-1.6666654611e-1f
                    + z * (8.3321608736e-3f
                    + z * (-1.9515295891e-4f))));
}

static inline float tvx_cospoly_(float x)
{
    float z = x * x;
    return 1.0f - 0.5f * z
                + z * z * (4.166664568298827e-2f
                    + z * (-1.388731625493765e-3f
                    + z * (2.443315711809948e-5f)));
}

static inline void tvx_sincos(uint32_t bams, float *s, float *c)
{
    uint32_t a = bams & 0xFFFFu;
    uint32_t q = (a >> 14) & 3u;          /* quadrant                     */
    uint32_t m = a & 0x3FFFu;             /* position within the quadrant */
    float x, sq, cq;

    if (m < 8192u) {                      /* first octant of the quadrant */
        x  = (float)m * TVX_BAMS_TO_RAD;
        sq = tvx_sinpoly_(x);
        cq = tvx_cospoly_(x);
    } else {                              /* fold about pi/4              */
        x  = (float)(16384u - m) * TVX_BAMS_TO_RAD;
        sq = tvx_cospoly_(x);
        cq = tvx_sinpoly_(x);
    }
    switch (q) {
    case 0:  *s =  sq; *c =  cq; break;
    case 1:  *s =  cq; *c = -sq; break;
    case 2:  *s = -sq; *c = -cq; break;
    default: *s = -cq; *c =  sq; break;
    }
}

/* ---- FRSQRT ----------------------------------------------------------- */
/* Bit-domain reciprocal square root: magic-constant seed plus three
 * Newton-Raphson steps.  The magic seed is only ~4.5 bits good, so two
 * steps stop at 2^-17.6; the third reaches near-single precision
 * (measured max rel err 2^-22.4 over all normals), past the SH-4
 * FSRRA's 2^-21.  The seed constant and the evaluation order are
 * normative.  Domain edges are made total: zero, denormal or negative
 * input returns the canonical qNaN, +inf returns +0.0, NaN propagates
 * as the canonical qNaN. */
static inline uint32_t tvx_rsqrt_bits(uint32_t bits)
{
    union { float f; uint32_t u; } v;
    float xhalf;

    if (bits & 0x80000000u)        return 0x7FC00000u;  /* negative/-0    */
    if ((bits & 0x7F800000u) == 0) return 0x7FC00000u;  /* zero/denormal  */
    if (bits == 0x7F800000u)       return 0;            /* +inf -> +0.0   */
    if (bits > 0x7F800000u)        return 0x7FC00000u;  /* NaN            */

    v.u = bits;
    xhalf = 0.5f * v.f;
    v.u = 0x5F37642Fu - (v.u >> 1);
    v.f = v.f * (1.5f - xhalf * v.f * v.f);
    v.f = v.f * (1.5f - xhalf * v.f * v.f);
    v.f = v.f * (1.5f - xhalf * v.f * v.f);
    return v.u;
}

/* ---- FRECIP ----------------------------------------------------------- */
/* Bit-domain reciprocal: magic-constant seed plus three Newton-Raphson
 * steps y <- y*(2 - x*y) (the seed is only ~4 bits good, so the third
 * step is what reaches single precision).  Sign is handled outside the
 * iteration.  Totalised edges: +-0 and denormals return the signed
 * infinity, |x| >= 2^126 (reciprocal would be subnormal) returns the
 * signed zero, infinities return the signed zero, NaN returns the
 * canonical qNaN. */
static inline uint32_t tvx_recip_bits(uint32_t bits)
{
    union { float f; uint32_t u; } v;
    uint32_t sign = bits & 0x80000000u;
    uint32_t mag  = bits & 0x7FFFFFFFu;
    float x;

    if ((mag & 0x7F800000u) == 0) return sign | 0x7F800000u; /* 0, denorm */
    if (mag >= 0x7F800000u)
        return (mag == 0x7F800000u) ? sign : 0x7FC00000u;    /* inf, NaN  */
    if (mag >= 0x7E800000u)       return sign;               /* underflow */

    v.u = mag;
    x = v.f;
    v.u = 0x7EF127EAu - mag;
    v.f = v.f * (2.0f - x * v.f);
    v.f = v.f * (2.0f - x * v.f);
    v.f = v.f * (2.0f - x * v.f);
    return sign | v.u;
}

/* ---- FTCULL ----------------------------------------------------------- */
/* Signed distance to a plane {a,b,c,d}, strictly left-to-right. */
static inline float tvx_plane_dist(const float p[4], const float v[3])
{
    return p[0]*v[0] + p[1]*v[1] + p[2]*v[2] + p[3];
}

/* The sign is the raw IEEE bit 31, so -0.0 counts as outside and a NaN
 * distance takes its payload sign - a compare against 0.0f is NOT a
 * conforming implementation. */
static inline uint32_t tvx_signbit(float f)
{
    union { float f; uint32_t u; } v;
    v.f = f;
    return v.u >> 31;
}

static inline uint32_t tvx_cull_code(const float *planes, uint32_t nplanes,
                                     const float v[3], uint32_t mode,
                                     float radius, uint32_t xmask)
{
    uint32_t k, bits = 0;

    if (mode == 1) {
        for (k = 0; k < nplanes && k < 4; k++) {
            float d = tvx_plane_dist(planes + 4*k, v);
            bits |= tvx_signbit(d + radius) << k;
            bits |= tvx_signbit(d - radius) << (k + 4);
        }
    } else {
        for (k = 0; k < nplanes && k < 6; k++)
            bits |= tvx_signbit(tvx_plane_dist(planes + 4*k, v)) << k;
    }
    return (bits ^ xmask) & 0xFFu;
}

/* ---- FTSETLIT / FLITB ------------------------------------------------- */
typedef struct {
    float dir[3];               /* light direction, as given             */
    float ambient;              /* floor intensity                       */
    float intensity;            /* diffuse scale                        */
} tvx_light;

static inline void tvx_light_set(tvx_light *L, const float d[5])
{
    L->dir[0] = d[0];
    L->dir[1] = d[1];
    L->dir[2] = d[2];
    L->ambient = d[3];
    L->intensity = d[4];
}

/* One vertex of FLITB: rotate the (possibly unnormalised) normal by the
 * 3x3 matrix, renormalise on the FRSQRT unit, clamp the diffuse term at
 * zero (back faces), clamp the result to [0,1].  A degenerate normal
 * lights as ambient-only rather than NaN. */
static inline float tvx_lit_vertex(const float n[3], const float m9[9],
                                   const tvx_light *L)
{
    float nx = m9[0]*n[0] + m9[1]*n[1] + m9[2]*n[2];
    float ny = m9[3]*n[0] + m9[4]*n[1] + m9[5]*n[2];
    float nz = m9[6]*n[0] + m9[7]*n[1] + m9[8]*n[2];
    float nn = nx*nx + ny*ny + nz*nz;
    float d  = nx*L->dir[0] + ny*L->dir[1] + nz*L->dir[2];
    float I;

    if (nn <= 1e-12f) {
        d = 0.0f;
    } else {
        union { float f; uint32_t u; } r;
        r.f = nn;
        r.u = tvx_rsqrt_bits(r.u);
        d = d * r.f;
    }
    if (d < 0.0f) d = 0.0f;
    I = L->ambient + L->intensity * d;
    if (I > 1.0f) I = 1.0f;
    if (I < 0.0f) I = 0.0f;
    return I;
}

/* ---- VBLEND8 ---------------------------------------------------------- */
/* ARGB8888 blend of one pixel.  ctl <= 255 is a global alpha; anything
 * larger selects the src pixel's own alpha byte.  The (t + (t>>8)) >> 8
 * form is the exact-rounding /255, so 255 is a true opaque copy and 0 a
 * true no-op - both ends are lossless, which a plain >>8 gets wrong. */
static inline uint32_t tvx_blend_pixel(uint32_t d, uint32_t s, uint32_t ctl)
{
    uint32_t a  = (ctl > 0xFFu) ? (s >> 24) & 0xFFu : ctl;
    uint32_t na = 255u - a;
    uint32_t out = 0, sh;

    for (sh = 0; sh < 32; sh += 8) {
        uint32_t sc = (s >> sh) & 0xFFu;
        uint32_t dc = (d >> sh) & 0xFFu;
        uint32_t t = sc * a + dc * na + 128u;
        out |= (((t + (t >> 8)) >> 8) & 0xFFu) << sh;
    }
    return out;
}

/* ---- VFILL / CSUM ----------------------------------------------------- */
/* The byte VFILL writes at address a. */
static inline uint32_t tvx_fill_byte(uint32_t pattern, uint32_t a)
{
    return (pattern >> ((a & 3u) * 8u)) & 0xFFu;
}

/* End-around-carry fold of a ones-complement accumulator to 16 bits.
 * Two folds suffice for any 32-bit value. */
static inline uint32_t tvx_csum_fold(uint32_t acc)
{
    acc = (acc & 0xFFFFu) + (acc >> 16);
    acc = (acc & 0xFFFFu) + (acc >> 16);
    return acc;
}

/* ---- TSKIN ------------------------------------------------------------ */
/* One weighted-joint accumulation: acc += M * (wx,wy,wz,ww) with M a 3x4
 * row-major matrix.  The accumulator starts at +0.0 and the accumulation
 * order across a vertex's weights is normative (f32 sums are order-
 * sensitive). */
static inline void tvx_skin_acc(float acc[3], const float m[12],
                                const float w[4])
{
    acc[0] += m[0]*w[0] + m[1]*w[1] + m[2]*w[2]  + m[3]*w[3];
    acc[1] += m[4]*w[0] + m[5]*w[1] + m[6]*w[2]  + m[7]*w[3];
    acc[2] += m[8]*w[0] + m[9]*w[1] + m[10]*w[2] + m[11]*w[3];
}

#endif /* TVX_GFX_H */
