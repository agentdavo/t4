# occam vs TinyCC: the same ray tracer, instruction by instruction

The C port renders the occam's pictures bit for bit, tracing the same
405061 rays for model 3, so any difference in speed is in the code.  It is
also the strictest test of a compiler change there is: a float operation
reordered or a value lost changes the picture.  Measured on t4, one node
(`-st -sn -1`), model 3, 256 x 256, instructions from t4's `heartbeat` at
exit (load-independent; t4's clock is the host's, so times are not):

| step | instructions | vs occam |
|------|-------------:|---------:|
| occam (`../bin/raytrace1.btl`) | 763 M | 1.00x |
| C as first ported, TinyCC HEAD 4a446dbd | 1813 M | 2.38x |
| + three fixes to the port (programming) | 1573 M | 2.06x |
| + FP results kept in FA, constant subscripts folded (compiler) | 1267 M | 1.66x |
| + dead parking removed at the end of a statement (compiler) | 1065 M | 1.40x |
| + builtin operands pushed directly, straight-line ALT (compiler, conc.h) | 1062 M | 1.39x |
| + indexed FP loads, `fpldnlsni`/`fpldnldbi` (compiler) | 1062 M | 1.39x |
| + `x * 2.0` as `fpumulby2` on the T800 (compiler) | 1059 M | 1.39x |
| + FP values kept on the FPU stack, as occam (compiler) | 954 M | 1.25x |
| + squares as `fpdup; fpmul` (compiler) | 954 M | 1.25x |

Exactly: 1,062,277,565 instructions before the indexed loads,
1,061,832,482 after (they save 0.04% here, where most subscripts are
constants and fold into the address), 1,059,480,926 with `x * 2.0` as
`fpumulby2`, 953,702,409 with the FPU stack and 953,888,262 with the
squares.  These are t4 dispatches; see the next section for real
instruction counts and cycles (t4 fuses `ldlp; fpldnlmulsn` into one
dispatch but not `fpdup; fpmul`, so the squares raise this count while
the real one falls).  Code size: the ray tracer is 14096
bytes of code in C (`raytrace.o`, was 19006), 12248 in occam
(`raytrace.lku`, which also holds `writeReal`).  The compiler changes are
on the compiler repo's main branch: a4dbeab2 (FA residency, constant
subscripts), 54530ac1 (statement-end cleanup, builtin operands, `conc.h`)
5f3c9a77 (indexed loads), a621df2b (`fpumulby2`/`fpudivby2` on the
T800), d355e94f (the FPU stack) and 0092bd43 (squares).  `transputer/CLAUDE.md` there, "Using the
stacks well", turns what this comparison taught into rules for the
backend.

## Every instruction, in T800 cycles

`heartbeat` counts dispatches, and t4 fuses common pairs (`stl; ldl`,
`ldlp; fpldnlsn`, `ldl; ldnl`, ...) into one, so the counts above
understate both programs by about 28%.  With fusion off and every executed
instruction counted at its address, model 3 at 256 x 256 on one node is:

| | instructions | T800 cycles | cycles vs occam |
|-|------:|------:|------:|
| occam | 1,058,821,118 | 2,947,336,424 | 1.00x |
| TinyCC 5f3c9a77 | 1,458,633,335 | 3,576,878,024 | 1.21x |
| a621df2b, `x * 2.0` as `fpumulby2` | 1,455,324,733 | 3,552,061,421 | 1.21x |
| d355e94f, FP values kept on the FPU stack | 1,284,656,245 | 3,277,368,522 | 1.11x |
| 0092bd43, squares as `fpdup; fpmul` | 1,284,142,073 | 3,265,578,901 | 1.11x |

The tables below this one are the first measurement, at 5f3c9a77; "Where
it stands now" after them is the last.

Cycles are INMOS's per-instruction timings (`../../../utils/instructions.txt`,
from the TCWG): on-chip RAM, one more cycle for every prefix byte, and the
operand-dependent ones (prod b+4, shifts, `in`/`out`/`move` by words, `norm`,
`cj` taken or not, `lend` looping or not, single or double FPU timings)
from the operands t4 saw.  The cycle gap is smaller than the instruction
gap because most of TinyCC's extra instructions are one- and two-cycle
loads and stores.  Where the 630 M cycles go, C minus occam:

| opcode | occam | C | C - occam, cycles |
|--------|------:|--:|------:|
| `ldl` | 311 M | 400 M | +178 M |
| `fpldnladdsn` | 5.1 M | 24.0 M | +151 M |
| `ldc` | 65.7 M | 74.3 M | +143 M |
| `stl` | 58.4 M | 200.2 M | +142 M |
| `fpldnlmulsn` | 32.9 M | 39.9 M | +90 M |
| `cj` | 30.9 M | 55.1 M | +87 M |
| `ldlp` | 56.8 M | 138.9 M | +82 M |
| `gt` | 2.1 M | 27.2 M | +50 M |
| `adc` | 5.8 M | 46.2 M | +40 M |
| `fpstnlsn` | 31.7 M | 51.6 M | +40 M |
| `fpordered` | 0 | 6.3 M | +19 M |
| `ldnl` | 57.5 M | 33.5 M | -48 M |
| `fpmul` | 5.3 M | 0 | -58 M |
| `fpadd` | 18.9 M | 0 | -113 M |
| `lend` | 19.9 M | 0 | -174 M |

(instruction counts; the last column is cycles.)  In groups:

- **Parking**, `ldl`/`stl`/`ldlp`: +402 M, less about 79 M that occam
  spends on `ldnl`/`ldnlp` reaching outer-scope variables.  Compiler.
- **FP operands from memory**: TinyCC adds with `fpldnladdsn` (8 cycles)
  where occam's `fpadd` (6) finds both operands on the FPU stack, squares
  with `fpldnlmulsn` (13) where occam does `fpdup; fpmul` (12), and stores
  9 partial results a `TransLocal` call to occam's 3.  About +105 M.
  Compiler: an FPU-stack allocator.  Done in d355e94f and 0092bd43 (below).
- **Global addresses**: `worldModel`, `tree`, `grunData`, `freeNode`,
  `lights`, ... are absolute addresses, each a 7-byte `ldc` (7 cycles),
  15 M times: 106 M cycles, where occam reaches the same arrays through
  its static link for 2-3.  Compiler, or the C (a local pointer per
  function).  Keeping an address in a frame word from its second use in a
  basic block, labels forgetting it, saved only 2.5 M - blocks seldom use
  a global three times - and hung model 1, so it was dropped; what would
  pay is a base held for the whole function.
- **FP constants**: 1.0, 2.0, 0.001, 512.0 as 32-bit `ldc` literals parked
  in a slot (`ldc #3F800000; stl 15; ldlp 15; fpldnlmulsn`): 36 M.  occam
  does `x * 2.0` as `fpumulby2` (1.65 M times) and loads the rest from a
  constant table.  Compiler; `x * 2.0` done in a621df2b (-24.8 M).
- **NaN-correct comparisons**: `fpordered` before every T800 relation, as
  C99 requires; occam does not care (+19 M).
- **Loops**: a wash.  TinyCC's `gt`/`cj`/`adc`/`eqc` (+200 M) against
  occam's `lend` (-174 M, 10 cycles each).

By routine (cycles a call, pairs matched by call counts and by their
`fpdiv`/`fpgt`/`fpeq`/square-root counts, which the algorithm fixes):

| routine | calls | occam | C | C/occam |
|---------|------:|------:|--:|------:|
| coneSect | 405,061 | 1706 | 2004 | 1.17 |
| TransLocal | 1,725,284 | 325 | 420 | 1.29 |
| ellipsoidSect | 360,594 | 1048 | 1360 | 1.30 |
| sceneSect | 405,061 | 707 | 680 | 0.96 |
| planeSect | 349,743 | 471 | 589 | 1.25 |
| dotProduct | 1,557,135 | 82 | 112 | 1.37 |
| shadeNode | 221,834 | 498 | 666 | 1.34 |
| reflectRay | 219,529 | 425 | 568 | 1.34 |
| normalize | 223,933 | 290 | 358 | 1.23 |
| pointSample | 464,272 | 115 | 105 | 0.91 |
| TransGlobal | 193,974 | 91 | 68 | 0.75 |

`TransLocal`, per call: 95 cycles more, of which 12 are the six additions
from memory, about 80 the parking (9 FP stores to 3, 18 more `ldl`, 19
more `stl`, 15 more `ldlp`), and its loop control costs the same as
occam's three `lend`s.

### Where it stands now (0092bd43)

318 M cycles, C minus occam:

| opcode | occam | C | C - occam, cycles |
|--------|------:|--:|------:|
| `ldc` | 65.7 M | 74.3 M | +133 M |
| `cj` | 30.9 M | 55.1 M | +87 M |
| `stl` | 58.4 M | 143.2 M | +85 M |
| `ldl` | 310.7 M | 343.7 M | +66 M |
| `gt` | 2.1 M | 27.2 M | +50 M |
| `adc` | 5.8 M | 46.2 M | +40 M |
| `ldlp` | 56.8 M | 93.4 M | +37 M |
| `eqc` | 11.0 M | 22.0 M | +22 M |
| `fpordered` | 0 | 6.3 M | +19 M |
| `j` | 7.8 M | 13.2 M | +15 M |
| `rev` | 0.01 M | 13.3 M | +13 M |
| `ldnlp` | 93.5 M | 62.2 M | -31 M |
| `ldnl` | 57.5 M | 33.5 M | -48 M |
| `lend` | 19.9 M | 0 | -174 M |

The floating point is occam's: `fpdup` and `fpmul` 5,268,802 each in
both, `fpldnlmulsn` 32.93 M in both, `fpadd` 18.5 M to occam's 18.9 M,
FP stores 32.0 M to 31.7 M.  What is left is integer code:

- **Parking**, `ldl`/`stl`/`ldlp`: +187 M (it was +402 M), less the
  79 M occam spends reaching outer-scope variables.
- **Constants**, `ldc`: +133 M, the global addresses and FP literals
  above.
- **Loops and conditions**: `cj`/`gt`/`adc`/`eqc`/`j` +214 M against
  `lend` -174 M.  ellipsoidSect shows it is a wash: its five small loops
  cost C about 220 cycles a call (`cj`, `gt`, `adc`, `j`) and occam about
  200 (21.7 `lend`s, 191 cycles); its FP is occam's instruction for
  instruction, and the 218 cycles a call it still loses are parking
  (`ldl` +109, `stl` +72, `ldlp` +16).

| routine | calls | occam | C | C/occam |
|---------|------:|------:|--:|------:|
| coneSect | 405,061 | 1706 | 1854 | 1.09 |
| TransLocal | 1,725,284 | 325 | 345 | 1.06 |
| ellipsoidSect | 360,594 | 1048 | 1266 | 1.21 |
| sceneSect | 405,061 | 707 | 680 | 0.96 |
| planeSect | 349,743 | 471 | 530 | 1.12 |
| dotProduct | 1,557,135 | 82 | 91 | 1.11 |
| shadeNode | 221,834 | 498 | 654 | 1.31 |
| reflectRay | 219,529 | 425 | 487 | 1.15 |
| normalize | 223,933 | 290 | 308 | 1.06 |
| pointSample | 464,272 | 115 | 105 | 0.91 |
| TransGlobal | 193,974 | 91 | 68 | 0.75 |

### Variable subscripts on local arrays (2026-11-05)

In the first loop of `ellipsoidSect`, TinyCC parked the address of
`point` before computing `&point[i]`. The same pattern occurred in
the loops over `normal` and `matrix`. The backend can push the index
first and the frame address second, in the order `wsub` takes them:

    before: ldlp 37; stl 9; ldl 18; ldl 9; wsub
    after:  ldl 18; ldlp 37; wsub

Measured against compiler `310efe5b`, one T800, model 3 at 256 x 256,
with fusion off and the same timing table for all three runs:

| Build | Executed instructions | Modeled cycles | ellipsoidSect cycles/call |
|---|---:|---:|---:|
| Rebuilt Occam BTL from t4 `7dbdf73` | 1,058,991,394 | 2,947,938,372 | 1,048 |
| TinyCC baseline | 1,284,141,947 | 3,265,578,467 | 1,266 |
| TinyCC with direct local-array addressing | 1,240,136,003 | 3,201,615,573 | 1,180 |

The change saved 63,962,894 cycles across the render, including
31,168,530 in the 360,594 calls to `ellipsoidSect`. All four reference
PPMs matched byte for byte on one and four nodes. Model 3 also matched
on 16 and 64 nodes. Both C builds used the same T800 runtime archive.

### Keeping a store address across FPU instructions (2026-11-05)

In the first loops of `ellipsoidSect` and `coneSect`, Occam loads both
indexed source floats, subtracts, then computes the destination address
and executes `fpstnlsn`. C computes the destination first. Its address
can stay on the integer stack while each `fpldnlsni` consumes the two
words above it. The existing stack scheduler now knows those effects.
From the disassembly of the first `ellipsoidSect` loop, with the same
source loads and subtraction in both C builds:

    before: ldl 18; ldlp 37; wsub; stl 9; <source loads and fpsub>; ldl 9; fpstnlsn
    after:  ldl 18; ldlp 37; wsub;        <source loads and fpsub>;        fpstnlsn

One T800, model 3, 256 x 256. The C builds used the same runtime built
from `3fae103e`. Counts are for the whole render with instruction fusion off.

| Build | Executed instructions | Modeled T800 cycles | coneSect cycles/call | ellipsoidSect cycles/call |
|---|---:|---:|---:|---:|
| Occam BTL | 1,058,991,381 | 2,947,938,342 | 1,706 | 1,048 |
| TinyCC `3fae103e` | 1,240,130,302 | 3,201,610,690 | 1,809 | 1,180 |
| With FPU instructions modeled by the scheduler | 1,203,123,620 | 3,142,698,837 | 1,753 | 1,124 |

The scheduler saved 58,911,853 modeled cycles (1.84% of the C render).
It eliminated 24.6 million `stl` and 21.8 million `ldl` executions,
while adding 9.9 million `rev` executions where a store needed its two
operands swapped. `coneSect` and `ellipsoidSect` together account for
42.5 million cycles saved. The C gap to Occam fell from 8.61% to 6.61%.
All four models matched the reference PPMs on one and four nodes. Model
3 also matched on the 16- and 64-node chains.

Reproducing: `ophist_patch.py` adds a `T4_OPHIST=file` counter to a copy
of t4's `p.c` (fusion off; every instruction counted at its address with
its length and timing operand, written at exit); `oph.py` reads the INMOS
table, attributes the counts to routines and pairs them:

    mkdir -p /tmp/t4op
    git -C ../../.. archive HEAD | tar -x -C /tmp/t4op
    python3 ophist_patch.py /tmp/t4op/p.c && make -C /tmp/t4op -f Makefile.linux t4
    printf '3\n' | SPYNET=../bin/raytrace1.map T4_OPHIST=occam.txt \
        /tmp/t4op/t4 -st -s8 -sn -1 -sb ../bin/raytrace1.btl
    printf '3\n' | SPYNET=bin256/raytrace1.map T4_OPHIST=c.txt \
        /tmp/t4op/t4 -st -s8 -sn -1 -sb bin256/raytrace1.btl
    base=$(awk '/Code from .raytrace\.lku./ {getline; sub(/^#/, "0x", $1); print $1; exit}' ../bin/raytrace1.map)
    python3 oph.py ../../../utils/instructions.txt occam.txt occ_code.bin \
        "$base" c.txt bin256/obj/raytrace.elf.tccsym out

`occ_code.bin` is the occam's `raytrace.lku` as loaded (the
`T4_DUMP_WORDS` dump under "Reproducing" below, as bytes).

### Global addresses kept in the frame (2026-11-05)

Occam reaches `tree`, `worldModel` and the other globals through its
static link, `ldl 25; ldnl 31`, about 4 cycles. C loaded each address with
an `ldc` of eight nibbles, since RAM starts at MinInt: 8 cycles a use, and
133 M cycles more than occam spent on `ldc` across the render. `sceneSect`
reloaded `worldModel` three times for every object in the scene.

The compiler's relaxation pass now loads such an address once, after the
prologue, into a frame word added on top of the frame, and every use
becomes an `ldl` of that word (`tp_gaddr_cache`). `sceneSect`, before and
after:

    before: ajw -46; ...; ldl 23; ldc worldModel; wsub; ldnl 0; eqc -1; ...
    after:  ajw -47; ldc worldModel; stl 46; ...; ldl 23; ldl 46; wsub; ldnl 0; ...

One T800, model 3, 256 x 256, both C builds on compiler main `073f2681`
with the same runtime; counts for the whole render, fusion off.

| Build | Executed instructions | Modeled T800 cycles | sceneSect | shadeNode | claim |
|---|---:|---:|---:|---:|---:|
| Occam BTL | 1,058,991,381 | 2,947,938,342 | 707 | 498 | 61 |
| TinyCC `073f2681` | 1,203,123,548 | 3,142,698,589 | 680 | 646 | 89 |
| Globals' addresses in the frame | 1,204,632,279 | 3,097,290,400 | 625 | 634 | 75 |
| Repeated float constants too | 1,201,334,990 | 3,084,034,732 | 625 | 634 | 75 |

(cycles per call for the three routines.) The addresses save 45,408,189
modeled cycles (1.44%): `ldc` falls from 202.6 M to 138.5 M cycles,
offset by 19 M of the `ldl`s that replace it. `sceneSect` is now cheaper
than occam's. What remains of `ldc` in `shadeNode` (41 cycles a call
against occam's 9) is mostly the three entry loads themselves, `lights`,
`grunData` and `tree`, plus `worldModel`, used once.

The same frame words hold a float constant a function loads repeatedly.
`coneSect` built 1.0 five times and 0.001 four times from eight-nibble
`ldc`s; now each is stored once at entry and loaded where it lies, a
new scheduler rule (S10) folding the copy to a scratch word:

    before: ldc #3F800000; stl 15; ldlp 15; fpldnlsn
    after:  ldlp 62; fpldnlsn

That is another 13,255,668 cycles (`coneSect` 1,753 -> 1,721 a call,
occam 1,706). Together the C gap to Occam fell from 6.61% to 4.62%. All
four models matched the reference PPMs on one and four nodes, and model
3 on 16 and 64, with each build.

## Programming or compiler?

Both, in that order of discovery.

**Programming** - the C asked for more work than the occam:

- `closest := iNode` is an array assignment, one block `move` in occam;
  the port copied it a word at a time.  10.5% of the run.
- `initWORDvec` fills the sample grid with doubling `move`s in occam; the
  port had a word loop.  3.6%.
- `SQRT` is inline in occam; the port called a wrapper.
- `ChanAlt`, a generic ALT over an array of channels with optional guards,
  where occam compiles each ALT inline: see "Channels and links".

**Compiler** - TinyCC emitted more for the same work, in every routine
about equally, which is why the profiles of the two programs had the same
shape (below):

| | what TinyCC did | status |
|-|-----------------|--------|
| 1 | stored every FP result and loaded it straight back | fixed: the operand just computed stays in FA (`tp_fpu_take_resident`) |
| 2 | allocated the result before freeing the operands, spilling a third live product | fixed: a consumed float slot takes the result |
| 3 | `p[0]` on a local array: `ldlp; stl; ldc 0; stl; ldl; ldl; wsub; stl` | fixed: folded into the frame offset, no code |
| 4 | parked every value in a slot so an assignment could have a value: `i = 0;` was `ldc 0; stl 9; ldl 9; stl i`, `i++;` seven instructions | fixed: at the end of a statement the dead parking in the window goes (`tp_stmt_end`); `i++;` is `ldl i; adc 1; stl i` |
| 5 | `return x;` went through slot 7: `stl 7; ldl 7` | fixed |
| 6 | parked every builtin operand, constants included: `__in(4, c, &w)` was 9 instructions | fixed: `ldlp w; ldl c; ldc 4; in`, as occam |
| 7 | `static inline` functions are calls: TinyCC does not inline | worked round in conc.h (macros); open in general |
| 8 | every prologue copies the parameters into the frame (`ldl 16; stl 14`) | open: `alt_core`'s first-parameter trick depends on that layout |
| 9 | a pointer is still parked when another operand's load comes between (`ldl a; ldnlp k; stl 9; ...; ldl 9; fpldnlmulsn`) | open |
| 10 | FP values not consumed by the next operation still go to memory (a dot product's first two products) | open: needs an FPU-stack allocator, occam's Sethi-Ullman ordering |

## Where the time goes

`T4_IPTR_SAMPLE` (8192 IPtr samples over the run), C by its symbol map,
occam by splitting the code at `ret` and matching each routine to its PROC
by the floating-point operations in it:

| routine        | occam | C, first port | C, now |
|----------------|------:|------:|------:|
| coneSect       | 22.8% | 23.4% | 21.9% |
| TransLocal     | 16.7% | 18.2% | 19.0% |
| ellipsoidSect  | 12.3% | 12.2% | 16.9% |
| sceneSect      | 11.0% | 4.2% + 10.5% copyNode | 7.2% |
| planeSect      |  5.3% |  4.5% |  5.7% |
| shadeNode      |  4.1% |  3.4% |  4.3% |
| reflectRay     |  3.7% |  2.7% |  3.8% |
| dotProduct     |  3.2% |  3.8% |  4.1% |

The shapes agree: the two compilers spend their time in the same places,
and the first C spent about twice as many instructions in each - so the
gap was never one routine or one construct, but how expressions were
compiled everywhere.

| routine       | occam | C, first port | C, now |
|---------------|------:|------:|------:|
| dotProduct    | 21 | 58 | 40 |
| normalize     | 58 | 126 | 96 |
| TransLocal    | 82 | 220 | 127 |
| coneSect      | 1121 | 2075 | 959 |

(instructions, static)

## dotProduct, side by side

`(a[0] * b[0]) + ((a[1] * b[1]) + (a[2] * b[2]))`

```
occam (D7205-era oc, -T8)                 TinyCC -mt800
                                          60 b1  ajw -15
                                          21 70  ldl 16        a
                                          de     stl 14
                                          21 71  ldl 17        b
                                          dd     stl 13
                                          7e     ldl 14
                                          d9     stl 9
                                          7d     ldl 13
                                          da     stl 10
72     ldl 2       a                      79     ldl 9
51     ldnlp 1                            28 fe  fpldnlsn      a[0]
28 fe  fpldnlsn    a[1]                   7a     ldl 10
73     ldl 3       b                      2a fc  fpldnlmulsn   * b[0]
51     ldnlp 1                            13     ldlp 3
2a fc  fpldnlmulsn * b[1]                 28 f8  fpstnlsn      -> tmp3
72     ldl 2                              7e     ldl 14
52     ldnlp 2                            51     ldnlp 1
28 fe  fpldnlsn    a[2]                   d9     stl 9
73     ldl 3                              7d     ldl 13
52     ldnlp 2                            51     ldnlp 1
2a fc  fpldnlmulsn * b[2]                 da     stl 10
28 f7  fpadd       (FPU stack: 2 deep)    79     ldl 9
72     ldl 2                              28 fe  fpldnlsn      a[1]
28 fe  fpldnlsn    a[0]                   7a     ldl 10
73     ldl 3                              2a fc  fpldnlmulsn   * b[1]
2a fc  fpldnlmulsn * b[0]                 14     ldlp 4
28 f7  fpadd                              28 f8  fpstnlsn      -> tmp4
71     ldl 1       &result                ...    (a[2] * b[2] the same -> tmp5)
28 f8  fpstnlsn                           73     ldl 3
22 f0  ret                                dc     stl 12
                                          14     ldlp 4
                                          28 fe  fpldnlsn      tmp4
                                          15     ldlp 5
                                          2a fa  fpldnladdsn   + tmp5
                                          13     ldlp 3
                                          28 f8  fpstnlsn      -> tmp3
                                          1c     ldlp 12
                                          28 fe  fpldnlsn
                                          13     ldlp 3
                                          2a fa  fpldnladdsn
                                          14     ldlp 4
                                          28 f8  fpstnlsn      -> tmp4
                                          74     ldl 4
                                          d7     stl 7         return value
                                          77     ldl 7
                                          bf     ajw 15
                                          22 f0  ret
21 instructions, no frame                 58 instructions, 15-word frame
```

(The occam adds the deeper bracket first.  IEEE addition commutes, so
the result is the same bits.)

## The first measurement, in detail

What the side-by-side showed before any of the fixes (the table under
"Programming or compiler?" says what became of each), in order of what
they cost:

1. **Floating-point intermediates go to memory.**  Each product is stored
   (`ldlp n; fpstnlsn`) and loaded back for the add (`ldlp n;
   fpldnlsn`/`fpldnladdsn`).  The T800 FPU has a three-deep stack (FA, FB,
   FC); the occam keeps the whole expression tree on it, ordering operands
   so the deeper subtree goes first, and stores once, at the assignment.
   This is most of the difference in the small kernels (dotProduct,
   TransLocal); in coneSect, whose temporaries are named variables in
   both, it is 92 `fpstnlsn` and 141 `fpldnlsn` against 62 and 120.
2. **Every pointer goes through a scratch slot before use.**  `ldl 14;
   stl 9; ... ldl 9; fpldnlsn` where `ldl 14; fpldnlsn` would do - the
   always-STL policy (CLAUDE.md, "Every load() stores to workspace via
   STL").  This is the big one in the large routines: coneSect has 445
   `stl` against the occam's 52, and 490 `ldl` against 290.
3. **Parameters are copied into the frame.**  `ldl 16; stl 14; ldl 17;
   stl 13`: the arguments already sit in the caller's frame at `ldl
   N+1..`, where the occam uses them directly (`ldl 2`, `ldl 3`).
4. **Constant indexing is not folded.**  TransLocal computes
   `&newpoint[0]` as `ldlp 15; stl 9; ldc 0; stl 10; ldl 10; ldl 9;
   wsub` - seven instructions for `ldlp 15`.
5. **A float result comes back through the integer return slot**
   (`ldl 4; stl 7; ldl 7`) and is stored again by the caller, where the
   occam leaves it in FA or stores it through the result pointer.
6. **Frames are larger than they need to be**: 15 and 20 words for leaf
   routines that need two or none.

1 and 2 were the bulk of it.  What is left of them is items 9 and 10
above: a full FPU-stack allocator - the Sethi-Ullman labelling occam
uses, which the integer side would want for A/B/C too - is the next
step for the arithmetic kernels.

## Channels and links

The routers (`throughPut`, `pixelBypass`, `loadBalance`) are all channel
I/O and ALTs.  The transfer instructions are the same in both - `in`,
`out`, `outword`, `enbc`/`altwt`/`disc` - and the difference is round them:

| operation | occam | C, first port | C, now |
|-----------|------:|------:|------:|
| PRI ALT of two channels | 17, inline | ~150: `ChanAlt` + `alt_core`'s loops over a guard array | 33: `ChanAlt2`, straight-line, one call |
| input a word | 4: `ldlp w; ldl c; ldc 4; in` | ~13: a call to `ChanInInt` | 4 |
| output a word | 3: `ldl w; ldl c; outword` | ~9: a call to `ChanOutInt` | 3-4 |

The occam ALT, from the `raytrace.lku` dump:

```
alt; ldl 10; ldc 1; enbc; ldl 9; ldc 1; enbc; altwt;
ldl 10; ldc 1; ldc 0; disc; ldl 9; ldc 1; ldc 76; disc; altend
```

`__tp_alt2` (`ChanAlt2`) now:

```
ajw -15; ldl 17; stl 13; ldl 18; stl 12; ldc -1; stl 11;
alt; ldl 13; ldc 1; enbc; ldl 12; ldc 1; enbc; altwt; stl 7;
ldl 13; ldc 1; ldc 0; disc; cj; ldc 0; stl 11;
ldl 12; ldc 1; ldc 1; disc; cj; ldc 1; stl 11; ldl 11; ajw 15; ret
```

- the body is the occam's; the rest is the call (item 8, the prologue's
  parameter copies, is half of it).  An inline ALT would need the
  compiler to keep workspace slot 0 free - the ALT's `Temp` - which is
  why it is a function.
- the conc.h channel wrappers were `static inline` functions, and TinyCC
  does not inline (item 7): each was a call, a frame and a return round one
  instruction.  They are macros now.
- builtin operands were parked in slots before use (item 6).

On one node this is a rounding error (the run is rendering).  On a long
chain it is latency: every job and every patch crosses every router in
front of the node that takes it, at high priority, so each hop's cost is
paid up to 255 times.

## Dhrystone against icc

The same question on integer code: `../../dhrystone/dhrystone.c` with
`LOOPS=500000`, INMOS icc's `dhrystone.b8h` against TinyCC `-mt800` with
the c-testsuite runtime (a wrapper calls `__seterr()` after `main` so that
`t4 -se` stops and prints `heartbeat`):

| build | instructions | vs icc |
|-------|-------------:|-------:|
| icc (`dhrystone.b8h`) | 229.0 M | 1.00x |
| TinyCC 4a446dbd (before this work) | 632.5 M | 2.76x |
| TinyCC now | 626.0 M | 2.73x |

Half of TinyCC's is `strcpy` (IPtr samples: 49.9%), the runtime's
`while ((*d++ = *src++))`, at 25 instructions a byte.  icc's library
`strcpy` (#80003774 in memory) checks that both pointers are word aligned
and copies a word at a time, finding the zero byte with #01010101 and
#80808080; it is 5.4% of icc's run.  That part is the library - a
programming difference.  Without it the two are about 313 M and 217 M,
1.44x, the ray tracer's ratio, and the byte loop itself shows three
compiler issues: `p++` through two slots, a sign extension (`ldc 128;
xword`) that only `sb` uses, and the loop test extending the byte again
for a zero test.  The statement-end cleanup does little here (-1%): this
code's waste is in the char and pointer handling, not in parked
expression values.

## Reproducing

```
# Read the node 0 code range from the map shipped with the Occam BTL.
read base size < <(awk '/Code from .raytrace\.lku./ {getline; sub(/^#/, "0x", $1); print $1, $3; exit}' ../bin/raytrace1.map)
W=
for ((off=0; off<size; off+=1024)); do
    W+=$(printf '0x%08X:256,' "$((base + off))")
done
printf '3\n' | SPYNET=../bin/raytrace1.map T4_IPTR_SAMPLE=93001 T4_DUMP_WORDS=${W%,} \
    t4 -st -s8 -sn -1 -sb ../bin/raytrace1.btl >occ-dump.log 2>&1
python3 - "$base" "$size" <<'PY'
import re, struct, sys
base, size = int(sys.argv[1], 0), int(sys.argv[2])
words = {}
with open('occ-dump.log', 'rb') as log:
    for line in log:
        m = re.match(rb'M 0x([0-9A-F]+) 0x([0-9A-F]+)', line)
        if m:
            words[int(m[1], 16)] = int(m[2], 16)
code = b''.join(struct.pack('<I', words[a]) for a in range(base, base + size, 4))
with open('occ_code.bin', 'wb') as f:
    f.write(code[:size])
PY
# C: the same, and the symbol map beside the ELF
SPYNET=bin256/raytrace1.map T4_IPTR_SAMPLE=221001 \
    t4 -st -s8 -sn -1 -sb bin256/raytrace1.btl
dast -O <addr> code.bin                            # transputer/ttools/dast
```

The older BTL put `raytrace.lku` at #80002B8C. The rebuilt one-node BTL
in t4 `7dbdf73` puts its 12,248 bytes at #80002C94. Always use the map
paired with the BTL being profiled.
