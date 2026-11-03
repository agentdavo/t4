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

Exactly: 1,062,277,565 instructions before the indexed loads and
1,061,832,482 after (they save 0.04% here, where most subscripts are
constants and fold into the address).  Code size: the ray tracer is 14096
bytes of code in C (`raytrace.o`, was 19006), 12248 in occam
(`raytrace.lku`, which also holds `writeReal`).  The compiler changes are
on the compiler repo's main branch: a4dbeab2 (FA residency, constant
subscripts), 54530ac1 (statement-end cleanup, builtin operands, `conc.h`)
and 5f3c9a77 (indexed loads).  `transputer/CLAUDE.md` there, "Using the
stacks well", turns what this comparison taught into rules for the
backend.

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
# occam: profile, and its code as loaded (T4_DUMP_WORDS, 256 words an address)
W=; for i in $(seq 0 11); do W="$W$(printf 0x%08X $((0x80002B8C + i*1024))):256,"; done
SPYNET=../bin/raytrace1.map T4_IPTR_SAMPLE=93001 T4_DUMP_WORDS=${W%,} \
    t4 -st -s8 -sn -1 -sb ../bin/raytrace1.btl     # answer 3
# C: the same, and the symbol map beside the ELF
SPYNET=bin256/raytrace1.map T4_IPTR_SAMPLE=221001 \
    t4 -st -s8 -sn -1 -sb bin256/raytrace1.btl
dast -O <addr> code.bin                            # transputer/ttools/dast
```

`raytrace.lku` is at #80002B8C on node 0 (`../bin/raytrace1.map`).  The
1991 `oc` in `../../itools` (2.01.01) does not compile this version of the
source, so the occam side is the shipped boot file's code, dumped from
memory.
