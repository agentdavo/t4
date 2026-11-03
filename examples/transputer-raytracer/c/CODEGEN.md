# occam vs TinyCC: the same ray tracer, instruction by instruction

The C port renders the occam's pictures bit for bit, tracing the same
405061 rays for model 3, so any difference in speed is code generation.
Measured 2026-11-03 on t4, one node (`-st -sn -1`), model 3, 256 x 256:

|                              | occam (`../bin/raytrace1.btl`) | C (`-mt800`, HEAD 4a446dbd) |
|------------------------------|-------------------:|--------------------:|
| instructions executed        | 763 M              | 1573 M (2.06x)      |
| time, loaded host            | 8.75 s - 21.0 s    | 14.7 s - 33.2 s (about 1.6-1.9x) |
| ray tracer code              | 12248 bytes (`raytrace.lku`) | 19006 bytes (`raytrace.o` .text) |

Instruction counts are t4's `heartbeat` at exit; times are the program's
own and moved with the load on the machine, the ratio less so.  The C
figure is after three fixes to the port itself, which had cost another
240 M instructions: `closest := iNode` is an array assignment, a block
`move` in occam, and was a word loop in C (10.5% of samples); the sample
grid was filled word by word instead of by the occam's doubling `move`s
(3.6%); and `SQRT` went through a function instead of inline.

## Where the time goes

`T4_IPTR_SAMPLE` (8192 IPtr samples over the run), C by its symbol map,
occam by splitting the code at `ret` and matching each routine to its PROC
by the floating-point operations in it:

| routine        | occam | C (before the port fixes) |
|----------------|------:|------:|
| coneSect       | 22.8% | 23.4% |
| TransLocal     | 16.7% | 18.2% |
| ellipsoidSect  | 12.3% | 12.2% |
| sceneSect      | 11.0% | 4.2% + 10.5% copyNode |
| planeSect      |  5.3% |  4.5% |
| shadeNode      |  4.1% |  3.4% |
| reflectRay     |  3.7% |  2.7% |
| dotProduct     |  3.2% |  3.8% |

The shapes agree: the compilers spend their time in the same places, and
C spends about twice as many instructions in each.  So the gap is not one
routine or one construct; it is how expressions are compiled everywhere.

| routine       | occam bytes / instructions | C bytes / instructions |
|---------------|------:|------:|
| dotProduct    |  30 / 21  |   77 / 58   |
| normalize     |  89 / 58  |  177 / 126  |
| TransLocal    | 111 / 82  |  293 / 220  |
| coneSect      | 1793 / 1121 | 2972 / 2075 |

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

## What the C pays for

In order of what they cost here:

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

1 and 2 are the bulk of it.  Keeping float expression trees on the FPU
stack - the same Sethi-Ullman labelling the integer side would want for
A/B/C - would bring the arithmetic kernels close to the occam's
instruction counts on its own.

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
