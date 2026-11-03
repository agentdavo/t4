# The INMOS ray tracer in C

A port of `../occ` (INMOS Technical Note 7, *Exploiting concurrency: a ray
tracing example*) to plain C for TinyCC's transputer target, booted over
the links of a chain of 1 to 256 T800s on t4, using the occam build's map
files.  It renders 2048 x 2048 by default; built at 256 x 256 it renders
the original picture, and must match `../pic/rayM.ppm` byte for byte -
which it does, for every model and every network size tried.

| occam                  | C            | what                                           |
|------------------------|--------------|------------------------------------------------|
| `raytrace.inc`         | `raytrace.h` | protocol, tree and object layouts, build options |
| `framebuf.occ`         | `framebuf.c` | node 0: picture, timing, `ray.ppm` via the host |
| `cntlsys.occ`          | `cntlsys.c`  | node 0: world models, job farm                 |
| `raytrace.occ`         | `raytrace.c` | every node: `throughPut`, `pixelBypass`, `renderer` |
| `raytrace.cfs`         | `node.c`     | which process runs where, on which channels    |
| hostio library         | `hostio.c`   | the iserver SP protocol over link 0            |
| `icollect`             | `mkbtl.py`   | network boot file from the linked image and a map |
|                        | `start.S`    | image entry                                    |

`conc.h` supplies the occam primitives: `ChanIn`/`ChanOut` on soft channels
and links, `ChanAlt2`/`ChanAltGuarded3` for `PRI ALT` (scan order is the
priority; `raytrace.h`'s `ALT2`/`ALTG3` fall back to `ChanAlt`/
`ChanAltGuarded` with a `conc.h` that lacks them), and `PSetup`/`PRun` with
`PRI_HIGH`/`PRI_LOW` for `PRI PAR`.  How the code TinyCC makes of it
compares with the occam's, instruction by instruction: `CODEGEN.md`.

## Build

In WSL, with a built compiler tree (`transputer-tcc`, `include/`,
`transputer-libtcc1-t800.a`):

    TCC_DIR=~/tinycc-transputer bash build.sh          # 2048 x 2048 -> bin/
    TCC_DIR=... RES=256 OUT=bin256 bash build.sh       # the occam's size

`bin/raytraceN.btl` and `bin/raytraceN.map` come out for N = 1, 2, 4, 8,
16, 32, 64, 128 and 256.  Every size boots the same image (`-mt800`, about
27 KB); only the boot header differs.  The maps are `../bin/raytraceN.map`
where the occam build has one - the 16-node map chains on links 1 -> 0, the
others on 2 -> 1 - and `mkbtl.py map N` writes 128 and 256 in the same form.
Options: `RES` (a multiple of 256), `PATCH` (job edge, 8 or 16: 16 from 1024
up), `CHATTY=1` (the occam's two lines a job), `SIZES`.

## Run

    bash run.sh 64 3            # 64 nodes as threads of one t4, model 3
    bash run.sh -sl 8 1         # 8 t4 processes over shared-memory links
    python3 ppm2png.py ray.ppm  # run.sh does this itself

`run.sh` answers the menu, keeps `ray-nN-mM.ppm`, `.png` and `.log`, and at
256 x 256 compares with `../pic`.  By hand it is what the occam needs:

    export SPYNET=bin/raytrace8.map
    printf '3\n' | t4 -st -s8 -sm 24 -sn -8 -sb bin/raytrace8.btl

Node 0 keeps the picture, 12 MB at 2048 x 2048, at #80200000, so it needs
`-sm 24`; at 256 x 256 it fits t4's default 2 MB.

## Booting a chain

`mkbtl.py` writes `[len] bootstrap`, a five-word header (index, nodes,
chain link, image size, image + BSS size) and the image.  The 220-byte
bootstrap, assembled by `mkbtl.py` itself (`mkbtl.py dump`), copies itself
to #801F0000, reads the header and the image from its boot link into
MemStart, sends bootstrap, header (index + 1) and image on down the chain
unless it is the last node, zeroes the BSS and jumps in.  So node 0 boots
from the file and each node boots the next, as the occam network loader
does.  The image reads its place from the header (`node.c`); the link back
to node 0 is the one it was booted through.

## Differences from the occam

- **The farm cannot deadlock.**  `loadBalance` could block handing a job
  to `outputBuffer` while the pixels it should be reading were stuck
  behind that job (`cntlsys.c` has the cycle; `../bin/testall.sh` sees it
  on emulated links).  It now queues jobs itself and gives one to
  `jobSender` when asked.  Credits (two jobs a worker) are unchanged.
- **Resolution** is a build option.  The camera and scene are the occam's:
  above 256 the 256 x 256 screen is sampled more finely, in power-of-two
  steps, so RES=256 is the original arithmetic exactly.
- **Messages** carry a patch as one block and text as bytes, where the
  occam sent rows and words; the tag-then-fields shape is the same.
- The shade tree has room for the worst case (the occam's 132 nodes stop
  with a range error on a deep enough tree; no reference picture needs
  more), the per-job progress lines are off unless `CHATTY=1`, and
  `writeReal`, which nothing calls, is not ported.

## Results

- **256 x 256**: every model, on every network size tried (1 to 256
  nodes), renders `../pic/rayM.ppm` byte for byte.
- **2048 x 2048**: `pic/rayM-2048.png`, rendered on 16 nodes (`bash run.sh
  16 M`) with the BTLs in `bin/`, which TinyCC built at compiler main
  5f3c9a77.  They are the same bytes as the first port's renders (TinyCC
  4a446dbd), so none of the compiler work moved a pixel.  On t4, as
  threads on a busy 20-core host, the 16-node renders reported 51 s (model 1),
  65 s (2), 40 s (3) and 87 s (4) -
  host time, so a rough guide only.
- **Speed**: on one node, model 3 at 256 x 256 takes 1,061,832,482
  instructions against the occam's 763 M (1.39x; the first port took
  1813 M).  `CODEGEN.md` has where the difference was, what was a
  programming issue and what a compiler one, and what is still open.
