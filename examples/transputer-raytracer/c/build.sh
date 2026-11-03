#!/usr/bin/env bash
#
# build.sh - build the C ray tracer and one boot file per network size.
#
#   ./build.sh                          # 2048 x 2048, every size, into bin/
#   RES=256 OUT=bin256 ./build.sh       # the occam's own size, to compare
#                                       # with ../pic/rayM.ppm
#   SIZES="1 4" ./build.sh              # just these
#
# env:
#   TCC_DIR  a built compiler tree: transputer-tcc, include/ and
#            transputer-libtcc1-t800.a (default: the checkout this is in)
#   RES      picture size, a multiple of 256 (2048)
#   PATCH    pixels per job edge, 8 or 16 (16 at RES >= 1024, else 8)
#   CHATTY   1 prints the occam's two lines a job (0)
#   SIZES    network sizes (1 2 4 8 16 32 64 128 256)
#   OUT      where raytraceN.btl and raytraceN.map go (bin)
#
# Every size runs the same image; only the boot header differs.  The maps
# are the occam build's (../bin/raytraceN.map) where there is one - the
# 16-node one chains on links 1 -> 0, the rest on 2 -> 1 - and are made in
# the same form (2 -> 1) for 128 and 256.
set -eu

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../../../.." && pwd)"
TCC_DIR="${TCC_DIR:-$ROOT}"
RES="${RES:-2048}"
SIZES="${SIZES:-1 2 4 8 16 32 64 128 256}"
OUT="${OUT:-bin}"
case "$OUT" in /*) ;; *) OUT="$HERE/$OUT" ;; esac

TCC="$TCC_DIR/transputer-tcc"
LIB="$TCC_DIR/transputer-libtcc1-t800.a"
for f in "$TCC" "$LIB"; do
    [ -f "$f" ] || { echo "build.sh: $f not found (set TCC_DIR)" >&2; exit 1; }
done

DEFS="-DRES=$RES"
[ -n "${PATCH:-}" ] && DEFS="$DEFS -DPATCH=$PATCH"
[ -n "${CHATTY:-}" ] && DEFS="$DEFS -DCHATTY=$CHATTY"

OBJ="$OUT/obj"
mkdir -p "$OUT" "$OBJ"
for src in start.S node.c framebuf.c cntlsys.c raytrace.c hostio.c; do
    "$TCC" -mt800 -nostdlib -nostdinc -I"$TCC_DIR/include/transputer" \
        -I"$TCC_DIR/include" $DEFS -c -o "$OBJ/${src%.*}.o" "$HERE/$src"
done
"$TCC" -mt800 -nostdlib -o "$OBJ/raytrace.elf" \
    "$OBJ/start.o" "$OBJ/node.o" "$OBJ/framebuf.o" "$OBJ/cntlsys.o" \
    "$OBJ/raytrace.o" "$OBJ/hostio.o" "$LIB" 2>&1 |
    grep -v 'start_kernel symbol not found\|Could not patch BSS load points' || true

for n in $SIZES; do
    map="$OUT/raytrace$n.map"
    if [ -f "$HERE/../bin/raytrace$n.map" ]; then
        cp "$HERE/../bin/raytrace$n.map" "$map"
    else
        python3 "$HERE/mkbtl.py" map "$n" > "$map"
    fi
    python3 "$HERE/mkbtl.py" btl "$OBJ/raytrace.elf" "$map" "$OUT/raytrace$n.btl"
done
echo "RES=$RES $DEFS" > "$OUT/BUILD"
