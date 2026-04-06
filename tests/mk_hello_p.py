#!/usr/bin/env python3
"""Build hello_p.btl — T4 guest that exercises goprot / syscall (Phase 1/3).

Boot format (netload-compatible):
  byte 0 = 0  (no bootstrap)
  then chunks: len (1..255), data...
  end: len 0

Loaded at MemStart (T806: 0x80000070). IPtr starts at MemStart; WPtr after image.

Supervisor (L):
  - Build PDS + RODS in RAM (absolute addresses via mint+bsub)
  - goprot → P
  - On return from trap: write magic 0xC0DE0000|cause to a result slot, stopp

P-process:
  - syscall  (forces trap back to L)

Region 2 identity map (code lives at 0x8xxxxxxx): RWX, 256MB bottom of quarter.
"""
from __future__ import annotations
import struct
import sys
from pathlib import Path

MEMSTART = 0x80000070  # T806 default in T4 main.c


def pfix_digits(n: int, final_op: int) -> bytes:
    """Emit pfix/nfix chain then final primary/secondary nybble.
    final_op: 0x4=ldc, 0xF=opr, 0x7=ldl, etc. n is the operand value (non-neg).
    """
    if n < 0:
        raise ValueError("use encode_negative_opr for negative OReg")
    digits = []
    v = n
    if v == 0:
        digits = [0]
    else:
        while v:
            digits.append(v & 0xF)
            v >>= 4
        digits.reverse()
    out = bytearray()
    for i, d in enumerate(digits):
        if i < len(digits) - 1:
            out.append(0x20 | d)  # pfix
        else:
            out.append((final_op << 4) | d)
    return bytes(out)


def ldc(n: int) -> bytes:
    if n < 0:
        # nfix path for negative constants
        # OReg = ~0 << 4 after nfix 0, then add
        # Simpler: only use non-negative here
        raise ValueError("ldc negative not implemented")
    return pfix_digits(n, 0x4)


def opr(n: int) -> bytes:
    """Secondary instruction with OReg = n (non-negative)."""
    return pfix_digits(n, 0xF)


def opr_neg(code: int) -> bytes:
    """Secondary with negative operation code from ISM App.B (e.g. -6 goprot).
    Encoding: nfix 0; opr |low4|  where OReg becomes 0xFFFFFFF0 + low4.
    For -6: 0xFFFFFFF0+0xA = 0xFFFFFFFA
    For -8: 0xFFFFFFF0+0x8 = 0xFFFFFFF8
    """
    # code is negative, e.g. -6
    assert code < 0
    low = (-code) & 0xF  # -6 -> need 0xA added to FFFFFFF0 → actually
    # After nfix 0: OReg = (~0)<<4 = 0xFFFFFFF0
    # We need OReg = code as uint32 (e.g. -6 = 0xFFFFFFFA)
    # 0xFFFFFFF0 + x = 0xFFFFFFFA => x = 0xA
    # 0xFFFFFFF0 + x = 0xFFFFFFF8 => x = 0x8
    target = code & 0xFFFFFFFF
    x = (target - 0xFFFFFFF0) & 0xF
    return bytes([0x60, 0xF0 | x])  # nfix 0; opr x


def mint() -> bytes:
    return opr(0x42)  # mint


def rev() -> bytes:
    return opr(0x00)


def bsub() -> bytes:
    return opr(0x02)


def stl(n: int) -> bytes:
    return pfix_digits(n, 0xD)


def ldl(n: int) -> bytes:
    return pfix_digits(n, 0x7)


def ldlp(n: int) -> bytes:
    return pfix_digits(n, 0x1)


def ajw(n: int) -> bytes:
    # ajw primary 0xB; negative needs nfix
    if n >= 0:
        return pfix_digits(n, 0xB)
    # nfix for negative: similar to OReg build then ajw
    # For small negative workspace adjust use nfix
    t = (-n) & 0xFFFFFFFF
    # simpler: only use positive ajw in this program
    raise ValueError("ajw neg")


def stopp() -> bytes:
    return opr(0x15)


def j_forever() -> bytes:
    """j -1 encoded carefully — use stopp instead for halt."""
    return stopp()


def ld_abs(addr: int) -> bytes:
    """A = addr. mint; ldc (addr - MostNeg); bsub  OR  full ldc chain.
    MostNeg = 0x80000000. addr = 0x80000070 + off.
    mint -> A=0x80000000; ldc off; sum or bsub: A = mint + off if off positive.
    bsub: A = A + B after rev? bsub: A = A + B, B = C.
    Sequence: mint; ldc delta; bsub  => A = mint+delta if B was mint and A was delta?
    After mint: A=mint. After ldc d: A=d, B=mint. bsub: A = A+B = d+mint. Yes.
    """
    mostneg = 0x80000000
    if addr >= mostneg:
        delta = addr - mostneg
        return mint() + ldc(delta) + bsub()
    return ldc(addr)


def make_region_identity_quarter2() -> int:
    """RWX region covering 0x80000000..0x8FFFFFFF (256MB), identity phys."""
    n = 28
    size_bit = 1 << (n - 1)
    flags = 0x1 | 0x2  # write + execute
    reloc = 0x80000000  # bits n..31
    return reloc | size_bit | flags


def build() -> tuple[bytes, dict]:
    # Layout within payload (offsets from MemStart)
    # We assemble code first, then place data after with known offsets.
    # Simpler: fixed layout in a 4KB image.
    #
    # [0x000 code supervisor]
    # [0x100 P code]
    # [0x140 PDS 11 words]
    # [0x180 RODS 4 words]
    # [0x1C0 P workspace]
    # [0x200 result word written by supervisor after trap]
    #
    OFF_SUP = 0x000
    OFF_PCODE = 0x100
    OFF_PDS = 0x140
    OFF_RODS = 0x180
    OFF_PWSP = 0x1C0
    OFF_RESULT = 0x200
    IMG = 0x400

    base = MEMSTART
    addr = lambda off: base + off

    # --- P code: just syscall ---
    pcode = opr_neg(-8)  # syscall
    assert len(pcode) == 2

    # --- RODS ---
    r0 = 0x80000000  # null for quarter 0
    null = 0x80000000
    r2 = make_region_identity_quarter2()
    rods = struct.pack("<IIII", null, null, r2, null)

    # --- PDS ---
    pds_words = [0] * 11
    pds_words[0] = 0  # cntl
    pds_words[4] = addr(OFF_PWSP)  # sWptr
    pds_words[5] = addr(OFF_PCODE)  # sIptr
    pds = struct.pack("<" + "I" * 11, *pds_words)

    # --- Supervisor ---
    # ld PDS into A, RODS into B, goprot
    # after trap: A has undefined; load cause from PDS cntl low 16, store to RESULT
    # Actually after trap, PDS has cause in cntl low bits.
    # ldlp-relative is hard without knowing WPtr. Use absolute.
    s = bytearray()
    # A=PDS, B=RODS
    s += ld_abs(addr(OFF_PDS))
    s += ld_abs(addr(OFF_RODS))
    s += rev()  # A=PDS, B=RODS
    s += opr_neg(-6)  # goprot
    # After P-trap: load PDS.cntl, store to RESULT, halt.
    # ld_abs PDS; ldnl 0 → A=cntl; ld_abs RESULT → A=RESULT B=cntl; stnl 0.
    s += ld_abs(addr(OFF_PDS))
    s += pfix_digits(0, 0x3)  # ldnl 0 (primary)
    s += ld_abs(addr(OFF_RESULT))
    s += pfix_digits(0, 0xE)  # stnl 0 → mem[RESULT]=cntl
    # Spin in-place (j -1): stays in RAM so T4_WATCH on timeout works.
    # nfix 0; j 0xF → OReg = -1.
    s += bytes([0x60, 0x0F])

    sup = bytes(s)

    # Assemble image
    img = bytearray(IMG)
    img[OFF_SUP:OFF_SUP + len(sup)] = sup
    img[OFF_PCODE:OFF_PCODE + len(pcode)] = pcode
    img[OFF_PDS:OFF_PDS + len(pds)] = pds
    img[OFF_RODS:OFF_RODS + len(rods)] = rods
    # P workspace zeroed
    # RESULT zeroed

    meta = {
        "memstart": hex(MEMSTART),
        "entry": hex(addr(OFF_SUP)),
        "pcode": hex(addr(OFF_PCODE)),
        "pds": hex(addr(OFF_PDS)),
        "rods": hex(addr(OFF_RODS)),
        "result": hex(addr(OFF_RESULT)),
        "region2": hex(r2),
        "sup_len": len(sup),
        "sup_hex": sup.hex(),
        "p_hex": pcode.hex(),
    }
    return bytes(img), meta


def to_btl(payload: bytes) -> bytes:
    """Zero-length bootstrap + chunked payload."""
    out = bytearray([0])  # no bootstrap
    i = 0
    while i < len(payload):
        chunk = payload[i:i + 255]
        out.append(len(chunk))
        out.extend(chunk)
        i += len(chunk)
    out.append(0)
    return bytes(out)


def main():
    out_dir = Path(__file__).resolve().parent
    payload, meta = build()
    btl = to_btl(payload)
    bin_path = out_dir / "hello_p.bin"
    btl_path = out_dir / "hello_p.btl"
    meta_path = out_dir / "hello_p.meta"
    bin_path.write_bytes(payload)
    btl_path.write_bytes(btl)
    meta_path.write_text("\n".join(f"{k}={v}" for k, v in meta.items()) + "\n")
    print(f"wrote {btl_path} ({len(btl)} bytes)")
    print(f"  entry={meta['entry']} result={meta['result']}")
    print(f"  goprot/syscall encoding: sup={meta['sup_hex'][:32]}... p={meta['p_hex']}")


if __name__ == "__main__":
    main()
