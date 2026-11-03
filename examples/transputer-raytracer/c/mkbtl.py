#!/usr/bin/env python3
"""mkbtl.py - boot a C program down a chain of transputers.

    mkbtl.py btl  raytrace.elf raytraceN.map raytraceN.btl
    mkbtl.py map  N [--down 2 --up 1] > raytraceN.map
    mkbtl.py dump                       # the bootstrap, as bytes

icollect builds an occam network's boot file; this does the same job for
one C image that every node runs.  The .btl is

    [len] bootstrap[len]                     the standard primary bootstrap
    index n down filesz memsz                five words, the boot header
    image[filesz]                            the program, for MemStart

t4 -sb loads the bootstrap at MemStart on node 0 and feeds it the rest over
link 0; every other node gets the same through its boot link from the node
before it.  The bootstrap

  1. copies itself to B_HI and carries on there, so the image can go to
     MemStart, where the linker put it;
  2. takes its boot link from C (a link-booted transputer has the input
     channel there; node 0, booted from the file, has rubbish, and gets
     link 0), reads the header and the image;
  3. if index + 1 < n, sends [len] bootstrap, the header with index + 1,
     and the image out on link `down` - booting the next node;
  4. zeroes the BSS (filesz..memsz) and enters the image at MemStart with
     Wptr = WS_TOP.

Its workspace is B_WS, so the C program finds the header there (node.c's
BOOTINFO), and word 5 is the boot link.  The map decides `down` and n; a
map that is not a chain (HOST on node 0 link 0, node i-1 link `down` to
node i, the same `down` all along) is refused.
"""
import re
import struct
import sys

MEMSTART = 0x80000070
LINK0IN = 0x80000010
B_HI = 0x801F0000          # the running copy of the bootstrap
B_WS = 0x801FF100          # its workspace = BOOTINFO
B_BLOB = 0x801FF200        # [len] bootstrap, assembled to send on
WS_TOP = 0x80180000        # main process's workspace top
IMAGE_LIMIT = WS_TOP - 0x10000  # image + BSS must end below this

# --- a small transputer assembler ---------------------------------------

FN = dict(j=0x0, ldlp=0x1, pfix=0x2, ldnl=0x3, ldc=0x4, ldnlp=0x5,
          nfix=0x6, ldl=0x7, adc=0x8, call=0x9, cj=0xA, ajw=0xB, eqc=0xC,
          stl=0xD, stnl=0xE, opr=0xF)
OP = dict(rev=0x00, diff=0x04, gcall=0x06, inp=0x07, gt=0x09, wsub=0x0A,
          out=0x0B, sb=0x3B, gajw=0x3C, mint=0x42, and_=0x46, move=0x4A,
          sum=0x52)


def s32(v):
    v &= 0xFFFFFFFF
    return v - (1 << 32) if v & 0x80000000 else v


def encode(fn, v):
    v = s32(v) if isinstance(v, int) and (v > 0x7FFFFFFF) else v
    if 0 <= v < 16:
        return [fn << 4 | v]
    if v >= 16:
        return encode(FN['pfix'], v >> 4) + [fn << 4 | (v & 15)]
    return encode(FN['nfix'], (~v) >> 4) + [fn << 4 | (v & 15)]


def assemble(prog):
    """prog: list of ('label', name) | (fn, operand) | ('op', name).
    An operand may be a function of (labels, address after the
    instruction).  Sizes are iterated until they settle."""
    sizes = [0 if it[0] == 'label' else 1 for it in prog]
    for _ in range(50):
        labels, addrs, addr = {}, [], 0
        for it, size in zip(prog, sizes):
            addrs.append(addr)
            if it[0] == 'label':
                labels[it[1]] = addr
            addr += size
        out, new = [], []
        for it, size, at in zip(prog, sizes, addrs):
            if it[0] == 'label':
                new.append(0)
                continue
            if it[0] == 'op':
                code = encode(FN['opr'], OP[it[1]])
            else:
                fn, opnd = it
                if callable(opnd):
                    opnd = opnd(labels, at + size)
                code = encode(FN[fn], opnd)
            new.append(len(code))
            out += code
        if new == sizes:
            return bytes(out), labels
        sizes = new
    raise SystemExit('bootstrap: sizes did not settle')


def rel(name):
    return lambda labels, end: labels[name] - end


def bootstrap():
    """The primary bootstrap.  Locals (at B_WS once it gets there):
    0 index, 1 n, 2 down, 3 filesz, 4 memsz, 5 boot link in, 6 down link
    out, 7 zero pointer, 8 BSS end."""
    def body(length):
        L = lambda n: ('ldl', n)
        S = lambda n: ('stl', n)
        C = lambda v: ('ldc', v)
        O = lambda n: ('op', n)
        return [
            ('label', 'start'),
            S(1), S(1), S(1),                   # keep C, the boot link
            C(MEMSTART), C(B_HI), C(length), O('move'),
            ('ldc', lambda lab, end: B_HI + lab['hi']), O('gcall'),
            ('label', 'hi'),                    # now running at B_HI
            L(1), C(B_WS), O('gajw'), O('rev'), S(5),
            L(5), C(LINK0IN), O('diff'), C(-13), O('and_'),
            ('cj', rel('linkok')),
            C(LINK0IN), S(5),                   # booted from the file
            ('label', 'linkok'),
            ('ldlp', 0), L(5), C(20), O('inp'),             # header
            C(MEMSTART), L(5), L(3), O('inp'),              # image
            L(1), L(0), ('adc', 1), O('gt'), ('cj', rel('last')),
            L(2), O('mint'), O('wsub'), S(6),               # down link out
            C(length), C(B_BLOB), O('sb'),
            C(B_HI), C(B_BLOB + 1), C(length), O('move'),
            C(B_BLOB), L(6), C(length + 1), O('out'),       # bootstrap
            L(0), ('adc', 1), S(0),
            ('ldlp', 0), L(6), C(20), O('out'),             # header
            L(0), ('adc', -1), S(0),
            C(MEMSTART), L(6), L(3), O('out'),              # image
            ('label', 'last'),
            L(3), C(MEMSTART), O('sum'), S(7),
            L(4), C(MEMSTART), O('sum'), S(8),
            ('label', 'zero'),
            L(8), L(7), O('gt'), ('cj', rel('go')),
            C(0), L(7), ('stnl', 0),
            L(7), ('adc', 4), S(7),
            ('j', rel('zero')),
            ('label', 'go'),
            C(WS_TOP), O('gajw'),
            C(MEMSTART), O('gcall'),
        ]
    # its own length is one of its operands: settle that too
    length = 128
    for _ in range(10):
        code, labels = assemble(body(length))
        padded = (len(code) + 3) & ~3   # Wptr after a link boot is
        if padded == length:            # MemStart + length: keep it aligned
            break
        length = padded
    else:
        raise SystemExit('bootstrap: length did not settle')
    code += bytes(length - len(code))
    if length > 252:
        raise SystemExit('bootstrap is %d bytes, more than a boot packet'
                         % length)
    return code


# --- ELF -----------------------------------------------------------------

def load_elf(path):
    data = open(path, 'rb').read()
    if data[:4] != b'\x7fELF' or data[4] != 1 or data[5] != 1:
        raise SystemExit('%s: not a 32-bit little-endian ELF' % path)
    phoff, = struct.unpack_from('<I', data, 0x1C)
    phentsize, phnum = struct.unpack_from('<HH', data, 0x2A)
    segs = []
    for i in range(phnum):
        (ptype, off, vaddr, paddr, filesz, memsz,
         flags, align) = struct.unpack_from('<8I', data, phoff + i * phentsize)
        if ptype == 1 and memsz:
            segs.append((vaddr, data[off:off + filesz], memsz))
    if not segs:
        raise SystemExit('%s: no PT_LOAD segment' % path)
    segs.sort()
    if segs[0][0] != MEMSTART:
        raise SystemExit('%s: loads at #%08X, not MemStart' % (path, segs[0][0]))
    image = bytearray()
    end = MEMSTART
    for vaddr, body, memsz in segs:
        if vaddr < MEMSTART + len(image):
            raise SystemExit('%s: overlapping segments' % path)
        image += bytes(vaddr - MEMSTART - len(image)) + body
        end = max(end, vaddr + memsz)
    image += bytes(-len(image) % 4)
    memsz = (end - MEMSTART + 3) & ~3
    if MEMSTART + memsz > IMAGE_LIMIT:
        raise SystemExit('%s: image and BSS reach #%08X, past #%08X'
                         % (path, MEMSTART + memsz, IMAGE_LIMIT))
    return bytes(image), memsz


# --- maps ----------------------------------------------------------------

CONNECT_HOST = re.compile(r'\s*Connect\s+HOST\s+to\s+processor\s+(\d+)\s+link\s+(\d+)')
CONNECT = re.compile(r'\s*Connect\s+processor\s+(\d+)\s+link\s+(\d+)\s+to\s+'
                     r'processor\s+(\d+)\s+link\s+(\d+)')


def read_chain(path):
    host, links, nodes = None, {}, set()
    for line in open(path):
        m = CONNECT_HOST.match(line)
        if m:
            host = (int(m.group(1)), int(m.group(2)))
            nodes.add(host[0])
            continue
        m = CONNECT.match(line)
        if m:
            a, la, b, lb = map(int, m.groups())
            links[(a, la)] = (b, lb)
            links[(b, lb)] = (a, la)
            nodes.update((a, b))
    if host != (0, 0):
        raise SystemExit('%s: HOST must be on processor 0 link 0' % path)
    n = max(nodes) + 1
    down = None
    for i in range(1, n):
        ups = [(la, lb) for (a, la), (b, lb) in links.items()
               if a == i - 1 and b == i]
        if len(ups) != 1:
            raise SystemExit('%s: processor %d is not chained to %d'
                             % (path, i, i - 1))
        if down is None:
            down = ups[0][0]
        elif ups[0][0] != down:
            raise SystemExit('%s: the chain leaves processor %d on link %d,'
                             ' not %d' % (path, i - 1, ups[0][0], down))
    if len(links) != 2 * (n - 1):
        raise SystemExit('%s: more links than a chain' % path)
    return n, down if down is not None else 2


def write_map(n, up, down, out):
    out.write('mkbtl.py : chain of %d transputers for the C ray tracer\n\n\n'
              % n)
    out.write('Boot path for network\n\n')
    out.write('  Boot processor 0 down link 0 from HOST\n')
    for i in range(1, n):
        out.write('  Boot processor %d down link %d from processor %d link %d\n'
                  % (i, up, i - 1, down))
    out.write('\n\nConnectivity for network\n\n')
    out.write('  Connect HOST to processor 0 link 0\n')
    for i in range(1, n):
        out.write('  Connect processor %d link %d to processor %d link %d\n'
                  % (i, up, i - 1, down))


def main(argv):
    if len(argv) >= 2 and argv[1] == 'dump':
        sys.stdout.write(bootstrap().hex() + '\n')
        return
    if len(argv) == 5 and argv[1] == 'btl':
        elf, mapfile, btl = argv[2:]
        image, memsz = load_elf(elf)
        n, down = read_chain(mapfile)
        boot = bootstrap()
        with open(btl, 'wb') as f:
            f.write(bytes([len(boot)]) + boot)
            f.write(struct.pack('<5i', 0, n, down, len(image), memsz))
            f.write(image)
        print('%s: %d nodes, chain on link %d, bootstrap %d bytes, image %d'
              ' bytes (%d with BSS)' % (btl, n, down, len(boot), len(image),
                                        memsz))
        return
    if len(argv) >= 3 and argv[1] == 'map':
        n, up, down = int(argv[2]), 1, 2
        args = argv[3:]
        while args:
            if args[0] == '--up':
                up = int(args[1])
            elif args[0] == '--down':
                down = int(args[1])
            else:
                raise SystemExit('unknown option ' + args[0])
            args = args[2:]
        write_map(n, up, down, sys.stdout)
        return
    raise SystemExit(__doc__)


if __name__ == '__main__':
    main(sys.argv)
