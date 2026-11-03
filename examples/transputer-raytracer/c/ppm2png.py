#!/usr/bin/env python3
"""ppm2png.py - convert a binary PPM (P6, maxval 255) to PNG.

    ppm2png.py ray.ppm [ray.png]

Standard library only (zlib), because the machines this runs on have no
ImageMagick or Pillow.  Rows are written with the PNG "Up" filter, which
suits the ray tracer's large flat areas.
"""
import struct
import sys
import zlib


def read_ppm(path):
    data = open(path, 'rb').read()
    fields, pos = [], 0
    while len(fields) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(data[start:pos])
    if fields[0] != b'P6' or int(fields[3]) != 255:
        raise SystemExit('%s: not a P6 PPM with maxval 255' % path)
    w, h = int(fields[1]), int(fields[2])
    pixels = data[pos + 1:pos + 1 + 3 * w * h]
    if len(pixels) != 3 * w * h:
        raise SystemExit('%s: %d bytes of pixels, want %d'
                         % (path, len(pixels), 3 * w * h))
    return w, h, pixels


def chunk(kind, body):
    return (struct.pack('>I', len(body)) + kind + body +
            struct.pack('>I', zlib.crc32(kind + body) & 0xFFFFFFFF))


def write_png(path, w, h, pixels):
    stride = 3 * w
    raw = bytearray()
    prev = bytes(stride)
    for y in range(h):
        row = pixels[y * stride:(y + 1) * stride]
        raw.append(2)                           # Up
        raw += bytes((a - b) & 0xFF for a, b in zip(row, prev))
        prev = row
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(bytes(raw), 9)))
        f.write(chunk(b'IEND', b''))


def main(argv):
    if len(argv) not in (2, 3):
        raise SystemExit(__doc__)
    src = argv[1]
    dst = argv[2] if len(argv) == 3 else src.rsplit('.', 1)[0] + '.png'
    w, h, pixels = read_ppm(src)
    write_png(dst, w, h, pixels)
    print('%s: %d x %d' % (dst, w, h))


if __name__ == '__main__':
    main(sys.argv)
