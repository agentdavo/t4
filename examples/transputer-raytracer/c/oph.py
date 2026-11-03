#!/usr/bin/env python3
"""oph.py - itemise two T4_OPHIST runs (occam and C) in instructions and in
T800 cycles, by opcode and by routine.

  oph.py instructions.txt occam.txt occ_code.bin 0x80002B8C  c.txt c.elf.tccsym
         [outdir]

Cycles follow the INMOS table (t4/utils/instructions.txt, from the TCWG):
on-chip RAM, plus one cycle for each prefix byte.  Variable timings use
the operand t4 recorded (prod b+4, shifts n+k, in/out 2w+19, move 2w+8,
norm n+5, cj 2 or 4, lend 10 or 5); FPU a/b timings take b for executions
whose FA was double length.  Routines: C by its symbol map; occam by
splitting its code at `ret`.  An occam routine is paired with the C
function that is entered the same number of times.
"""
import re, sys, os, collections

def parse_cycles(path):
    """key -> (kind, args): ('n', c) fixed; ('lin', k, mul) = mul*var + k;
    ('sd', s, d) single/double; ('cj',); ('lend',)"""
    tab, names = {}, {}
    for line in open(path):
        m = re.match(r'([0-9A-F])x\s+(\w+)\s+(\S+)', line)
        if m:
            key = int(m.group(1), 16) << 4
            name, cyc = m.group(2), m.group(3)
        else:
            m = re.match(r'([0-9A-F]{2})\s+(s\s+)?(\w+)\s+(\S+)(\s+(\S+))?', line)
            if not m:
                continue
            code = int(m.group(1), 16)
            key = (0x300 if m.group(2) else 0x100) + code
            name, cyc = m.group(3), m.group(4)
            if cyc.endswith('/') and m.group(6):     # "27/ 29" split by spaces
                cyc = cyc + m.group(6)
        names[key] = name
        if name == 'cj':
            tab[key] = ('cj',)
        elif name == 'lend':
            tab[key] = ('lend',)
        elif re.fullmatch(r'\d+', cyc):
            tab[key] = ('n', int(cyc))
        elif re.fullmatch(r'(\d+)/\s*(\d+)', cyc):
            a, b = re.fullmatch(r'(\d+)/\s*(\d+)', cyc).groups()
            tab[key] = ('sd', int(a), int(b))
        else:
            m2 = re.fullmatch(r'(\d*)([bnw])\+(\d+)', cyc)
            if m2:
                mul = int(m2.group(1) or 1)
                tab[key] = ('lin', int(m2.group(3)), mul)
            else:
                tab[key] = ('n', 0)                  # '-', 'f': unknown
    names.setdefault(0x20, 'pfix'); names.setdefault(0x60, 'nfix')
    return tab, names

def cycles_of(tab, key, cnt, var, dbl, length):
    t = tab.get(key, ('n', 0))
    pre = cnt * max(length - 1, 0)
    k = t[0]
    if k == 'n':   c = t[1] * cnt
    elif k == 'sd': c = t[1] * (cnt - dbl) + t[2] * dbl
    elif k == 'lin': c = t[1] * cnt + t[2] * var
    elif k == 'cj': c = 2 * cnt + 2 * var
    elif k == 'lend': c = 5 * cnt + 5 * var
    else: c = 0
    return c + pre

def load(path):
    rows = []
    for line in open(path):
        p = line.split()
        if len(p) == 6:
            rows.append((int(p[0], 16), int(p[1], 16), int(p[2]), int(p[3]),
                         int(p[4]), int(p[5])))
    return rows

def c_ranges(tccsym):
    syms = []
    for line in open(tccsym):
        p = line.split()
        if len(p) >= 3 and p[1] in 'Tt':
            syms.append((int(p[0], 16), p[2]))
    syms.sort()
    return [(a, syms[i + 1][0] if i + 1 < len(syms) else a + 0x10000, n)
            for i, (a, n) in enumerate(syms)]

def occ_ranges(codebin, base):
    """split at ret (#22 #F0) by decoding the byte stream"""
    data = open(codebin, 'rb').read()
    starts, i = [base], 0
    while i < len(data):
        b = data[i]; i += 1
        if b >> 4 == 0xF and (b & 15) == 0 and i >= 2 and data[i - 2] == 0x22:
            starts.append(base + i)
    starts.append(base + len(data))
    return [(starts[j], starts[j + 1], 'R%02d@%08X' % (j, starts[j]))
            for j in range(len(starts) - 1) if starts[j + 1] > starts[j]]

def attribute(rows, ranges, tab):
    per = collections.defaultdict(lambda: collections.Counter())
    entry = {}
    for (a, key, ln, cnt, var, dbl) in rows:
        name = 'other@%05X' % ((a >> 8) & 0xFFFFF)
        for s, e, n in ranges:
            if s <= a < e:
                name = n
                if a == s or n not in entry:
                    entry[n] = cnt if a == s else entry.get(n, cnt)
                break
        cyc = cycles_of(tab, key, cnt, var, dbl, ln)
        per[name]['ins'] += cnt
        per[name]['cyc'] += cyc
        per[name]['pre'] += cnt * max(ln - 1, 0)
        per[name][('k', key)] += cnt
        per[name][('c', key)] += cyc
    return per, entry

def opcode_totals(rows, tab):
    ins, cyc = collections.Counter(), collections.Counter()
    for (a, key, ln, cnt, var, dbl) in rows:
        ins[key] += cnt
        cyc[key] += cycles_of(tab, key, cnt, var, dbl, ln)
    return ins, cyc

def main():
    itab, occf, occbin, occbase, cf, csym = sys.argv[1:7]
    out = sys.argv[7] if len(sys.argv) > 7 else '.'
    tab, names = parse_cycles(itab)
    nm = lambda k: names.get(k, ('opr#%X' % (k - 0x100)) if k >= 0x100 else '#%02X' % k)
    orow, crow = load(occf), load(cf)
    oi, oc = opcode_totals(orow, tab)
    ci, cc = opcode_totals(crow, tab)
    O, C = sum(oi.values()), sum(ci.values())
    OC, CC = sum(oc.values()), sum(cc.values())
    unknown = sorted({k for k in list(oi) + list(ci) if tab.get(k, ('n', 0)) == ('n', 0)})
    print('instructions: occam %d, C %d (%.2fx)' % (O, C, C / O))
    print('T800 cycles:  occam %d, C %d (%.2fx)' % (OC, CC, CC / OC))
    print('  (no timing for: %s)' % ', '.join(nm(k) for k in unknown))
    print()
    print('%-14s %12s %12s %12s %12s %12s' % ('opcode', 'occam ins', 'C ins',
                                              'occam cyc', 'C cyc', 'C-occ cyc'))
    keys = sorted(set(oi) | set(ci), key=lambda k: -(cc[k] - oc[k]))
    for k in keys:
        if oi[k] + ci[k] < (O + C) / 20000:
            continue
        print('%-14s %12d %12d %12d %12d %+12d' % (nm(k), oi[k], ci[k], oc[k],
                                                   cc[k], cc[k] - oc[k]))
    # routines
    op, oent = attribute(orow, occ_ranges(occbin, int(occbase, 16)), tab)
    cp, cent = attribute(crow, c_ranges(csym), tab)
    # the algorithm fixes how many divides, compares and square roots each
    # routine does, whichever compiler made it: pair on those and the calls
    SIG = (0x18C, 0x194, 0x195, 0x301, 0x196, 0x197)
    sig = lambda r: [r[('k', k)] for k in SIG]
    pairs, used = [], set()
    for on, ocnt in sorted(oent.items(), key=lambda x: -op[x[0]]['cyc']):
        cands = [cn for cn, ccnt in cent.items() if ccnt == ocnt and cn not in used]
        if cands:
            d = lambda cn: sum(abs(a - b) for a, b in zip(sig(op[on]), sig(cp[cn])))
            cands.sort(key=d)
            cn = cands[0]
            if len(cands) > 1 and d(cands[0]) == d(cands[1]):
                cn = '?' + '|'.join(cands[:3])
            else:
                used.add(cn)
        else:
            cn = '-'
        pairs.append((on, cn, ocnt))
    print()
    print('%-16s %-22s %10s %12s %12s %12s %12s %6s' % (
        'occam routine', 'C function', 'calls', 'occam ins', 'C ins',
        'occam cyc', 'C cyc', 'C/occ'))
    for on, cn, calls in pairs:
        if op[on]['cyc'] < OC / 1000:
            continue
        c = cp.get(cn, collections.Counter())
        print('%-16s %-22s %10d %12d %12d %12d %12d %6s' % (
            on, cn[:22], calls, op[on]['ins'], c['ins'], op[on]['cyc'], c['cyc'],
            '%.2f' % (c['cyc'] / op[on]['cyc']) if c['cyc'] else ''))
    for cn, c in sorted(cp.items(), key=lambda x: -x[1]['cyc']):
        if cn not in used and c['cyc'] > CC / 1000:
            print('%-16s %-22s %10s %12s %12d %12s %12d' % (
                '(unpaired)', cn[:22], cent.get(cn, ''), '', c['ins'], '', c['cyc']))
    # per-routine opcode diffs for the paired routines
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'routines.txt'), 'w') as f:
        for on, cn, calls in pairs:
            if cn not in cp or op[on]['cyc'] < OC / 200:
                continue
            o, c = op[on], cp[cn]
            f.write('== %s (occam %s), %d calls: instructions %d -> %d, '
                    'cycles %d -> %d (%.2fx), per call %.0f -> %.0f cycles\n' % (
                        cn, on, calls, o['ins'], c['ins'], o['cyc'], c['cyc'],
                        c['cyc'] / max(o['cyc'], 1), o['cyc'] / calls, c['cyc'] / calls))
            ks = {k[1] for k in list(o) + list(c) if isinstance(k, tuple)}
            for k in sorted(ks, key=lambda k: -(c[('c', k)] - o[('c', k)])):
                if o[('k', k)] == c[('k', k)] == 0:
                    continue
                f.write('   %-14s per call: occam %7.1f ins %8.1f cyc   C %7.1f ins %8.1f cyc\n' % (
                    nm(k), o[('k', k)] / calls, o[('c', k)] / calls,
                    c[('k', k)] / calls, c[('c', k)] / calls))
            f.write('\n')
    print('\nper-routine opcode breakdown: %s/routines.txt' % out)

main()
