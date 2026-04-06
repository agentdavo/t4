/*
 * decode_cache.c — Pre-decode transputer instruction stream
 *
 * Scans code memory and resolves all PFIX/NFIX prefix chains ahead of time.
 * The emulator can then skip prefix processing entirely in the hot loop.
 */
#include "decode_cache.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

decoded_insn_t *decode_cache_build(const uint8_t *mem, uint32_t mem_mask,
                                    uint32_t start, uint32_t size)
{
    decoded_insn_t *cache = calloc(size, sizeof(decoded_insn_t));
    if (!cache) return NULL;

    uint32_t pos = 0;
    int32_t oreg = 0;
    uint32_t insn_start = 0;
    int prefix_count = 0;

    while (pos < size) {
        uint32_t addr = start + pos;
        uint8_t byte = mem[addr & mem_mask];
        int op = (byte >> 4) & 0xF;
        int nibble = byte & 0xF;

        if (op == 0x2) {  /* PFIX */
            if (prefix_count == 0) insn_start = pos;
            oreg = (oreg | nibble) << 4;
            cache[pos].is_prefix = 1;
            cache[pos].opcode = 0x2;
            prefix_count++;
            pos++;
            continue;
        }

        if (op == 0x6) {  /* NFIX */
            if (prefix_count == 0) insn_start = pos;
            oreg = (~(oreg | nibble)) << 4;
            cache[pos].is_prefix = 1;
            cache[pos].opcode = 0x6;
            prefix_count++;
            pos++;
            continue;
        }

        /* Non-prefix instruction: resolve final operand */
        int32_t operand = oreg | nibble;
        uint32_t next_ip = start + pos + 1;

        /* Fill the entry for THIS byte */
        cache[pos].opcode = op;
        cache[pos].operand = operand;
        cache[pos].next_ip = next_ip;
        cache[pos].is_prefix = 0;

        /* Also fill entries for all prefix bytes in this chain,
         * pointing to the same resolved instruction. This way,
         * if IPtr points to a prefix byte, we can jump straight
         * to the resolved instruction. */
        if (prefix_count > 0) {
            uint32_t p;
            for (p = insn_start; p < pos; p++) {
                cache[p].opcode = op;
                cache[p].operand = operand;
                cache[p].next_ip = next_ip;
                /* Keep is_prefix=1 so we know these are prefix bytes */
            }
        }

        /* Reset for next instruction */
        oreg = 0;
        prefix_count = 0;
        pos++;
    }

    return cache;
}
