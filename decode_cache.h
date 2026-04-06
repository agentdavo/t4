/*
 * decode_cache.h — Instruction decode cache for T4 emulator
 *
 * Small direct-mapped cache that stores pre-decoded instructions.
 * On cache hit, skips all PFIX/NFIX prefix processing.
 * On miss, decodes normally and fills the cache entry.
 *
 * Cache size: 64K entries × 16 bytes = 1MB (fits in L2 cache).
 */
#ifndef DECODE_CACHE_H
#define DECODE_CACHE_H

#include <stdint.h>
#include <string.h>

/* Cache size — must be power of 2. 64K entries = 1MB total. */
#define DCACHE_BITS   16
#define DCACHE_SIZE   (1 << DCACHE_BITS)
#define DCACHE_MASK   (DCACHE_SIZE - 1)

/* Decoded instruction entry */
typedef struct {
    uint32_t tag;       /* IPtr that this entry was decoded from */
    uint32_t next_ip;   /* IPtr after this instruction (past all prefixes) */
    int32_t  operand;   /* Fully resolved operand */
    uint8_t  opcode;    /* High nibble (0x0-0xF) */
    uint8_t  valid;     /* 1 if entry is valid */
    uint8_t  _pad[2];
} dcache_entry_t;

/* The cache array — allocated statically for simplicity */
typedef struct {
    dcache_entry_t entries[DCACHE_SIZE];
    uint64_t hits;
    uint64_t misses;
} dcache_t;

/* Initialize (zero all entries) */
static inline void dcache_init(dcache_t *dc) {
    memset(dc, 0, sizeof(*dc));
}

/* Lookup — returns entry if hit, NULL if miss */
static inline dcache_entry_t *dcache_lookup(dcache_t *dc, uint32_t iptr) {
    uint32_t idx = iptr & DCACHE_MASK;
    dcache_entry_t *e = &dc->entries[idx];
    if (e->valid && e->tag == iptr) {
        dc->hits++;
        return e;
    }
    dc->misses++;
    return NULL;
}

/* Decode an instruction at iptr, fill cache entry.
 * Walks the PFIX/NFIX chain in memory, resolves operand.
 * Returns the filled entry. */
static inline dcache_entry_t *dcache_fill(dcache_t *dc, uint32_t iptr,
                                           const uint8_t *mem, uint32_t mask)
{
    uint32_t idx = iptr & DCACHE_MASK;
    dcache_entry_t *e = &dc->entries[idx];
    uint32_t pos = iptr;
    int32_t oreg = 0;

    /* Walk prefix chain */
    while (1) {
        uint8_t byte = mem[pos & mask];
        int op = (byte >> 4) & 0xF;
        int nib = byte & 0xF;
        oreg += nib;

        if (op == 0x2) {        /* PFIX */
            oreg <<= 4;
            pos++;
            continue;
        }
        if (op == 0x6) {        /* NFIX */
            oreg = (~oreg) << 4;
            pos++;
            continue;
        }

        /* Non-prefix: this is the actual instruction */
        e->tag = iptr;
        e->opcode = op;
        e->operand = oreg;
        e->next_ip = pos + 1;
        e->valid = 1;
        return e;
    }
}

/* Invalidate all entries (call when code is modified) */
static inline void dcache_invalidate(dcache_t *dc) {
    memset(dc->entries, 0, sizeof(dc->entries));
}

/* Invalidate a specific address (call on self-modifying code) */
static inline void dcache_invalidate_addr(dcache_t *dc, uint32_t addr) {
    uint32_t idx = addr & DCACHE_MASK;
    dc->entries[idx].valid = 0;
}

#endif /* DECODE_CACHE_H */
