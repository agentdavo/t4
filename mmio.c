/*
 * mmio.c — Memory-mapped I/O device dispatch
 *
 * Thin wrapper that routes MMIO reads/writes to the correct device.
 * During the transition, this calls back into p.c's existing handlers.
 * Eventually, each device gets its own implementation file.
 */
#include "mmio.h"
#include <stdio.h>

/*
 * These are the existing handlers in p.c.
 * We declare them extern here and call through.
 * Once p.c is fully refactored, these move into per-device files.
 */
extern uint32_t word_int(uint32_t ptr);
extern void writeword_int(uint32_t ptr, uint32_t value);
extern unsigned char byte_int(uint32_t ptr);
extern void writebyte_int(uint32_t ptr, unsigned char value);

/*
 * MMIO word read — dispatches to the correct device.
 * Currently just calls p.c's word_int which handles all MMIO inline.
 * Future: direct dispatch to per-device handlers.
 */
uint32_t mmio_read_word(uint32_t addr)
{
    return word_int(addr);
}

void mmio_write_word(uint32_t addr, uint32_t value)
{
    writeword_int(addr, value);
}

uint8_t mmio_read_byte(uint32_t addr)
{
    return byte_int(addr);
}

void mmio_write_byte(uint32_t addr, uint8_t value)
{
    writebyte_int(addr, value);
}
