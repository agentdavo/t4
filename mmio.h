/*
 * mmio.h — Memory-mapped I/O device interface for T4 emulator
 *
 * Separates device emulation from the CPU dispatch loop.
 * Each device handles its own address range.
 */
#ifndef MMIO_H
#define MMIO_H

#include <stdint.h>

/* -------------------------------------------------------------------
 * Device address map
 * ------------------------------------------------------------------- */
#define MMIO_UART0_DATA     0x10000000u
#define MMIO_UART0_STATUS   0x10000004u
#define MMIO_UART1_DATA     0x11000000u  /* Keyboard */
#define MMIO_UART1_STATUS   0x11000004u
#define MMIO_UART2_DATA     0x12000000u  /* Mouse */
#define MMIO_UART2_STATUS   0x12000004u
#define MMIO_SD_BASE        0x20000000u
#define MMIO_SD_SIZE        (128u * 1024 * 1024)
#define MMIO_NET_BASE       0x30000000u
#define MMIO_FB_BASE        0x81E00000u  /* RAM carveout; was 0x90000000 */
#define MMIO_FB_LEGACY_BASE 0x90000000u
#define MMIO_VGA_CTRL       0xA0000000u

/* UART status bits */
#define UART_STATUS_TXRDY   0x01
#define UART_STATUS_RXAVAIL 0x02

/* -------------------------------------------------------------------
 * MMIO dispatch — called from memory read/write when address is
 * in the MMIO range (0x10000000-0xAFFFFFFF).
 * ------------------------------------------------------------------- */

/* Check if address falls in any MMIO device range */
static inline int mmio_is_device_addr(uint32_t addr) {
    return (addr >= 0x10000000u && addr < 0xB0000000u);
}

/*
 * These will be implemented in mmio.c, calling into the existing
 * device handlers in p.c during the transition period.
 * Eventually, each device gets its own file (uart.c, sd.c, etc.).
 */

/* Word-level MMIO (most common) */
uint32_t mmio_read_word(uint32_t addr);
void     mmio_write_word(uint32_t addr, uint32_t value);

/* Byte-level MMIO */
uint8_t  mmio_read_byte(uint32_t addr);
void     mmio_write_byte(uint32_t addr, uint8_t value);

#endif /* MMIO_H */
