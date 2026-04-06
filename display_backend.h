#ifndef DISPLAY_BACKEND_H
#define DISPLAY_BACKEND_H

#include <stdint.h>

#ifdef T4_SDL_FB
#define FB_BASE         ((uint32_t)0x90000000)
#define FB_WIDTH        800
#define FB_HEIGHT       600
#define FB_BPP          4
#define FB_SIZE         (FB_WIDTH * FB_HEIGHT * FB_BPP)
#define FB_TOTAL        (FB_SIZE * 2)  /* front + back buffer */
#define FB_LIMIT        (FB_BASE + FB_TOTAL)  /* cover both buffers */
#define FB_CONSOLE_COLS (FB_WIDTH / 8)
#define FB_CONSOLE_ROWS (FB_HEIGHT / 8)

#define VGA_CTRL_BASE       ((uint32_t)0xA0000000)
#define VGA_CTRL_CONTROL    (VGA_CTRL_BASE + 0x00)
#define VGA_CTRL_FB_BASE    (VGA_CTRL_BASE + 0x04)
#define VGA_CTRL_WIDTH      (VGA_CTRL_BASE + 0x08)
#define VGA_CTRL_HEIGHT     (VGA_CTRL_BASE + 0x0C)
#define VGA_CTRL_STRIDE     (VGA_CTRL_BASE + 0x10)
#define VGA_CTRL_FLIP       (VGA_CTRL_BASE + 0x14)
#define VGA_CTRL_SCREENSHOT (VGA_CTRL_BASE + 0x18)
#define VGA_CTRL_ENABLE     0x01

/* ---------------------------------------------------------------------------
 * Input UARTs — same register layout as the console UART (0x10000000)
 *
 * Each UART has:
 *   +0x00  DATA    — read: pop byte from RX FIFO; write: ignored
 *   +0x04  STATUS  — bit 0: TX ready (always 1), bit 1: RX data available
 *
 * Keyboard UART: SDL key events → VT100 escape sequences (byte stream)
 *   Linux sees this as a serial keyboard — works with any tty/console driver.
 *   Printable keys: ASCII byte.  Special keys: ESC [ A/B/C/D etc.
 *
 * Mouse UART: SDL mouse events → simple 3-byte packet protocol
 *   Byte 0: 0x80 | buttons (bit0=L, bit1=M, bit2=R)
 *   Byte 1: dx (signed 8-bit, positive = right)
 *   Byte 2: dy (signed 8-bit, positive = down)
 *   Linux can use this with a simple serial mouse driver.
 * --------------------------------------------------------------------------- */

#define KBD_UART_BASE       ((uint32_t)0x11000000)
#define KBD_UART_DATA       (KBD_UART_BASE + 0x00)
#define KBD_UART_STATUS     (KBD_UART_BASE + 0x04)

#define MOUSE_UART_BASE     ((uint32_t)0x12000000)
#define MOUSE_UART_DATA     (MOUSE_UART_BASE + 0x00)
#define MOUSE_UART_STATUS   (MOUSE_UART_BASE + 0x04)

#define INPUT_UART_STATUS_TXRDY   0x01
#define INPUT_UART_STATUS_RXAVAIL 0x02

/* FIFO size for each input UART */
#define INPUT_FIFO_SIZE     512

/* Keyboard FIFO */
extern unsigned char kbd_fifo[INPUT_FIFO_SIZE];
extern int kbd_fifo_head;
extern int kbd_fifo_tail;

/* Mouse FIFO */
extern unsigned char mouse_fifo[INPUT_FIFO_SIZE];
extern int mouse_fifo_head;
extern int mouse_fifo_tail;

/* Push bytes into FIFOs (called from SDL event handler) */
void kbd_fifo_push(unsigned char ch);
void kbd_fifo_push_str(const char *s);
void mouse_fifo_push(unsigned char ch);

/* Check data available */
int kbd_fifo_available(void);
int mouse_fifo_available(void);

/* Pop byte from FIFO */
unsigned char kbd_fifo_pop(void);
unsigned char mouse_fifo_pop(void);

/* VGA / display */
extern uint32_t vga_ctrl_control;
extern uint32_t vga_ctrl_fb_base;
extern uint32_t vga_ctrl_width;
extern uint32_t vga_ctrl_height;
extern uint32_t vga_ctrl_stride;
extern unsigned char vga_framebuffer[FB_TOTAL];
extern int vga_dirty;

void display_init(void);
void display_resize(uint32_t width, uint32_t height);
void display_present(void);
void display_pump(void);
void display_shutdown(void);
#endif

#endif
