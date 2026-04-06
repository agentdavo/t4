#ifndef DISPLAY_BACKEND_H
#define DISPLAY_BACKEND_H

#include <stdint.h>

/* -------------------------------------------------------------------
 * Framebuffer constants — keep in sync with linux-7
 * arch/transputer/include/asm/simplefb.h.
 *
 * Guest surface is the top 2 MiB carveout of the 32 MiB RAM window
 * (0x81E00000). Host still keeps a private double buffer (FB_TOTAL)
 * for FLIP; only FB_SIZE bytes are guest-visible (FB_LIMIT).
 * Legacy MMIO window 0x90000000 is aliased for older images.
 * ------------------------------------------------------------------- */
/* Display windows are runtime variables so that large-memory runs can move
 * them out of the RAM range; see t4_relocate_mmio_for_ram() in p.c.
 * Defaults keep the historic map exactly, so the kernel and fbwm are
 * unaffected at the sizes they actually use (<= 256 MB). */
extern uint32_t fb_base;
#define FB_BASE         fb_base
#define FB_BASE_DEFAULT ((uint32_t)0x81E00000)
/* The legacy framebuffer sits inside the transputer RAM window
 * (MemStart = 0x80000000), so it caps RAM at 256 MB.  For large-memory runs
 * the emulator relocates it below MemStart; see t4_relocate_mmio_for_ram(). */
extern uint32_t fb_legacy_base;
extern uint32_t vga_ctrl_base;
#define FB_LEGACY_BASE  fb_legacy_base
#define FB_LEGACY_DEFAULT ((uint32_t)0x90000000)
#define VGA_CTRL_DEFAULT  ((uint32_t)0xA0000000)
#define FB_WIDTH        800
#define FB_HEIGHT       600
#define FB_BPP          4
#define FB_SIZE         (FB_WIDTH * FB_HEIGHT * FB_BPP)
#define FB_TOTAL        (FB_SIZE * 2)  /* host front + back buffer */
#define FB_LIMIT        (FB_BASE + FB_SIZE)  /* guest-visible front only */
#define FB_LEGACY_LIMIT (FB_LEGACY_BASE + FB_SIZE)
#define FB_CONSOLE_COLS (FB_WIDTH / 8)
#define FB_CONSOLE_ROWS (FB_HEIGHT / 8)

#define VGA_CTRL_BASE       vga_ctrl_base
#define VGA_CTRL_CONTROL    (VGA_CTRL_BASE + 0x00)
#define VGA_CTRL_FB_BASE    (VGA_CTRL_BASE + 0x04)
#define VGA_CTRL_WIDTH      (VGA_CTRL_BASE + 0x08)
#define VGA_CTRL_HEIGHT     (VGA_CTRL_BASE + 0x0C)
#define VGA_CTRL_STRIDE     (VGA_CTRL_BASE + 0x10)
#define VGA_CTRL_FLIP       (VGA_CTRL_BASE + 0x14)
#define VGA_CTRL_SCREENSHOT (VGA_CTRL_BASE + 0x18)
/* Guest write → t9k_leave_p(): abandon P without trap (spawn longjmp path). */
#define VGA_CTRL_LEAVE_P    (VGA_CTRL_BASE + 0x1C)
#define VGA_CTRL_ENABLE     0x01

/* -------------------------------------------------------------------
 * Input UART addresses — also available to all build paths.
 * ------------------------------------------------------------------- */
#define KBD_UART_BASE       ((uint32_t)0x11000000)
#define KBD_UART_DATA       (KBD_UART_BASE + 0x00)
#define KBD_UART_STATUS     (KBD_UART_BASE + 0x04)

#define MOUSE_UART_BASE     ((uint32_t)0x12000000)
#define MOUSE_UART_DATA     (MOUSE_UART_BASE + 0x00)
#define MOUSE_UART_STATUS   (MOUSE_UART_BASE + 0x04)

#define INPUT_UART_STATUS_TXRDY   0x01
#define INPUT_UART_STATUS_RXAVAIL 0x02

#if defined(T4_SDL_FB) || defined(T4_X11_FB)
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

/* P2: which buffer is currently "front" for present (byte offset into
 * vga_framebuffer). FLIP can swap this instead of full-frame memcpy. */
extern uint32_t vga_front_offset;

/* Dirty scanline range (inclusive). -1 = clean. */
extern int vga_dirty_y0;
extern int vga_dirty_y1;

/* Headless: no SDL window (UART-only boots). Set by display_init(). */
extern int display_headless;

void display_init(void);
void display_resize(uint32_t width, uint32_t height);
/* present_mode: 0 = only if dirty + rate-limit (legacy timer path, often no-op
 * when T4_PRESENT_ON_FLIP=1); 1 = force present now (FLIP register). */
void display_present(void);
void display_present_force(void);
void display_pump(void);
void display_shutdown(void);

/* Mark dirty rows after a pixel write (called from p.c MMIO path) */
static inline void vga_mark_dirty_offset(uint32_t fb_off)
{
	uint32_t stride = vga_ctrl_stride ? vga_ctrl_stride : (FB_WIDTH * FB_BPP);
	int y;
	if (stride == 0) return;
	y = (int)(fb_off / stride);
	if (y < 0) y = 0;
	if (vga_dirty_y0 < 0 || y < vga_dirty_y0) vga_dirty_y0 = y;
	if (vga_dirty_y1 < 0 || y > vga_dirty_y1) vga_dirty_y1 = y;
	vga_dirty = 1;
}
#endif

#endif
