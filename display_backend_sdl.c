#include "display_backend.h"

#ifdef T4_SDL_FB
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <SDL2/SDL.h>

extern int32_t quit;

static SDL_Window *display_window = NULL;
static SDL_Renderer *display_renderer = NULL;
static SDL_Texture *display_texture = NULL;
static int sdl_ready = 0;
static int quit_pending = 0;  /* 1 = first X click, waiting for confirm */
static struct timeval last_present;

/* -----------------------------------------------------------------------
 * Input UART FIFOs — keyboard and mouse byte streams
 * ----------------------------------------------------------------------- */

unsigned char kbd_fifo[INPUT_FIFO_SIZE];
int kbd_fifo_head = 0;
int kbd_fifo_tail = 0;

unsigned char mouse_fifo[INPUT_FIFO_SIZE];
int mouse_fifo_head = 0;
int mouse_fifo_tail = 0;

void kbd_fifo_push(unsigned char ch)
{
    int next = (kbd_fifo_head + 1) % INPUT_FIFO_SIZE;
    if (next != kbd_fifo_tail) {
        kbd_fifo[kbd_fifo_head] = ch;
        kbd_fifo_head = next;
    }
}

void kbd_fifo_push_str(const char *s)
{
    while (*s)
        kbd_fifo_push((unsigned char)*s++);
}

int kbd_fifo_available(void)
{
    return kbd_fifo_head != kbd_fifo_tail;
}

unsigned char kbd_fifo_pop(void)
{
    if (kbd_fifo_head == kbd_fifo_tail)
        return 0;
    unsigned char ch = kbd_fifo[kbd_fifo_tail];
    kbd_fifo_tail = (kbd_fifo_tail + 1) % INPUT_FIFO_SIZE;
    return ch;
}

void mouse_fifo_push(unsigned char ch)
{
    int next = (mouse_fifo_head + 1) % INPUT_FIFO_SIZE;
    if (next != mouse_fifo_tail) {
        mouse_fifo[mouse_fifo_head] = ch;
        mouse_fifo_head = next;
    }
}

int mouse_fifo_available(void)
{
    return mouse_fifo_head != mouse_fifo_tail;
}

unsigned char mouse_fifo_pop(void)
{
    if (mouse_fifo_head == mouse_fifo_tail)
        return 0;
    unsigned char ch = mouse_fifo[mouse_fifo_tail];
    mouse_fifo_tail = (mouse_fifo_tail + 1) % INPUT_FIFO_SIZE;
    return ch;
}

/* -----------------------------------------------------------------------
 * SDL key → VT100 byte sequence translation
 *
 * Printable ASCII: literal byte
 * Arrow keys:  ESC [ A/B/C/D
 * F1-F12:      ESC O P/Q/R/S, ESC [ 15~, ESC [ 17~ ... ESC [ 24~
 * Home/End:    ESC [ H / ESC [ F
 * PgUp/PgDn:   ESC [ 5~ / ESC [ 6~
 * Insert/Del:  ESC [ 2~ / ESC [ 3~
 * Backspace:   0x7F (or 0x08)
 * Tab:         0x09
 * Enter:       0x0D (CR)
 * Escape:      0x1B
 * ----------------------------------------------------------------------- */

static void sdl_key_to_uart(SDL_Keysym *ks)
{
    SDL_Keycode sym = ks->sym;
    uint16_t mod = ks->mod;

    /* Ctrl+C → 0x03 (interrupt) */
    if (sym == SDLK_c && (mod & KMOD_CTRL)) { kbd_fifo_push(0x03); return; }
    /* Ctrl+D → 0x04 (EOF) */
    if (sym == SDLK_d && (mod & KMOD_CTRL)) { kbd_fifo_push(0x04); return; }
    /* Ctrl+Z → 0x1A (suspend) */
    if (sym == SDLK_z && (mod & KMOD_CTRL)) { kbd_fifo_push(0x1A); return; }
    /* Ctrl+L → 0x0C (form feed / clear) */
    if (sym == SDLK_l && (mod & KMOD_CTRL)) { kbd_fifo_push(0x0C); return; }

    /* Arrow keys → VT100 */
    if (sym == SDLK_UP)    { kbd_fifo_push_str("\x1b[A"); return; }
    if (sym == SDLK_DOWN)  { kbd_fifo_push_str("\x1b[B"); return; }
    if (sym == SDLK_RIGHT) { kbd_fifo_push_str("\x1b[C"); return; }
    if (sym == SDLK_LEFT)  { kbd_fifo_push_str("\x1b[D"); return; }

    /* Home / End */
    if (sym == SDLK_HOME)  { kbd_fifo_push_str("\x1b[H"); return; }
    if (sym == SDLK_END)   { kbd_fifo_push_str("\x1b[F"); return; }

    /* Page Up / Page Down */
    if (sym == SDLK_PAGEUP)   { kbd_fifo_push_str("\x1b[5~"); return; }
    if (sym == SDLK_PAGEDOWN) { kbd_fifo_push_str("\x1b[6~"); return; }

    /* Insert / Delete */
    if (sym == SDLK_INSERT) { kbd_fifo_push_str("\x1b[2~"); return; }
    if (sym == SDLK_DELETE) { kbd_fifo_push_str("\x1b[3~"); return; }

    /* Function keys F1-F12 */
    if (sym == SDLK_F1)  { kbd_fifo_push_str("\x1bOP");   return; }
    if (sym == SDLK_F2)  { kbd_fifo_push_str("\x1bOQ");   return; }
    if (sym == SDLK_F3)  { kbd_fifo_push_str("\x1bOR");   return; }
    if (sym == SDLK_F4)  { kbd_fifo_push_str("\x1bOS");   return; }
    if (sym == SDLK_F5)  { kbd_fifo_push_str("\x1b[15~"); return; }
    if (sym == SDLK_F6)  { kbd_fifo_push_str("\x1b[17~"); return; }
    if (sym == SDLK_F7)  { kbd_fifo_push_str("\x1b[18~"); return; }
    if (sym == SDLK_F8)  { kbd_fifo_push_str("\x1b[19~"); return; }
    if (sym == SDLK_F9)  { kbd_fifo_push_str("\x1b[20~"); return; }
    if (sym == SDLK_F10) { kbd_fifo_push_str("\x1b[21~"); return; }
    if (sym == SDLK_F11) { kbd_fifo_push_str("\x1b[23~"); return; }
    if (sym == SDLK_F12) { kbd_fifo_push_str("\x1b[24~"); return; }

    /* Special keys */
    if (sym == SDLK_ESCAPE)    { kbd_fifo_push(0x1B); return; }
    if (sym == SDLK_RETURN)    { kbd_fifo_push(0x0D); return; }
    if (sym == SDLK_TAB)       { kbd_fifo_push(0x09); return; }
    if (sym == SDLK_BACKSPACE) { kbd_fifo_push(0x7F); return; }

    /* Printable ASCII — handle shift for uppercase */
    if (sym >= SDLK_SPACE && sym <= SDLK_z) {
        char ch = (char)sym;
        if (sym >= SDLK_a && sym <= SDLK_z && (mod & KMOD_SHIFT))
            ch = ch - 'a' + 'A';
        kbd_fifo_push((unsigned char)ch);
        return;
    }

    /* Shifted number row symbols */
    if (mod & KMOD_SHIFT) {
        switch (sym) {
        case SDLK_1: kbd_fifo_push('!'); return;
        case SDLK_2: kbd_fifo_push('@'); return;
        case SDLK_3: kbd_fifo_push('#'); return;
        case SDLK_4: kbd_fifo_push('$'); return;
        case SDLK_5: kbd_fifo_push('%'); return;
        case SDLK_6: kbd_fifo_push('^'); return;
        case SDLK_7: kbd_fifo_push('&'); return;
        case SDLK_8: kbd_fifo_push('*'); return;
        case SDLK_9: kbd_fifo_push('('); return;
        case SDLK_0: kbd_fifo_push(')'); return;
        case SDLK_MINUS:        kbd_fifo_push('_'); return;
        case SDLK_EQUALS:       kbd_fifo_push('+'); return;
        case SDLK_LEFTBRACKET:  kbd_fifo_push('{'); return;
        case SDLK_RIGHTBRACKET: kbd_fifo_push('}'); return;
        case SDLK_BACKSLASH:    kbd_fifo_push('|'); return;
        case SDLK_SEMICOLON:    kbd_fifo_push(':'); return;
        case SDLK_QUOTE:        kbd_fifo_push('"'); return;
        case SDLK_COMMA:        kbd_fifo_push('<'); return;
        case SDLK_PERIOD:       kbd_fifo_push('>'); return;
        case SDLK_SLASH:        kbd_fifo_push('?'); return;
        case SDLK_BACKQUOTE:    kbd_fifo_push('~'); return;
        default: break;
        }
    }

    /* Ignore modifiers (Shift, Ctrl, Alt) as standalone keypresses */
}

/* -----------------------------------------------------------------------
 * SDL mouse → 3-byte serial mouse packet
 *
 * Byte 0: 0x80 | buttons (bit0=L, bit1=M, bit2=R) — sync byte
 * Byte 1: dx (signed 8-bit, clamped to -127..+127)
 * Byte 2: dy (signed 8-bit, clamped to -127..+127)
 * ----------------------------------------------------------------------- */

static uint32_t mouse_buttons = 0;

static void mouse_send_packet(int dx, int dy)
{
    /* Clamp deltas to signed 8-bit range */
    if (dx > 127)  dx = 127;
    if (dx < -127) dx = -127;
    if (dy > 127)  dy = 127;
    if (dy < -127) dy = -127;

    mouse_fifo_push(0x80 | (mouse_buttons & 0x07));
    mouse_fifo_push((unsigned char)(dx & 0xFF));
    mouse_fifo_push((unsigned char)(dy & 0xFF));
}

/* -----------------------------------------------------------------------
 * Display timing
 * ----------------------------------------------------------------------- */

static long elapsed_us_since_last_present(void)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - last_present.tv_sec) * 1000000L +
           (now.tv_usec - last_present.tv_usec);
}

/* -----------------------------------------------------------------------
 * Display lifecycle
 * ----------------------------------------------------------------------- */

void display_init(void)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "[DISPLAY] SDL_Init failed: %s\n", SDL_GetError());
        return;
    }

    display_window = SDL_CreateWindow("Transputer T4 - VGA Display",
                                      SDL_WINDOWPOS_UNDEFINED,
                                      SDL_WINDOWPOS_UNDEFINED,
                                      FB_WIDTH, FB_HEIGHT, 0);
    if (!display_window) {
        fprintf(stderr, "[DISPLAY] SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return;
    }

    display_renderer = SDL_CreateRenderer(display_window, -1,
                                          SDL_RENDERER_ACCELERATED |
                                          SDL_RENDERER_PRESENTVSYNC);
    if (!display_renderer)
        display_renderer = SDL_CreateRenderer(display_window, -1, SDL_RENDERER_SOFTWARE);
    if (!display_renderer) {
        fprintf(stderr, "[DISPLAY] SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(display_window);
        display_window = NULL;
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return;
    }

    display_texture = SDL_CreateTexture(display_renderer,
                                        SDL_PIXELFORMAT_ARGB8888,
                                        SDL_TEXTUREACCESS_STREAMING,
                                        FB_WIDTH, FB_HEIGHT);
    if (!display_texture) {
        fprintf(stderr, "[DISPLAY] SDL_CreateTexture failed: %s\n", SDL_GetError());
        SDL_DestroyRenderer(display_renderer);
        display_renderer = NULL;
        SDL_DestroyWindow(display_window);
        display_window = NULL;
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return;
    }

    SDL_RenderSetLogicalSize(display_renderer, FB_WIDTH, FB_HEIGHT);
    memset(vga_framebuffer, 0, FB_SIZE);
    vga_dirty = 1;
    last_present.tv_sec = 0;
    last_present.tv_usec = 0;
    sdl_ready = 1;
    fprintf(stderr, "[DISPLAY] SDL backend ready (%dx%d, kbd UART 0x%X, mouse UART 0x%X)\n",
            FB_WIDTH, FB_HEIGHT, KBD_UART_BASE, MOUSE_UART_BASE);
}

void display_resize(uint32_t width, uint32_t height)
{
    if (!sdl_ready) return;
    if (width == 0 || height == 0 || width > 4096 || height > 4096) return;

    if (display_texture) {
        SDL_DestroyTexture(display_texture);
        display_texture = NULL;
    }
    display_texture = SDL_CreateTexture(display_renderer,
                                        SDL_PIXELFORMAT_ARGB8888,
                                        SDL_TEXTUREACCESS_STREAMING,
                                        width, height);
    if (!display_texture) {
        fprintf(stderr, "[DISPLAY] SDL_CreateTexture failed on resize: %s\n", SDL_GetError());
        return;
    }
    SDL_SetWindowSize(display_window, width, height);
    SDL_RenderSetLogicalSize(display_renderer, width, height);
    fprintf(stderr, "[DISPLAY] Resized to %ux%u\n", width, height);
}

void display_present(void)
{
    int pitch;
    void *pixels;
    uint32_t w = vga_ctrl_width;
    uint32_t h = vga_ctrl_height;
    uint32_t stride = vga_ctrl_stride;

    if (!sdl_ready || !vga_dirty) return;
    if (!(vga_ctrl_control & VGA_CTRL_ENABLE)) return;
    if (elapsed_us_since_last_present() < 16666) return;
    if (w == 0 || h == 0 || stride == 0) return;

    if (SDL_LockTexture(display_texture, NULL, &pixels, &pitch) != 0) {
        fprintf(stderr, "[DISPLAY] SDL_LockTexture failed: %s\n", SDL_GetError());
        return;
    }

    if ((uint32_t)pitch == stride) {
        uint32_t copy_size = stride * h;
        if (copy_size > FB_SIZE) copy_size = FB_SIZE;
        memcpy(pixels, vga_framebuffer, copy_size);
    } else {
        uint32_t y;
        uint32_t row_bytes = w * FB_BPP;
        const unsigned char *src = vga_framebuffer;
        unsigned char *dst = pixels;
        for (y = 0; y < h; y++) {
            memcpy(dst, src, row_bytes);
            src += stride;
            dst += pitch;
        }
    }

    SDL_UnlockTexture(display_texture);
    SDL_RenderClear(display_renderer);
    SDL_RenderCopy(display_renderer, display_texture, NULL, NULL);
    SDL_RenderPresent(display_renderer);
    vga_dirty = 0;
    gettimeofday(&last_present, NULL);
}

/* -----------------------------------------------------------------------
 * Event pump — SDL events → keyboard UART + mouse UART byte streams
 * ----------------------------------------------------------------------- */

void display_pump(void)
{
    SDL_Event event;

    if (!sdl_ready) return;

    while (SDL_PollEvent(&event)) {
        switch (event.type) {

        case SDL_QUIT:
            if (quit_pending) {
                /* Second click — force quit */
                fprintf(stderr, "[T4] Shutdown confirmed.\n");
                quit = 1;
            } else {
                /* First click — ask for confirmation */
                quit_pending = 1;
                fprintf(stderr, "[T4] Close requested — click X again to confirm, or keep running.\n");
                if (display_window)
                    SDL_SetWindowTitle(display_window, "Transputer T4 - Click X again to quit");
            }
            break;

        case SDL_KEYDOWN:
            if (!event.key.repeat)
                sdl_key_to_uart(&event.key.keysym);
            break;

        /* SDL_KEYUP: serial console is byte-stream, no key-up concept.
         * DOOM can parse VT100 sequences if needed, or use raw MMIO. */

        case SDL_MOUSEMOTION:
            if (event.motion.xrel || event.motion.yrel)
                mouse_send_packet(event.motion.xrel, event.motion.yrel);
            break;

        case SDL_MOUSEBUTTONDOWN: {
            if (event.button.button == SDL_BUTTON_LEFT)   mouse_buttons |= 1;
            if (event.button.button == SDL_BUTTON_MIDDLE) mouse_buttons |= 2;
            if (event.button.button == SDL_BUTTON_RIGHT)  mouse_buttons |= 4;
            mouse_send_packet(0, 0);  /* button-change packet with zero motion */
            break;
        }

        case SDL_MOUSEBUTTONUP: {
            if (event.button.button == SDL_BUTTON_LEFT)   mouse_buttons &= ~1;
            if (event.button.button == SDL_BUTTON_MIDDLE) mouse_buttons &= ~2;
            if (event.button.button == SDL_BUTTON_RIGHT)  mouse_buttons &= ~4;
            mouse_send_packet(0, 0);
            break;
        }

        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_EXPOSED)
                vga_dirty = 1;
            break;
        }
    }
}

void display_shutdown(void)
{
    if (display_texture)  { SDL_DestroyTexture(display_texture);   display_texture = NULL; }
    if (display_renderer) { SDL_DestroyRenderer(display_renderer); display_renderer = NULL; }
    if (display_window)   { SDL_DestroyWindow(display_window);     display_window = NULL; }
    if (sdl_ready) {
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        SDL_Quit();
        sdl_ready = 0;
    }
}
#endif
