/*
 * debug.h — Debug and trace infrastructure for T4 emulator
 *
 * All environment variable checks, symbol tables, trace output,
 * and debug state are collected here. The hot loop should call
 * debug_is_active() to decide whether to enter the debug path.
 */
#ifndef T4_DEBUG_H_NEW
#define T4_DEBUG_H_NEW

#include <stdint.h>

/* -------------------------------------------------------------------
 * Debug configuration — initialized once from env vars at startup
 * ------------------------------------------------------------------- */
typedef struct {
    /* Master enable flags (cached from env vars) */
    int bootdbg_enabled;
    int outbyte_trace_enabled;
    int uart_trace_enabled;
    int uart_console_enabled;
    int mmio_trace_enabled;
    int early_printk_trace_enabled;
    int nearest_sym_enabled;
    int link0_trace_enabled;
    int fb_console_enabled;
    int noisy_boot_trace;

    /* Symbol/address trace */
    uint32_t sym_trace_entry;
    int      sym_trace_remaining;
    int      sym_trace_active;
    const char *sym_trace_name;

    uint32_t iptr_trace_start;
    uint32_t iptr_trace_end;
    int      iptr_trace_enabled;

    /* Any debug feature active? (fast check for hot loop) */
    int any_active;
} debug_config_t;

/* Initialize debug config from environment variables.
 * Call once at startup. Sets any_active=1 if any feature is enabled. */
void debug_init(debug_config_t *dbg);

/* Fast check: is ANY debug/trace feature active?
 * Returns 0 for production runs (skips all debug overhead). */
static inline int debug_is_active(const debug_config_t *dbg) {
    return dbg->any_active;
}

#endif /* T4_DEBUG_H_NEW */
