/*
 * debug_config.c — Initialize debug configuration from environment variables
 *
 * This replaces the ~30 individual getenv() calls scattered throughout
 * p.c's mainloop startup. All debug state is collected into one struct.
 */
#include "debug.h"
#include <stdlib.h>
#include <string.h>

static int env_bool(const char *name)
{
    const char *val = getenv(name);
    return (val && *val) ? 1 : 0;
}

void debug_init(debug_config_t *dbg)
{
    memset(dbg, 0, sizeof(*dbg));

    dbg->bootdbg_enabled         = env_bool("T4_BOOTDBG");
    dbg->outbyte_trace_enabled   = env_bool("T4_OUTBYTE_TRACE");
    dbg->uart_trace_enabled      = env_bool("T4_UART_TRACE");
    dbg->uart_console_enabled    = env_bool("T4_UART_CONSOLE");
    dbg->mmio_trace_enabled      = env_bool("T4_MMIO_TRACE");
    dbg->early_printk_trace_enabled = env_bool("T4_EARLY_PRINTK_TRACE");
    dbg->nearest_sym_enabled     = env_bool("T4_NEAREST_SYM");
    dbg->link0_trace_enabled     = env_bool("T4_LINK0_TRACE");
    dbg->fb_console_enabled      = env_bool("T4_FB_CONSOLE");
    dbg->noisy_boot_trace        = env_bool("T4_NOISY_BOOT");

    /* Override: force console mode */
    if (env_bool("T4_UART_CONSOLE_FORCE")) {
        dbg->uart_console_enabled = 1;
        dbg->uart_trace_enabled = 0;
    }

    /* Symbol trace */
    {
        const char *sym = getenv("T4_SYM_TRACE");
        if (sym && *sym) {
            dbg->sym_trace_name = sym;
            dbg->sym_trace_remaining = 1000; /* default trace length */
            const char *count = getenv("T4_SYM_TRACE_COUNT");
            if (count && *count)
                dbg->sym_trace_remaining = atoi(count);
        }
    }

    /* IPtr range trace */
    {
        const char *start = getenv("T4_IPTR_TRACE_START");
        const char *end   = getenv("T4_IPTR_TRACE_END");
        if (start && *start && end && *end) {
            dbg->iptr_trace_start = (uint32_t)strtoul(start, NULL, 16);
            dbg->iptr_trace_end   = (uint32_t)strtoul(end, NULL, 16);
            dbg->iptr_trace_enabled = 1;
        }
    }

    /* Set master flag */
    dbg->any_active = dbg->bootdbg_enabled ||
                      dbg->outbyte_trace_enabled ||
                      dbg->uart_trace_enabled ||
                      dbg->mmio_trace_enabled ||
                      dbg->early_printk_trace_enabled ||
                      dbg->nearest_sym_enabled ||
                      dbg->noisy_boot_trace ||
                      dbg->sym_trace_name != NULL ||
                      dbg->iptr_trace_enabled;
}
