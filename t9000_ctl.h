/*
 * IMS T9000 control system for the T4 emulator: the control links CLink0
 * and CLink1, the packet handler, the control unit (command handler and
 * autonomous control) and the system services registers.
 *
 * Spec: t9000hrm.txt Part 2 ch.8 (control system: 8.3 functional
 * description, 8.4 commands, 8.4.1 gross state, 8.5 errors, 8.6.1 booting
 * from link, 8.7 reset levels), ch.13.5-13.6 (link errors, link registers),
 * ch.15 (configuration register map), Part 1 ch.7 (routing control
 * channels).
 *
 * A control link is wired like a data link: in the network map it is link 4
 * (CLink0) or 5 (CLink1), "processor N clink C".  It carries packets in the
 * VCP's format, always with 2-byte headers:
 *
 *     header (2 bytes) | data (0..32 bytes) | token (EOP or EOM)
 *
 * The controlling process is ordinary T9000 code using a virtual link on one
 * of its data links (HRM 8.1: "a control link network is generally connected
 * to one of the data links of a controlling IMS T9000"), so the controlling
 * side needs nothing here.
 *
 * Command codes.  The figures that give them (HRM fig 8.6, 8.7) are
 * illegible in the available text; three survive: IdentifyResult #80,
 * CPeekHandshake #82, PeekHandshake #F4, so Identify = #00, CPeek = #02 and
 * Peek = #74 ("response codes are the same as the command codes except with
 * the top bit inverted").  The rest are T4 assignments, chosen around those.
 * Every handshake is: response code, status byte (0 = valid, 1 = invalid or
 * failed), then the data for IdentifyResult, CPeek and Peek (4 bytes,
 * least significant first, zero on failure).
 */
#ifndef T9000_CTL_H
#define T9000_CTL_H

#include <stdint.h>

/* Commands (HRM 8.4, figure 8.6); response = command ^ #80 */
#define T9K_CC_IDENTIFY         0x00u   /* HRM fig 8.7 */
#define T9K_CC_CPEEK            0x02u   /* HRM fig 8.7 */
#define T9K_CC_CPOKE            0x03u
#define T9K_CC_START            0x10u
#define T9K_CC_RESET            0x11u
#define T9K_CC_STOP             0x12u
#define T9K_CC_RECOVERERROR     0x13u
#define T9K_CC_ERRORHANDSHAKE   0x60u   /* controller's reply to an Error */
#define T9K_CC_PEEK             0x74u   /* HRM fig 8.7 */
#define T9K_CC_POKE             0x75u
#define T9K_CC_BOOT             0x76u
#define T9K_CC_BOOTDATA         0x77u
#define T9K_CC_RUN              0x78u
#define T9K_CC_REBOOT           0x79u
#define T9K_CC_ERROR            0xE0u   /* Error message, device -> controller */
#define T9K_CC_RESPONSE         0x80u

/* Error codes (HRM table 8.9), highest priority first */
#define T9K_CE_CLINK0_CMD       0xC0u   /* invalid command code */
#define T9K_CE_CLINK0_PROTO     0xC1u   /* length, termination, protocol */
#define T9K_CE_CLINK1           0xC2u   /* parity or disconnect on CLink1 */
#define T9K_CE_LINK(n)          (0x80u + (uint32_t)(n))
#define T9K_CE_CPU              0x01u
#define T9K_CE_VCP              0x02u
#define T9K_CE_PMI              0x03u

/* System services and control link registers (HRM tables 8.3-8.6, 15.11) */
#define T9K_CR_MODESTATUS       0x1003u
#define T9K_CR_ERRORCODE        0x1004u
#define T9K_CR_DSLINKPLL        0x1005u
#define T9K_CR_SYSSERVWRITELOCK 0x1006u
#define T9K_CR_CLINK0MODE       0xFD01u /* +0x100 for CLink1: Mode, Command, Status, WriteLock */

#define T9K_MS_ERRORSINCERESET  0x1u
#define T9K_MS_STANDALONE       0x2u

/* t9k_ctl_poll() result */
#define T9K_CTL_ACTIVE          1       /* a control packet moved */
#define T9K_CTL_START           2       /* Run or Reboot: the CPU is set up to start */

struct cpu_t;

void t9k_ctl_reset_node (void);         /* init_processor: power-on state, wire CLinks */
void t9k_ctl_destroy (struct cpu_t *c);

/* Service both control links without waiting.  Called from linkcomms(). */
int  t9k_ctl_poll (void);
/* A control packet is waiting (idle wake-up test). */
int  t9k_ctl_ready (void);
/* Control traffic since the last call: a boot wait is not idle. */
int  t9k_ctl_busy (void);
/* Wait up to ms for any link traffic (a boot wait with nothing else to poll). */
void t9k_ctl_wait (int ms);

/* A subsystem error (HRM 8.5): recorded, and for a system error the CPU is
 * halted and the error reported on CLink0, or in stand-alone mode the chip
 * is reset and rebooted from ROM.  Returns 0 if there is no control system
 * to take it: no control link wired and StandAloneMode clear, a
 * configuration HRM 8.1.2 rules out, where T4 leaves the CPU running. */
int  t9k_ctl_error (uint32_t code);

/* mainloop() has entered / returned.  t9k_ctl_after_halt() serves the
 * control links while the CPU is stopped; 1 = Run or Reboot started it
 * again (call mainloop() once more), 0 = finish. */
void t9k_ctl_cpu_entry (void);
int  t9k_ctl_after_halt (void);
/* The control system set Wdesc and Iptr (Run/Reboot) for this start. */
int  t9k_ctl_started_cpu (void);

/* Link faults (HRM 13.5, 8.5.3); link 0-3 = Link0-3, 4-5 = CLink0-1.
 * t9k_line_signal: this node's output on `link` gives the far end a parity
 * (disconnect = 0) or disconnect error.  t9k_line_poll: act on the faults
 * the far ends have signalled here.  Threaded fabric and single-node
 * loopback only. */
void t9k_line_signal (int link, int disconnect);
void t9k_line_poll (void);

/* Configuration space: system services #1003-#1006 and CLink0-1.  Return 0
 * if a is not one of these, else 1 (done) or -1 (refused: not readable /
 * not writable / write-locked against the CPU). */
int  t9k_ctl_conf_read (uint32_t a, uint32_t *val);
int  t9k_ctl_conf_write (uint32_t a, uint32_t val, int by_cpu);

#endif /* T9000_CTL_H */
