/*
 * IMS T9000 external channels for the T4 emulator: the Virtual Channel
 * Processor (virtual links multiplexed over the physical links), event
 * channels, resource channels, variable-length communication and the
 * configuration register space.
 *
 * Spec: t9000ism.txt ch.8.4-8.8 and ch.12, appendix C (instruction
 * definitions); t9000hrm.txt ch.11 (VCP, packets, VLCBs) and ch.15
 * (configuration register map).
 *
 * Channel address space (ISM fig 12.5):
 *   #80000000..#8000000C  byte-stream outputs  (= T800 Link0-3Out)
 *   #80000010..#8000001C  byte-stream inputs   (= T800 Link0-3In)
 *   #80000020..#8000003C  event channels, input = #80000020 + 8n,
 *                         output = input + 4
 *   #80000040..           virtual channels, input = #80000040 + 8vl,
 *                         output = input + 4, up to MinInvalidChannel
 *   ..MemStart            illegal
 *   MemStart..            internal (soft) channels
 *
 * The VLCBs themselves are private to the VCP (ISM 12.2.1: "the details of
 * the information stored in this block are private to the VCP") and are
 * kept here rather than in guest memory below MemStart, which T4 loads boot
 * code into.  Packet buffers, the header region and the external resource
 * channel data structures are guest memory, addressed by the registers.
 */
#ifndef T9000_VCP_H
#define T9000_VCP_H

#include <stdint.h>

/* Constants (ISM tables A.12 / A.13) */
#define T9K_MIN_EVENT_CHAN      0x80000020u
#define T9K_MIN_VIRTUAL_CHAN    0x80000040u
#define T9K_BYTES_PER_VLCB      32u
#define T9K_MAX_HEADER_OFFSET   0xFFFEu
#define T9K_MAX_HEADER_LEN      0xFFu
#define T9K_MAX_SHORT_HEADER    3u
#define T9K_MAX_PACKET          32u
#define T9K_NUM_LINKS           4

#define T9K_DEACTIVATED_P       0x80000001u
#define T9K_RESCHAN_P           0x80000002u
#define T9K_STOPPING_P          0x80000003u
#define T9K_LENGTHERROR_P       0xFFFFFFFFu
#define T9K_NULHEADER           0xFFFFFFFFu
#define T9K_NULOFFSET           0xFFFFFFFFu

/* Packet terminators on the emulated link (HRM 11.2): the last byte of every
 * link message is the token.  A packet with no data ending in EOP is an
 * acknowledge. */
#define T9K_TOK_EOP             0x01u
#define T9K_TOK_EOM             0x02u

/* Configuration register addresses (HRM tables 15.3-15.10) */
#define T9K_CR_MININVALIDCHAN   0x010Fu
#define T9K_CR_INITIALIPTR      0x0111u
#define T9K_CR_INITIALWPTR      0x0112u
#define T9K_CR_REASON           0x0113u
#define T9K_CR_EMIBADADDRESS    0x0115u
#define T9K_CR_VCPSTATUS        0x0802u
#define T9K_CR_VCPCOMMAND       0x0804u
#define T9K_CR_VCPLINK0MODE     0x0805u   /* +4n: Mode, MaxHeader, MinHeader, HdrOffset */
#define T9K_CR_HDRAREABASE      0x0901u
#define T9K_CR_DEVICEID         0x1001u
#define T9K_CR_DEVICEREVISION   0x1002u   /* #1003-#1006: t9000_ctl.h */
#define T9K_CR_MEMSTART         0x410Eu
#define T9K_CR_EXTERNALRCBASE   0x4110u
#define T9K_CR_CHANWRITELOCK    0x4900u
#define T9K_CR_LINK0MODE        0x8001u   /* +0x100n: Mode, Command, Status, WriteLock */

/* VCPCommand bits (HRM table 11.7) */
#define T9K_VCPCMD_RESET        0x1u
#define T9K_VCPCMD_START        0x2u
#define T9K_VCPCMD_STOP         0x4u

/* VCPLinkNMode bits (HRM table 11.14) */
#define T9K_VLM_BYTEMODE        0x1u
#define T9K_VLM_LOCALIZEERROR   0x2u
#define T9K_VLM_HEADER2         0x4u

/* Hot-path test used by the communication instructions in p.c: true for any
 * channel address the CPU must hand to the VCP rather than treat as an
 * internal or byte-stream channel. */
#define T9K_EXTCHAN(a) \
        ((uint32_t)((a) - T9K_MIN_EVENT_CHAN) < t4cpu->s_vcp_span)

/* Kinds of output/input operation, for the packet engine. */
#define T9K_IO_FIXED            0
#define T9K_IO_VARIABLE         1

struct cpu_t;

void t9k_vcp_reset_node(void);          /* init_processor */
void t9k_vcp_destroy(struct cpu_t *c);

/* Communication instructions on external channels.  The caller has already
 * advanced IPtr; these may deschedule the current process. */
void t9k_ext_out(uint32_t chan, uint32_t src, uint32_t len, int kind);
void t9k_ext_in(uint32_t chan, uint32_t dst, uint32_t len, int kind);
/* enbc / disc on an external channel: 1 if the channel is ready. */
int  t9k_ext_enbc(uint32_t chan);
int  t9k_ext_disc(uint32_t chan);
/* resetch on an external channel: the new Areg. */
uint32_t t9k_ext_resetch(uint32_t chan);

/* Resource claim on an internal channel whose word is ResChan.p (out,
 * outbyte, outword and vout); the channel becomes an ordinary waiting
 * channel afterwards. */
void t9k_internal_claim(uint32_t chan);

/* T9000 channel and configuration instructions (OReg as executed).  The
 * caller has advanced IPtr.  Returns 0 if op is not handled here. */
int  t9k_vcp_exec(uint32_t op);

/* VXFER: start the n {channel, buffer, length} transfers at list, deschedule
 * once and wake when all are done. */
void t9k_vcp_group(uint32_t list, uint32_t n);

/* Interrupt save-area handling: once the T9000 channel space is in use the
 * T800 save area at #8000002C.. overlaps event channel words. */
int  t9k_chanspace_active(void);

/* linkcomms(): physical links in virtual-channel mode carry packets. */
int  t9k_link_virtual(int link);
int  t9k_link_listening(int link);
int  t9k_link_tx_pending(int link);
void t9k_link_rx(int link);
void t9k_link_tx(int link);
void t9k_vcp_serve_links(void);         /* CPU stopped (t9000_ctl.c) */

/* Event pins (HRM: EventIn0-3 / EventOut0-3). */
void t9k_event_in_pin(int n, int level);
int  t9k_event_out_pin(int n);

/* The configuration bus (HRM 15.1) for either master: the CPU (ldconf,
 * stconf; by_cpu = 1) or the control unit (CPeek, CPoke), whose writes no
 * write lock stops (HRM 15.3).  0 = invalid address or refused. */
int  t9k_cfg_read(uint32_t a, uint32_t *val);
int  t9k_cfg_write(uint32_t a, uint32_t val, int by_cpu);

/* Signals from the control unit to the VCP (HRM 11.8.2, table 8.10). */
#define T9K_VCPSIG_STOP         1       /* Stop command */
#define T9K_VCPSIG_ERROR        2       /* system error: discard until reset */
#define T9K_VCPSIG_RESET2       3       /* reset 2: channels cleared, configuration kept */
#define T9K_VCPSIG_RESET1       4       /* reset 1: back to the power-on configuration */
void t9k_vcp_signal(int sig);

#endif /* T9000_VCP_H */
