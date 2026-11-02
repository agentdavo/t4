# T9000 channels, links and the VCP in T4

T4 implements the IMS T9000 communication architecture: the Virtual Channel
Processor (VCP) multiplexing virtual links over the physical links, event
channels, resource channels, variable-length communication, the
configuration register space, and the shadow / trap-return state
instructions.  Code: `t9000_vcp.c` / `t9000_vcp.h` (channels, VCP,
configuration space) and `p.c` (instruction dispatch, shadow state,
`tret`, `restart`).

Sources, in authority order (see `../CLAUDE.md`): T9000 ISM ch.8.4-8.8
(communication, ALT, resources), ch.12 (channels), ch.13.3-13.4 (shadow
state, restart), appendix C (instruction definitions); T9000 HRM ch.6
(links, packet and token protocol), ch.7 (routing, header deletion), ch.11
(communications, VLCBs, VCP registers, errors), ch.12 (events), ch.13
(DS-links), ch.15 (configuration register map).

## Channel address space (ISM fig 12.5, HRM 11.6)

| Address | Channels |
|---|---|
| `#80000000`-`#8000000C` | byte-stream outputs (= T800 `Link0-3Out`) |
| `#80000010`-`#8000001C` | byte-stream inputs (= T800 `Link0-3In`) |
| `#80000020`-`#8000003C` | event channels: input `#80000020+8n`, output `+4` |
| `#80000040`-MinInvalidChannel | virtual channels: input `#80000040+8vl`, output `+4` |
| MinInvalidChannel-MemStart | illegal (IntegerError) |
| MemStart- | internal (soft) channels |

`in`, `out`, `outbyte`, `outword`, `enbc`, `disc` and `resetch` test the
channel address once (`T9K_EXTCHAN()`, one subtract and compare) and hand
event and virtual channels to the VCP.  Byte-stream addresses keep T4's
existing link code.

### Compatibility gate

Until software touches the T9000 channel space - a VCP/configuration write,
a VLCB instruction, `setchmode`, or communication on an event/virtual
channel - only the event range is routed to the VCP and the illegal range
behaves as T800 memory channels did.  On first use the four event channels
are initialised to *empty, activated* (HRM 12.2 requires software to do
this; doing it here keeps T805-style `EventIn` code working) and the range
up to MemStart becomes live.  `T4_T9K_CHANNELS=1` enables it from reset.

The interrupted-process save area moved into the T9000 shadow registers.
It is still mirrored into the T800 locations `#8000002C..` while the
channel space is unused, because once it is in use those words are event
channel words.

## Virtual links and the VCP (ISM 12.2-12.6, HRM 11.2-11.8)

* **VLCBs** are kept inside the VCP, not in guest memory below MemStart: the
  ISM calls their contents private to the VCP, and T4 loads boot code at
  MemStart, where VLCB 1 onward would sit.  Packet buffers, the header
  region (`HdrAreaBase`) and the external resource channel block
  (`ExternalRCBase`) are guest memory, as on the chip.
* **Packets** (HRM 6.3.2): one emulated link message per packet -
  `header | 0..32 data bytes | token`, token `EOP` (1) or `EOM` (2).  Every
  packet but a message's last carries exactly 32 bytes; a zero-length `EOP`
  packet is an acknowledge.  The receiver takes 1 or 2 header bytes
  according to `VCPLinkNMode.HeaderLength` and maps
  `(header - HdrOffset) mod 2^16` to a virtual link, after the
  `MinHeader`/`MaxHeader` check.
* **Protocol** (HRM 11.2.1, 11.3): each data packet waits for its
  acknowledge before the next is sent; the outputting process is
  rescheduled when the last packet is acknowledged.  A packet that arrives
  before its input is waiting goes to the virtual link's packet buffer and
  is acknowledged when the input takes it.
* **Link queues** (HRM 11.2.3): per physical link, high- and low-priority
  data and acknowledge queues threaded through the VLCBs; high priority
  first, acknowledges and data alternately within a priority.
* **Deactivated channels** (HRM 11.3.1) send nothing and perform no
  scheduling; acknowledges, completions, ALT wake-ups and resource claims
  are held and carried out by `setchmode` on activation.
* **VCP states** (HRM 11.8): reset, waiting (the initial state: requests
  are queued, links are not served), running, stopping (deactivates each
  channel it acts on), discarding.
* **Errors** (HRM 11.9, 11.7.2): header out of range, header too short,
  short non-terminal packet and over-long packet are recorded in
  `VCPStatus`.  With the link's `LocalizeError` bit clear the error is
  reported and the VCP goes to *discarding* (accept and drop everything,
  send nothing) until a VCPCommand reset - the closest equivalent of the
  control system halting a reliable link.  A `vin` overrun stores
  `LengthError.p` and is read back with `ldcnt`.

No IMS C104 is modelled: emulated links connect nodes directly, so a
receiver sees exactly the sender's header (HRM 7.2.1: header deletion
happens in the router).

### Instructions

`initvlcb`, `sethdr`, `writehdr`, `readhdr`, `insphdr`, `readbfr`,
`swapbfr`, `setchmode`, `stopch`, `resetch`, `ldchstatus`, `chantype`,
`vin`, `vout`, `ldcnt`, `ldconf`, `stconf`, `ldmemstartval`.

## Configuration space (HRM ch.15)

`ldconf`/`stconf` implement MemStart, MinInvalidChannel, ExternalRCBase,
HdrAreaBase, ChanWriteLock, InitialIptr/Wptr, Reason, EmiBadAddress,
VCPStatus, VCPCommand, VCPLink0-3 Mode/MaxHeader/MinHeader/HdrOffset,
DeviceID, DeviceRevision and Link0-3 Mode/Command/Status/WriteLock.  PMI,
system services, cache and control-link registers are stored and read
back.  Any other address signals IntegerError, as does a write blocked by
ChanWriteLock or a link WriteLock.

Physical links reset into **byte-stream mode** (so T800 code, the link boot
protocol and the host link are unchanged); software selects virtual mode
with `stconf` on `VCPLinkNMode`.  In byte-stream mode the link carries T4's
existing OS-link messages, i.e. the T9000 behaves as if an IMS C100 joined
it to a T8-series neighbour (HRM 11.5).

## Event channels (ISM 12.2.3, HRM ch.12)

The event pins are modelled.  `T4_EVENT_LOOPBACK=1` wires EventOut0 to
EventIn1 and EventOut1 to EventIn0 (likewise 2/3), so one event pair can
handshake with another; `t9k_event_in_pin()` is the hook for a device
model.  Input and output follow the four-phase handshake of HRM figure
12.1; channel words follow HRM tables 12.1 and 12.2.

## Resource channels (ISM 8.8, 12.7)

`mkrc`, `unmkrc`, `grant`, `enbg`, `disg`, `erdsq`, `irdsq`, `ldresptr`,
`stresptr` on internal channels (RCDS after the channel word), virtual
input channels and event input channels (RCDS in the ExternalRCBase
block).  `out`, `outbyte`, `outword` and `vout` make the claim on an
internal resource channel; the VCP makes it for a virtual or event input.

## Shadow state and trap return (ISM 13.2-13.4, 10.4)

An interrupt saves the low-priority process into shadow registers,
including RegionReg0-3, PstateReg and WdescStubReg of an interrupted
P-process, which is resumed under protection.  `stshadow` / `ldshadow`
transfer the groups of figure 13.1 (the floating-point group in the
`fpstall` layout).  `restart` resumes a process from a PDS; `tret`
returns from an L-process trap-handler through its THDS.  Status words use
the ISM table 5.5 bits T4 models (IntOv, FPError, TimesliceDisabled,
IsPprocess) plus bit 28 for T4's HaltOnError.

`swapgstatus` (OReg -9) is not an ISM instruction and still stops the
emulator with a message.

## Testing

* `tests/test_t9000_vcp.c` - single node, with
  `T4_LINK_LOOPBACK=1,2-3` (Link1 cabled to itself carries the virtual
  links; Link2/Link3 crossed, byte-stream) and `T4_EVENT_LOOPBACK=1`.
  Covers the configuration space, VLCB and header instructions, message
  lengths 0-1000 bytes, LengthError, both directions of one virtual link,
  two virtual links multiplexed on one physical link, packet buffering,
  ALT over virtual channels, resource channels (queueing, waiting
  servers, queue surgery, `enbg`/`disg`, virtual-input resource),
  deactivation and `resetch`, `stopch`, event handshakes, byte-stream
  loopback, malformed packets, the discarding state and its recovery,
  `cir`/`ciru`, `tret`, `restart` and `stshadow`/`ldshadow`.
* `tests/test_t9000_vcp_net.c` - two threaded nodes (`-st -sn -2`): node 0
  boots node 1 down byte-stream Link1, both switch Link1 to virtual mode,
  and node 1 echoes messages over virtual link 0.

Run both with `make -f Makefile.linux test-vcp` or `tests/run_t9000_vcp.sh`
(`T4_BIN` picks the emulator).  They build with `transputer-tcc` and
`transputer-netload` from the top of the tinycc_transputer tree, and use
`tests/test_boot.S` (a copy of the compiler tree's test boot stub) and
`tests/test_t9000_vcp_fn.S` (raw-encoded probes).  Each must print exactly
`S`, `ALL-PASS`, `H` on the UART.

`T4_VCP_DEBUG=1` traces VCP activity on stderr.  `T4_LINK_LOOPBACK` takes
effect only for a stand-alone node (no `-sn`).
