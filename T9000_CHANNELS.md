# T9000 channels, links and the VCP in T4

T4 implements the IMS T9000 communication architecture: the Virtual Channel
Processor (VCP) multiplexing virtual links over the physical links, event
channels, resource channels, variable-length communication, the
configuration register space, the shadow / trap-return state
instructions, and the control system with its control links.  Code:
`t9000_vcp.c` / `t9000_vcp.h` (channels, VCP, configuration space),
`t9000_ctl.c` / `t9000_ctl.h` (control links, control unit, system
services) and `p.c` (instruction dispatch, shadow state, `tret`,
`restart`).

Sources, in authority order (see `../CLAUDE.md`): T9000 ISM ch.8.4-8.8
(communication, ALT, resources), ch.12 (channels), ch.13.3-13.4 (shadow
state, restart), appendix C (instruction definitions); T9000 HRM ch.6
(links, packet and token protocol), ch.7 (routing, header deletion), ch.11
(communications, VLCBs, VCP registers, errors), ch.12 (events), ch.13
(DS-links), ch.15 (configuration register map), and part 2 ch.8 (control
system).

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
  is acknowledged when the input takes it.  One whose input is waiting is
  acknowledged as soon as its header is in ("the VCP sends an acknowledge
  packet as soon as it receives the start of the data packet", HRM 11.2.3),
  so on a quiet path the next packet follows with no gap.
* **Link queues** (HRM 11.2.3): per physical link, high- and low-priority
  data and acknowledge queues threaded through the VLCBs; high priority
  first, acknowledges and data alternately within a priority.  On a timed
  network (`T4_T9K_NET`) the queued packets become ready at different
  cycles, and the one ready first goes first; the alternation breaks ties.
* **Deactivated channels** (HRM 11.3.1) send nothing and perform no
  scheduling; acknowledges, completions, ALT wake-ups and resource claims
  are held and carried out by `setchmode` on activation.
* **VCP states** (HRM 11.8): reset, waiting (the initial state: requests
  are queued, links are not served), running, stopping (deactivates each
  channel it acts on), discarding.
* **Errors** (HRM 11.9, 11.7.2): header out of range, header too short,
  short non-terminal packet and over-long packet are recorded in
  `VCPStatus`.  With the link's `LocalizeError` bit clear (a reliable
  link, HRM 13.5.1) the error goes to the control system: the CPU stops,
  the VCP goes to *discarding* (accept and drop everything, send nothing)
  until reset, and a VCP error (#02) is reported on CLink0 - see "Control
  system" below.  A node with no control link wired and StandAloneMode
  clear has nothing to report to (HRM 8.1.2 rules that configuration out);
  there T4 keeps the CPU running and only the VCP discards.  A `vin`
  overrun stores `LengthError.p` and is read back with `ldcnt`.

Between two directly connected nodes a receiver sees exactly the sender's
header.  Longer headers - a routing header in front of the virtual-link
header - need an IMS C104 in the path to delete the routing part; see
"Routing switch" below.

### Instructions

`initvlcb`, `sethdr`, `writehdr`, `readhdr`, `insphdr`, `readbfr`,
`swapbfr`, `setchmode`, `stopch`, `resetch`, `ldchstatus`, `chantype`,
`vin`, `vout`, `ldcnt`, `ldconf`, `stconf`, `ldmemstartval`.

**VXFER** (TVX OPR `0x1FA`, not a T9000 instruction; `__vxfer` in
`include/t9000.h`).  A = a list of `{channel, buffer, length}` words, B = the
number of entries.  It starts every transfer on the list, deschedules once,
and wakes the process when all are done - one instruction where a collective
step otherwise needs a process per transfer.  An input entry takes up to
`length` bytes, and the count received (or `LengthError.p`) replaces the
entry's length.  An output entry is *posted*: it is done once its last
packet has left the node, when its buffer is free again.  The VCP still
takes the final acknowledge, and that virtual link's next message waits for
it as any next packet does, so ordering and flow control are unchanged; the
process just does not wait for the acknowledge's round trip.  `vout` and `out`
keep the T9000's synchronous completion.  VXFER is privileged like `vin` and
`vout`: in a P-process it traps to the supervisor.  Its cost in the cycle
model is 20 cycles plus 3 per entry.

## Configuration space (HRM ch.15)

`ldconf`/`stconf` implement MemStart, MinInvalidChannel, ExternalRCBase,
HdrAreaBase, ChanWriteLock, InitialIptr/Wptr, Reason, EmiBadAddress,
VCPStatus, VCPCommand, VCPLink0-3 Mode/MaxHeader/MinHeader/HdrOffset,
DeviceID, DeviceRevision, Link0-3 Mode/Command/Status/WriteLock, and
(`t9000_ctl.c`) ModeStatus, ErrorCode, DSLinkPLL, SysServWriteLock and
CLink0-1 Mode/Command/Status/WriteLock.  PMI and cache registers are
stored and read back.  Any other address signals IntegerError, as does a
write blocked by a write lock.  The control unit's CPeek/CPoke use the
same bus and are not stopped by write locks; a lock stops every CPU write
to its subsystem, its own included (HRM 15.3), so it is cleared by the
control unit or by a reset to level 1 or 2.

Physical links reset into **byte-stream mode** (so T800 code, the link boot
protocol and the host link are unchanged); software selects virtual mode
with `stconf` on `VCPLinkNMode`.  In byte-stream mode the link carries T4's
existing OS-link messages, i.e. the T9000 behaves as if an IMS C100 joined
it to a T8-series neighbour (HRM 11.5).

A byte-stream output completes only once the far process has input all of
the message (ISM 8.4.2, 12.2.2): an OS-link acknowledges a byte only to a
ready input (HRM, C100 section 1.3.1), so a link holds nothing that has not
been asked for.  On threaded (`-st`), shared-memory (`-sl`) and loopback
links a packet stays in the far node's one-packet mailbox until the inputs
there have taken every byte of it, and emptying the mailbox is the
acknowledge: the sender's `out`, `outbyte` or `outword` deschedules and is
rescheduled then (`Channel.ackwait` in `p.c`); a threaded node idle in
such an `out` is woken when the far end empties the mailbox
(`t4_node_any_ready` counts `ackwait`).  Until 2026-11-03 the output
completed as soon as the packet was in the mailbox, which let a sender run
up to a message ahead of its receiver and allowed orderings that real links
cannot produce (`tests/test_link_sync.c`).  Over nanomsg `ipc://` an output
still completes once nanomsg has accepted it: that transport has no shared
word to acknowledge with, and would need acknowledge messages of its own.
With `T4_LOCKSTEP=1` a threaded network also orders these packets and
acknowledges by node time, carried in the mailbox's `SCH_STAMP` slot when
`T4_T9K_NET` is off, plus a transfer time if `T4_LOCKSTEP_LINK` gives one
(`THREADED-NODES.md`, "Node time").

## Routing switch: IMS C104 (HRM part 1 ch.7, part 3 ch.2)

A threaded network (`-st -sn -N`) can contain IMS C104 packet routing
switches, so virtual channels reach nodes that are not directly connected.
Code: the router engine in `t4_fabric.c`, its description in the network
map in `netcfg.c`.

* **Node.** A router is a node of the map with its own id - any id that is
  not a processor's, so at least N - and up to 32 links.  It runs as one
  more thread on the fabric with an input mailbox per link, the same format
  as a transputer's link inputs; an output writes into the input mailbox at
  the far end of its link, which may be a processor or another router.
* **Wormhole routing** (HRM 7.2, figure 7.3) is approximated a packet at a
  time: the VCP sends each packet as one link message, and the switch
  forwards whole packets.  A packet stays in its input until an output takes
  it, which stalls that input - and through it the sender - as a blocked
  worm does.  Packets of any length pass; anything after the header bytes
  is just packet body (HRM 7.2.1).
* **Interval routing** (HRM 7.3.1, figure 7.9).  The first one or two bytes
  of a packet are its header, according to the input link's header length.
  Each input link has an interval selector: `lo <= header < hi` leaves on a
  given output link.  Intervals must not overlap on a link.  Two-byte
  headers are read first byte least significant, which is the HRM's general
  rule for values sent as several components (HRM preface, "Significance":
  the least significant component is transmitted first) and how the T4 VCP
  sends them.
* **Header deletion** (HRM 7.2.1, figures 7.4-7.6) is a property of an
  output link: it drops the header bytes the switch routed on, so the next
  bytes become the header seen by the device beyond.  With a 1-byte network
  header in front of the virtual-link header, every T9000 can number its
  virtual links from 0 (figure 7.5b); without deletion the network uses one
  global label per virtual channel and each receiver subtracts its
  `VCPLinkNHdrOffset` (figure 7.5a).  Deleting at every level gives the
  hierarchical routing of figure 7.6.
* **Grouped adaptive routing.**  An interval may name a group of
  consecutive links, `a-b`: the packet leaves on the first of them that is
  free.  The HRM only lists the feature ("Grouped adaptive routing"); this is
  T4's reading of it, meant for parallel cables between switches.
* **Arbitration.**  Each output serves the inputs waiting for it round
  robin, starting after the input it served last, so a waiting packet gets
  its output within one packet from each other contender.  Different
  input/output pairs move concurrently, as in the 32x32 crossbar.
* **Errors.**  The HRM leaves these to the C104's control system, which is
  not modelled.  A header that falls in no interval, or a packet that ends
  inside its header, is discarded with a `-W-EMU104` message (once per input
  link; every time with `T4_ROUTER_DEBUG=1`).  An interval leading to an
  unconnected link blocks its input, as an unanswered worm would.
* **Configuration.**  On the chip the interval tables, header lengths and
  deletion flags are programmed over the control link.  T4 takes a static
  description from the network map instead; a router programmed over
  CLink0 is not modelled yet.  The other features of the product preview -
  universal routing and the congestion-reduction circuitry - are not
  modelled either.  The per-link tables do let one C104 serve two
  independently labelled networks, as the test below does.

Network-map syntax (numbers decimal, `#hex` or `0xhex`; `<links>` a
comma-separated list of `n` and `a-b`; without `on` a statement applies to
every input link):

```
router <id> [links <n>]                          a C104 with n links (32)
router <id> header 1|2 [on <links>]              input header length (1)
router <id> interval <lo> <hi> <out>|<a-b> [on <links>]
router <id> delete <links>                       header deletion on outputs
Connect processor <p> link <l> to router <id> link <m>
Connect router <id> link <m> to router <id2> link <n>
```

Use `Connect` for router links: it defines both directions, and a processor
finds its links from its own side.  A map with routers is refused without
`-st`, as are overlapping intervals, links a router does not have, and a
router id that a processor uses.  A node still waiting to boot from a link
takes the first message on *any* link as boot data, so route nothing to a
node until it has booted (the test has node 1 report back on its boot link
first).  Example: two T9000s on one C104, one
1-byte network header deleted on the way out, so node 0 is reached as
header 0 and node 1 as header 1:

```
Connect HOST to processor 0 link 0
Connect processor 0 link 3 to processor 1 link 3   -- boot
router 2 links 2
Connect processor 0 link 1 to router 2 link 0
Connect processor 1 link 1 to router 2 link 1
router 2 interval 0 1 0
router 2 interval 1 2 1
router 2 delete 0-1
```

`T4_ROUTER_DEBUG=1` traces every routing decision on stderr.

## Control system: CLink0 and CLink1 (HRM part 2 ch.8)

Each node has the two control links, a packet handler, a control unit and
the system services registers.  Code: `t9000_ctl.c`.

* **Wiring.**  The control links are links 4 (CLink0) and 5 (CLink1) of the
  network map, written `clink`:
  `Connect processor 0 link 2 to processor 1 clink 0`.  They work on the
  threaded fabric (`-st`), shared-memory links and ipc sockets; in a
  node's 8 mailbox slots CLink0-1 In take 0-1 (data links use 4-7).
* **The controlling side is ordinary T9000 code.**  HRM 8.1: "a control
  link network is generally connected to one of the data links of a
  controlling IMS T9000", so the controller drives a virtual link on a data
  link in virtual mode with 2-byte headers: the virtual link's header is
  the device's label, and the return header it gives the device maps back
  to that virtual link.  The VCP does the packet work.
* **Packets** are the VCP's, always with 2-byte headers: a command is one
  packet `label | code | operands | EOM`, every message packet is
  acknowledged with an empty EOP packet, and a device sends nothing more
  until its last message is acknowledged (HRM 8.1.1).
* **Packet handler** (HRM 8.3.1-8.3.2).  The first header seen on CLink0
  after power-on becomes the device's label (Reset 1 keeps it).  Packets
  with another header go down CLink1, once software has started it with a
  CPoke of CLink1Command (StartLink), or are destroyed with a #C1 error.
  Packets arriving on CLink1 go up CLink0, alternating fairly with the
  device's own.  Start programs the return header put on every outgoing
  packet; a later Start re-programs it, the acknowledge going under the old
  header and the handshake under the new.  CLink0 starts on its first
  token.  Only Reset, RecoverError and ErrorHandshake may overtake an
  unacknowledged handshake; any other command is discarded with #C1.

### Commands

The command figures of HRM 8.4 (figures 8.6/8.7) are illegible in the text
T4 is built from.  Three codes survive (IdentifyResult #80, CPeekHandshake
#82, PeekHandshake #F4) and the HRM states that a response is its command
with the top bit inverted; the others are T4's.

| Command | Code | Operands (lsb first) | Valid in (HRM table 8.7) |
|---|---|---|---|
| Identify | #00 | - | every state |
| CPeek | #02 | address (2) | Ready, Booting |
| CPoke | #03 | address (2), data (4) | Ready, Booting |
| Start | #10 | return header (2) | every state; must come first |
| Reset | #11 | level (1): 1, 2 or 3 | every state |
| Stop | #12 | - | Running, Stopping |
| RecoverError | #13 | - | every state |
| ErrorHandshake | #60 | - | answers an Error message; no handshake |
| Peek | #74 | address (4) | Ready, Booting |
| Poke | #75 | address (4), data (4) | Ready, Booting |
| Boot | #76 | length (4), address (4) | Ready |
| BootData | #77 | 16 bytes | Booting |
| Run | #78 | Wptr (4), Iptr (4) | Ready |
| Reboot | #79 | - | Ready, Booting |
| Error message (device to controller) | #E0 | error code (1) | |

Every handshake is `code ^ #80, status` - status 0 valid, 1 invalid or
failed - followed by 4 data bytes for Identify, CPeek and Peek (zero on
failure).  A command that is not valid in the current state is handshaken
with status 1, as are an unaligned Peek, Poke, Boot or Run address, an
invalid configuration address, a write-only register read by CPeek and a
Reset level other than 1-3.

* **Gross state** (HRM 8.4.1): Initial (no Start yet), Ready, Booting,
  Running, Stopping, Error.  A CPU booted down a data link, from a file or
  from ROM is Running without a Start, the "doubling" HRM 8.4.1 notes.
* **Boot, BootData** load 16 bytes at a time into consecutive blocks of
  four words; the excess of the last block is discarded and the last one
  returns to Ready.  **Run** and **Reboot** (Wptr and Iptr from #7FFFFFFC
  and #7FFFFFF8) start a high-priority process, as a boot from ROM does
  (HRM 10.5.2), after writing the state the CPU stopped in at Wptr+0..6:
  Wdesc, Iptr, Status (ISM table 5.5 layout), ThReg, Reason, error type,
  Eptr (HRM 8.5.4; T4 records the last two only for traps, so they are 0).
* **Reset** (HRM 8.7, table 8.10).  Every level stops and clears the CPU
  (queues empty, Wdesc NotProcess.p) and the handshake state.  Level 2 also
  clears the error records and the CPU write locks, resets the VCP's channel
  state and the link engines but keeps their configuration, and resets the
  byte-stream link channels; level 1 also returns the VCP and link
  configuration to its power-on values and clears ErrorSinceReset.  The
  label, the return header and memory survive every level.  The handshake
  goes once the reset is done.
* **Stop** (HRM 8.5.4) puts the VCP in 'stopping' and stops the CPU when
  its current process next deschedules or reaches a timeslice point.
* **RecoverError** (HRM 8.5.3) clears the handshake and acknowledge state,
  so that an Error message that was sent but not handshaken goes again
  after the RecoverError handshake.

### Errors (HRM 8.5)

One record per error code (HRM table 8.9), sent highest priority first:
#C0 invalid command code, #C1 control link protocol error (wrong length,
EOP-terminated command, unexpected acknowledge, a command overtaking a
handshake, a packet for another device with CLink1 not started, anything
before Start), #C2 CLink1, #80-#83 Link0-3, #01 CPU, #02 VCP, #03 PMI.
An Error message `#E0, code` goes when every earlier message has been
acknowledged and every earlier Error message handshaken; ErrorHandshake
removes the record.  ErrorCode (#1004) holds the latest error.

| Error | CPU | ErrorSinceReset | Reported on CLink0 |
|---|---|---|---|
| control link (#Cx) | runs on | no | yes |
| system (#01-#03, #8x) | stopped, VCP discarding | set | yes |
| system, StandAloneMode set | reset, rebooted from ROM | set (kept) | no |

The system errors T4 raises are the VCP's (#02, reliable links only), the
CPU's (#01) - T4's halt-on-error stands for the T9000 halting on an error
with a null trap-handler (ISM 10.3.3) - and link faults (#80-#83, below);
the PMI raises nothing.  With no control link wired and StandAloneMode
clear a system error is recorded in ErrorCode/ModeStatus but the CPU runs
on, as described under "Errors" above.

### Link failures (HRM 13.5, 8.5.3)

T4's links never fail on their own; software makes them fail the way the
HRM gives it to: `WrongParity` in LinkNCommand or CLinkNCommand (stconf or
CPoke) gives the far end of the cable a parity error, and `ResetOutput`
(outputs held low) a disconnect error.  The fault is a line event, carried
outside the link's mailbox (`s_line_err` on the far node's `cpu_t`), so it
arrives even when the mailbox is full.

* **At the link that sees it** (data or control): LinkStatus gets
  LinkError and ParityError or DiscError, LinkStarted clears, and the link
  engine resets - what its input held is lost.  A link that is not running
  sees no fault.
* **Propagation.**  A link with a parity error halts its output, which the
  far end sees as a disconnect, halting that end in turn (HRM 13.5); a
  disconnect goes no further.
* **Reporting.**  Link0-3: with LinkNMode.LocalizeError (bit 3) clear - a
  reliable link, the power-on default - the error goes to the control unit
  as #80-#83, a system error (CPU stopped, VCP discarding, Error message);
  with it set the link just resets ("unreliable", HRM 13.5.2).  CLink1:
  #C2, a control-link error.  CLink0: nothing can be reported; its halted
  output is what the neighbour sees (HRM 8.5.3).
* **Frozen until restarted.**  A halted link carries nothing - no VCP
  packets, no byte-stream messages (an output started then waits, and not
  even its first packet leaves), no control packets - until software
  starts it again: LinkNCommand/CLinkNCommand StartLink (ResetLink empties
  its buffers first), `resetch` on a byte-stream channel (reset and
  restart, HRM 13.7.2; on an input it also loses a packet the link had
  received and nobody had input), `setchmode` on a byte-stream channel, or
  a control unit Reset 1 or 2.  CLink0 restarts on its next token (HRM 8.3.1); the
  controller then resynchronises the device with RecoverError.

Faults reach nodes of the threaded fabric (`-st`) and a stand-alone node's
looped-back links (`T4_LINK_LOOPBACK`); a cable to a C104 or to a node in
another process carries none.  Not modelled: the disconnect that a link
reset causes at the far end (HRM 13.7.2's enforced reset delay) - so
`resetch`-based recovery of T800-style links behaves as before - and link
timing.

### The CPU under control

The control unit stops a running CPU through `start_process()`, which
then returns from `mainloop()`; anything that needs the CPU stopped (a
reset, a reboot) is finished there.  While the CPU is stopped - before a
control-link boot, or after an error, Stop or Reset - the node keeps
serving its control links and its virtual links (HRM 8.5.4: Stop leaves
the links alone; a discarding VCP accepts and drops packets, 11.8.2), and
Run or Reboot enters `mainloop()` again.  The restarted process starts
outside any P-process with the PMI as it was (HRM table 8.10).  The
control unit's own memory accesses (Peek, Poke, Boot, the state block)
are physical, never translated through a P-process's regions.

**Booting.**  A node waiting to boot serves its control links and its
byte-stream data links together: the T800 link boot (`handleboot()`) is
unchanged, and control traffic does not count against the boot timeout.
A data link put into virtual mode by a CPoke during that wait belongs to
the VCP and is no longer a boot link.  The usual control-link boot (HRM
8.6.1) is: Start, CPoke the VCP and link configuration, Boot + BootData,
Run.

`T4_CTL_DEBUG=1` traces the control unit on stderr.  Not modelled: link
speeds (CLinkNMode and DSLinkPLL are stored), the cache, and a C104
controlled over its CLink0.

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

## Network timing and C programs on every node

**Booting.**  In a threaded network each node other than 0 normally waits
for boot code on a link.  `T4_NODE_BTL=n%d.btl` makes node N load its own
netload image instead (`%d` becomes N), as node 0 does with `-sb`, so every
node - with routers in between - can run a C program.  The C side is
`include/t9000.h` in the compiler tree: builtins for the configuration
space and the VCP (`__stconf`, `__initvlcb`, `__vin`, ...) and helpers that
bring up virtual links.

**Virtual links per node.**  Their channel words run from `#80000040` to
MemStart, and T4 (and the compiler's linker) put MemStart at `#80000070`,
so a node starts with **six** virtual links; `initvlcb` on a seventh raises
IntegerError.  MemStart is a writable configuration register (HRM ch.15):
`__stconf(T9_CR_MEMSTART, (unsigned)T9_VIN(n))` before
`t9_vcp_channels(n)` gives n.  The channel addresses from `#80000070` up then
coincide with the boot code, which is harmless: VLCBs live inside the VCP,
and only channel instructions treat those addresses as channels.
`ffn_cube.c -DSPLIT=k` uses 6k.

**Cycle model.**  With `T4_T9000=1` and `T4_T9K_NET=1|2` (see
`t9000_group.h`), every link message carries its sender's ready cycle, in a
slot after the mailbox data in a threaded network.  The model is per node:
`t9k` is thread-local, as are the TVX and move2d latches and TVX scratch
buffers, and `T4_T9K_MARKS` lines carry the node (`[T9000-MARK] n3 tag`).
The VCP keeps per-virtual-link times: a data packet leaves when the previous
one was acknowledged, an acknowledge when its data arrived, and the CPU
waits only when the VCP wakes a process.  Even then only an idle CPU waits:
while it runs another process, the woken one joins the queue when the CPU's
clock reaches the cycle its transfer finished, so a node computes while its
VCP moves data (`ffn_cube.c -DOVERLAP`).  A C104 adds `T4_T9K_LINKLAT` per
hop and `T4_T9K_C104LAT` (20 cycles) to route, and serialises each output
link; a packet's bytes are counted once along a path (wormhole), and an
output goes to the waiting packet that reached the switch first.  Packets
reach a link in host order, which through a switch is not cycle order, so
each link keeps its last 16 transfers (`t9000_tl.h`): a packet stamped
before ones already placed takes the first gap at or after its stamp.
In-order traffic is timed as before.  This replaced a rule that charged a
late packet's bytes to every later one, which let host thread timing swing
the 8-node C104 FFN between 5.13 and 5.65 ms; it now repeats to within
0.2%.

Measured (transputer/tests/net in the compiler tree): 24 KB over one cable,
one virtual link 1.06x the raw link time at 9 MB/s and 2.42x at 90 MB/s,
three links 1.06x and 1.09x; byte stream 1.00x (`vlbw.sh`).  Each virtual
link waits for every 32-byte packet's acknowledge, so a fast link or a path
through switches needs several in flight.  A collective step through one
C104 (`msglat.sh`, 4-byte messages at 9 MB/s): 139 cycles with one VXFER,
of which 99 are the wire; 213 with a process per output, which waits for
the last acknowledge.  DeepSeek's shared-expert FFN on 8 nodes and three
C104s (`c104.sh`, VXFER, three virtual links per peer and plane): 5.16 ms
at 9 MB/s and 4.13 ms at 90 MB/s - with one per peer, 5.35 and 4.55 ms; the
same FFN on an 8-node hypercube of byte-stream links takes 6.73 ms.

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
* `tests/test_t9000_c104_net.c` - two threaded T9000s and two C104s.
  Node 0 boots node 1 down a direct byte-stream Link3; the Link1s meet
  through both switches, which are joined by a group of two cables, using
  `[node][virtual link]` headers whose first byte is deleted on the way out
  (figure 7.5b); the Link2s meet through router 2 using global 2-byte labels
  - node 0 `#000-#0FF`, node 1 `#100-#1FF`, `VCPLink2HdrOffset = #100` -
  with no deletion (figure 7.5a), read on two links of router 2 that have
  their own interval table.  Node 1 echoes on three virtual links; node 0
  uses them one at a time, then all at once from three processes so that
  packets of two channels meet in the switches.  The map is in
  `tests/run_t9000_vcp.sh`.
* `tests/test_t9000_ctl_net.c` - two threaded nodes, node 0's Link2 cabled
  to node 1's CLink0.  Node 1 is never booted down a data link: node 0
  labels and identifies it, configures its VCP with CPoke, loads a program
  with Boot + BootData and starts it with Run; sends it a packet its VCP
  rejects on a reliable link and receives the VCP error, resets it to
  level 3 and reads the evidence with CPeek/Peek; resets it to level 2 and
  reboots it into a program that halts on error (CPU error); stops a
  running program; and checks the state table, invalid operands and the
  #C0/#C1 errors, RecoverError re-sending an unhandshaken error message.
  Then link failures on the Link1 cable: WrongParity from node 0 is a
  parity error at node 1, reported as #81 on its reliable link, and a
  disconnect back at node 0; after restarting both ends (CPoke
  Link1Command, `resetch`), the same fault with LocalizeError set is
  recorded in Link1Status but not reported.
* `tests/test_t9000_ctl_chain.c` - three nodes on a daisy-chained control
  network (node 0 Link2 - node 1 CLink0, node 1 CLink1 - node 2 CLink0):
  node 2 is labelled, identified, booted, started and reset through node 1,
  and its Error message comes back the same way.  Then WrongParity on node
  1's CLink1: a parity error at node 2's CLink0, a disconnect back at node
  1 reported as #C2; the controller restarts CLink1 and resynchronises
  node 2 with RecoverError.
* `tests/test_link_sync.c` - two threaded nodes, byte-stream Link1 and
  Link2.  Node 1 inputs a word on Link2, waits 200 ms, then inputs an item
  on Link1; node 0's output of that item must still be outstanding when its
  Link2 output has completed (byte-stream mode, above).
* `tests/test_alt_guard.c` and `tests/test_fused_cj.c` - not T9000: an ALT
  guard of 2 is true and a false guard's `disc` leaves the channel word
  alone (ISM `enbc`, `disc`, `enbs`, `diss`); and the `eqc;cj` and `gt;cj`
  pairs t4 fuses leave a taken jump's 0 in A, as `cj` does (ISM `cj`).

Run them with `make -f Makefile.linux test-vcp` or `tests/run_t9000_vcp.sh`
(`T4_BIN` picks the emulator).  They build with `transputer-tcc` and
`transputer-netload` from the top of the tinycc_transputer tree, and use
`tests/test_boot.S` (a copy of the compiler tree's test boot stub) and
`tests/test_t9000_vcp_fn.S` (raw-encoded probes).  Each must print exactly
`S`, `ALL-PASS`, `H` on the UART.

`T4_VCP_DEBUG=1` traces VCP activity on stderr.  `T4_LINK_LOOPBACK` takes
effect only for a stand-alone node (no `-sn`).
