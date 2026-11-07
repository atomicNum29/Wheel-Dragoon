# CAN RX FIFO / interrupt queue

2026-11-07. This change replaces single-mailbox polling with:

`FlexCAN RX FIFO (6 frames) -> bounded CAN ISR -> RAM queue (32 frames) -> main loop`

This replaces the earlier single-mailbox polling implementation.

## Hardware configuration

- Teensy 3.2 / MK20DX256, existing 500 kbit/s timing and pins unchanged.
- RFEN enabled, Format A, RFFN=0: FIFO uses MB0..5 and eight filter words use MB6..7.
- TX moves from MB0 to MB8. Other ordinary mailboxes remain inactive.
- RXFGMASK compares RTR and IDE, accepts all standard data IDs (including bridge
  traffic); extended and remote frames are not part of the existing `CanFrame` API.
- Only FIFO available/warning/overflow interrupt bits are enabled; TX remains polled.
- CAN IRQ priority is 64 (numerically smaller is higher priority). The core's
  default priority is 128. Other interrupt priorities are not changed.
- The ISR reads at most six frames per entry, copies each frame and `micros()` into
  RAM, and acknowledges BUF5I to advance FIFO output. A still nonempty FIFO
  reasserts the interrupt. There is no USB, PID decoding, motor action or wait in ISR.
- FIFO reads do not use the standalone-mailbox BUSY/TIMER unlock protocol.

References: [K20 reference manual](https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf),
44.3.35 (FIFO storage/filter format), 44.4.6.3 (mailbox locking excludes FIFO),
44.4.7 (FIFO flags and advancement), 44.4.11 (interrupts).

## Queue and timing semantics

The ring is single-producer (ISR), single-consumer (main). Lock-free 32-bit atomics
publish a completed copy and protect slot reuse. No global interrupt-disable region
is needed to copy frames. All 32 slots are usable. When full, the newest frame is
dropped and counted; existing queued order is preserved and hardware is still drained.

`can_receive()` retains its public interface for raw bridge traffic. New
`can_receive_timed()` also returns the ISR's FIFO-read timestamp. Both consume only
the RAM queue; a nonzero timeout waits for the interrupt producer.

The timestamp is **not** the exact wire arrival time or the hardware CAN timestamp.
It includes IRQ latency and any hardware FIFO residence time, but excludes subsequent
RAM queue residence. Software consumers must run in main context with CAN interrupts
enabled. A prolonged IRQ mask can still overflow the hardware FIFO.

Normal CONTROL and benchmark processing use the ISR timestamp. Before closing a
response window they perform a bounded queue drain (up to 32 entries), so responses
received before the deadline are not rejected solely because main-loop processing
was delayed. The 3 ms response window, 10 ms command cadence, and steering/PWM code
are unchanged. A pre-command benchmark timestamp is rejected as stale without
terminating the new attempt. There are still no on-wire sequence numbers, so a late
old response arriving during a later request cannot be uniquely identified.

The extra deadline drain is CAN processing, not a new timing probe. Existing
`rx_poll_gap_us` continues to measure entry spacing of the normal main-loop drain;
it no longer measures how often hardware RX is serviced. A deadline drain's time is
included in its enclosing `can_schedule` or `bench_update` stage.

## Diagnostics and validation

`can_rx_stats()` exposes FIFO overflow observations, RAM queue drops, queue
high-water mark and maximum enqueue-to-dequeue age. The existing 0x82 CAN overrun
report includes hardware FIFO and RAM queue losses. These counters require no
separate timing analysis feature. `can_rx_pending()` checks RAM and hardware state.

The existing 0x82 and 0xA1 layouts are preserved; their receive timestamps now refer
to ISR FIFO-read time rather than main-loop read time. Hardware overflow flags can
coalesce multiple losses, so the overflow count is not an exact lost-frame total.

Focused host tests model FIFO initialization/advancement, IRQ budget, queue ordering,
overflow, index/timestamp wrap and CONTROL/benchmark deadline handling. They do not
prove hardware interrupt latency. Hardware testing of the combined working firmware
was reported by the user to no longer show CAN errors; the isolated CAN commit is
also checked by a target build.

From the repository root (create an output directory first):

```powershell
g++ -std=c++14 -Wall -Wextra -Werror -pedantic -Iinclude -Itest/flexcan_fakes test/test_flexcan_rx.cpp -o test_flexcan_rx.exe
.\test_flexcan_rx.exe
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Iinclude -Itest/fakes test/test_can_rx_queue_integration.cpp src/md200t_can.cpp -o test_can_rx_queue.exe
.\test_can_rx_queue.exe
pio run -e teensy32
```
