# Basic Operation of the RSbusMaster library #

This document describes the basic operation of the RSbusMaster library.
For a detailed description of the RS-Bus protocol itself, see:
- [RSbus decoder library - Basic Operation](https://github.com/aikopras/RSbus/blob/master/extras/BasicOperation.md)
- [Der-Moba: RS-Rückmeldebus](http://www.der-moba.de/index.php/RS-Rückmeldebus) (in German)

For a detailed description of the PIO assembly code that drives the pulse generation
and feedback reception, see [pio.md](pio.md).


## The RS-Bus protocol ##

The RS-Bus is a polling protocol. The master station generates a continuous train of 130 pulses, each consisting of a
92us high phase followed by a 108us low phase. The first pulse serves as a
synchronisation pulse; decoders may use it to reset their internal counter.
During the low phase of pulse N (N = 2..129), the decoder at RS-Bus address N-1
may respond by sending an 8-bit feedback message at 4800 baud.
Pulse 130 serves as an end-of-cycle marker.


### RS-Bus address ###
The RS-Bus supports addresses 1 to 128. The address is derived from the pulse counter:

```
addr = 129 - pulse_counter
```

### Feedback message format ###
Each feedback message consists of a start bit followed by 8 data bits:

```
Bit: 7      6   5   4     3   2   1   0
     Parity T1  T0  N     D3  D2  D1  D0
```

- **Parity**: odd parity over bits 0..6
- **T1, T0**: message type (feedback decoder: T1=1, T0=0)
- **N**: nibble indicator (0 = lower 4 bits, 1 = upper 4 bits)
- **D3..D0**: 4 feedback data bits

Because each message carries only 4 bits (one nibble), two messages are needed to
transmit all 8 bits of a feedback address. The library stores both nibbles separately
in `fbState[addr].nibble[0]` and `fbState[addr].nibble[1]`.

### Cycle timing ###
- One pulse: 92us + 108us = 200us
- One cycle (no decoders): 130 × 200us = 26ms + 6.85ms gap = ~33ms
- One decoder response: 9 bits at 4800 baud = 9 × 208us = ~1.875ms
- One cycle (all 128 decoders): 33ms + 128 × 1.875ms = ~273ms


## PIO hardware ##

Pulse generation and feedback reception are handled entirely by the RP2040/RP2350 PIO
(Programmable I/O) hardware. The PIO runs autonomously once started; no CPU intervention
is needed per pulse. This leaves the CPU free for application code.

Two PIO interrupts are used:
- **PIOx_IRQ_0**: fires when the RX FIFO contains a received feedback message
- **PIOx_IRQ_1**: fires when a complete 130-pulse cycle has finished

For a detailed description of the PIO assembly code, see [pio.md](pio.md).

### Received data format ###
The PIO pushes a 32-bit word into the RX FIFO for each received feedback message:

```
bits [31:24] : pulse counter value at time of reception
bits [23: 0] : 3 samples per data bit, 8 bits total (24 bits of majority voting input)
```

The CPU interrupt handler reads this word and derives the RS-Bus address and data.


## Majority voting ##

Each data bit is sampled 3 times, at 30%, 40% and 50% of the bit period (62us, 83us
and 104us after the start of the bit). The majority of the 3 samples determines the
bit value: if 2 or 3 samples are high, the bit is considered high.

Sampling is deliberately limited to the first half of the bit period. Some RS-Bus
decoders start transmitting on the rising edge of the polling pulse rather than the
falling edge. Such decoders may already terminate their bit before the second half of
the nominal bit period, making late samples (e.g. at 70%) unreliable.

If the 3 samples for any bit are not unanimous (i.e. not all 0 or all 1), the
`errorsSampling` counter for that address is incremented. A high sampling error count
indicates noise or interference on the RS-Bus, or a decoder with marginal timing.

## Parity ##

The RS-Bus uses odd parity. After majority voting and bit inversion, the library
checks the parity of all 8 bits. If the check fails:
- The `errorsParity` counter for that address is incremented
- The gap after the current cycle is extended from 6.85ms to 10.7ms
- The message is discarded and not forwarded to the application

The extended gap signals to the decoder that its message was received with an error,
allowing it to retransmit in the next cycle.

For messages that pass the parity check, the parity bit is stripped before the data
is stored in `rsQueue` and `fbState`. The application always receives 7-bit data
with the parity bit set to 0.

Note that sampling errors and parity errors are independent: a message can have
sampling errors without a parity error (if the majority voting corrected the noise),
or a parity error without sampling errors (if all 3 samples agreed on a wrong value).


## Startup sequence ##

When `start()` is called, the library performs a two-phase startup sequence before
beginning normal polling:

**Phase 1: Reset pulse (88ms)**
The TX pin is driven high for 88ms. This resets all RS-Bus decoders on the bus,
clearing their internal state and preparing them for a fresh registration.

**Phase 2: Initialisation wait (562ms)**
After the reset pulse, the library waits 562ms. During this time the decoders
initialise and configure themselves. The PIO is started at the end of this phase.

**Normal operation**
After the startup sequence, the PIO begins generating 130-pulse cycles. All decoders
re-register by sending their current feedback state in the first 1..2 cycles.
With many decoders on the bus, one cycle can take up to 273ms, so allow at least
600ms after `start()` before expecting stable feedback data.

### Stop and restart ###
`stop()` waits for the current 130-pulse cycle to complete before stopping.
`abort()` halts the pulse train immediately.

After either method, `start()` must be called to resume. `start()` always performs
the full startup sequence, so all decoders will re-register after a restart.


## Error counters ##

The `fbState[]` table maintains per-address error counters that can be used to
diagnose RS-Bus problems:

| Counter | Description |
|---|---|
| `count` | Total messages received for this address |
| `errorsParity` | Messages with a parity error |
| `errorsSampling` | Messages where majority voting was not unanimous |

A healthy RS-Bus installation should show zero or very few errors. If errors are
consistently high for a specific address, check the wiring and decoder at that address.
If errors are high across all addresses, check the RS-Bus termination and power supply.

See the [Statistics example](../examples/Statistics/Statistics.ino) for a sketch that
prints these counters in a formatted table.
