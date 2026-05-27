# PIO Assembly Code #

This document describes the PIO (Programmable I/O) assembly code that drives the
RS-Bus master. The PIO runs on the RP2040/RP2350 and handles pulse generation and
feedback reception autonomously, without CPU intervention.

For background on the RS-Bus protocol and timing, see [BasicOperation.md](BasicOperation.md).

The assembly source code is embedded as a comment in
[src/rsbus_pio.h](../src/rsbus_pio.h).
Since the Arduino IDE cannot compile PIO assembly directly, the code was compiled
offline using the [Wokwi PIO assembler](https://wokwi.com/tools/pioasm), and the
resulting machine code is stored as a `uint16_t` array in the same file.


## PIO basics ##

The RP2040/RP2350 PIO is a small, dedicated processor with its own instruction set.
It runs independently of the main CPU at a configurable clock rate.
Key resources used by this program:

| Resource | Role |
|---|---|
| `OSR` | Output Shift Register — receives the pulse count (130) from the TX FIFO |
| `X` | Scratch register — current pulse counter (counts down from 130 to 0) |
| `Y` | Scratch register — loop counter (input check loop and bit counter) |
| `ISR` | Input Shift Register — accumulates address and sampled data bits |
| `SET pin` | TX pin — drives the RS-Bus pulse output |
| `JMP pin` | RX pin — monitored for incoming decoder feedback |
| `IN pin` | RX pin — sampled for majority voting |

The clock is configured at 250 kHz (4us per tick).
All timing in this document is expressed in ticks and microseconds.


## Program flow ##

The program has two main paths:

```
main_loop
    ↓
pulse_high  ←───────────────────────────────-───┐
    ↓                                           │
pulse_low                                       │
    ↓                                           │
check_input                                     │
    ├── no input → x-- → pulse_high  ───────────┘
    ├── x == 0  → irq 0 → main_loop (cycle done)
    └── input   → get_input
                      ↓
                  next_bit (× 8)
                      ↓
                  push → x-- → pulse_high ──────┘
```


## Instruction-by-instruction walkthrough ##

### main_loop (instructions 0..2) ###

```asm
pull                    ; wait for TX FIFO, load into OSR
mov x, osr              ; x = 130 (pulse counter)
jmp x-- pulse_high      ; x becomes 129; jump to pulse_high
```

The program starts by waiting for the CPU to push a value (130) into the TX FIFO.
This value is loaded into OSR, then copied to X. The `jmp x--` decrements X to 129
before jumping — this compensates for the fact that the initial value is one too high.

The CPU pushes a new value into the TX FIFO to start each cycle. The `pull` at the
top of the loop blocks until that value arrives, so the PIO waits here between cycles.


### pulse_high (instruction 3) ###

```asm
set pins, 1 [22]        ; TX high for 23 ticks = 92us
```

The TX pin is driven high for 23 ticks (1 tick execution + 22 delay = 23 × 4us = 92us).
On the RS-Bus hardware, a high TX signal opens the MOSFET, pulling the bus towards 0V.


### pulse_low (instructions 4..8) ###

```asm
set pins, 0             ; TX low
set y, 11               ; y = 11 (12 input checks)
check_input:
jmp pin, get_input      ; RX high? → decoder is responding
jmp y-- check_input     ; no input, keep checking (y-- times)
jmp x-- pulse_high      ; no input after all checks: next pulse
```

The TX pin is driven low. The program then checks the RX pin 12 times in a tight loop.
Each iteration takes 2 ticks (8us), so the total check window is 12 × 8us = 96us,
leaving 12us margin before the end of the 108us low phase.

If no input is detected after all 12 checks, `x` is decremented and the next pulse
begins. If `x` reaches 0, the cycle is complete.

### End of cycle (instructions 9..10) ###

```asm
irq 0                   ; signal CPU: cycle complete (triggers PIOx_IRQ_1)
jmp main_loop           ; wait for next TX FIFO value
```

After 130 pulses, the PIO raises IRQ 0, which triggers the `cycleDoneIsrHandler` on
the CPU. The CPU then starts a gap timer (6.85ms or 10.7ms) and pushes a new value
into the TX FIFO when the gap expires, restarting the cycle.


### get_input (instructions 11..14) ###

```asm
in x, 8                 ; shift pulse counter (address) into ISR [bits 31:24]
set y, 7                ; y = 7 (8 bits to receive)
nop [24]                ; wait 25 ticks = 100us
nop [23]                ; wait 24 ticks = 96us
```

When input is detected, the current value of X (the pulse counter) is shifted into
the ISR. This will become the address field of the 32-bit word sent to the CPU.

The program then waits 196us total. This skips the start bit (208us at 4800 baud)
and positions the sampling point at 30% into the first data bit:

```
Start bit: 208us
196us wait lands at: 208us - 196us + overhead ≈ 62us into bit 0 = ~30%
```


### next_bit (instructions 15..22) ###

```asm
next_bit:
nop [14]                ; wait 15 ticks = 60us
in pins, 1              ; sample 1  (~30% of bit period)
nop [3]                 ; wait 4 ticks = 16us
in pins, 1              ; sample 2  (~40% of bit period)
nop [3]                 ; wait 4 ticks = 16us
in pins, 1              ; sample 3  (~50% of bit period)
nop [24]                ; wait 25 ticks = 100us (remainder of bit + into next bit)
jmp y-- next_bit        ; next bit (y = 7..0)
```

Each data bit is sampled 3 times. The 3 samples are shifted into the ISR,
consuming 3 bits of the ISR per data bit. After 8 data bits, the ISR contains
24 bits of sample data in bits [23:0], plus the 8-bit address in bits [31:24].

Sampling at 30/40/50% rather than 30/50/70% is deliberate: some decoders start
transmitting on the rising edge of the polling pulse rather than the falling edge,
and may terminate their bit before the second half of the nominal bit period.

After the 3 samples, the program waits 100us. This covers the remainder of the
current bit period and advances slightly into the next bit, where the 60us `nop`
at the start of `next_bit` brings the sampling point back to 30%.

The bit period at 4800 baud is 208us. The timing per bit is:
```
60us (nop) + sample1 + 16us + sample2 + 16us + sample3 + 100us = ~196us + 3 samples
```


### push and continue (instructions 23..25) ###

```asm
push                    ; send ISR (32 bits) to RX FIFO → triggers PIOx_IRQ_0
nop [4]                 ; safety margin of 20us for slow decoders
jmp x-- pulse_high      ; continue pulse train
```

After all 8 bits are sampled, the ISR is pushed to the RX FIFO. This triggers
`fifoIsrHandler` on the CPU, which reads the 32-bit word and extracts the address
and data.

The 20us safety margin ensures that even slightly slow decoders have finished
transmitting before the next pulse begins.


## ISR bit layout ##

After `push`, the 32-bit word in the RX FIFO has the following layout:

```
bits [31:24] : pulse counter (X) at time of reception
bits [23:21] : 3 samples of data bit 7 (MSB / parity bit)
bits [20:18] : 3 samples of data bit 6 (T1)
bits [17:15] : 3 samples of data bit 5 (T0)
bits [14:12] : 3 samples of data bit 4 (N = nibble indicator)
bits [11: 9] : 3 samples of data bit 3 (D3)
bits [ 8: 6] : 3 samples of data bit 2 (D2)
bits [ 5: 3] : 3 samples of data bit 1 (D1)
bits [ 2: 0] : 3 samples of data bit 0 (D0, LSB)
```

The CPU handler extracts the address as `129 - (raw >> 24)` and passes the lower
24 bits to `majorityVote()`, which processes 3 bits at a time for each of the 8
data bits.


## Compiling the PIO assembly ##

The assembly source is in the comment block at the top of `rsbus_pio.h`.
To recompile after changes, paste the assembly into the
[Wokwi PIO assembler](https://wokwi.com/tools/pioasm) and replace the
`RSBUS_program_instructions` array in `rsbus_pio.h` with the new output.
