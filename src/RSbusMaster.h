//************************************************************************************************
//
// file:      RSbusMaster.h
// purpose:   RS-Bus Master library for the RP2040/RP2350 (Arduino / Raspberry Pi Pico).
//            Implements an RS-Bus master that generates the 130-pulse polling cycle and
//            receives feedback messages from RS-Bus decoders, using the RP2040/2350 PIO hardware.
// author:    Aiko Pras
// version:   2026-05-27 ap V1.0 First version
//
// Interface:
// - init()           : configure TX/RX pins and PIO
// - start()          : begin RS-Bus polling (130-pulse cycle, continuous)
// - stop()           : graceful stop after current cycle completes
// - abort()          : immediate stop
// - rsQueue[]        : circular buffer (256 entries) of received feedback messages
// - newestIndex      : index of the most recently received message in rsQueue
// - getRsFeedbacks() : returns up to 7 address/data pairs since last call
// - fbState[]        : per-address state table (nibbles, message count, error counts)
//
// Once started, the PIO runs autonomously via interrupts.
// The application does not need to call update() or similar polling functions.

//
// This source file is subject of the GNU general public license 3,
// that is available at the world-wide-web at http://www.gnu.org/licenses/gpl.txt
//
//************************************************************************************************
#pragma once

#include <Arduino.h>
#include "hardware/pio.h"


class RsMaster {
  public:

    // init() sets the TX and RX pins, and (optionally) selects which PIO will be used for the RSBus
    void init(uint txPin, uint rxPin, PIO pio = pio1); 

    // Start, stop and abort the RS-Bus signal generation
    void start();                          // Should be called after init(), but also after stop() / abort()
    void stop();                           // Graceful stop. Completes current pulse train cycle.
    void abort();                          // Stops the pulse train immediately. Intended for short-cuts

    // Circular buffer that stores the received feedback messages in the order in which they are received
    // newestIndex points to the most recently received feedback message
    struct RsEntry {
      uint8_t address;                     // 1..128
      uint8_t data;                        // bits: [6]=nibble, [5:4]=type, [3:0]=feedback data
    };
    volatile RsEntry rsQueue[256];         // Circular buffer, holding recent RSBus feedbacks
    volatile uint8_t newestIndex;          // Points to the last feedback message received by the ISR

    // getRsFeedbacks() checks if new feedback data is available in the rsQueue.
    // Returns true if data is available; false if the queue has no new entries.
    // message: interleaved address/data pairs: [addr1, data1, addr2, data2, ...]
    // length:  number of pairs (0..7); message buffer must hold at least 14 bytes
    bool getRsFeedbacks(uint8_t *message, uint8_t &length);

    // Table holding state information per feedback (RS-Bus) address.
    struct FbState {
      uint8_t nibble[2];                   // To store both nibbles
      uint32_t count;                      // Number of received feedback messages
      uint16_t errorsParity;               // Number of parity (XOR) errors
      uint16_t errorsSampling;             // Number of sampling errors
    };
    FbState fbState[129];                  // Index 0 unused; RS-Bus addresses are 1..128
};


