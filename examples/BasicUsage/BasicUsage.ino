//************************************************************************************************
// File:    BasicUsage.ino
// Purpose: Demonstrates the simplest way to read RS-Bus feedback messages.
//
// This example reads individual feedback messages from the rsQueue circular buffer.
// Each message contains an RS-Bus address (1..128) and a data byte with 4 feedback
// bits and a nibble indicator (0 = lower 4 bits, 1 = upper 4 bits).
//
// Expected output (example):
// Address:   1, 0110 (1)
// Address:  22, 0110 (1)
// Address: 127, 1010 (0)
//
// Connections:
// - PIN_RSTX (14): RS-Bus transmit pin
// - PIN_RSRX (15): RS-Bus receive pin
//
// Tested on: Raspberry Pi Pico 2
// Should work on all RP2040/RP2350 boards.
//
//************************************************************************************************
#include <Arduino.h>
#include <RSbusMaster.h>

#define monitor                Serial
#define PIN_RSTX               14
#define PIN_RSRX               15

extern RsMaster rsBus;
uint8_t lastProcessedIndex;

//************************************************************************************************
void setup() {
  monitor.begin(115200);
  delay(2000);
  monitor.println("RSBusMaster - Basic Usage Example");
  rsBus.init(PIN_RSTX, PIN_RSRX);
  lastProcessedIndex = rsBus.newestIndex;  // start from current position
  rsBus.start();
}


void loop() {
  while (lastProcessedIndex != rsBus.newestIndex) {
    lastProcessedIndex++;
    uint8_t addr = rsBus.rsQueue[lastProcessedIndex].address;
    uint8_t data = rsBus.rsQueue[lastProcessedIndex].data;
    uint8_t nb   = (data >> 4) & 0x01;
    uint8_t bits = data & 0x0F;          // lower 4 data bits

    // Print address, 3 chars wide with leading spaces
    monitor.print("Address: ");
    if (addr < 10)        monitor.print("  ");
    else if (addr < 100)  monitor.print(" ");
    monitor.print(addr);
    monitor.print(", ");

    // Print 4 data bits in binary
    for (int i = 3; i >= 0; i--)
      monitor.print((bits >> i) & 1);

    // Print nibble
    monitor.print(" (");
    monitor.print(nb);
    monitor.println(")");
  }
}


