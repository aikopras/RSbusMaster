//************************************************************************************************
// File:    FeedbackMonitor.ino
// Purpose: Demonstrates the use of getRsFeedbacks() to receive multiple RS-Bus feedback
//          messages per call. This approach is particularly suited for command stations
//          that forward RS-Bus feedback data via XpressNet Feedback Broadcast messages.
//
// Instead of reading individual messages from rsQueue, this example uses
// getRsFeedbacks() which returns up to 7 address/data pairs per call.
// The application calls getRsFeedbacks() periodically (every 10ms), allowing
// multiple feedback messages to accumulate between calls.
//
// Command stations that forward RS-Bus feedback data via XpressNet should prefer
// this variant over BasicUsage, since XpressNet Feedback Broadcast messages carry
// up to 7 address/data pairs per message. Batching feedback data therefore reduces
// the number of XpressNet messages and makes more efficient use of the bus.
//
// Recommended call interval: 10..50ms, depending on the application:
// - One RS-Bus polling cycle takes around 33ms, so at a 33ms interval roughly
//   the maximum of 7 feedback messages may accumulate per call.
// - A shorter interval (e.g. 10ms) reduces latency for XpressNet users
//   at the cost of smaller batches (typically 2..4 pairs per call).
// - A longer interval (e.g. 50ms) maximises batching efficiency but
//   introduces more latency.
// - During normal operation (after startup) state changes are
//   infrequent, so batching efficiency matters less than latency.
// The 10ms interval in this example is a reasonable default for most
// command station applications.
//
// Expected output (example):
// Address:   1, 0110 (1)
// Address:   1, 1010 (0)
// Address:  22, 0110 (1)
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

uint8_t fbMessage[14];         // buffer for up to 7 address/data pairs
uint8_t fbLength;              // number of pairs returned by getRsFeedbacks()

//************************************************************************************************
void setup() {
  monitor.begin(115200);
  delay(2000);
  monitor.println("RSBusMaster - Feedback Monitor Example");
  rsBus.init(PIN_RSTX, PIN_RSRX);
  rsBus.start();
}

//************************************************************************************************
long previousCheckTime = 0;

void loop() {
  if ((millis() - previousCheckTime) < 10) return;
  previousCheckTime = millis();

  if (rsBus.getRsFeedbacks(fbMessage, fbLength)) {
    for (uint8_t i = 0; i < fbLength; i++) {
      uint8_t addr = fbMessage[i * 2];
      uint8_t data = fbMessage[i * 2 + 1];
      uint8_t nb   = (data >> 4) & 0x01;
      uint8_t bits = data & 0x0F;          // lower 4 data bits

      // Print address, 3 chars wide with leading spaces
      monitor.print("Address: ");
      if (addr < 10)        monitor.print("  ");
      else if (addr < 100)  monitor.print(" ");
      monitor.print(addr);
      monitor.print(", ");

      // Print 4 data bits in binary
for (int b = 3; b >= 0; b--)
  monitor.print((bits >> b) & 1);
      // Print nibble
      monitor.print(" (");
      monitor.print(nb);
      monitor.println(")");
    }
  }
}