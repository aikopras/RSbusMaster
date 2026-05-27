//************************************************************************************************
// File:    StopAndRestart.ino
// Purpose: Demonstrates the stop(), abort() and start() methods of the RSbusMaster library.
//
// The RS-Bus master can be stopped and restarted at any time. Two stop methods
// are provided:
// - stop()  : graceful stop. Waits for the current 130-pulse cycle to complete
//             before stopping. The RS-Bus decoders finish their current cycle
//             cleanly. Use this in normal operation.
// - abort() : immediate stop. Halts the pulse train at once, which may leave
//             decoders in an undefined state. Use this only when a fast stop
//             is required, for example during a DCC emergency stop (Notaus),
//             or a short-cut detection.
//
// After stop() or abort(), start() must be called to resume operation.
// start() always performs the full startup sequence:
// - An 88ms high pulse resets all decoders on the bus
// - A 562ms wait allows decoders to reinitialise
// - Normal polling cycles resume
// This means that after every restart, all decoders will re-register and send
// their current feedback state. With many decoders on the bus, one cycle can
// take up to 273ms (128 decoders × 1.875ms + 33ms base cycle time).
// Allow at least 1..2 full cycles (300..600ms) before expecting stable
// feedback data after a restart.
//
// This example stops and restarts the RS-Bus every 10 seconds using stop(),
// and prints the number of feedback messages received in each run.
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

uint8_t  lastProcessedIndex;
uint16_t messageCount;         // number of feedback messages received in current run
uint16_t runCount;             // number of completed stop/start cycles

//************************************************************************************************
void setup() {
  monitor.begin(115200);
  delay(2000);
  monitor.println("RSBusMaster - Stop and Restart Example");
  rsBus.init(PIN_RSTX, PIN_RSRX);
  lastProcessedIndex = rsBus.newestIndex;
  messageCount = 0;
  runCount = 0;
  rsBus.start();
}

//************************************************************************************************
void countMessages() {
  // Count incoming feedback messages without printing them
  while (lastProcessedIndex != rsBus.newestIndex) {
    lastProcessedIndex++;
    messageCount++;
  }
}

//************************************************************************************************
long cycleTime = 0;

void loop() {
  countMessages();

  if ((millis() - cycleTime) > 10000) {
    cycleTime = millis();
    runCount++;

    // Print results for this run
    monitor.print("Run ");
    monitor.print(runCount);
    monitor.print(": received ");
    monitor.print(messageCount);
    monitor.println(" feedback messages");

    // Graceful stop: waits for current cycle to complete
    // Replace with rsBus.abort() for an immediate stop
    rsBus.stop();

    // Brief pause to allow stop() to complete before restarting
    // stop() is graceful but not instantaneous; one cycle takes around 33ms
    delay(100);

    // Reset counters for next run
    messageCount = 0;
    lastProcessedIndex = rsBus.newestIndex;

    // Restart: performs full startup sequence (around 650ms before first feedback)
    rsBus.start();

    monitor.println("Restarted. Waiting for decoders to re-register...");
  }
}