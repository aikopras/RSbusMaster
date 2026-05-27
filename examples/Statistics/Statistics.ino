//************************************************************************************************
// File:    Statistics.ino
// Purpose: Demonstrates the use of the fbState[] table to monitor RS-Bus feedback quality.
//
// For each active RS-Bus address, the following statistics are shown:
// - count  : total number of feedback messages received
// - parity : number of parity errors detected
// - sample : number of majority voting (sampling) errors detected
// - error% : percentage of messages with at least one error
//
// Statistics are printed every 10 seconds. Only addresses that have received
// at least one message are shown.
//
// Expected output (example):
//      addr |  count | parity | sample | error%
//      -----|--------|--------|--------|-------
//         1 |    142 |      0 |      2 |   1.4%
//        22 |    138 |      1 |      0 |   0.7%
//      -----|--------|--------|--------|-------
//       tot |    280 |      1 |      2 |   1.1%
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

//************************************************************************************************
void setup() {
  monitor.begin(115200);
  delay(2000);
  monitor.println("RSBusMaster - Statistics Example");
  rsBus.init(PIN_RSTX, PIN_RSRX);
  rsBus.start();
}

//************************************************************************************************
static void printField(uint32_t value, uint8_t width) {
  // Print a number right-aligned in a field of 'width' characters
  uint32_t threshold = 1;
  for (uint8_t i = 1; i < width; i++) threshold *= 10;
  while (threshold > 1 && value < threshold) {
    monitor.print(" ");
    threshold /= 10;
  }
  monitor.print(value);
}

static void printSeparator() {
  monitor.println("-----|--------|--------|--------|-------");
}

void showStatistics() {
  uint32_t totalCount   = 0;
  uint32_t totalParity  = 0;
  uint32_t totalSample  = 0;
  uint8_t  activeCount  = 0;

  monitor.println();
  monitor.println("addr |  count | parity | sample | error%");
  printSeparator();

  for (uint8_t addr = 1; addr <= 128; addr++) {
    uint32_t count   = rsBus.fbState[addr].count;
    uint16_t parity  = rsBus.fbState[addr].errorsParity;
    uint16_t sample  = rsBus.fbState[addr].errorsSampling;
    if (count == 0) continue;

    activeCount++;
    totalCount  += count;
    totalParity += parity;
    totalSample += sample;

    // Address
    printField(addr, 4);
    monitor.print(" | ");
    // Count
    printField(count, 6);
    monitor.print(" | ");
    // Parity errors
    printField(parity, 6);
    monitor.print(" | ");
    // Sampling errors
    printField(sample, 6);
    monitor.print(" | ");
    // Error percentage
    uint32_t errors = parity + sample;
    uint32_t percent_x10 = (errors * 1000) / count;  // one decimal place
    printField(percent_x10 / 10, 4);
    monitor.print(".");
    monitor.print(percent_x10 % 10);
    monitor.println("%");
  }

  printSeparator();
  // Totals
  monitor.print(" tot | ");
  printField(totalCount,  6); monitor.print(" | ");
  printField(totalParity, 6); monitor.print(" | ");
  printField(totalSample, 6); monitor.print(" | ");
  uint32_t totalErrors = totalParity + totalSample;
  uint32_t totalPercent_x10 = totalCount > 0 ? (totalErrors * 1000) / totalCount : 0;
  printField(totalPercent_x10 / 10, 4);
  monitor.print(".");
  monitor.print(totalPercent_x10 % 10);
  monitor.println("%");

  monitor.print("Active addresses: ");
  monitor.println(activeCount);
}

//************************************************************************************************
long cycleTime = 0;

void loop() {
  if ((millis() - cycleTime) > 10000) {
    cycleTime = millis();
    showStatistics();
  }
}