//************************************************************************************************
//
// file:      RSbusMaster.cpp
// purpose:   RS-bus Master library.
//
// author:    Aiko Pras
// version:   2026-05-27 ap V1.0 First version
//
// Hardware:
//   The RS-Bus is a polling protocol. The master generates a continuous train of 130 pulses.
//   Each pulse consists of a 92us high phase followed by a 108us low phase.
//   During the low phase, the decoder whose address matches the current pulse counter may
//   respond by sending an 8-bit feedback message at 4800 baud (one start bit + 8 data bits).
//
// PIO:
//   Pulse generation and feedback reception are handled entirely by the RP2040/2350 PIO.
//   The PIO runs autonomously once started; no CPU intervention is needed per pulse.
//   Two PIO interrupts are used:
//   - PIOx_IRQ_0: fires when the RX FIFO contains a received feedback message
//   - PIOx_IRQ_1: fires when a complete 130-pulse cycle has finished
//
// Received data format:
//   The PIO pushes a 32-bit word into the RX FIFO per received message:
//   - bits [31:24] : pulse counter value at time of reception (used to derive RS-Bus address)
//   - bits [23: 0] : 3 samples per data bit, 8 bits total (24 bits of majority voting input)
//   The RS-Bus address is derived as: addr = 129 - (raw >> 24)
//
// Majority voting:
//   Each data bit is sampled 3 times (at 30%, 40% and 50% of the bit period).
//   Sampling is limited to the first half of the bit period, rather than spread evenly
//   (e.g. 30/50/70%), because some decoders start transmitting on the rising edge of the
//   pulse rather than the falling edge. Such decoders may already terminate their bit before
//   the second half of the nominal bit period, making late samples unreliable.
//   The majority of the 3 samples determines the bit value.
//   If the 3 samples are not unanimous, an errorsSampling counter is incremented.
//
// Parity:
//   The RS-Bus uses odd parity. If the parity check fails, errorsParity is incremented
//   and the gap between pulse trains is extended to RS_GAP_US_ERROR (10.7ms) to signal
//   the error to the decoder.
//
// Gap timing:
//   After each 130-pulse cycle, the master waits before starting the next cycle:
//   - RS_GAP_US       (6.85ms) : normal gap
//   - RS_GAP_US_ERROR (10.7ms) : extended gap after a parity error
//
// Startup sequence:
//   1. An 88ms high pulse resets all decoders on the bus
//   2. A 562ms wait allows decoders to initialise
//   3. Normal pulse train cycling begins
//
// This source file is subject of the GNU general public license 3,
// that is available at the world-wide-web at http://www.gnu.org/licenses/gpl.txt
//
//************************************************************************************************
#include "RSbusMaster.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "rsbus_pio.h"

constexpr uint NUMBER_RS_PULSES =  130;  // Addresses 1..128, plus extra pulses for start / end
constexpr uint RS_GAP_US =        6850;  // Silence period between 2 RSBus pulse trains (in us)
constexpr uint RS_GAP_US_ERROR = 10700;  // Silence period in case of an XOR error
constexpr uint START_DELAY_1 =   88000;  // Pulse to tell the decoders we have started
constexpr uint START_DELAY_2 =  562000;  // Time for decoders to configure / reset

typedef enum {                           // Tracks the two-phase RSBus startup sequence
    RS_IDLE,
    RS_START_PULSE,                      // 88ms high pulse to reset decoders
    RS_START_WAIT                        // 562ms wait for decoders to initialise
} StartPhase;

struct localData {                       // Local variables
  bool isInitialized = false;            // Flag to ensure init is called once only
  volatile bool isRunning = false;       // Flag to ensure start is called once only
  volatile bool stopRequested = false;   // A flag that tells the PIO ISR that we stop
  StartPhase startPhase = RS_IDLE;       // Current phase of the startup sequence
  PIO pio;                               // The PIO to be used for the RS-Bus
  uint sm;                               // State machine in the PIO
  pio_sm_config pioConfig;               // The PIO configuration
  uint pioOffset;                        // The PIO start offset
  uint txPin;                            // Pin used for transmitting RSBus pulses
  uint rxPin;                            // Pin used to receive RSBus decoder feedback
  struct repeating_timer startTimer;     // The timer used during start up
  struct repeating_timer gapTimer;       // The timer for the 7ms / 9ms gap
  uint32_t gapUs = RS_GAP_US;            // Default value 
  uint8_t lastProcessedIndex;            // getRsFeedbacks: latest processed rsQueue entry
};

// Object instantiation
RsMaster rsBus;                          // Create the object that main should use (via extern)
static localData rs;                     // This object holds local data


//************************************************************************************************
//                                        Support routines
//************************************************************************************************
static void createPioConfig(uint offset) {
  // Assign to the variable pioConfig a default config, "offset" is the PIO programs offset
  // RSBUS_program_get_default_config is a static inline function generated by the pioasm
  rs.pioConfig = RSBUS_program_get_default_config(offset);
  // Specify which pins the PIO SET instruction will operate upon
  // The first parameter is the start pin, and "1" specifies the number of pins
  sm_config_set_set_pins(&rs.pioConfig, rs.txPin, 1);
  // Same, but now for the JMP and IN instructions (which operate on single pins only)
  sm_config_set_jmp_pin(&rs.pioConfig, rs.rxPin);
  sm_config_set_in_pins(&rs.pioConfig, rs.rxPin);
  // Set the shift direction for the input shift register
  // Shift left, no autopush, threshold 32 bits
  sm_config_set_in_shift(&rs.pioConfig, false, false, 32);
  // Configure the clock to tick every 4 us. Use F_CPU to compensate for different clock speeds.
  float pio_clk = 250000; // 250 kHz = 4us per PIO clock tick
  float clkdiv = (float)F_CPU / pio_clk;
  sm_config_set_clkdiv(&rs.pioConfig, clkdiv);
}

static int getPioIrq(PIO pio, uint irqIndex) {
  // Determines which Interrupt Request belongs to this PIO
  if (pio == pio0) return irqIndex ? PIO0_IRQ_1 : PIO0_IRQ_0;
  if (pio == pio1) return irqIndex ? PIO1_IRQ_1 : PIO1_IRQ_0;
   #ifdef PIO2_IRQ_0
    if (pio == pio2) return irqIndex ? PIO2_IRQ_1 : PIO2_IRQ_0;
  #endif
  return -1;  // should never happen; caller must check for invalid PIO
}


//************************************************************************************************
//                           Interrupt Service Routines and Call backs
//************************************************************************************************
//                                   start timer call back
//------------------------------------------------------------------------------------------------
static bool startTimerCallback(struct repeating_timer *t) {
  switch (rs.startPhase) {
    case RS_START_PULSE:
      // Decoders are ready; hand the TX pin over to the PIO and start the first pulse train
      gpio_put(rs.txPin, 0);  // End the 88ms start pulse
      rs.startPhase = RS_START_WAIT;
      add_repeating_timer_us(START_DELAY_2, startTimerCallback, NULL, &rs.startTimer);
      break;
    case RS_START_WAIT:
      // Tell the pinmux to attach the TX pin to the PIO
      pio_gpio_init(rs.pio, rs.txPin);
      // Initialise the state machine with the provided configuration
      pio_sm_init(rs.pio, rs.sm, rs.pioOffset, &rs.pioConfig);
      // Start the state machine
      pio_sm_set_enabled(rs.pio, rs.sm, true);
      // Start the normal 130-pulse cycle
      pio_sm_put_blocking(rs.pio, rs.sm, NUMBER_RS_PULSES);
      rs.startPhase = RS_IDLE;
      break;
    default:     // should never happen
      break;
  }
  return false;  // false = do not repeat, next timer is started explicitly above
}


//------------------------------------------------------------------------------------------------
//                                         FIFO has Data 
//------------------------------------------------------------------------------------------------
//  Parity, T1, T0, N, D3, D2, D1, D0
struct VoteResult {
  uint8_t data;          // voted result
  bool uncertain;        // true if any bit had non-unanimous samples
};

static VoteResult majorityVote(uint32_t samples) {
  VoteResult result = {0, 0};
  for (int bit = 0; bit < 8; bit++) {
    uint8_t s = (samples >> (bit * 3)) & 0x07;     // extract 3 samples for this bit
    if (__builtin_popcount(s) >= 2)                // 2 or 3 samples are high: majority wins
      result.data |= (1 << bit);
    if (s != 0x00 && s != 0x07)                    // not all bits 0 or 1 => sampling error
      result.uncertain = true;
  }
  return result;
}

static void fifoIsrHandler() {
  const uint32_t DATA_MASK = 0x00FFFFFF;           // remaining 24 bits hold data samples
  while (!pio_sm_is_rx_fifo_empty(rs.pio, rs.sm)) {
    uint32_t raw = pio_sm_get(rs.pio, rs.sm);      // get the 32 bits from the PIO
    uint8_t addr = 129 - (raw >> 24);              // first 8 bits
    if (addr < 1 || addr > 128) return;            // should never happen! Is the PIO working well?
    rsBus.fbState[addr].count++;                   // increase the counter for the number of messages
    // Get the data from the raw 24 bits
    VoteResult voted;
    voted = majorityVote(raw & DATA_MASK);         // extract 8 bits from 3 samples each
    uint8_t data = ~voted.data;                    // we receive inverted bits
    if (voted.uncertain)
      rsBus.fbState[addr].errorsSampling++;        // counted independently of parity errors
    // Check for parity errors
    uint8_t parity = 1;                            // odd parity: XOR of all 8 bits plus 1 should equal 0
    for (int i = 0; i < 8; i++) parity ^= (data >> i) & 1;
    // store the results
    if (parity != 0) {
      rs.gapUs = RS_GAP_US_ERROR;                  // extended gap to tell decoder of the error
      rsBus.fbState[addr].errorsParity++;          // increase the counter for parity errors
    } else {
      rs.gapUs = RS_GAP_US;                        // restore normal gap
      data = data & 0x7F;                          // remove parity bit
      uint8_t nb = (data >> 4) & 0x01;             // nibble 0 or 1
      // Fill the circular rsQueue.
      uint8_t idx = rsBus.newestIndex + 1;         // Point to the next entry. Wraps at 255 
      rsBus.rsQueue[idx].address = addr;           // 1..128
      rsBus.rsQueue[idx].data = data;              // 7 bits: nibble bit + 2 type bits + 4 data bits
      rsBus.newestIndex = idx;                     // Tell the user we have new data
      // Fill the state information table
      rsBus.fbState[addr].nibble[nb] = data;       // Store the received data at the right place
    }
  }
}

//------------------------------------------------------------------------------------------------
//                              Enable FIFO has data interrupt handler
//------------------------------------------------------------------------------------------------
static void enableFifoHasDataIsrHandler() {
  // Step 1: Route the RX FIFO not empty event for this SM to CPU interrupt line PIOx_IRQ_0
  // Each state machine has its own interrupt source; select the right one for rs.sm
  switch (rs.sm) {
    case 0: pio_set_irq0_source_enabled(rs.pio, pis_sm0_rx_fifo_not_empty, true); break;
    case 1: pio_set_irq0_source_enabled(rs.pio, pis_sm1_rx_fifo_not_empty, true); break;
    case 2: pio_set_irq0_source_enabled(rs.pio, pis_sm2_rx_fifo_not_empty, true); break;
    case 3: pio_set_irq0_source_enabled(rs.pio, pis_sm3_rx_fifo_not_empty, true); break;
    default: pio_set_irq0_source_enabled(rs.pio, pis_sm0_rx_fifo_not_empty, true);  break; // should never happen
  }
  // Step 2: Determine the CPU IRQ number for PIOx_IRQ_0, then attach and enable the handler
  int irq0 = getPioIrq(rs.pio, 0);
  irq_set_exclusive_handler(irq0, fifoIsrHandler);
  irq_set_enabled(irq0, true);
}


//------------------------------------------------------------------------------------------------
//                          Restart PIO pulse generation for RSBus polling
//------------------------------------------------------------------------------------------------
static bool gapTimerCallback(struct repeating_timer *t) {
  pio_sm_put_blocking(rs.pio, rs.sm, NUMBER_RS_PULSES);  // trigger next 130-pulse cycle
  return false;  // false = do not repeat; next timer is started by cycleDoneIsrHandler
}

//------------------------------------------------------------------------------------------------
//                                      Cycle is done 
//------------------------------------------------------------------------------------------------
static void cycleDoneIsrHandler() {
  pio_interrupt_clear(rs.pio, 0);              // Clear this interrupt
  if (rs.stopRequested) {                      // The application requested us to stop
    pio_sm_set_enabled(rs.pio, rs.sm, false);  // Stop the SM
    pio_sm_clear_fifos(rs.pio, rs.sm);         // flush RX FIFO to discard data from the stopped cycle
    rs.isRunning = false;
    rs.stopRequested = false;
  } else {  // Start timer to trigger next pulse train after gap
    add_repeating_timer_us(rs.gapUs, gapTimerCallback, NULL, &rs.gapTimer); // schedule next cycle
  }
}

static void enableCycleIsDoneIsrHandler() {
  // Step 1: Route the PIO internal flag 0 (pis_interrupt0) to CPU interrupt line PIOx_IRQ_1
  pio_set_irq1_source_enabled(rs.pio, pis_interrupt0, true);
  // Step 2: Determine the CPU interrupt number for PIOx_IRQ_1, then register and enable the handler
  int irq1 = getPioIrq(rs.pio, 1);
  irq_set_exclusive_handler(irq1, cycleDoneIsrHandler);
  irq_set_enabled(irq1, true);
}


//************************************************************************************************
//                                  RsMaster class methods
//************************************************************************************************
void RsMaster::init(uint txPin, uint rxPin, PIO pio) {
  if (rs.isInitialized) return;
  // Step 1: Store the pins for TX and RX
  rs.txPin = txPin;
  rs.rxPin = rxPin;
  // Step 2: Select which PIO we will use. The RP2040 has pio0 and pio1, RP2350 has also pio2
  rs.pio = pio;
  // Step 3: On this PIO, ask which of the four state machines is available
  // The parameter "true" says that this state machine will be assigned to sm (0..3)
  rs.sm = pio_claim_unused_sm(rs.pio, true);
  // Step 4: Load the PIO-programme (RSBUS_program) in this PIO's instruction memory
  // The start address of this programme is returned as parameter and assigned to "offset"
  rs.pioOffset = pio_add_program(rs.pio, &RSBUS_program);
  //Step 5: Make for this PIO the configuration (clock, pins being used for SET, JMP and IN)
  createPioConfig(rs.pioOffset);
  // Step 6: Tell the PIO which pins will be used for input / output. 
  // The parameter "1" represents the number of pins.
  pio_sm_set_consecutive_pindirs(rs.pio, rs.sm, rs.txPin, 1, true);   // true: pin is used for output
  pio_sm_set_consecutive_pindirs(rs.pio, rs.sm, rs.rxPin, 1, false);  // false: pin is used for input
  // Step 7: Attach the input pin to the PIO. The input pin has negative logic
  // Note: the TX pin is deliberately not attached to the PIO here; start() does that
  // after the 88ms startup pulse, which requires SIO (gpio) control of the TX pin.
  pio_gpio_init(rs.pio, rs.rxPin);
  gpio_set_inover(rs.rxPin, GPIO_OVERRIDE_INVERT);  
  // Step 8: Assign and enable the interrupt handlers for the PIO
  // We use two Interrupts:
  // - PIOx_IRQ_0: signals that the PIO has put data in the RX FIFO
  // - PIOx_IRQ_1: signals the end of a 130-pulse cycle
  enableFifoHasDataIsrHandler();
  enableCycleIsDoneIsrHandler();
  // Step 9: Initialise the pointer into the circular output rsQueue
  rsBus.newestIndex       = 0;  // circular buffer starts empty
  rs.lastProcessedIndex   = 0;  // getRsFeedbacks starts at same position
  // Step 10: The PIO is now initialized
  rs.isInitialized = true;
}

void RsMaster::start() {
  if (rs.isRunning) return;
  // Step 1: Attach the TX pin to SIO (Standard IO) for the start pulse
  gpio_init(rs.txPin);
  gpio_set_dir(rs.txPin, GPIO_OUT);
  // Step 2: Send an RS-Bus start pulse (~88ms high) and wait for decoders to reset (~562ms)
  // The startTimerCallback handles the rest (including start of first pulse train) 
  gpio_put(rs.txPin, 1);
  rs.startPhase = RS_START_PULSE;
  add_repeating_timer_us(START_DELAY_1, startTimerCallback, NULL, &rs.startTimer);
  rs.isRunning = true;
}

void RsMaster::stop() {
  if (!rs.isRunning) return;
  // Ask the end of cycle ISR to stop the state machine
  rs.stopRequested = true;
}

void RsMaster::abort() {
  gpio_init(rs.txPin);  // reclaim TX pin from PIO, reset to SIO
  gpio_set_dir(rs.txPin, GPIO_OUT);
  gpio_put(rs.txPin, 0);
  stop();
}

bool RsMaster::getRsFeedbacks(uint8_t *message, uint8_t &length) {
  // See RSbusMaster.h for interface description
  // Check if there are unprocessed entries in the rsQueue
  if (rs.lastProcessedIndex == rsBus.newestIndex) {
    length = 0;
    return false;
  }
  length = 0;
  while (rs.lastProcessedIndex != rsBus.newestIndex && length < 14) {
    rs.lastProcessedIndex++;        // wraps at 255
    message[length++] = rsBus.rsQueue[rs.lastProcessedIndex].address;
    message[length++] = rsBus.rsQueue[rs.lastProcessedIndex].data;
  }
  length /= 2;                     // return number of pairs, not bytes
  return true;
}