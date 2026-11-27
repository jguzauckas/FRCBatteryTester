#include<ADS1115_WE.h>
#include<Wire.h>
#include<SPI.h>
#include<Adafruit_MCP4725.h>
#include<WDT.h> // Uno R4 (Renesas core) hardware watchdog

#define DAC_I2C_ADDRESS 0x62 // Default address for Adafruit MCP4725
#define ADS_BATTERY_CURRENT_I2C_ADDRESS 0x48 // U6, ADDR to GND: HASS 400-S output (AIN0) vs. its reference (AIN1)
#define ADS_BATTERY_VOLTAGE_I2C_ADDRESS 0x49 // U3, ADDR to 5V: battery sense wires, Sense+ divider on AIN0 minus Sense- divider on AIN1
#define ADS_5V_RAIL_I2C_ADDRESS 0x4A // U26, ADDR to SDA: +5V rail wired straight to AIN0 (+/-6.144 V range)

// Pin numbers follow the KiCad schematic (Arduino Uno R4 WiFi shield)
const int S0_PIN = 2; // CD74HC4067 address lines, shared by all four muxes
const int S1_PIN = 3;
const int S2_PIN = 4;
const int S3_PIN = 5;
const int E_STOP_PIN = 6; // HIGH = 2N7000 kill switch pulls the DAC control line to ground
const int CONTACTOR_PIN = A0; // HIGH = Q34 energizes the LEV200 coil (K1), connecting the battery to the MOSFETs.
                              // R303 pulls the gate low, so K1 stays open while the Arduino resets.
const unsigned long CONTACTOR_CLOSE_MS = 100; // LEV200 closes in tens of ms; wait this long before loading it
const unsigned long CONTACTOR_OPEN_DELAY_MS = 10; // Let the kill switch drop the current before the contacts open

// ADS8320 chip selects. All four share SPI SCK and MISO (ICSP header).
const int ADC_CURRENT_0_CS_PIN = 7;     // U18, V0_MUX: shunt voltages for MOSFETs 0-15
const int ADC_CURRENT_1_CS_PIN = 8;     // U19, V1_MUX: shunt voltages for MOSFETs 16-31
const int ADC_TEMPERATURE_0_CS_PIN = 9; // U20, T0_MUX: thermistors for MOSFETs 0-15
const int ADC_TEMPERATURE_1_CS_PIN = 10; // U21, T1_MUX: thermistors for MOSFETs 16-31

const float ADS8320_VREF = 4.096; // REF3040 reference
const float ADS8320_COUNTS = 65536.0; // 16-bit, straight binary, 0 V to VREF
const SPISettings ADS8320_SPI_SETTINGS(2000000, MSBFIRST, SPI_MODE0); // DCLOCK max is 2.4 MHz at 5 V

// Current ADCs read each shunt differentially (top on +IN, bottom on -IN) through 100 ohm / 1 nF after the
// mux. Each channel's 0.1 uF C1 sits across its shunt and holds the reading, so switching the mux only
// disturbs it by ~1% of the difference between neighboring channels; 150 us is plenty.
// Thermistor ADCs still have 100 ohm / 0.1 uF and the dividers are ~3k (tau ~0.3 ms), so temperatures get
// a long dwell on one address per loop instead.
const unsigned int CURRENT_SETTLE_US = 150;
const unsigned int TEMPERATURE_SETTLE_US = 3000; // ~9 time constants

// Serial link to the host (Pi). Each loop sends one compact CSV data line (~370 characters, ~32 ms at
// 115200 baud). The old labeled format was ~1,000 characters, about 1 s per loop at 9600 baud, which slowed
// every safety check and made match playback impossible. Raise SERIAL_BAUD (e.g. 921600) if the host keeps up;
// both ends must match.
//   H,<field names...>   header: sent at startup and whenever the host sends "H"
//   D,<values...>        one data line per loop, in header order
//   anything else        events and faults: WARNING:..., <NAME>:<details> when a fault trips, then
//                        ERROR:<NAME> once a second while latched
const unsigned long SERIAL_BAUD = 115200;

const unsigned long COMMAND_TIMEOUT_MS = 1000; // Drop the load to 0 A if no command arrives for this long

// Hardware watchdog. If the sketch ever stops running loop() (a stuck I2C bus, a hung library call), the
// software checks stop too and the load would keep running at its last setting. The watchdog resets the
// Arduino if loop() hasn't refreshed it within this time. On reset every pin goes high-impedance, so R303
// opens the contactor and R1 turns on the kill switch: the hardware fails safe without any code running.
// The Uno R4 watchdog allows up to about 5.5 s. A normal loop takes ~40 ms.
const unsigned long WATCHDOG_TIMEOUT_MS = 2000;
bool watchdog_running = false; // Only refresh after WDT.begin(): on this chip a refresh is what starts it

Adafruit_MCP4725 dac;
ADS1115_WE ads_battery_current = ADS1115_WE(ADS_BATTERY_CURRENT_I2C_ADDRESS);
ADS1115_WE ads_battery_voltage = ADS1115_WE(ADS_BATTERY_VOLTAGE_I2C_ADDRESS);
ADS1115_WE ads_5v_rail = ADS1115_WE(ADS_5V_RAIL_I2C_ADDRESS);

const float ROBOT_BATTERY_V_SCALING_FACTOR = (10000.0 + 3300.0) / 3300.0; // 10k top resistor and 3.3k bottom resistor
const float ROBOT_BATTERY_I_SCALING_FACTOR = 400 / 0.625; // HASS 400-S: 0.625 V (Vout - Vref) per 400 A
// The HASS has a small zero error. setup() measures it while the contactor is open (true current is exactly
// 0 A) and subtracts it from every reading after that.
const int BATTERY_I_ZERO_SAMPLES = 32; // ADS1115 runs at 128 SPS, so ~8 ms per sample
const float BATTERY_I_ZERO_MAX_V = 0.1; // ~64 A. A bigger "zero" means the sensor is unpowered or miswired
float battery_i_zero_v = 0.0;

const float THERMISTOR_R_FIXED = 3900.0; // Fixed (bottom) resistor for voltage divider, 3.9k so 85 C stays under the 4.096 V reference
// The thermistor dividers run from the +5V rail, but the ADS8320s measure against the 4.096 V reference, so
// the conversion needs the rail's actual voltage. U26 measures it every loop; a 2% low rail would otherwise
// make every MOSFET read ~1.5 C cool at 60 C.
const float THERMISTOR_V_IN_DEFAULT = 5.0; // Used only if U26 doesn't respond at startup
const float V5_RAIL_MIN = 4.5; // Outside this window the rail (or U26) is faulty, so the tester shuts down
const float V5_RAIL_MAX = 5.5;
float thermistor_v_in = THERMISTOR_V_IN_DEFAULT; // Measured +5V rail, used in the thermistor math
bool v5_rail_adc_ok = false;
const float THERMISTOR_R_NOMINAL = 10000.0; // Thermistors rated resistance at base temperature
const float THERMISTOR_T_NOMINAL = 25.0 + 273.15; // 25 degrees Celsius converted to Kelvin
const float THERMISTOR_BETA = 3988.0; // Rated Beta of Thermistor
const float THERMISTOR_V_MIN_VALID = 0.05; // Below this the thermistor is open/disconnected
// The ADS8320 tops out at 4.096 V but the divider runs from 5 V. With the 4.7k bottom resistor, 85 C reads
// about 4.08 V, so readings above THERMISTOR_V_MAX_VALID (about 84 C on a 5.0 V rail) count as saturated.
// A saturated reading (too hot to measure, or a shorted thermistor) is reported as TEMPERATURE_SATURATED,
// which trips the over-temperature shutdown.
const float THERMISTOR_V_MAX_VALID = ADS8320_VREF - 0.05;
const float TEMPERATURE_FAULT = -999.0; // Open or disconnected thermistor
const float TEMPERATURE_SATURATED = 999.0; // Above the measurable range

// const float SHUNT_RESISTOR = 0.01; // Shunt resistor is 0.01 ohms // Old

const float DAC_V_IN = 5.0; // DAC voltage input is 5 volts
const int DAC_COUNT = 4095; // DAC resolution is 12 bits (2^12 -1)
// DAC Control (buffered DAC output, 0-5 V) feeds a 47k / 1.2k divider at each channel's op-amp + input,
// referenced to that channel's shunt bottom, so full scale per channel is about 0.124 V (about 13 A per MOSFET).
const float DAC_DIVIDER_RATIO = 1200.0 / (47000.0 + 1200.0);
const float MAX_CURRENT_REQUEST = 10.0 * 32; // 10 A average per MOSFET

// Over-current trips. At a full 320 A request the channels don't share evenly: channel 0's 9.3 mohm shunt
// alone puts it ~2% above average (~10.2 A), and LM324A offset (up to 3 mV, ~0.3 A), divider tolerance,
// the 0.1 mohm rounding of the measured shunts and the outer gain loop (if a channel drops out) can each
// add a few tenths more. An 11 A trip could fire with nothing wrong, so it's 12 A. A real failure (a
// shorted MOSFET) is tens to hundreds of amps, so it still trips immediately.
const float MOSFET_CURRENT_TRIP_A = 12.0;
const float BATTERY_CURRENT_TRIP_A = MAX_CURRENT_REQUEST * 1.1; // 352 A, from the HASS

// Outer current loop. The DAC uses the 5 V rail as its reference and the 47k / 1.2k dividers have tolerance,
// so the commanded current is off by a few percent. Each loop compares the total measured across all 32
// shunts with what was asked for last loop and nudges a gain correction applied to the DAC command.
// Correcting gain (not adding amps) keeps the correction right as the request changes during playback.
const float GAIN_LEARN_MIN_CURRENT = 30.0; // A total (~1 A per MOSFET). Below this, op-amp offsets (up to
                                          // ~0.3 A per channel) would skew the estimate, so the gain isn't updated
const float GAIN_LEARN_RATE = 0.5; // Fraction of the measured error removed each loop (stable for any loop time)
const float GAIN_CORRECTION_MIN = 0.85;
const float GAIN_CORRECTION_MAX = 1.15; // Limits how hard the other channels get pushed if some drop out
float gain_correction = 1.0;
float last_current_command = 0.0; // Request that produced the currents measured this loop
bool last_command_saturated = false; // DAC was clipped at full scale, so the measurement can't be trusted for learning

// Battery voltage limits. Real FRC batteries sag to 7-8 V on current spikes, so the running cutoff only trips
// on a sustained sag (about what a roboRIO tolerates). The startup check is stricter: a resting battery
// below 10.5 V is deeply discharged and shouldn't be loaded at all.
const float BATTERY_CUTOFF_V = 6.5;
const unsigned long BATTERY_CUTOFF_MS = 250; // Must stay below BATTERY_CUTOFF_V this long to trip
const float BATTERY_START_MIN_V = 10.5;
unsigned long battery_low_since_ms = 0; // 0 = battery currently above BATTERY_CUTOFF_V

// No-current check. The battery sense wires read the battery directly, so an open CB1 (or a contactor that
// didn't close) no longer shows up as a low battery. Instead, if a real request produces almost no current
// for a while, something in the high-current path is open: report it and shut down.
const float NO_CURRENT_CHECK_MIN_A = 30.0; // Only check requests at least this big (A total)
const float NO_CURRENT_FRACTION = 0.1; // "Almost no current" = under 10% of the request
const unsigned long NO_CURRENT_MS = 500; // Must last this long, so normal steps never trip it
unsigned long no_current_since_ms = 0; // 0 = current is flowing (or no check needed)

float robot_battery_v = 12.0; // Starting voltage (high enough to prevent shutdown)
float robot_battery_i = 0.0; // Starting current (low enough to prevent shutdown)
float mosfet_current_total = 0.0; // Starting current of all mosfets
float mosfet_avg_temp = 0.0; // Starting temperature
float current_requested = 0.0; // Current requested at any given time
unsigned long last_command_ms = 0; // millis() of the last valid command from the host

// Incoming commands are collected a character at a time, so reading them never waits on the host.
const byte COMMAND_BUFFER_SIZE = 32; // Longest valid command is "A320.0" or "ESTOP"
char command_buffer[COMMAND_BUFFER_SIZE];
byte command_length = 0;
bool command_overflow = false; // Line too long: ignore it up to the next newline

const int MOSFET_COUNT = 32;
const float SHUNT_RESISTANCES[MOSFET_COUNT] = {0.0093, 0.0095, 0.0095, 0.0095, 
                                             0.0095, 0.0094, 0.0095, 0.0095,
                                             0.0095, 0.0095, 0.0096, 0.0096,
                                             0.0095, 0.0096, 0.0095, 0.0095,
                                             0.0096, 0.0094, 0.0095, 0.0094,
                                             0.0095, 0.0096, 0.0096, 0.0095,
                                             0.0095, 0.0095, 0.0096, 0.0095,
                                             0.0095, 0.0095, 0.0095, 0.0096};
float total_system_conductance = 0.0;
float mosfet_currents[MOSFET_COUNT];
float mosfet_temperatures[MOSFET_COUNT];
byte next_temperature_address = 0; // Mux address whose thermistors get read on the next loop

// Dead-channel warning. If one MOSFET channel stops conducting (failed MOSFET, op-amp, gate or sense wire),
// the outer gain loop quietly pushes the other 31 harder to make up the total. That's safe, but you'd want
// to know. Every channel gets the same setpoint, so a healthy one stays near the average.
const float CHANNEL_CHECK_MIN_AVG_A = 2.0; // Only check above this average per channel (offsets matter less)
const float CHANNEL_LOW_FRACTION = 0.5; // "Low" = under half the average
const unsigned long CHANNEL_LOW_MS = 500; // Must last this long before warning
unsigned long channel_low_since_ms[MOSFET_COUNT]; // 0 = channel currently fine
bool channel_low_warned[MOSFET_COUNT]; // Warn once per channel until it recovers or the load drops

void setup() {
  // Battery disconnected and load killed before anything else happens.
  pinMode(CONTACTOR_PIN, OUTPUT);
  digitalWrite(CONTACTOR_PIN, LOW);
  pinMode(E_STOP_PIN, OUTPUT);
  digitalWrite(E_STOP_PIN, HIGH);

  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);

  pinMode(ADC_CURRENT_0_CS_PIN, OUTPUT);
  pinMode(ADC_CURRENT_1_CS_PIN, OUTPUT);
  pinMode(ADC_TEMPERATURE_0_CS_PIN, OUTPUT);
  pinMode(ADC_TEMPERATURE_1_CS_PIN, OUTPUT);
  digitalWrite(ADC_CURRENT_0_CS_PIN, HIGH);
  digitalWrite(ADC_CURRENT_1_CS_PIN, HIGH);
  digitalWrite(ADC_TEMPERATURE_0_CS_PIN, HIGH);
  digitalWrite(ADC_TEMPERATURE_1_CS_PIN, HIGH);

  Serial.begin(SERIAL_BAUD);
  print_log_header();

  initialize_mosfet_arrays();

  Wire.begin();
  SPI.begin();

  dac.begin(DAC_I2C_ADDRESS);
  dac.setVoltage(0, false); // Make sure the load starts at 0 A before the kill switch is released

  // Each ADS1115 now reads a single input, so the channel is set once and never switched.
  ads_battery_current.init();
  ads_battery_current.setVoltageRange_mV(ADS1115_RANGE_4096);
  ads_battery_current.setCompareChannels(ADS1115_COMP_0_1);
  ads_battery_current.setMeasureMode(ADS1115_CONTINUOUS);

  ads_battery_voltage.init();
  ads_battery_voltage.setVoltageRange_mV(ADS1115_RANGE_4096);
  ads_battery_voltage.setCompareChannels(ADS1115_COMP_0_1);
  ads_battery_voltage.setMeasureMode(ADS1115_CONTINUOUS);

  v5_rail_adc_ok = ads_5v_rail.init();
  if (v5_rail_adc_ok) {
    ads_5v_rail.setVoltageRange_mV(ADS1115_RANGE_6144); // AIN0 sits at VDD, allowed up to VDD + 0.3 V
    ads_5v_rail.setCompareChannels(ADS1115_COMP_0_GND);
    ads_5v_rail.setMeasureMode(ADS1115_CONTINUOUS);
  } else {
    Serial.println("WARNING:V5_RAIL_ADC_MISSING, assuming 5.0 V for thermistors");
  }

  // A watchdog reset means the sketch hung while the battery was connected. Don't quietly reconnect it:
  // stay latched until someone presses reset or power-cycles the tester (both clear this flag).
  if (R_SYSTEM->RSTSR1_b.WDTRF) {
    Serial.println("WATCHDOG_RESET_FAULT:sketch hung, press reset to restart");
    shut_down("WATCHDOG_RESET_FAULT");
  }

  delay(1000); // Give hardware time to stabilize
  read_5v_rail(); // Before any thermistor is read

  // Read every thermistor once so no MOSFET starts with an unchecked temperature.
  for (byte address = 0; address < 16; address++) {
    read_mosfet_temperatures();
  }

  // Contactor is still open, so no current flows through the HASS: whatever it reads now is its zero error.
  calibrate_battery_current_zero();

  // The battery voltage divider sits on the battery side of K1, so the battery can be checked before
  // it is connected. Refuse to start (and latch) if anything is already out of range.
  read_robot_battery();
  if (robot_battery_v < BATTERY_START_MIN_V) {
    Serial.print("BATT_TOO_LOW_TO_START:");
    Serial.println(robot_battery_v, 2);
    shut_down("BATT_TOO_LOW_TO_START");
  }
  const char *fault = should_shut_down();
  if (fault != nullptr) {
    shut_down(fault);
  }

  // Start the watchdog just before the battery is connected. Everything above runs with the contactor
  // open, so a hang there is harmless. Refuse to connect the battery without it.
  if (!WDT.begin(WATCHDOG_TIMEOUT_MS)) {
    Serial.println("WATCHDOG_START_FAULT");
    shut_down("WATCHDOG_START_FAULT");
  }
  watchdog_running = true;

  // Close the contactor with the load still killed and the DAC at 0, then release the kill switch.
  digitalWrite(CONTACTOR_PIN, HIGH);
  delay(CONTACTOR_CLOSE_MS);

  last_command_ms = millis();
  digitalWrite(E_STOP_PIN, LOW);
}

void initialize_mosfet_arrays() {
  for (byte i = 0; i < MOSFET_COUNT; i++) {
    mosfet_currents[i] = 0.0;
    mosfet_temperatures[i] = 0.0;
    channel_low_since_ms[i] = 0;
    channel_low_warned[i] = false;
    total_system_conductance += (1.0 / SHUNT_RESISTANCES[i]);
  }
}

void loop() {
  WDT.refresh(); // Proves loop() is still running; see WATCHDOG_TIMEOUT_MS

  // Monitor Values
  read_robot_battery();
  read_5v_rail();

  read_mosfets();

  log_data();

  read_commands();

  // Watchdog: if the host stops talking (crash, unplugged cable), stop sinking current.
  // The host must resend its "A<amps>" command at least every COMMAND_TIMEOUT_MS.
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    current_requested = 0.0;
  }

  check_load_current(); // Before learning, so an open circuit never winds up the gain
  check_channel_balance();
  update_gain_correction();
  set_current_request(current_requested); // Use this to control the voltage of the DAC for mosfet control.

  const char *fault = should_shut_down();
  if (fault != nullptr) {
    shut_down(fault);
  }
}

// Handle every complete line the host has sent since the last loop, not just one. During playback the
// host may send faster than the ~40 ms loop; reading one line per loop would let a queue build up, so
// playback would fall further and further behind (and an ESTOP would wait at the back of the line).
// Lines are handled in order, so the newest "A" wins and ESTOP acts as soon as it's read.
// A partial line just waits in command_buffer for the rest of it next loop.
void read_commands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (!command_overflow && command_length > 0) {
        command_buffer[command_length] = '\0';
        handle_command(command_buffer);
      }
      command_length = 0;
      command_overflow = false;
    } else if (command_length < COMMAND_BUFFER_SIZE - 1) {
      command_buffer[command_length++] = c;
    } else {
      command_overflow = true;
    }
  }
}

void handle_command(const char *command) {
  if (strcmp(command, "ESTOP") == 0) {
    Serial.println("ESTOP_COMMAND:");
    shut_down("ESTOP_COMMAND");
  } else if (strcmp(command, "H") == 0) {
    print_log_header();
  } else if (command[0] == 'A') {
    float requested = atof(command + 1);
    if (requested < 0.0) {
      requested = 0.0;
    }
    if (requested > MAX_CURRENT_REQUEST) {
      requested = MAX_CURRENT_REQUEST;
    }
    current_requested = requested;
    last_command_ms = millis();
  }
}

void calibrate_battery_current_zero() {
  float sum = 0.0;
  for (int i = 0; i < BATTERY_I_ZERO_SAMPLES; i++) {
    delay(10); // Let a fresh continuous-mode conversion finish
    sum += ads_battery_current.getResult_V();
  }
  battery_i_zero_v = sum / BATTERY_I_ZERO_SAMPLES;

  if (fabs(battery_i_zero_v) > BATTERY_I_ZERO_MAX_V) {
    Serial.print("BATT_I_ZERO_FAULT:");
    Serial.println(battery_i_zero_v, 4);
    shut_down("BATT_I_ZERO_FAULT"); // Can't trust the current sensor, so never connect the battery
  }
}

void read_5v_rail() {
  if (!v5_rail_adc_ok) {
    return; // Keep THERMISTOR_V_IN_DEFAULT
  }
  float v = ads_5v_rail.getResult_V();
  if (v < V5_RAIL_MIN || v > V5_RAIL_MAX) {
    Serial.print("V5_RAIL_FAULT:");
    Serial.println(v, 3);
    shut_down("V5_RAIL_FAULT"); // Every sensor and the thermistor math depend on this rail
  }
  thermistor_v_in = v;
}

void read_robot_battery() {
  robot_battery_i = (ads_battery_current.getResult_V() - battery_i_zero_v) * ROBOT_BATTERY_I_SCALING_FACTOR;
  robot_battery_v = ads_battery_voltage.getResult_V() * ROBOT_BATTERY_V_SCALING_FACTOR;
}

void read_mosfets() {
  read_mosfet_currents();
  read_mosfet_temperatures();
}

// Sweep all 16 mux addresses. At each address U18 reads MOSFET n and U19 reads MOSFET n + 16.
void read_mosfet_currents() {
  float total_current = 0.0;
  for (byte address = 0; address < 16; address++) {
    set_mosfet_mux_address(address);
    delayMicroseconds(CURRENT_SETTLE_US);

    byte low = address;
    byte high = address + 16;
    mosfet_currents[low] = read_ads8320_volts(ADC_CURRENT_0_CS_PIN) / SHUNT_RESISTANCES[low];
    mosfet_currents[high] = read_ads8320_volts(ADC_CURRENT_1_CS_PIN) / SHUNT_RESISTANCES[high];
    total_current += mosfet_currents[low] + mosfet_currents[high];
  }
  mosfet_current_total = total_current;
}

// Temperatures change slowly and their inputs settle slowly, so each loop reads one mux address
// (two MOSFETs). Every thermistor is refreshed once every 16 loops.
void read_mosfet_temperatures() {
  byte address = next_temperature_address;
  set_mosfet_mux_address(address);
  delayMicroseconds(TEMPERATURE_SETTLE_US);

  mosfet_temperatures[address] = temperature(read_ads8320_volts(ADC_TEMPERATURE_0_CS_PIN));
  mosfet_temperatures[address + 16] = temperature(read_ads8320_volts(ADC_TEMPERATURE_1_CS_PIN));
  next_temperature_address = (address + 1) % 16;

  float total_temperature = 0.0;
  for (byte i = 0; i < MOSFET_COUNT; i++) {
    total_temperature += mosfet_temperatures[i];
  }
  mosfet_avg_temp = total_temperature / (float) MOSFET_COUNT;
}

// One ADS8320 conversion. Pulling CS low starts sampling; the first ~5 clocks are the sample period,
// then a null bit, then 16 data bits MSB first. Clocking 24 bits leaves the result in bits 17..2.
uint16_t read_ads8320(int cs_pin) {
  SPI.beginTransaction(ADS8320_SPI_SETTINGS);
  digitalWrite(cs_pin, LOW);
  uint32_t raw = SPI.transfer(0);
  raw = (raw << 8) | SPI.transfer(0);
  raw = (raw << 8) | SPI.transfer(0);
  digitalWrite(cs_pin, HIGH);
  SPI.endTransaction();
  return (uint16_t) ((raw >> 2) & 0xFFFF);
}

float read_ads8320_volts(int cs_pin) {
  return read_ads8320(cs_pin) * (ADS8320_VREF / ADS8320_COUNTS);
}

void set_mosfet_mux_address(int channel) {
  digitalWrite(S0_PIN, channel & 1);
  digitalWrite(S1_PIN, (channel >> 1) & 1);
  digitalWrite(S2_PIN, (channel >> 2) & 1);
  digitalWrite(S3_PIN, (channel >> 3) & 1);
}

float temperature(float voltage) {
  if (voltage < THERMISTOR_V_MIN_VALID) {
    return TEMPERATURE_FAULT; // Open or disconnected: treated as a fault by should_shut_down()
  }
  if (voltage > THERMISTOR_V_MAX_VALID) {
    return TEMPERATURE_SATURATED; // Too hot to measure or shorted: trips the over-temperature check
  }

  float resistance = THERMISTOR_R_FIXED * ((thermistor_v_in / voltage) - 1.0);
  float steinhart = 1.0 / ((1.0 / THERMISTOR_T_NOMINAL) + (1.0 / THERMISTOR_BETA) * log(resistance / THERMISTOR_R_NOMINAL));
  return steinhart - 273.15;
}

void print_log_header() {
  Serial.print("H,ms,batt_v,batt_i,v5_rail,req_i,fet_total_i,gain_corr,fet_avg_t");
  for (int i = 0; i < MOSFET_COUNT; i++) {
    Serial.print(",i");
    Serial.print(i);
  }
  for (int i = 0; i < MOSFET_COUNT; i++) {
    Serial.print(",t");
    Serial.print(i);
  }
  Serial.println();
}

void log_data() {
  Serial.print("D,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(robot_battery_v, 2);
  Serial.print(',');
  Serial.print(robot_battery_i, 1);
  Serial.print(',');
  Serial.print(thermistor_v_in, 3);
  Serial.print(',');
  Serial.print(current_requested, 1);
  Serial.print(',');
  Serial.print(mosfet_current_total, 1);
  Serial.print(',');
  Serial.print(gain_correction, 4);
  Serial.print(',');
  Serial.print(mosfet_avg_temp, 1);
  for (int i = 0; i < MOSFET_COUNT; i++) {
    Serial.print(',');
    Serial.print(mosfet_currents[i], 2);
  }
  for (int i = 0; i < MOSFET_COUNT; i++) {
    Serial.print(',');
    Serial.print(mosfet_temperatures[i], 1);
  }
  Serial.println();
}

// Prints a "<NAME>:<details>" line and returns NAME for the first limit that's exceeded, or nullptr if
// everything is in range. The latched ERROR lines repeat NAME, so a host that connects late still sees why.
const char *should_shut_down() {
  // fabs: the HASS reads negative if it's mounted (or its leads swapped) backwards. This trip is the only
  // one that sees current bypassing the shunts, like the drain bus shorted to ground, so it must work
  // either way. (A backwards sensor also shows up as negative batt_i in the log.)
  if (fabs(robot_battery_i) >= BATTERY_CURRENT_TRIP_A) {
    Serial.print("BATT_OVERCURRENT_FAULT:");
    Serial.println(robot_battery_i, 1);
    return "BATT_OVERCURRENT_FAULT";
  }
  if (robot_battery_v < BATTERY_CUTOFF_V) {
    if (battery_low_since_ms == 0) {
      battery_low_since_ms = millis() | 1; // | 1 so a millis() of exactly 0 doesn't read as "not low"
    } else if (millis() - battery_low_since_ms >= BATTERY_CUTOFF_MS) {
      Serial.print("BATT_LOW_FAULT:");
      Serial.println(robot_battery_v, 2);
      return "BATT_LOW_FAULT";
    }
  } else {
    battery_low_since_ms = 0;
  }

  for (byte i = 0; i < MOSFET_COUNT; i++) {
    if (mosfet_currents[i] > MOSFET_CURRENT_TRIP_A) {
      print_channel_fault("FET_OVERCURRENT_FAULT", i, mosfet_currents[i]);
      return "FET_OVERCURRENT_FAULT";
    }
    if (mosfet_temperatures[i] > 60.0) {
      print_channel_fault("FET_OVERTEMP_FAULT", i, mosfet_temperatures[i]); // 999.0 = above measurable range
      return "FET_OVERTEMP_FAULT";
    }
    if (mosfet_temperatures[i] == TEMPERATURE_FAULT) {
      print_channel_fault("THERMISTOR_OPEN_FAULT", i, mosfet_temperatures[i]);
      return "THERMISTOR_OPEN_FAULT"; // A thermistor we can't read means we can't protect that MOSFET
    }
  }
  return nullptr;
}

// e.g. "FET_OVERCURRENT_FAULT:ch5,12.31"
void print_channel_fault(const char *name, byte channel, float value) {
  Serial.print(name);
  Serial.print(":ch");
  Serial.print(channel);
  Serial.print(',');
  Serial.println(value, 2);
}

void shut_down(const char *reason) {
  // Remove the load first: engage the hardware kill switch, then open the contactor once the current has
  // dropped so the LEV200 is not breaking full load, then latch. Zeroing the DAC goes last because it
  // needs I2C: if the bus is what failed, the battery is already disconnected before that call can hang.
  current_requested = 0.0;
  digitalWrite(E_STOP_PIN, HIGH);
  delay(CONTACTOR_OPEN_DELAY_MS);
  digitalWrite(CONTACTOR_PIN, LOW);
  dac.setVoltage(0, false);

  while (true) {
    // Keep refreshing so the watchdog doesn't reset us out of the latch
    if (watchdog_running) {
      WDT.refresh();
    }
    Serial.print("ERROR:");
    Serial.println(reason);
    delay(1000);
  }
}

// Called once per loop, after read_mosfets() has measured the result of last loop's command.
// Warning only (no shutdown): e.g. "WARNING:CHANNEL_LOW:ch5,0.21,avg,6.10"
void check_channel_balance() {
  float average = mosfet_current_total / MOSFET_COUNT;
  unsigned long now = millis();
  for (byte i = 0; i < MOSFET_COUNT; i++) {
    if (average < CHANNEL_CHECK_MIN_AVG_A || mosfet_currents[i] >= average * CHANNEL_LOW_FRACTION) {
      channel_low_since_ms[i] = 0;
      channel_low_warned[i] = false;
    } else if (channel_low_since_ms[i] == 0) {
      channel_low_since_ms[i] = now | 1; // | 1 so a millis() of exactly 0 doesn't read as "fine"
    } else if (!channel_low_warned[i] && now - channel_low_since_ms[i] >= CHANNEL_LOW_MS) {
      Serial.print("WARNING:CHANNEL_LOW:ch");
      Serial.print(i);
      Serial.print(',');
      Serial.print(mosfet_currents[i], 2);
      Serial.print(",avg,");
      Serial.println(average, 2);
      channel_low_warned[i] = true;
    }
  }
}

// Called once per loop, after read_mosfets() has measured the result of last loop's command.
void check_load_current() {
  if (last_current_command >= NO_CURRENT_CHECK_MIN_A &&
      mosfet_current_total < last_current_command * NO_CURRENT_FRACTION) {
    if (no_current_since_ms == 0) {
      no_current_since_ms = millis() | 1; // | 1 so a millis() of exactly 0 doesn't read as "flowing"
    } else if (millis() - no_current_since_ms >= NO_CURRENT_MS) {
      Serial.print("NO_CURRENT_FAULT:requested ");
      Serial.print(last_current_command, 1);
      Serial.print(" A, measured ");
      Serial.print(mosfet_current_total, 1);
      Serial.println(" A (check CB1 and the contactor)");
      shut_down("NO_CURRENT_FAULT");
    }
  } else {
    no_current_since_ms = 0;
  }
}

// Called once per loop, after read_mosfets() has measured the result of last loop's command.
void update_gain_correction() {
  if (last_current_command < GAIN_LEARN_MIN_CURRENT || last_command_saturated || mosfet_current_total < 0.5) {
    return; // Nothing trustworthy to learn from. Keep the last correction.
  }
  // If the system delivered `measured` for `last_current_command`, this correction would have hit it exactly.
  float ideal = gain_correction * last_current_command / mosfet_current_total;
  gain_correction += GAIN_LEARN_RATE * (ideal - gain_correction);
  gain_correction = constrain(gain_correction, GAIN_CORRECTION_MIN, GAIN_CORRECTION_MAX);
}

void set_current_request(float current) {
  last_current_command = current;

  // Every op-amp holds its shunt at the per-channel setpoint, so total current = V * (sum of 1/R).
  float controlVoltage = (current * gain_correction) / total_system_conductance;
  float targetVoltage = controlVoltage / DAC_DIVIDER_RATIO; // Undo the per-channel 47k / 1.2k divider

  float rawCounts = (targetVoltage / DAC_V_IN) * (float) DAC_COUNT;
  int dacOutput = round(rawCounts);

  last_command_saturated = dacOutput > DAC_COUNT;
  if (dacOutput < 0) {
    dac.setVoltage(0, false);
  } else if (dacOutput > DAC_COUNT) {
    dac.setVoltage(DAC_COUNT, false);
  } else {
    dac.setVoltage(dacOutput, false);
  }
}
