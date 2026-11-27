#include<ADS1115_WE.h>
#include<Wire.h>
#include<SPI.h>
#include<Adafruit_MCP4725.h>

#define DAC_I2C_ADDRESS 0x62 // Default address for Adafruit MCP4725
#define ADS_BATTERY_CURRENT_I2C_ADDRESS 0x48 // U6, ADDR to GND: HTFS400-P output (AIN0) vs. its reference (AIN1)
#define ADS_BATTERY_VOLTAGE_I2C_ADDRESS 0x49 // U3, ADDR to 5V: battery voltage divider on AIN0

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

// Each ADC input has a 100 ohm / 0.1 uF filter after the mux, so the cap has to recharge every time the
// mux address changes. Shunt inputs are low impedance (tau ~17 us with mux Ron); the thermistor dividers
// are ~5k (tau ~0.5 ms), so temperatures get a long dwell on one address per loop instead.
const unsigned int CURRENT_SETTLE_US = 150; // ~8 time constants
const unsigned int TEMPERATURE_SETTLE_US = 3000; // ~6 time constants

const unsigned long COMMAND_TIMEOUT_MS = 1000; // Drop the load to 0 A if no command arrives for this long

Adafruit_MCP4725 dac;
ADS1115_WE ads_battery_current = ADS1115_WE(ADS_BATTERY_CURRENT_I2C_ADDRESS);
ADS1115_WE ads_battery_voltage = ADS1115_WE(ADS_BATTERY_VOLTAGE_I2C_ADDRESS);

const float ROBOT_BATTERY_V_SCALING_FACTOR = (10000.0 + 3300.0) / 3300.0; // 10k top resistor and 3.3k bottom resistor
const float ROBOT_BATTERY_I_SCALING_FACTOR = 400 / 0.625; // 400 amps across 0.625 volt range

const float THERMISTOR_R_FIXED = 3900.0; // Fixed (bottom) resistor for voltage divider, 3.9k so 85 C stays under the 4.096 V reference
const float THERMISTOR_V_IN = 5.0; // Voltage going into Thermistor
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
const float MAX_CURRENT_REQUEST = 10.0 * 32; // 10 A average per MOSFET, under the 11 A per-MOSFET trip

float robot_battery_v = 12.0; // Starting voltage (high enough to prevent shutdown)
float robot_battery_i = 0.0; // Starting current (low enough to prevent shutdown)
float mosfet_current_total = 0.0; // Starting current of all mosfets
float mosfet_avg_temp = 0.0; // Starting temperature
float current_requested = 0.0; // Current requested at any given time
unsigned long last_command_ms = 0; // millis() of the last valid command from the host

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

  Serial.begin(9600);
  Serial.setTimeout(10);

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
  ads_battery_voltage.setCompareChannels(ADS1115_COMP_0_GND);
  ads_battery_voltage.setMeasureMode(ADS1115_CONTINUOUS);

  delay(1000); // Give hardware time to stabilize

  // Read every thermistor once so no MOSFET starts with an unchecked temperature.
  for (byte address = 0; address < 16; address++) {
    read_mosfet_temperatures();
  }

  // The battery voltage divider sits on the battery side of K1, so the battery can be checked before
  // it is connected. Refuse to start (and latch) if anything is already out of range.
  read_robot_battery();
  if (should_shut_down()) {
    shut_down();
  }

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
    total_system_conductance += (1.0 / SHUNT_RESISTANCES[i]);
  }
}

void loop() {
  // Monitor Values
  read_robot_battery();

  read_mosfets();

  log_data();

  if (Serial.available() > 0) {
    String incomingCommand = Serial.readStringUntil('\n');
    incomingCommand.trim();

    if (incomingCommand == "ESTOP") {
      shut_down();
    }
    else if (incomingCommand.substring(0, 1) == "A") {
      float requested = incomingCommand.substring(1).toFloat();
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

  // Watchdog: if the host stops talking (crash, unplugged cable), stop sinking current.
  // The host must resend its "A<amps>" command at least every COMMAND_TIMEOUT_MS.
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    current_requested = 0.0;
  }

  set_current_request(current_requested); // Use this to control the voltage of the DAC for mosfet control.

  if (should_shut_down()) {
    shut_down();
  }
}

void read_robot_battery() {
  robot_battery_i = ads_battery_current.getResult_V() * ROBOT_BATTERY_I_SCALING_FACTOR;
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

  float resistance = THERMISTOR_R_FIXED * ((THERMISTOR_V_IN / voltage) - 1.0);
  float steinhart = 1.0 / ((1.0 / THERMISTOR_T_NOMINAL) + (1.0 / THERMISTOR_BETA) * log(resistance / THERMISTOR_R_NOMINAL));
  return steinhart - 273.15;
}

void log_data() {
  Serial.print("RBT_BATT_V:");
  Serial.print(robot_battery_v);
  Serial.print(",RBT_BATT_I:");
  Serial.print(robot_battery_i);
  Serial.print(",FET_TOTAL_I:");
  Serial.print(mosfet_current_total);
  Serial.print(",FET_AVG_T:");
  Serial.print(mosfet_avg_temp);

  for (int i = 0; i < MOSFET_COUNT; i++) {
    Serial.print(",FET_I_");
    Serial.print(i);
    Serial.print(":");
    Serial.print(mosfet_currents[i]);
    Serial.print(",FET_T_");
    Serial.print(i);
    Serial.print(":");
    Serial.print(mosfet_temperatures[i]);
  }
  Serial.println();
}

bool should_shut_down() {
  if (robot_battery_i >= (MOSFET_COUNT * 11.0)) {
    return true;
  }
  if (robot_battery_v < 9.0) {
    return true;
  }

  for (byte i = 0; i < MOSFET_COUNT; i++) {
    if (mosfet_currents[i] > 11.0) {
      return true;
    }
    if (mosfet_temperatures[i] > 60.0) {
      return true;
    }
    if (mosfet_temperatures[i] == TEMPERATURE_FAULT) {
      return true; // A thermistor we can't read means we can't protect that MOSFET
    }
  }
  return false;
}

void shut_down() {
  // Remove the load first: engage the hardware kill switch and zero the DAC, then open the contactor
  // once the current has dropped so the LEV200 is not breaking full load, then latch.
  current_requested = 0.0;
  digitalWrite(E_STOP_PIN, HIGH);
  dac.setVoltage(0, false);
  delay(CONTACTOR_OPEN_DELAY_MS);
  digitalWrite(CONTACTOR_PIN, LOW);

  while (true) {
    Serial.println("ERROR");
    delay(1000);
  }
}

void set_current_request(float current) {
  // Every op-amp holds its shunt at the DAC Control voltage, so total current = V * (sum of 1/R).
  float controlVoltage = current / total_system_conductance;
  float targetVoltage = controlVoltage / DAC_DIVIDER_RATIO; // Undo the 4.7k / 120 ohm divider

  float rawCounts = (targetVoltage / DAC_V_IN) * (float) DAC_COUNT;
  int dacOutput = round(rawCounts);

  if (dacOutput < 0) {
    dac.setVoltage(0, false);
  } else if (dacOutput > DAC_COUNT) {
    dac.setVoltage(DAC_COUNT, false);
  } else {
    dac.setVoltage(dacOutput, false);
  }
}
