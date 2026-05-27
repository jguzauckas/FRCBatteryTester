#include<ADS1115_WE.h>
#include<Wire.h>
#include<Adafruit_MCP4725.h>

#define DAC_I2C_ADDRESS 0x62 // Default address for Adafruit MCP4725
#define ADS_ROBOTBATTERY_I2C_ADDRESS 0x48 // Default address for ADS1115 (Ground)
#define ADS_MOSFETVOLTAGE_I2C_ADDRESS 0x49 // Modified Address for ADS1115 (5V)
#define ADS_MOSFETTEMPERATURE_I2C_ADDRESS 0x4A // Modified address for ADS1115 (SDA)

const int S0_PIN = 0;
const int S1_PIN = 1;
const int S2_PIN = 2;
const int S3_PIN = 3;
const int E_STOP_PIN = 4;

Adafruit_MCP4725 dac;
ADS1115_WE ads_robot_battery = ADS1115_WE(ADS_ROBOTBATTERY_I2C_ADDRESS);
ADS1115_WE ads_mosfet_current = ADS1115_WE(ADS_MOSFETVOLTAGE_I2C_ADDRESS);
ADS1115_WE ads_mosfet_temperature = ADS1115_WE(ADS_MOSFETTEMPERATURE_I2C_ADDRESS);

const float ROBOT_BATTERY_V_SCALING_FACTOR = (12000 + 4700) / 4700; // 12k top resistor (10k + 1k + 1k) and 4.7k bottom resistor
const float ROBOT_BATTERY_I_SCALING_FACTOR = 400 / 0.625; // 400 amps across 0.625 volt range
const float MOSFET_CURRENT_SCALING_FACTOR = 1.0 / 0.01; // voltage over 0.01 ohm shunt resistor

const float THERMISTOR_R_FIXED = 10000.0; // Fixed resistor for voltage divider
const float THERMISTOR_V_IN = 5.0; // Voltage going into Thermistor
const float THERMISTOR_R_NOMINAL = 10000.0; // Thermistors rated resistance at base temperature
const float THERMISTOR_T_NOMINAL = 25.0 + 273.15; // 25 degrees Celsius converted to Kelvin
const float THERMISTOR_BETA = 3988.0; // Rated Beta of Thermistor

const float SHUNT_RESISTOR = 0.01; // Shunt resistor is 0.01 ohms
const float DAC_V_IN = 5.0; // DAC voltage input is 5 volts
const int DAC_COUNT = 4095; // DAC resolution is 12 bits (2^12 -1)

float robot_battery_v = 12.0; // Starting voltage (high enough to prevent shutdown)
float robot_battery_i = 0.0; // Starting current (low enough to prevent shutdown)
float mosfet_current_total = 0.0; // Starting current of all mosfets
float mosfet_avg_temp = 0.0; // Starting temperature
float current_requested = 0.0; // Current requested at any given time

const int MOSFET_COUNT = 32;
float mosfet_currents[MOSFET_COUNT];
float mosfet_temperatures[MOSFET_COUNT];

void setup() {
  pinMode(E_STOP_PIN, OUTPUT);
  digitalWrite(E_STOP_PIN, HIGH);

  Serial.begin(9600);
  Serial.setTimeout(10);

  initialize_mosfet_arrays();

  dac.begin(DAC_I2C_ADDRESS); 
  
  ads_robot_battery.setVoltageRange_mV(ADS1115_RANGE_4096);
  ads_robot_battery.setMeasureMode(ADS1115_CONTINUOUS);

  ads_mosfet_current.setVoltageRange_mV(ADS1115_RANGE_0256);
  ads_mosfet_current.setMeasureMode(ADS1115_CONTINUOUS);

  ads_mosfet_temperature.setVoltageRange_mV(ADS1115_RANGE_4096);
  ads_mosfet_temperature.setMeasureMode(ADS1115_CONTINUOUS);

  delay(1000); // Give hardware time to stabilize

  digitalWrite(E_STOP_PIN, LOW);
}

void initialize_mosfet_arrays() {
  for (byte i = 0; i < MOSFET_COUNT; i++) {
    mosfet_currents[i] = 0.0;
    mosfet_temperatures[i] = 0.0;
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
      current_requested = incomingCommand.substring(1).toFloat();
    }
  }

  set_current_request(current_requested); // Use this to control the voltage of the DAC for mosfet control.

  if (should_shut_down()) {
    shut_down();
  }
}

void read_robot_battery() {
  ads_robot_battery.setCompareChannels(ADS1115_COMP_0_1);
  robot_battery_i = ads_robot_battery.getResult_V() * ROBOT_BATTERY_I_SCALING_FACTOR;
  
  ads_robot_battery.setCompareChannels(ADS1115_COMP_2_GND);
  robot_battery_v = ads_robot_battery.getResult_V() * ROBOT_BATTERY_V_SCALING_FACTOR;
}

void read_mosfets() {
  float total_current = 0.0;
  float total_temperature = 0.0;
  for (byte i = 0; i < MOSFET_COUNT; i++) {
    set_mosfet_mux_address(mosfet_index(i));
    
    if (i == 0) {
      ads_mosfet_current.setCompareChannels(ADS1115_COMP_0_GND);
      ads_mosfet_temperature.setCompareChannels(ADS1115_COMP_0_GND);
    } else if (i == 16) {
      ads_mosfet_current.setCompareChannels(ADS1115_COMP_1_GND);
      ads_mosfet_temperature.setCompareChannels(ADS1115_COMP_1_GND);
    }

    mosfet_currents[i] = ads_mosfet_current.getResult_V() * MOSFET_CURRENT_SCALING_FACTOR;
    total_current += mosfet_currents[i];

    mosfet_temperatures[i] = temperature(ads_mosfet_temperature.getResult_V());
    total_temperature += mosfet_temperatures[i];
  }
  mosfet_current_total = total_current;
  mosfet_avg_temp = total_temperature / (float) MOSFET_COUNT;
}

ADS1115_MUX mosfet_channel(int index) {
  if (index < 16) {
    return ADS1115_COMP_0_GND;
  }
  return ADS1115_COMP_1_GND;
}

int mosfet_index(int index) {
  if (index < 16) {
    return index;
  }
  return index - 16;
}

void set_mosfet_mux_address(int channel) {
  digitalWrite(S0_PIN, channel & 1);
  digitalWrite(S1_PIN, (channel >> 1) & 1);
  digitalWrite(S2_PIN, (channel >> 2) & 1);
  digitalWrite(S3_PIN, (channel >> 3) & 1);
}

float temperature(float voltage) {
  if (voltage <= 0) {
    return -999.0;
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
    if (mosfet_temperatures[i] > 85.0) {
      return true;
    }
  }
  return false;
}

void shut_down() {
  while (true) {
      Serial.println("ERROR");
      delay(1000);
    }
}

void set_current_request(float current) {
  float perChannelCurrent = current / (float) MOSFET_COUNT;
  float targetVoltage = perChannelCurrent * SHUNT_RESISTOR;

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