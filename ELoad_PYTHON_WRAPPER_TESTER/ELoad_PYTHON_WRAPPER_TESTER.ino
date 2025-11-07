#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_MCP4728.h>

// ADC Variable
Adafruit_ADS1115 ads1115;
bool ads_initialized = false;
float curr_sens[3]; // Read from ADS
float temp_sens[3];
float volt_sens[3];

// Thermocouple Readings
Adafruit_MAX31856 thermo_input1 = Adafruit_MAX31856(5);
Adafruit_MAX31856 thermo_input2 = Adafruit_MAX31856(6);
Adafruit_MAX31856 thermo_input3 = Adafruit_MAX31856(7);
float temperature_readings[3];

Adafruit_MCP4728 dac;
float channel_settings[3];

float external_reference = 4.096; // External reference voltage

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  analogReference(EXTERNAL); 

  if (!dac.begin()) {
  Serial.println("DAC initialization failure");
  while (1) {}
  }

  Serial.println("DAC Initialized");

  // Set each channel to 0V, with a gain of 1X and the Vref to be Vdd. 
  dac.setChannelValue(MCP4728_CHANNEL_A, 0);
  dac.setChannelValue(MCP4728_CHANNEL_B, 0);
  dac.setChannelValue(MCP4728_CHANNEL_C, 0);

  dac.saveToEEPROM(); // Set as default whenever DAC turns on

  /* Initialize ADC */
  if(!ads1115.begin()){ // Initialize with default address (0x48)
    Serial.println("Failure to Initialize ADC");
  }else{
    ads_initialized = true;
    Serial.println("ADS Initialized");
  }

    // Functionality needs to be tested.
  if (!thermo_input1.begin() || !thermo_input2.begin() || !thermo_input3.begin()) {
    Serial.println("Could not initialize thermocouples.");
    while (1) {};
  }

  thermo_input1.setThermocoupleType(MAX31856_TCTYPE_K);
  thermo_input2.setThermocoupleType(MAX31856_TCTYPE_K);
  thermo_input3.setThermocoupleType(MAX31856_TCTYPE_K);

  thermo_input1.setConversionMode(MAX31856_CONTINUOUS);
  thermo_input2.setConversionMode(MAX31856_CONTINUOUS);
  thermo_input3.setConversionMode(MAX31856_CONTINUOUS);
}

void loop() {
  // Current Readings
  for(int curr_idx = 0; curr_idx < 3; curr_idx++){
    if(ads_initialized){
      // Add multiplier
      curr_sens[curr_idx] = 10 * ads1115.computeVolts(ads1115.readADC_SingleEnded(curr_idx)); // 6.144 volts with precision = 3mV. 
    } else { // ADS loss of connectivity
      curr_sens[curr_idx] = -1;
    }
  }

  // Mosfet Temperature Readings
  for(int temp_idx = 0; temp_idx < 3; temp_idx++){
    temp_sens[temp_idx] = analogRead(2 - temp_idx) * (5 / 1023.0);
  }

  // Voltage Readings
  volt_sens[0] = analogRead(7) * (external_reference / 1023.0) * 5;
  volt_sens[1] = analogRead(6) * (external_reference / 1023.0) * 5;
  volt_sens[2] = analogRead(3) * (external_reference / 1023.0) * 5;
  

  // Present Results:
  // Serial.print("VALUE | "); Serial.print("CHANNEL 1 | ");   Serial.print("CHANNEL 2 | ");  Serial.println("CHANNEL 3");
  temperature_readings[0] = thermo_input1.readThermocoupleTemperature(); 
  temperature_readings[1] = thermo_input2.readThermocoupleTemperature(); 
  temperature_readings[2] = thermo_input3.readThermocoupleTemperature(); 
  // temperature_readings[2] = 50;

  Serial.print(temperature_readings[0]); Serial.print(","); Serial.print(temperature_readings[1]); Serial.print(","); Serial.print(temperature_readings[2]); Serial.print(",");
  Serial.print(curr_sens[0]); Serial.print(","); Serial.print(curr_sens[1]); Serial.print(","); Serial.print(curr_sens[2]); Serial.print(",");
  Serial.print(volt_sens[0]); Serial.print(","); Serial.print(volt_sens[1]); Serial.print(","); Serial.print(volt_sens[2]); Serial.print(",");
  Serial.print(temp_sens[0]); Serial.print(","); Serial.print(temp_sens[1]); Serial.print(","); Serial.println(temp_sens[2]); 

  // temperature_readings[0] = 

  String input_message;
  if(Serial.available()){
    input_message = Serial.readStringUntil(">");
    // Remove whitespace
    input_message.trim();

    // Split by commas
    float values[6] = {NAN, NAN, NAN, NAN, NAN, NAN};
    int index = 0;

    int lastIndex = 0;
    while (index < 6) {
      int commaIndex = input_message.indexOf(',', lastIndex);
      if (commaIndex == -1) commaIndex = input_message.length();

      String token = input_message.substring(lastIndex, commaIndex);
      token.trim();
      if (token.length() > 0) values[index] = token.toFloat();

      lastIndex = commaIndex + 1;
      index++;
      if (commaIndex >= input_message.length()) break;
    }

    // Interpret DAC command
    int target_channel = (int)values[0]; // First field: DAC channel
    float target_voltage = values[2] / 10;    // Third field: target voltage && account for 10X multiplier
    

    if (!isnan(target_channel) && !isnan(target_voltage)) {
      uint16_t output_bits = (uint16_t)((target_voltage / 5.0) * 4095.0);
      // Serial.print("Setting Channel ");
      // Serial.print(target_channel);
      // Serial.print(" to target voltage ");
      // Serial.print(target_voltage);
      // Serial.print("V or ");
      // Serial.print(output_bits);
      // Serial.println(" bits");

      bool success = false;
      Serial.println(target_channel); 
      switch (target_channel) {
        case 0: success = dac.setChannelValue(MCP4728_CHANNEL_A, output_bits); break;
        case 1: success = dac.setChannelValue(MCP4728_CHANNEL_B, output_bits); break;
        case 2: success = dac.setChannelValue(MCP4728_CHANNEL_C, output_bits); break;
      }

      if (success) {
        Serial.println("Command Successful");
      } else {
        Serial.println("Command Failed");
      }
    }

  }


  delay(100);
}
