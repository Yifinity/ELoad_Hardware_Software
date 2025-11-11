#include <Wire.h>
#include <Adafruit_MCP4728.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4728.h>
#include <ctype.h>

// Analog Readings
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

// PID Constants
// float current_error; 
float kP = 1; 
float deadband = 0.001; //  used to see if we've hit 
float last_time = 0; 
float target_current; 
int target_channel = -1; 
float start_time; 

void setup(void) {
  Serial.begin(115200);
  Serial.println("Digital-to-Analog Tester");

  if (!dac.begin()) {
    Serial.println("DAC initialization failure");
    // while (1) {}
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

char channel_input;
int output_bits; 

void loop() {
  while(Serial.available() == 0) {
        // Current Readings
    for(int curr_idx = 0; curr_idx < 3; curr_idx++){
      if(ads_initialized){
        curr_sens[curr_idx] = ads1115.computeVolts(ads1115.readADC_SingleEnded(curr_idx)); // 6.144 volts with precision = 3mV. 
      } else { // ADS loss of connectivity
        curr_sens[curr_idx] = -1;
      }
    }

    // Mosfet Temperature Readings
    for(int temp_idx = 0; temp_idx < 3; temp_idx++){
      temp_sens[temp_idx] = analogRead(2 - temp_idx) * (5 / 1023.0);
    }

    // Voltage Readings
    volt_sens[0] = analogRead(7) * (5 / 1023.0) * 4; // Account for voltage divider
    volt_sens[1] = analogRead(6) * (5 / 1023.0) * 4;
    volt_sens[2] = analogRead(3) * (5 / 1023.0) * 4;
    
    // Load Temperature Sense
    temperature_readings[0] = 0; 
    temperature_readings[1] = 100; 
    temperature_readings[2] = 200; 
  

    // Serial.println("--------------------------------------------------------------");
    // Serial.print("VALUE   | "); Serial.print("CHANNEL 1 | ");   Serial.print("CHANNEL 2 | ");  Serial.println("CHANNEL 3");
    // Serial.print("VOLT(V) |   "); Serial.print(volt_sens[0]); Serial.print("       "); Serial.print(volt_sens[1]); Serial.print("       "); Serial.println(volt_sens[2]); 
    Serial.println("CURR(V) |   "); Serial.print(curr_sens[0]);
    // Serial.print("TEMP(V) |   "); Serial.print(temp_sens[0]); Serial.print("       "); Serial.print(temp_sens[1]); Serial.print("       "); Serial.println(temp_sens[2]); 
    // Serial.print("THERM(C)|   "); Serial.print(temperature_readings[0]); Serial.print("      "); Serial.print(temperature_readings[1]); Serial.print("       "); Serial.println(temperature_readings[2]); 
    // Serial.print("LOAD(V) |   "); Serial.print(channel_settings[0]); Serial.print("       "); Serial.print(channel_settings[1]); Serial.print("       "); Serial.println(channel_settings[2]); 
    // Serial.println("Enter Target Channel (A-C): ");
    
    
    // Closed Loop
  if(target_channel != -1){
      if(last_time == 0){
      output_bits = target_current / 10 * (4096 / 5); // Starting point. 
    }

    last_time = millis(); 

    float present_current = curr_sens[target_channel] * 10; 
    float error = target_current - present_current; 
    int additional_output = error * kP  / 10 * (4096 / 5); 

    if(abs(error) >= deadband){
    // serial
      output_bits += additional_output; 
    }

    if(output_bits > 4096 || output_bits < 0){
      Serial.println("Bounds Abort"); 
      output_bits = 0; 
    }

    Serial.print("Present Current: ");
    Serial.print(present_current); 
    Serial.print(" ERROR: ");
    Serial.print(error);
    Serial.print("Additional:"); 
    Serial.print(additional_output); 
    Serial.print(" OUTPUT: ");
    Serial.println(output_bits); 

  }
  

    
    
    delay(200);
  } // Wait for response: 

  channel_input = Serial.readStringUntil('\n')[0];
  channel_input = toupper(channel_input);
  Serial.print("Target "); Serial.println(channel_input); 
  if(channel_input < 'A' || channel_input > 'C'){
    Serial.println("Invalid Channel");
    return; 
  }

  target_current = -1.0;  

  // float voltage_input; 
  do{
    Serial.println("Enter Target Voltage (0-0.5V) for Channel " + String(channel_input) + ": ");
    while(Serial.available() == 0) {} // Wait for response: 
    target_current = Serial.parseFloat();

    // Clear everything up to the next '/n' - nessisary to ensure no new line error. 
    while(Serial.available() > 0) {Serial.read();}

  } while (target_current < 0 || target_current > 5);


  switch(channel_input){
    case 'A':
      target_channel = 0; 
      break; 
    case 'B':
      target_channel = 1; 
      break;
    case 'C':
      target_channel = 2; 
      break;
    default:
      Serial.println("Channel Translation Error"); 
      return;
  }

  if (target_current == 0){
      Serial.println("Resetting OUR LAST TIME AND OUTPUT_BITs"); 
      output_bits = 0;
      last_time = 0;
    }


  // Serial.print("Setting Channel " + String(target_channel)  + " to target voltage " + String(target_current) + " or "); Serial.print(output_bits); Serial.println(" bits"); 

  int fixed_bits = target_current / 10 * (4096 / 5); // Used right now for comparison 
  start_time = millis();  // Set our start time to be now. --> used for comparision. 
  if(dac.setChannelValue(target_channel, fixed_bits)){
    // channel_settings[target_channel] = target_current; 
    Serial.println("Command Successful");
  }else{
    channel_settings[target_channel] = target_current;  // Comment OUT
    Serial.println("Command Failed"); 
  }
}
