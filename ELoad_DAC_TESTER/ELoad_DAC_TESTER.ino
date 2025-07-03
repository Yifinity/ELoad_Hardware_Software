#include <Adafruit_MCP4728.h>
#include <ctype.h>
#include <Wire.h>

Adafruit_MCP4728 dac;
float channel_settings[3];

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
}

char channel_input;

void loop() {
  while(Serial.available() == 0) {
    Serial.println("-------------------------------");
    Serial.print("VALUE | "); Serial.print("CHANNEL 1 | ");   Serial.print("CHANNEL 2 | ");  Serial.println("CHANNEL 3");
    Serial.print("Loads |   "); Serial.print(channel_settings[0]); Serial.print("       "); Serial.print(channel_settings[1]); Serial.print("       "); Serial.println(channel_settings[2]); 
    Serial.println("Enter Target Channel (A-C): ");
    Serial.println("-------------------------------");
    delay(100);
  } // Wait for response: 
  channel_input = Serial.readStringUntil('\n')[0];
  channel_input = toupper(channel_input);
  // channel_input.trim(); 
  // channel_input.toUpperCase(); 
  Serial.print("Target "); Serial.println(channel_input); 
  if(channel_input < 'A' || channel_input > 'C'){
    Serial.println("Invalid Channel");
    return; 
  }

  float target_voltage = -1.0;  

  // float voltage_input; 
  do{
    Serial.println("Enter Target Voltage (0-5V) for Channel " + String(channel_input) + ": ");
    while(Serial.available() == 0) {} // Wait for response: 
    target_voltage = Serial.parseFloat();

    // Clear everything up to the next '/n' - nessisary to ensure no new line error. 
    while(Serial.available() > 0) {Serial.read();}

  } while (target_voltage < 0 || target_voltage > 5);

  int output_bits = target_voltage * (4096 / 5); 
  int target_channel;
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

  Serial.print("Setting Channel " + String(target_channel)  + " to target voltage " + String(target_voltage) + " or "); Serial.print(output_bits); Serial.println(" bits"); 
  if(dac.setChannelValue(target_channel, output_bits)){
    // channel_settings[target_channel] = target_voltage; 
    Serial.println("Command Successful");
  }else{
    channel_settings[target_channel] = target_voltage;  // Comment OUT
    Serial.println("Command Failed"); 
  }
}