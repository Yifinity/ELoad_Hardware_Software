#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_MCP4728.h>

// ADC Variable
float curr_sens[3] = {1, 2, 3};
float temp_sens[3] = {4, 5, 6};
float volt_sens[3] = {7, 8, 9};
float temperature_readings[3] = {10, 11, 12};
float channel_settings[3];

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(temperature_readings[0]); Serial.print(","); Serial.print(temperature_readings[1]); Serial.print(","); Serial.print(temperature_readings[2]); Serial.print(",");
  Serial.print(curr_sens[0]); Serial.print(","); Serial.print(curr_sens[1]); Serial.print(","); Serial.print(curr_sens[2]); Serial.print(",");
  Serial.print(volt_sens[0]); Serial.print(","); Serial.print(volt_sens[1]); Serial.print(","); Serial.print(volt_sens[2]); Serial.print(",");
  Serial.print(temp_sens[0]); Serial.print(","); Serial.print(temp_sens[1]); Serial.print(","); Serial.println(temp_sens[2]); 

  if(Serial.available()){
    String input_message = Serial.readStringUntil('>');
    // Remove whitespace
    input_message.trim();

    // Split by commas
    /* 
    0: Target Load, 
    1: Test Type,
    2: Target
    3: Startpoint:
    4: Current Increment
    5: Seconds Per Step
    6: Voltage Cutoff
    */

    float values[7] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN};
    int index = 0;


    int lastIndex = 0;
    while (index < 7) {
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
    int target_channel = (int)values[0]; 
    int test_type = (int)values[1]; // 0: Constant Load, 1: Constant Current, 2: Profile
    // Split by commas
    /* 
    0: Target Load, 
    1: Test Type,
    2: Target
    3: Startpoint:
    4: Current Increment
    5: Seconds Per Step
    6: Voltage Cutoff
    */
    float target = values[2] / 10;    
    float start_current = values[3]; 
    float curr_increment = values[4];
    float secs_per_step = values[5];
    float volt_cutoff = values[6];

    if (!isnan(target_channel) && !isnan(target) && !isnan(test_type)) {
      if(test_type == 0){ // Constant Load:
        Serial.print("Channel "); Serial.print(target_channel); Serial.print(" has constant load of ");
        Serial.print(target); Serial.println("A"); 
        bool success = false; 
        uint16_t output_bits = (uint16_t)((target / 5.0) * 4095.0);
        if (success) {
          Serial.println("Command Successful");
        } else {
          Serial.println("Command Failed");
        }
      }else if(test_type == 1){
        Serial.print("Channel"); Serial.print(target_channel); Serial.print(" has CONSTANT CURRENT OF ");
        Serial.println(target);
        Serial.print("Channel"); Serial.print(target_channel); Serial.print(" has CONSTANT CURRENT OF ");
        Serial.println(target);
        Serial.print("Channel"); Serial.print(target_channel); Serial.print(" has CONSTANT CURRENT OF ");
        Serial.println(target);
      }else if(test_type == 2){
        Serial.print("Channel"); Serial.print(target_channel); Serial.print(" has PROFILE WITH START:");
        Serial.println(start_current); Serial.print(" | END:"); Serial.print(target); Serial.print(" | Inc: ");
        Serial.print(curr_increment); Serial.print(" | At steps/sec: "); Serial.print(secs_per_step); Serial.print(" | Voltage Cutoff"); Serial.println(volt_cutoff);
      }
    }
  }
  delay(100);
}