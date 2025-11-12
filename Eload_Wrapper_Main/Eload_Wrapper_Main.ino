#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_MCP4728.h>
#include <math.h>

// ADC Variable
float curr_sens[3];
float temp_sens[3] = {4, 5, 6};
float volt_sens[3];
float temperature_readings[3] = {10, 11, 12};
float channel_settings[3];

// Internal ADC
float external_reference = 4.096; 

Adafruit_MCP4728 dac;
Adafruit_ADS1115 ads1115;
bool ads_initialized = false;
Adafruit_MAX31856 thermo_input1 = Adafruit_MAX31856(5);
Adafruit_MAX31856 thermo_input2 = Adafruit_MAX31856(6);
Adafruit_MAX31856 thermo_input3 = Adafruit_MAX31856(7);

// For each load, have the following:
/*
 * 0: Test type (0-2, -1 being no test )
 * 1: Test Target
 * 2: Test Start
 * 3: Current Increment
 * 4: Seconds per step
 * 5: Voltage Cutoff
 * 6: Last Timestamp
 * 7: output setting
 * 8: Current Target --> used for profiles
 * 9: PID Timestamp
 * 10: kp
 */
float load_test_info[3][11]; 
int pid_update_millis = 1000; //update every 500 seconds
float error_threshold = 0.001;


void setup() {
  Serial.begin(115200);
  load_test_info[0][10] = 75; // Load 0's kp value
  load_test_info[1][10] = 75; // Load 0's kp value
  load_test_info[2][10] = 75; // Load 0's kp value

  // analogReference(EXTERNAL); 

  /* Initialize ADC */
  if(!ads1115.begin()){ // Initialize with default address (0x48)
    Serial.println("Failure to Initialize ADC");
  }else{
    ads_initialized = true;
    Serial.println("ADS Initialized");
  }
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

void clear_test_info(int target_channel){
  load_test_info[target_channel][0] = -1; 
  for(int i = 1; i < 10; i++){
    load_test_info[target_channel][i] = 0;
  }
}

void loop() {
  // Voltage Readings
  volt_sens[0] = analogRead(7) * (external_reference / 1023.0) * 5;
  volt_sens[1] = analogRead(6) * (external_reference / 1023.0) * 5;
  volt_sens[2] = analogRead(3) * (external_reference / 1023.0) * 5;
  
  // Mosfet Temperature Readings
  for(int temp_idx = 0; temp_idx < 3; temp_idx++){
    temp_sens[temp_idx] = 100 * analogRead(2 - temp_idx) * (5 / 1023.0);
  }
  
  for(int curr_idx = 0; curr_idx < 3; curr_idx++){
    if(ads_initialized){
      // Add multiplier
      curr_sens[curr_idx] = 10 * ads1115.computeVolts(ads1115.readADC_SingleEnded(curr_idx)); // 6.144 volts with precision = 3mV. 
    } else { // ADS loss of connectivity
      curr_sens[curr_idx] = -1;
    }
  }

  // Present Results:
  // Serial.print("VALUE | "); Serial.print("CHANNEL 1 | ");   Serial.print("CHANNEL 2 | ");  Serial.println("CHANNEL 3");
  temperature_readings[0] = thermo_input1.readThermocoupleTemperature(); 
  temperature_readings[1] = thermo_input2.readThermocoupleTemperature(); 
  temperature_readings[2] = thermo_input3.readThermocoupleTemperature(); 


  Serial.print(temperature_readings[0]); Serial.print(","); Serial.print(temperature_readings[1]); Serial.print(","); Serial.print(temperature_readings[2]); Serial.print(",");
  Serial.print(curr_sens[0]); Serial.print(","); Serial.print(curr_sens[1]); Serial.print(","); Serial.print(curr_sens[2]); Serial.print(",");
  Serial.print(volt_sens[0]); Serial.print(","); Serial.print(volt_sens[1]); Serial.print(","); Serial.print(volt_sens[2]); Serial.print(",");
  Serial.print(temp_sens[0]); Serial.print(","); Serial.print(temp_sens[1]); Serial.print(","); Serial.println(temp_sens[2]); 

  // Update the states for each load. 
  for(int load = 0; load < 3; load++){
    if(load_test_info[load][0] > 0){ // If it's constant current or a profile adjust output accordingly
      if(volt_sens[load] <= load_test_info[load][5]){ // If it's under the cuttoff, abort. 
        switch (load) {
          case 0:  dac.setChannelValue(MCP4728_CHANNEL_A, 0); break;
          case 1:  dac.setChannelValue(MCP4728_CHANNEL_B, 0); break;
          case 2:  dac.setChannelValue(MCP4728_CHANNEL_C, 0); break;
        }
        clear_test_info(load); // Clear our setpoint
      }

      if(curr_sens[load] == -1){
        Serial.println("Current for load IS DISCONNECTED --> abort");
        continue; 
      }

      if(load_test_info[load][0] == 2){ // If it's a profile 
        float present_time = millis();  
        // Serial.print("Elaspsed time"); 
        // Serial.println(present_time - load_test_info[load][6]);
        if(present_time - load_test_info[load][6] >= (load_test_info[load][4] * 1000)) { // If we've passed our seconds per step
          // Serial.print("Time Reached --> updating to ");
          load_test_info[load][1] += load_test_info[load][3]; //increase starting load by current increment
          // Serial.println(load_test_info[load][1]);
          if(load_test_info[load][1] >= load_test_info[load][8]){ // if we are over the target, then stop
            // Serial.println("We are now over or at the target so we stop"); 
            load_test_info[load][1] = load_test_info[load][8]; // Should we end test or not?
          }
          load_test_info[load][6] = present_time; // Update our time
        }

      }

      if(millis() - load_test_info[load][9] > pid_update_millis){ // Time to update
        float present_current = curr_sens[load];
        // Serial.print("Current Read: ");
        // Serial.print(present_current); 
        // Serial.print(" | Current Target: ");
        // Serial.print(load_test_info[load][1]); 
        float current_error = load_test_info[load][1] - present_current;
        // Serial.print(" | Current ERROR: ");
        // Serial.print(current_error); 

        if(abs(current_error) < error_threshold){
          continue; // We're within a good threshold
        }
        int correction = round(current_error * load_test_info[load][10]); // P loop
        correction = constrain(correction, -400, 400); // max corretion
        int bit_output = load_test_info[load][7] + correction; 
        // Serial.print(" | Current Output: ");
        // Serial.print(load_test_info[load][7]); 
        bit_output = constrain(bit_output, 0, 4095);
        bool success = false;
        switch (load) {
          case 0: success = dac.setChannelValue(MCP4728_CHANNEL_A, bit_output); break;
          case 1: success = dac.setChannelValue(MCP4728_CHANNEL_B, bit_output); break;
          case 2: success = dac.setChannelValue(MCP4728_CHANNEL_C, bit_output); break;
        }
        // Serial.print(" | Load: "); Serial.print(load); Serial.print(" Has correction: "); Serial.print(correction); Serial.print(" | NEW OUTPUT: "); Serial.print(bit_output);
        if(success){
          // Serial.println("Closed Loop Feedback Adjustment"); 
          load_test_info[load][7] = bit_output; // update the output such that you can add/subtract onto it. 
          // Serial.print(" ||| NEW Index seven: "); Serial.println(load_test_info[load][7]);
          load_test_info[load][9] = millis(); //Update last update time
        }
      }
    }
  }

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
    float target = values[2];    
    float start_current = values[3]; 
    float curr_increment = values[4];
    float secs_per_step = values[5];
    float volt_cutoff = values[6];

    if (!isnan(target_channel) && !isnan(target) && !isnan(test_type)) {
      if(test_type == 0){ // Constant Load:
        // Serial.print("Channel "); Serial.print(target_channel); Serial.print(" has constant load of ");
        // Serial.print(target); Serial.println("A"); 
        bool success = false; 
        uint16_t output_bits = (uint16_t)((target / 50.0) * 4095.0);
    
        switch (target_channel) {
          case 0: success = dac.setChannelValue(MCP4728_CHANNEL_A, output_bits); break;
          case 1: success = dac.setChannelValue(MCP4728_CHANNEL_B, output_bits); break;
          case 2: success = dac.setChannelValue(MCP4728_CHANNEL_C, output_bits); break;
        }
        
        // May have to move after success
        clear_test_info(target_channel); 

        if (success) {
          // Serial.println("Command Successful");
          // load_test_info[target_channel] = {0,0,0,0,0,0,0,0};
          load_test_info[target_channel][0] = test_type; 
          load_test_info[target_channel][1] = target;  
          load_test_info[target_channel][7] = output_bits; 
        } else {
          Serial.println("Command Failed");
        }
      }else if(test_type == 1){
        // Closed loop code: 
        clear_test_info(target_channel);  // Clear
        load_test_info[target_channel][0] = test_type;
        load_test_info[target_channel][1] = target;
        load_test_info[target_channel][9] = millis();
        // Serial.print("CHANNEL: "); Serial.print(target_channel); Serial.print(" GETS: "); Serial.println(target); 
        if(volt_cutoff > 0){
          load_test_info[target_channel][5] = volt_cutoff; 
        }


        // Serial.print("Channel "); Serial.print(target_channel); Serial.print(" has CONSTANT CURRENT OF "); Serial.print(load_test_info[target_channel][1]);
        // Serial.print(" | Voltage Cutoff: "); Serial.println(volt_cutoff);
        
      }else if(test_type == 2){
        if(!isnan(start_current) && !isnan(curr_increment) && !isnan(secs_per_step) && !isnan(volt_cutoff)){
          if(curr_increment > target){
            // Serial.println("ERROR --> increment is too high"); 
            return; 
          }
          clear_test_info(target_channel);  // May need to push under
          load_test_info[target_channel][0] = test_type; 
          load_test_info[target_channel][1] = curr_increment + start_current; 
          load_test_info[target_channel][2] = start_current;     
          load_test_info[target_channel][3] = curr_increment;
          load_test_info[target_channel][4] = secs_per_step;
          load_test_info[target_channel][5] = volt_cutoff; 
          load_test_info[target_channel][6] = millis(); // Get the most recent timestamp
          load_test_info[target_channel][8] = target; //Starting should be the increment
          load_test_info[target_channel][9] = millis(); // Time for pid update
        }

        // Serial.print("Channel"); Serial.print(target_channel); Serial.print(" has PROFILE WITH START:");
        // Serial.println(start_current); Serial.print(" | END:"); Serial.print(target); Serial.print(" | Inc: ");
        // Serial.print(curr_increment); Serial.print(" | At steps/sec: "); Serial.print(secs_per_step); Serial.print(" | Voltage Cutoff"); Serial.println(volt_cutoff);
      }else{
        Serial.println("INVALID TEST");
      }
    }
  }
  delay(100);
}