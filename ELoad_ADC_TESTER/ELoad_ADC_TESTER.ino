#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADC Variable
Adafruit_ADS1115 ads1115;
bool ads_initialized = false;
float curr_sens[3]; // Read from ADS
float temp_sens[3];
float volt_sens[3];

void setup() {
  Serial.begin(115200);
  /* Initialize ADC */
  if(!ads1115.begin()){ // Initialize with default address (0x48)
    Serial.println("Failure to Initialize ADC");
  }else{
    ads_initialized = true;
    Serial.println("ADS Initialized");
  }
}

void loop() {
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
  volt_sens[0] = analogRead(7) * (5 / 1023.0);
  volt_sens[1] = analogRead(6) * (5 / 1023.0);
  volt_sens[2] = analogRead(3) * (5 / 1023.0);
  

  // Present Results:
  Serial.print("VALUE | "); Serial.print("CHANNEL 1 | ");   Serial.print("CHANNEL 2 | ");  Serial.println("CHANNEL 3");
  Serial.print("VOLTS |   "); Serial.print(curr_sens[0]); Serial.print("       "); Serial.print(curr_sens[1]); Serial.print("       "); Serial.println(curr_sens[2]); 
  Serial.print("CURR  |   "); Serial.print(volt_sens[0]); Serial.print("       "); Serial.print(volt_sens[1]); Serial.print("       "); Serial.println(volt_sens[2]);
  Serial.print("TEMP  |   "); Serial.print(temp_sens[0]); Serial.print("       "); Serial.print(temp_sens[1]); Serial.print("       "); Serial.println(temp_sens[2]); 
  delay(100);
}
