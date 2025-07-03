#include <Adafruit_MAX31856.h>

Adafruit_MAX31856 thermo_input1 = Adafruit_MAX31856(5);
Adafruit_MAX31856 thermo_input2 = Adafruit_MAX31856(6);
Adafruit_MAX31856 thermo_input3 = Adafruit_MAX31856(7);
float temperature_readings[3] = {0,0,0};


void setup() {
  Serial.begin(115200);
  Serial.println("MAX31856 thermocouple test");

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
  // temperature_readings[0] = thermo_input1.readThermocoupleTemperature(); 
  // temperature_readings[1] = thermo_input1.readThermocoupleTemperature(); 
  // temperature_readings[2] = thermo_input1.readThermocoupleTemperature(); 
  
  temperature_readings[0] = 0; 
  temperature_readings[1] = 100; 
  temperature_readings[2] = 200; 
  Serial.print("TC(C) | "); Serial.print(temperature_readings[0]); Serial.print("       "); Serial.print(temperature_readings[1]); Serial.print("       "); Serial.println(temperature_readings[2]); 
 

}