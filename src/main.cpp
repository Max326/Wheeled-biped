#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#define CSN_PIN PB12

// Dedicated SPI2 bus
SPIClass SPI_2(PB15, PB14, PB13);

// --- 1. OUR PERFECT BARE-METAL READ FUNCTION ---
float readMyAS5047P() {
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  
  digitalWrite(CSN_PIN, LOW);
  // Unbroken 16-bit transfer fixes the STM32 byte-split bug
  uint16_t raw_data = SPI_2.transfer16(0xFFFF); 
  digitalWrite(CSN_PIN, HIGH);
  
  SPI_2.endTransaction();
  
  // Mask out the parity and error bits to get the raw 14-bit angle
  uint16_t angle_raw = raw_data & 0x3FFF;
  
  // Convert 0-16383 to 0-6.283 radians for SimpleFOC
  return ((float)angle_raw / 16384.0f) * _2PI;
}

// --- 2. OUR CUSTOM INIT FUNCTION ---
void initMyAS5047P() {
  pinMode(CSN_PIN, OUTPUT);
  digitalWrite(CSN_PIN, HIGH);
  SPI_2.begin();
}

// --- 3. TELL SIMPLEFOC TO USE OUR FUNCTIONS ---
GenericSensor sensor = GenericSensor(readMyAS5047P, initMyAS5047P);

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("Initializing Custom SPI2 Sensor...");
  
  // SimpleFOC will now call our initMyAS5047P() automatically
  sensor.init();

  Serial.println("Custom AS5047P Ready! Open Teleplot.");
}

void loop() {
  // Fetch the latest angle using our perfect readMyAS5047P() function
  sensor.update();

  Serial.print(">Angle_Rad:");
  Serial.println(sensor.getAngle());
  
  delay(10);
}