#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#define CSN_1 PB12
#define CSN_2 PB10

SPIClass SPI_2(PB15, PB14, PB13);

// --- ENCODER 1 (PB12) ---
float readEnc1() {
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(CSN_1, LOW);
  uint16_t raw_data = SPI_2.transfer16(0xFFFF); 
  digitalWrite(CSN_1, HIGH);
  SPI_2.endTransaction();
  return ((float)(raw_data & 0x3FFF) / 16384.0f) * _2PI;
}

// --- ENCODER 2 (PB10) ---
float readEnc2() {
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(CSN_2, LOW);
  uint16_t raw_data = SPI_2.transfer16(0xFFFF); 
  digitalWrite(CSN_2, HIGH);
  SPI_2.endTransaction();
  return ((float)(raw_data & 0x3FFF) / 16384.0f) * _2PI;
}

// --- INIT FUNCTION ---
void initEncoders() {
  pinMode(CSN_1, OUTPUT);
  pinMode(CSN_2, OUTPUT);
  digitalWrite(CSN_1, HIGH);
  digitalWrite(CSN_2, HIGH);
  SPI_2.begin();
}

// --- CREATE SIMPLEFOC SENSORS ---
GenericSensor sensor1 = GenericSensor(readEnc1, initEncoders);
GenericSensor sensor2 = GenericSensor(readEnc2, initEncoders); // Reuses the same init

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("Initializing Dual Custom SPI2 Sensors...");
  
  sensor1.init();
  sensor2.init();

  Serial.println("Dual AS5047P Ready! Open Teleplot.");
}

void loop() {
  sensor1.update();
  sensor2.update();

  Serial.print(">Enc1_Rad:");
  Serial.println(sensor1.getAngle());
  
  Serial.print(">Enc2_Rad:");
  Serial.println(sensor2.getAngle());
  
  delay(10);
}