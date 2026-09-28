#include <Arduino.h>
#include <SimpleFOC.h>

// 1. Tell SimpleFOC to use the AS5047/AS5147 profile and set CS to PB12
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, PB12);

// 2. Create a dedicated SPI2 bus (MOSI, MISO, SCK)
SPIClass SPI_2(PB15, PB14, PB13);

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("Initializing SPI2...");
  
  // 3. Pass our custom SPI2 bus to the sensor
  sensor.init(&SPI_2);

  Serial.println("AS5047P Ready! Open Teleplot.");
}

void loop() {
  // Fetch the latest angle over SPI
  sensor.update();

  // Print formatted for Teleplot
  Serial.print(">Angle_Rad:");
  Serial.println(sensor.getAngle());
  
  delay(10); // 100Hz plot update
}