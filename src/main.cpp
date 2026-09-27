#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Accelerometer (I2C)
#define LSM303_ADDR 0x19
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28

// Gyroscope (SPI)
#define GYRO_CS_PIN PE3
#define GYRO_CTRL1  0x20
#define GYRO_OUT_X  0x28

// Discovery Board Green LED
#define LED_PIN PD12 

float pitch_angle = 0.0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // WAIT FOR USB
  uint32_t t = millis();
  while (!Serial && (millis() - t < 3000)) { 
    delay(10); 
  }

  // --- I2C Setup for Accelerometer (PB9, PB6) ---
  Wire.setSDA(PB9);
  Wire.setSCL(PB6);
  Wire.setClock(100000); 
  Wire.begin();
  
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A);
  Wire.write(0x57); // 100Hz, all axes enabled
  Wire.endTransmission();

  // --- SPI Setup for Gyroscope (PA5, PA6, PA7, PE3) ---
  pinMode(GYRO_CS_PIN, OUTPUT);
  digitalWrite(GYRO_CS_PIN, HIGH);
  
  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GYRO_CS_PIN, LOW);
  SPI.transfer(GYRO_CTRL1); 
  SPI.transfer(0x0F); // Normal mode, all axes enabled
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.endTransaction();
  
  Serial.println("IMU Ready! Open Teleplot.");
  last_time = micros();
}

void loop() {
  // Heartbeat LED
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  // --- 1. Read Accelerometer ---
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(OUT_X_L_A | 0x80); 
  Wire.endTransmission(false);
  Wire.requestFrom(LSM303_ADDR, 6);
  
  int16_t ax=0, ay=0, az=0;
  if (Wire.available() == 6) {
    ax = (Wire.read() | (Wire.read() << 8)) >> 4;
    ay = (Wire.read() | (Wire.read() << 8)) >> 4;
    az = (Wire.read() | (Wire.read() << 8)) >> 4;
  }

  // --- 2. Read Gyroscope ---
  int16_t gx=0, gy=0, gz=0;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GYRO_CS_PIN, LOW);
  SPI.transfer(GYRO_OUT_X | 0x80 | 0x40); // Read bit + Auto-increment
  gx = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  gy = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  gz = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.endTransaction();

  // --- 3. Complementary Filter Math ---
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // Calculate absolute pitch from Accel. 
  // (We use float casts to prevent integer overflow when squaring)
  float accel_pitch = atan2(-ax, sqrt((float)ay * (float)ay + (float)az * (float)az)) * 180.0 / PI;

  // Convert raw gyro Y to degrees/sec (0.00875 dps/LSB for default 250dps range)
  float gyro_rate = gy * 0.00875;

  // Fuse the data: 98% Gyro (fast response) + 2% Accel (gravity anchor)
  pitch_angle = 0.98 * (pitch_angle + gyro_rate * dt) + 0.02 * accel_pitch;

  // --- 4. Output to Teleplot ---
  Serial.print(">Raw_Accel_Pitch:"); Serial.println(accel_pitch);
  Serial.print(">Filtered_Pitch:"); Serial.println(pitch_angle);
  
  delay(10);
}