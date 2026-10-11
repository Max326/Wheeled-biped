#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <SPI.h>

// --- IMU REGISTERS ---
#define LSM303_ADDR 0x19
#define CTRL_REG1_A 0x20
#define OUT_X_L_A   0x28
#define GYRO_CS_PIN PE3
#define GYRO_CTRL1  0x20
#define GYRO_OUT_X  0x28

// --- COMPRESSED DUAL ENCODER LOGIC (SPI2) ---
#define CSN_1 PB12
#define CSN_2 PB10
SPIClass SPI_2(PB15, PB14, PB13);

float readEnc1() {
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(CSN_1, LOW);
  uint16_t raw = SPI_2.transfer16(0xFFFF);
  digitalWrite(CSN_1, HIGH);
  SPI_2.endTransaction();
  return ((raw & 0x3FFF) / 16384.0f) * _2PI;
}

float readEnc2() {
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(CSN_2, LOW);
  uint16_t raw = SPI_2.transfer16(0xFFFF);
  digitalWrite(CSN_2, HIGH);
  SPI_2.endTransaction();
  return ((raw & 0x3FFF) / 16384.0f) * _2PI;
}

void initEncoders() { 
  pinMode(CSN_1, OUTPUT); digitalWrite(CSN_1, HIGH);
  pinMode(CSN_2, OUTPUT); digitalWrite(CSN_2, HIGH);
  SPI_2.begin(); 
}

GenericSensor sensor1(readEnc1, initEncoders);
GenericSensor sensor2(readEnc2, initEncoders);

// --- IMU VARIABLES ---
float pitch_angle = 0.0;
unsigned long last_imu_time = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("1. Initializing Encoders (SPI2)...");
  sensor1.init();
  sensor2.init();

  Serial.println("2. Waking up Accelerometer (I2C)...");
  Wire.setSDA(PB9);
  Wire.setSCL(PB6);
  Wire.setClock(100000); 
  Wire.begin();
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A);
  Wire.write(0x57); 
  Wire.endTransmission();

  Serial.println("3. Configuring Gyroscope (SPI1)...");
  pinMode(GYRO_CS_PIN, OUTPUT);
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GYRO_CS_PIN, LOW);
  SPI.transfer(GYRO_CTRL1); 
  SPI.transfer(0x0F); 
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.endTransaction();
  
  Serial.println("All Sensors Ready! Open Teleplot.");
  last_imu_time = micros();
}

void loop() {
  unsigned long current_time = micros();
  
  if (current_time - last_imu_time >= 10000) { 
    float dt = (current_time - last_imu_time) / 1000000.0;
    last_imu_time = current_time;

    // --- ACCELEROMETER FIX (No bit-shifting!) ---
    Wire.beginTransmission(LSM303_ADDR);
    Wire.write(OUT_X_L_A | 0x80); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM303_ADDR, (uint8_t)6);
    int16_t ax=0, ay=0, az=0;
    if (Wire.available() == 6) {
      ax = Wire.read() | (Wire.read() << 8);
      ay = Wire.read() | (Wire.read() << 8);
      az = Wire.read() | (Wire.read() << 8);
    }

    // --- GYROSCOPE FIX ---
    int16_t gy=0;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_X | 0x80 | 0x40); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    gy = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();

    // --- PURE SENSOR CALCULATION ---
    // Try atan2(-ax, az) first. If it's wrong, change to atan2(-ay, az).
    float pure_accel_pitch = atan2(-ax, az) * 180.0 / PI; 
    float gyro_rate = -gy * 0.00875;
    
    // Pure integration for diagnostic plotting only
    static float pure_gyro_pitch = 0.0;
    pure_gyro_pitch += gyro_rate * dt;

    // --- THE GOLDEN COMPLEMENTARY FILTER ---
    pitch_angle = 0.98 * (pitch_angle + gyro_rate * dt) + 0.02 * pure_accel_pitch;

    // --- TELEPLOT DIAGNOSTICS ---
    Serial.print(">Accel_Pitch:");
    Serial.println(pure_accel_pitch);
    
    Serial.print(">Gyro_Only_Pitch:");
    Serial.println(pure_gyro_pitch);
    
    Serial.print(">FILTERED_PITCH:");
    Serial.println(pitch_angle);
  }
}