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
#define GYRO_OUT_Z  0x2C

// --- ENCODER PINS & SPI (SWAPPED AS TESTED) ---
#define CSN_1 PB10
#define CSN_2 PB12
SPIClass SPI_2(PB15, PB14, PB13);

// --- MOTOR SETTINGS ---
BLDCMotor motor1 = BLDCMotor(7); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA1, PA2, PA3, PE7);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PD12, PD13, PD14, PD15);

// --- INNER LOOP PID ONLY (STABILIZATION) ---
// P, I, D, ramp, limit (max voltage)
// Reduced P and D for hand-testing
PIDController pid_stb(10.0, 0.0, 0.5, 100000.0, 8.0);
LowPassFilter lpf_pitch(0.01);

// --- IMU VARIABLES ---
float roll_angle = 0.0;
float gyro_bias = 0.0;
unsigned long last_imu_time = 0;

void initEncoders() { 
  pinMode(CSN_1, OUTPUT); digitalWrite(CSN_1, HIGH);
  pinMode(CSN_2, OUTPUT); digitalWrite(CSN_2, HIGH);
  SPI_2.begin(); 
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
}

float readEnc1() {
  // Ensure the other CS is HIGH and wait 2us for MISO line to release (Hi-Z)
  digitalWrite(CSN_2, HIGH);
  // delayMicroseconds(2);
  
  digitalWrite(CSN_1, LOW); 
  uint16_t raw = SPI_2.transfer16(0xFFFF); 
  digitalWrite(CSN_1, HIGH);
  return ((raw & 0x3FFF) / 16384.0f) * _2PI;
}

float readEnc2() {
  // Ensure the other CS is HIGH and wait 2us for MISO line to release (Hi-Z)
  digitalWrite(CSN_1, HIGH);
  // delayMicroseconds(2);

  digitalWrite(CSN_2, LOW); 
  uint16_t raw = SPI_2.transfer16(0xFFFF); 
  digitalWrite(CSN_2, HIGH);
  return ((raw & 0x3FFF) / 16384.0f) * _2PI;
}

GenericSensor sensor1(readEnc1, initEncoders);
GenericSensor sensor2(readEnc2, initEncoders);

void setup() {
  Serial.begin(115200);
  delay(2000); 
  
  // Enable SimpleFOC debug output to see exact calibration results for both motors
  SimpleFOCDebug::enable(&Serial);

  sensor1.init(); 
  sensor2.init();

  Serial.println("Configuring IMU...");
  Wire.setSDA(PB9); Wire.setSCL(PB6);
  Wire.setClock(400000); Wire.begin();
  
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A); Wire.write(0x57); Wire.endTransmission();

  pinMode(GYRO_CS_PIN, OUTPUT); digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.setSCLK(PA5); SPI.setMISO(PA6); SPI.setMOSI(PA7);
  SPI.begin();
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GYRO_CS_PIN, LOW);
  SPI.transfer(GYRO_CTRL1); SPI.transfer(0x0F); 
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.println("Calibrating Gyroscope... DO NOT MOVE ROBOT");
  long gyro_sum = 0;
  for (int i = 0; i < 500; i++) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_Z | 0xC0);
    SPI.transfer(0x00); SPI.transfer(0x00);
    gyro_sum += (int16_t)(SPI.transfer(0x00) | (SPI.transfer(0x00) << 8));
    SPI.transfer(0x00); SPI.transfer(0x00);
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();
    delay(2);
  }
  gyro_bias = (float)gyro_sum / 500.0;
  
  Serial.println("Configuring FOC...");
  driver1.voltage_power_supply = 12.0; driver1.init();
  motor1.linkDriver(&driver1); motor1.linkSensor(&sensor1);

  driver2.voltage_power_supply = 12.0; driver2.init();
  motor2.linkDriver(&driver2); motor2.linkSensor(&sensor2);
  
  motor1.controller = MotionControlType::torque; 
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.voltage_sensor_align = 2.0;
  motor1.voltage_limit = 6.0;

  motor2.controller = MotionControlType::torque; 
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.voltage_sensor_align = 2.0;
  motor2.voltage_limit = 6.0;
  
  Serial.println("--- CALIBRATING MOTOR 1 ---");
  motor1.init(); 
  motor1.initFOC();

  Serial.println("--- CALIBRATING MOTOR 2 ---");
  motor2.init(); 
  motor2.initFOC();

  last_imu_time = micros();
  Serial.println("GO!");
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  unsigned long current_time = micros();
  if (current_time - last_imu_time >= 4000) { 
    float dt = (current_time - last_imu_time) / 1000000.0;
    last_imu_time = current_time;

    // --- IMU READ ---
    Wire.beginTransmission(LSM303_ADDR);
    Wire.write(OUT_X_L_A | 0x80); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM303_ADDR, (uint8_t)6);
    int16_t ay=0, az=0;
    if (Wire.available() == 6) {
      Wire.read(); Wire.read(); 
      ay = Wire.read() | (Wire.read() << 8);
      az = Wire.read() | (Wire.read() << 8);
    }

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_Z | 0xC0); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    int16_t gy = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();

    // --- IMU MATH ---
    float accel_roll = atan2(-ay, -az); 
    float gyro_rate = (gy - gyro_bias) * 0.00875 * _PI / 180.0; 
    
    // Complementary filter to combine accelerometer and gyroscope data
    roll_angle = 0.99 * (roll_angle + gyro_rate * dt) + 0.01 * accel_roll;
    float filtered_pitch = lpf_pitch(roll_angle);

    // --- BALANCE CONTROL ---
    // target is always 0 (upright)
    float target_pitch = 0.0;
    float control_voltage = -pid_stb(target_pitch - filtered_pitch);

    // Safety cutout (45 degrees)
    if (abs(filtered_pitch) > 0.8) {
      control_voltage = 0;
      pid_stb.reset();
    }

    motor1.move(-control_voltage); 
    motor2.move(control_voltage); 

    // --- DIAGNOSTICS ---
    // static unsigned long last_print = 0;
    // if (millis() - last_print > 50) { 
    //   last_print = millis();
    //   Serial.print(">Pitch:"); Serial.println(filtered_pitch);
    //   Serial.print(">Voltage:"); Serial.println(control_voltage);

    //   Serial.print(">Angle 1:"); Serial.println(sensor1.getAngle());
    //   Serial.print(">Angle 2:"); Serial.println(sensor2.getAngle());
    // }

    // Serial.print(">Pitch:"); Serial.println(filtered_pitch);
    // Serial.print(">Voltage:"); Serial.println(control_voltage);

    // Serial.print(">Angle 1:"); Serial.println(sensor1.getAngle());
    // Serial.print(">Angle 2:"); Serial.println(sensor2.getAngle());
  }
}