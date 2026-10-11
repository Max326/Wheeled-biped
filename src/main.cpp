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

// --- ENCODER PINS & SPI ---
#define CSN_1 PB12
#define CSN_2 PB10
SPIClass SPI_2(PB15, PB14, PB13);

// --- MOTOR SETTINGS ---
// Motor 1 (Right wheel)
BLDCMotor motor1 = BLDCMotor(7); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA1, PA2, PA3, PE7);

// Motor 2 (Left wheel) - Ensure you assign correct pins for your second FOC mini!
// Replace PB4, PB5, PB1, PE8 with your actual pins for motor 2.
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PB4, PB5, PB1, PE8);

// --- PID CONTROLLERS ---
// 1. Stabilization PID (Inner Loop)
// P, I, D, ramp (voltage change limit), limit (max voltage)
PIDController pid_stb(40.0, 100.0, 1.0, 100000.0, 8.0);

// 2. Velocity/Position PID (Outer Loop)
// Limit max lean angle to ~17 degrees (0.3 rad)
PIDController pid_vel(0.05, 0.0, 0.001, 10000.0, 0.3); 

// Low pass filters to smooth out derivative kicks
LowPassFilter lpf_pitch(0.01);
LowPassFilter lpf_velocity(0.01);

// --- IMU VARIABLES ---
float roll_angle = 0.0;
float gyro_bias = 0.0;
unsigned long last_imu_time = 0;
float target_position = 0.0;

// --- CUSTOM ENCODER FUNCTIONS ---
void initEncoders() { 
  pinMode(CSN_1, OUTPUT); digitalWrite(CSN_1, HIGH);
  pinMode(CSN_2, OUTPUT); digitalWrite(CSN_2, HIGH);
  SPI_2.begin(); 
  // Keep SPI open and fast
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
}

float readEnc1() {
  digitalWrite(CSN_1, LOW);
  uint16_t raw = SPI_2.transfer16(0xFFFF);
  digitalWrite(CSN_1, HIGH);
  return ((raw & 0x3FFF) / 16384.0f) * _2PI;
}

float readEnc2() {
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

  Serial.println("1. Initializing Encoders...");
  sensor1.init();
  sensor2.init();

  Serial.println("2. Configuring IMU...");
  Wire.setSDA(PB9);
  Wire.setSCL(PB6);
  Wire.setClock(400000); // Fast I2C for IMU
  Wire.begin();
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A);
  Wire.write(0x57); 
  Wire.endTransmission();

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

  Serial.println("Calibrating Gyroscope... KEEP ROBOT STILL");
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

  Serial.println("3. Configuring FOC Motors...");
  driver1.voltage_power_supply = 12.0; driver1.init();
  driver2.voltage_power_supply = 12.0; driver2.init();
  
  motor1.linkDriver(&driver1); motor1.linkSensor(&sensor1);
  motor2.linkDriver(&driver2); motor2.linkSensor(&sensor2);

  // Pure voltage control for both motors
  motor1.controller = MotionControlType::torque; 
  motor2.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;

  motor1.voltage_limit = 8.0; motor2.voltage_limit = 8.0;
  
  Serial.println("Aligning Motors...");
  motor1.init(); motor1.initFOC();
  motor2.init(); motor2.initFOC();
  
  last_imu_time = micros();
  Serial.println("GO!");
}

void loop() {
  // 1. Run core FOC math as fast as possible
  motor1.loopFOC();
  motor2.loopFOC();

  unsigned long current_time = micros();
  
  // 2. Control loop running at ~250 Hz (4ms)
  if (current_time - last_imu_time >= 4000) { 
    float dt = (current_time - last_imu_time) / 1000000.0;
    last_imu_time = current_time;

    // --- Read IMU Data ---
    Wire.beginTransmission(LSM303_ADDR);
    Wire.write(OUT_X_L_A | 0x80); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM303_ADDR, (uint8_t)6);
    int16_t ay=0, az=0;
    if (Wire.available() == 6) {
      Wire.read(); Wire.read(); // Skip X
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

    // --- Filter Math ---
    float accel_roll = atan2(-ay, -az); // Output in RADIANS for PID
    float gyro_rate = (gy - gyro_bias) * 0.00875 * _PI / 180.0; // Convert to rad/s
    roll_angle = 0.98 * (roll_angle + gyro_rate * dt) + 0.02 * accel_roll;

    // Smooth the roll angle
    float filtered_pitch = lpf_pitch(roll_angle);

    // --- OUTER LOOP: Position/Velocity to Lean Angle ---
    // Average motor position
    float robot_position = (motor1.shaft_angle + motor2.shaft_angle) / 2.0;
    
    // Calculate how much we need to lean to reach/hold position
    // If we are pushed forward, the robot leans backward to counter it
    float target_pitch = pid_vel(target_position - robot_position);

    // --- INNER LOOP: Lean Angle to Motor Voltage ---
    // Calculate required voltage to maintain the target pitch
    // Negative because leaning forward requires accelerating forward
    float control_voltage = -pid_stb(target_pitch - filtered_pitch);

    // Safety cutout: If it falls over (> 45 degrees / 0.8 rad), cut power
    if (abs(filtered_pitch) > 0.8) {
      control_voltage = 0;
      pid_stb.reset();
      pid_vel.reset();
    }

    // Apply voltage. One motor might need a reversed sign depending on mounting direction
    motor1.move(-control_voltage);
    motor2.move(-control_voltage); // Use -control_voltage if the motor is mirrored

      // Wypisuj dane co 50 ms (20 Hz)
    static unsigned long last_print = 0;
    if (millis() - last_print > 50) { 
      last_print = millis();

      // Wyplotuj kąt docelowy
      Serial.print(">Target_Pitch:");
      Serial.println(target_pitch);
      
      // Wyplotuj aktualny kąt po filtrze
      Serial.print(">Filtered_Pitch:");
      Serial.println(filtered_pitch);
    }
  }
}