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
#define GYRO_OUT_Z  0x2C // Z-Axis for Roll

// --- COMPRESSED DUAL ENCODER LOGIC (SPI2) ---
#define CSN_1 PB12
#define CSN_2 PB10
SPIClass SPI_2(PB15, PB14, PB13);

float gyro_bias = 0.0;

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

// --- MOTOR SETUP ---
// Motor 1
BLDCMotor motor1 = BLDCMotor(7); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA1, PA2, PA3, PE7);

// Motor 2 (UPDATE THESE PINS TO YOUR ACTUAL WIRING!)
// BLDCMotor motor2 = BLDCMotor(7); 
// BLDCDriver3PWM driver2 = BLDCDriver3PWM(PB0, PB1, PB2, PE8);

// --- BALANCING PID CONTROLLER ---
// Starting with a gentle PID so it doesn't violently kick
PIDController pid_stb{0.4, 0.0, 0.005, 100000, 12.0}; 

// --- IMU VARIABLES ---
float roll_angle = 0.0;
unsigned long last_imu_time = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("1. Initializing Encoders & IMU...");
  sensor1.init();
  sensor2.init();

  // Accel Init
  Wire.setSDA(PB9); Wire.setSCL(PB6); Wire.setClock(100000); Wire.begin();
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A); Wire.write(0x57); 
  Wire.endTransmission();

  // Gyro Init
  pinMode(GYRO_CS_PIN, OUTPUT); digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.setSCLK(PA5); SPI.setMISO(PA6); SPI.setMOSI(PA7); SPI.begin();
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(GYRO_CS_PIN, LOW);
  SPI.transfer(GYRO_CTRL1); SPI.transfer(0x0F); 
  digitalWrite(GYRO_CS_PIN, HIGH);
  SPI.endTransaction();

  Serial.println("Calibrating Gyro... KEEP ROBOT STILL & UPSIDE DOWN.");
  long gyro_sum = 0;
  for (int i = 0; i < 500; i++) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_Z | 0x80 | 0x40);
    SPI.transfer(0x00); SPI.transfer(0x00);
    gyro_sum += (SPI.transfer(0x00) | (SPI.transfer(0x00) << 8));
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();
    delay(2);
  }
  gyro_bias = (float)gyro_sum / 500.0;

  Serial.println("2. Initializing Motors...");
  // Driver 1
  pinMode(PE7, OUTPUT); digitalWrite(PE7, HIGH); // Enable pin
  driver1.voltage_power_supply = 12.0;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.linkSensor(&sensor1);
  
  // Driver 2
  // pinMode(PE8, OUTPUT); digitalWrite(PE8, HIGH); // Enable pin
  // driver2.voltage_power_supply = 12.0;
  // driver2.init();
  // motor2.linkDriver(&driver2);
  // motor2.linkSensor(&sensor2);

  // Control logic
  motor1.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.voltage_limit = 4.0; 
  motor1.voltage_sensor_align = 3.0; 

  // motor2.controller = MotionControlType::torque;
  // motor2.torque_controller = TorqueControlType::voltage;
  // motor2.voltage_limit = 12.0; 
  // motor2.voltage_sensor_align = 3.0; 

  // ENABLE SIMPLEFOC VERBOSE DEBUGGING
  SimpleFOCDebug::enable(&Serial);

  Serial.println("3. Starting FOC Alignment... KEEP WHEELS IN AIR!");
  motor1.init(); 
  motor1.initFOC();

  // MUST BE COMMENTED OUT WHILE TESTING ONLY ONE MOTOR
  // motor2.init(); 
  // motor2.initFOC();

  Serial.println("--- BALANCING LOOP READY ---");
  last_imu_time = micros();
}

void loop() {
  // 1. Ultra-fast FOC Loop for both motors
  motor1.loopFOC();
  // motor2.loopFOC();

  // 2. The 100Hz IMU & Balancing Loop
  unsigned long current_time = micros();
  if (current_time - last_imu_time >= 10000) { 
    float dt = (current_time - last_imu_time) / 1000000.0;
    last_imu_time = current_time;

    // --- Read Accel ---
    Wire.beginTransmission(LSM303_ADDR);
    Wire.write(OUT_X_L_A | 0x80); Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM303_ADDR, (uint8_t)6);
    int16_t ay=0, az=0;
    if (Wire.available() == 6) {
      Wire.read(); Wire.read(); // Skip X
      ay = Wire.read() | (Wire.read() << 8);
      az = Wire.read() | (Wire.read() << 8);
    }

    // --- Read Gyro (Z-Axis) ---
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_Z | 0x80 | 0x40); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    int16_t gy = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8); 
    SPI.transfer(0x00); SPI.transfer(0x00); 
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();

    // --- Sensor Math ---
    float pure_accel_roll = atan2(-ay, -az) * 180.0 / PI; 
    
    // IF THE GYRO FIGHTS THE ACCEL, CHANGE THIS TO: -((gy - gyro_bias)...
    float gyro_rate = (gy - gyro_bias) * 0.00875;
    
    roll_angle = 0.98 * (roll_angle + gyro_rate * dt) + 0.02 * pure_accel_roll;

    // --- PID Control ---
    // Calculate the voltage required to push the robot back to 0 degrees
    float target_voltage = pid_stb(0.0 - roll_angle);

    // Apply the voltage to both motors (You may need to make one negative!)
    // motor1.target = target_voltage;

    motor1.target = 4.0;

    // motor2.target = target_voltage; 
  }

  // 3. Command the coils
  motor1.move();
  // motor2.move();
}