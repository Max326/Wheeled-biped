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

// --- SIMPLEFOC OBJECTS ---
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7); 
BLDCDriver3PWM driver = BLDCDriver3PWM(PA1, PA2, PA3, PE7);

// --- BALANCING PID CONTROLLER ---
// PID parameters: P, I, D, ramp, limit
// You will need to tune P and D! Start small.
PIDController pid_stb{0.1, 0.0, 0.005, 100000, 12.0};

float pitch_angle = 0.0;
unsigned long last_imu_time = 0;

void setup() {
  Serial.begin(115200);
  delay(2000); // Time to open the monitor

  Serial.println("\n--- ROBOT BOOT SEQUENCE START ---");

  // 1. Configure I2C Pins FIRST
  Serial.println("1. Configuring I2C & AS5600 Encoder...");
  Wire.setSDA(PB9);
  Wire.setSCL(PB6);
  
  // Let SimpleFOC do its internal Wire.begin() BEFORE we talk to the IMU!
  sensor.init(&Wire); 
  motor.linkSensor(&sensor);
  
  // Now enforce our 100kHz speed for stability
  Wire.setClock(100000); 

  // 2. NOW we can safely talk to the Accelerometer
  Serial.println("2. Waking up Accelerometer...");
  Wire.beginTransmission(LSM303_ADDR);
  Wire.write(CTRL_REG1_A);
  Wire.write(0x57); 
  byte i2c_err = Wire.endTransmission();
  Serial.print("   I2C Status (0 is success): "); Serial.println(i2c_err);

  Serial.println("3. Configuring SPI & Gyroscope...");
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

  Serial.println("4. Initializing Motor Driver...");
  pinMode(PE7, OUTPUT);
  digitalWrite(PE7, HIGH);
  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  
  // RESTORED TO YOUR ORIGINAL WORKING LIMITS
  motor.voltage_limit = 4.0; 
  motor.voltage_sensor_align = 3.0; 

  SimpleFOCDebug::enable(&Serial);
  Serial.println("5. Starting FOC Alignment...");
  motor.init();
  motor.initFOC();

  Serial.println("--- BALANCING LOOP READY ---");
  last_imu_time = micros();
}

void loop() {
  // 1. The FOC algorithm runs as fast as possible in the background
  motor.loopFOC();

  // 2. The IMU and Balancing loop runs exactly at 100Hz
  unsigned long current_time = micros();
  if (current_time - last_imu_time >= 10000) { 
    float dt = (current_time - last_imu_time) / 1000000.0;
    last_imu_time = current_time;

    // --- Read Accel ---
    Wire.beginTransmission(LSM303_ADDR);
    Wire.write(OUT_X_L_A | 0x80); 
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)LSM303_ADDR, (uint8_t)6);
    int16_t ax=0, ay=0, az=0;
    if (Wire.available() == 6) {
      ax = (Wire.read() | (Wire.read() << 8)) >> 4;
      ay = (Wire.read() | (Wire.read() << 8)) >> 4;
      az = (Wire.read() | (Wire.read() << 8)) >> 4;
    }

    // --- Read Gyro ---
    int16_t gy=0;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(GYRO_CS_PIN, LOW);
    SPI.transfer(GYRO_OUT_X | 0x80 | 0x40); 
    SPI.transfer(0x00); SPI.transfer(0x00); // Skip X
    gy = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8); // Read Y
    SPI.transfer(0x00); SPI.transfer(0x00); // Skip Z
    digitalWrite(GYRO_CS_PIN, HIGH);
    SPI.endTransaction();

    // --- Complementary Filter ---
    float accel_pitch = atan2(-ax, sqrt((float)ay * (float)ay + (float)az * (float)az)) * 180.0 / PI;
    float gyro_rate = gy * 0.00875;
    pitch_angle = 0.995 * (pitch_angle + gyro_rate * dt) + 0.005 * accel_pitch;

    // --- BALANCING MATH ---
    // The target angle is 0.0 (perfectly upright). 
    // We calculate the voltage needed to push back against the fall.
    float target_voltage = pid_stb(0.0 - pitch_angle);

    // Feed that voltage into the motor
    motor.target = target_voltage;
  }

  // 3. Apply the calculated voltage to the coils
  motor.move();
}