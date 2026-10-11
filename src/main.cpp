#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

// --- ENCODER SPI PINS ---
#define CSN_1 PB12
#define CSN_2 PB10
SPIClass SPI_2(PB15, PB14, PB13);

// --- MOTOR CONFIGURATION ---
// 7 pole pairs is standard for a 14-pole gimbal motor. 
// Change to 11 if using a 22-pole motor like the 5010 360KV.
BLDCMotor motor1 = BLDCMotor(7); 
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA1, PA2, PA3, PE7); // SimpleFOC Mini on PA1, PA2, PA3. EN on PE7

// --- CUSTOM HIGH-SPEED SENSOR ---
void initEncoders() {
  pinMode(CSN_1, OUTPUT); digitalWrite(CSN_1, HIGH);
  pinMode(CSN_2, OUTPUT); digitalWrite(CSN_2, HIGH);
  SPI_2.begin();
  // Keep the SPI bus open to avoid transaction overhead in the main loop
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

  Serial.println("1. Initializing Custom SPI Encoders...");
  sensor1.init();
  sensor2.init();

  Serial.println("2. Configuring BLDC Motor and Driver...");
  
  // Initialize driver variables
  driver1.voltage_power_supply = 12.0;
  driver1.init();

  // Link driver and sensor
  motor1.linkDriver(&driver1);
  motor1.linkSensor(&sensor1); // Motor 1 driven by Encoder 1

  // Set control modes
  // We use pure voltage torque mode. No PIDs. 
  // This is the rawest, purest way to test FOC timing.
  motor1.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;

  // IMPORTANT: Limits to prevent burning the motor on the bench
  motor1.voltage_limit = 4.0; // Max 4 Volts applied to the phases
  motor1.voltage_sensor_align = 2.0; // 2 Volts during the alignment beep

  Serial.println("3. Starting FOC Alignment...");
  // This will make the motor beep and turn slightly to find the electric zero
  motor1.init();
  motor1.initFOC();

  Serial.println("4. Setup Complete. Motor should spin silently now.");
  
  // Set a target of 1.5 Volts. 
  // It should result in a smooth, silent spin.
  motor1.target = 1.5; 
}

void loop() {
  // These two lines are the heart of SimpleFOC.
  // loopFOC() reads the sensor, does the vector math, and updates the PWM.
  // Because our sensor read is now lightning-fast, this will run at >10kHz.
  motor1.loopFOC();
  motor1.move();

  // We can add simple Teleplot output for monitoring, but we slow it down 
  // significantly so we don't block the FOC loop with Serial prints.
  static unsigned long last_print = 0;
  if (millis() - last_print > 100) { // Print at 10Hz
    last_print = millis();
    
    // We can print the electrical angle to ensure the loop is tracking it
    // Serial.print(">Angle_Elec:");
    // Serial.println(motor1.electrical_angle(), 4); // throws an error, no such function
    
    // Print the raw position to confirm it matches the spin
    Serial.print(">Enc1_Rad:");
    Serial.println(sensor1.getAngle(), 4);
  }
}