#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7); 
BLDCDriver3PWM driver = BLDCDriver3PWM(PA1, PA2, PA3, PE7);
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  delay(1000); 

  // --- I2C SETUP ---
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.setClock(400000); 
  Wire.begin();

  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  pinMode(PE7, OUTPUT);
  digitalWrite(PE7, HIGH);

  driver.voltage_power_supply = 12.0;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 3.0; 
  
  // 1. Set to Torque Mode
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  
  // 2. Set absolute maximum voltage you want to test
  // (Start lower, like 6.0V, to avoid melting the motor during stall)
  motor.voltage_limit = 12.0; 

  // // Initialize FOC
  // motor.init();
  // motor.initFOC();

  Serial.println("Initializing Motor...");
  motor.init();

  Serial.println("Starting Sensor Alignment Dance...");
  motor.initFOC();
  command.add('T', doTarget, "target angle");

  // Lock to 0 radians instantly upon boot
  motor.target = 0.0; 
  Serial.println("READY! Motor locked at 0 radians.");
}

void loop() {
//   motor.loopFOC();
//   motor.move();
//   command.run();
    Serial.println("Motor angle: " + String(sensor.getMechanicalAngle()) + " rad");
    delay(100);
}