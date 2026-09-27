#include <SimpleFOC.h>
#include <Wire.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7); 
BLDCDriver3PWM driver = BLDCDriver3PWM(PA1, PA2, PA3, PE7);
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  Serial.begin(115200);
  
  // 1. Wait for Serial Monitor to open before continuing
  while (!Serial) {} 
  delay(100); 

  SimpleFOCDebug::enable(&Serial);

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
  
  // 2. Change to Angle mode to lock position
  motor.controller = MotionControlType::angle;
  
  // --- GIMBAL MOTOR TUNING ---
  
  // 1. Smooth out the sensor noise so the PID doesn't panic on micro-jitters
  motor.LPF_velocity.Tf = 0.01; 

  // 2. Lower the inner loop Velocity P and I gains
  motor.PID_velocity.P = 0.1; // Much softer response
  motor.PID_velocity.I = 10.0;
  
  // 3. Lower the outer loop Angle P gain
  motor.P_angle.P = 15.0; 

  // 4. Limit the maximum speed the motor will use to correct its position
  motor.velocity_limit = 10.0; // rad/s
  motor.voltage_limit = 3.0; 

  // 3. Cleaned up initialization (only called once!)
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
  motor.loopFOC();
  motor.move();
  command.run();

  // sensor.update(); 
  
  // Serial.println("Motor angle: " + String(sensor.getMechanicalAngle()) + " rad");
  // delay(100);
}