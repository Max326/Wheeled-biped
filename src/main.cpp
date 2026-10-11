#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>

#define CSN_1 PB12
#define CSN_2 PB10

SPIClass SPI_2(PB15, PB14, PB13);

// Inicjalizacja układów z zachowaniem sprzętowego twardego pull-upu (żeby nic nie pływało na starcie)
void initEncoders() {
  pinMode(CSN_1, OUTPUT); digitalWrite(CSN_1, HIGH);
  pinMode(CSN_2, OUTPUT); digitalWrite(CSN_2, HIGH);

  SPI_2.begin();
  // Zostawiamy raz otwartą sesję SPI na 500 kHz z odpowiednim trybem, 
  // zamiast wywoływać beginTransaction 20000 razy na sekundę.
  SPI_2.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
}

// Błyskawiczny, "brutalny" odczyt na wzór Twojego działającego skryptu
float readEnc1() {
  digitalWrite(CSN_1, LOW);
  // Ślepy, ale sprawny strzał z jedynek, który omija problem pipeliningu AS5047
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

// Przekazanie szybkich funkcji do GenericSensor
GenericSensor sensor1(readEnc1, initEncoders);
GenericSensor sensor2(readEnc2, initEncoders);

void setup() {
  Serial.begin(115200);
  delay(2000); 

  Serial.println("Initializing Lightning-Fast Custom SPI Encoders...");
  
  sensor1.init();
  sensor2.init();

  Serial.println("All Sensors Ready! Open Teleplot.");
}

void loop() {
  sensor1.update();
  sensor2.update();

  static unsigned long last_print = 0;
  if (millis() - last_print > 50) { 
    last_print = millis();

    Serial.print(">Enc1_Rad:");
    Serial.println(sensor1.getAngle(), 4);
    Serial.print(">Enc2_Rad:");
    Serial.println(sensor2.getAngle(), 4);
  }
}