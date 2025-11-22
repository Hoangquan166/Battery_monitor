#include <DallasTemperature.h> 
#include <OneWire.h> 
#include <Wire.h> 
#include <Adafruit_INA219.h>

#define ONE_WIRE_BUS 18 
OneWire oneWire(ONE_WIRE_BUS); // khai bao chan GPIO 18 la chan onewire (DQ)
DallasTemperature sensors(&oneWire); // setup oneWire lam chan truyen nhan data

Adafruit_INA219 ina219;

  float tempC = 0;
  float shuntvoltage = 0;
  float loadvoltage = 0;
  float current_mA = 0;
  float batteryvoltage = 0;

void setup() {
  Serial.begin(115200);
  sensors.begin();
  ina219.begin();
  Wire.begin(21,22); // SDA, SCL
  
  // Cấu hình 12-bit, 128 samples, bus range 32V, PGA ÷8, continuous mode
  uint16_t config = 0;
  config |= (1 << 13);       // BRNG = 1 
  config |= (3 << 11);       // PG = 11 
  config |= (0xF << 7);      // BADC = 1111 
  config |= (0xF << 3);      // SADC = 1111 
  config |= 0x07;            // MODE continuous = 111 
  Wire.beginTransmission(0x40);
  Wire.write(0x00);
  Wire.write((config >> 8) & 0xFF);  // High byte
  Wire.write(config & 0xFF);         // Low byte
  Wire.endTransmission();
  delay(70); 

  // Vgs = 3.3V
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
}

void loop() {
    //INA219
  shuntvoltage = ina219.getShuntVoltage_mV();
  loadvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  batteryvoltage = loadvoltage + (shuntvoltage / 1000);
  if (batteryvoltage <= 6) {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    Serial.printf("LOW BATTERY - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(19, LOW);
  } else {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
  } 
  if (current_mA > 1000) {
    Serial.printf("Current: %.1f mA\n", current_mA);
    Serial.printf("OVERCURRENT - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(19, LOW);
  } else {
    Serial.printf("Current: %.1f mA\n", current_mA);
  }
/* Serial.printf("shuntvolt: %.1f mV\n", shuntvoltage);
  Serial.printf("loadvolt: %.2f V\n", loadvoltage); */

  // DS18B20
  sensors.requestTemperatures();
  delay(750); 
  tempC = sensors.getTempCByIndex(0); // lấy data từ con cảm biến đầu tiên
if (tempC == DEVICE_DISCONNECTED_C) {
  Serial.printf("Sensor not found or CRC error!\n");
} else if (tempC >= 60) {
  Serial.printf("Temperature: %.1f°C\n", tempC);
  Serial.printf("TEMPERATURE CRITICAL, SAFETY CUTOFF ACTIVATED\n"); 
  digitalWrite(19, LOW);
} else {
  Serial.printf("Temperature: %.1f°C\n", tempC);
}
  delay(1000); 
}
