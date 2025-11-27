#include <DallasTemperature.h> 
#include <Onewire.h> 
#include <WiFi.h>
#include <PubSubClient.h>
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
  float temp_sta=0;
  float cur_sta=0;
  float bat_sta=0;
  float vol_sta=0;
  float load_sta=0;
  float charge_sta=1;
  float sw=0;
  float change_load=1;
  float change_char=1;

const char* ssid = "abc";
const char* password = "12345678";

const char* mqtt_server = "10.136.50.129";
const int mqtt_port = 1883;
const char* access_token = "2RP9CzGLNXtBpLiiiXs2";

unsigned long reconnectTs = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void updateReconnectTs() {
    reconnectTs = millis();
    Serial.printf("MQTT reconnected, reconnectTs = %lu\n", reconnectTs);
}

void rpcCallback(char* topic, byte* payload, unsigned int length) {
      if (millis() - reconnectTs < 100) { // 100 ms margin
        return;
    }

    String msg = "";
    for (int i = 0; i < length; i++) msg += (char)payload[i];
    Serial.println("RPC: " + msg);

  if (msg.indexOf("setLoad") != -1){
      if (msg.indexOf("true") != -1) {
        load_sta=0;
        digitalWrite(19, HIGH);
        change_load=1;
        Serial.println("Command: RESTORE LOAD");
  }   else {
        load_sta=1;
        digitalWrite(19, LOW);
        change_load=1;
        Serial.println("Command: CUTOFF LOAD");
      }
  }

  if (msg.indexOf("Sw") != -1){
      if (msg.indexOf("true") != -1) {
        sw=1;

        load_sta=1;
        digitalWrite(19, LOW);
        change_load=1;
        Serial.println("Command: SWITCH TO CHARGING");

        charge_sta=0;
        digitalWrite(17, HIGH);
        change_char=1;
  }   else {
        sw=0;

        charge_sta=1;
        digitalWrite(17, LOW);
        change_char=1;
        Serial.println("Command: SWITCH TO LOAD");

        load_sta=0;
        digitalWrite(19, HIGH);
        change_load=1;
      }
  }

    String requestId = String(topic).substring(String(topic).lastIndexOf('/') + 1);
    String responseTopic = "v1/devices/me/rpc/response/" + requestId;
    client.publish(responseTopic.c_str(), "{}");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sensors.begin();
  ina219.begin();
  Wire.begin(21,22);
  
  // Cấu hình 12-bit, 128 samples, bus range 32V, PGA ÷8, continuous mode
  uint16_t config = 0;
  config |= (1 << 13);       // BRNG = 1 
  config |= (1 << 11);       // PGA : 11 
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

  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
}
  Serial.println("\nConnected to WiFi!");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(rpcCallback);
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("ESP32_Device", access_token, NULL)) {
      Serial.println("connected!");
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
    //INA219
  shuntvoltage = ina219.getShuntVoltage_mV();
  loadvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  batteryvoltage = loadvoltage + (shuntvoltage / 1000);
  if(sw==1){
  if (batteryvoltage >=5) {
    Serial.printf("Current: %.2f mA\n", current_mA);
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    Serial.printf("FULL - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(17, LOW);
    charge_sta=1;
    change_char=1;
  }
  else {
    if (batteryvoltage <= 2.5) 
  {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    Serial.printf("LOW VOLTAGE - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(17, LOW);
    vol_sta=1;
  } else {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    vol_sta=0;
  } 
    if (current_mA > 800) {
    Serial.printf("Current: %.1f mA\n", current_mA);
    Serial.printf("OVERCURRENT - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(17, LOW);
    cur_sta=1;
  } else {
    Serial.printf("Current: %.1f mA\n", current_mA);
    cur_sta=0;
  }
}
  }
  if(sw==0){
  if (batteryvoltage <= 3) {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    Serial.printf("LOW BATTERY - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(19, LOW);
    vol_sta=1;
  } else {
    Serial.printf("Battery Voltage: %.2f V\n", batteryvoltage);
    vol_sta=0;
  } 
  if (current_mA > 800) {
    Serial.printf("Current: %.1f mA\n", current_mA);
    Serial.printf("OVERCURRENT - SAFETY CUTOFF ACTIVATED\n");
    digitalWrite(19, LOW);
    cur_sta=1;
  } else {
    Serial.printf("Current: %.1f mA\n", current_mA);
    cur_sta=0;
  }
}
/* Serial.printf("shuntvolt: %.1f mV\n", shuntvoltage);
  Serial.printf("loadvolt: %.2f V\n", loadvoltage); */

  // DS18B20
  sensors.requestTemperatures();
  delay(750); 
  tempC = sensors.getTempCByIndex(0); // lấy data từ con cảm biến đầu tiên
if (tempC == DEVICE_DISCONNECTED_C) {
  Serial.printf("Sensor not found or CRC error!\n");
} else if (tempC >= 30) {
  Serial.printf("Temperature: %.1f°C\n", tempC);
  Serial.printf("TEMPERATURE CRITICAL, SAFETY CUTOFF ACTIVATED\n");
  if(load_sta==0) digitalWrite(19, LOW);
  if(charge_sta==0) digitalWrite(17, LOW);
  temp_sta=1;
} else {
  Serial.printf("Temperature: %.1f°C\n", tempC);
  temp_sta=0;
}
  if(cur_sta==1||temp_sta==1||vol_sta==1){
    if(load_sta==0){
      load_sta=1;
      change_load=1;
    }
    if(charge_sta==0){
      charge_sta=1;
      change_char=1;
    }
  }
//Database
  if (!client.connected()) {
    while (!client.connected()) {
      Serial.println("Reconnecting to ThingsBoard...");
      if (client.connect("ESP32_Device", access_token, NULL)) {
        Serial.println("Reconnected!");
      } else {
        delay(1000);
      }
    }
  }

String payload = "{";
payload += "\"Temperature(℃)\":" + String(tempC);
payload += ",\"Voltage(V)\":" + String(batteryvoltage);
payload += ",\"Current(mA)\":" + String(current_mA);
payload += ",\"Temperature status\":" + String(temp_sta);
payload += ",\"Current status\":" + String(cur_sta);
payload += ",\"Voltage status\":" + String(vol_sta);
payload += ",\"Switch\":" + String(sw);

if (change_load == 1) {
    payload += ",\"Load status\":" + String(load_sta);
    change_load =0;
}

if (change_char == 1) {
    payload += ",\"Charge status\":" + String(charge_sta);
    change_char =0;
}
payload += "}";

  Serial.println("Publishing data: " + payload);

  client.publish("v1/devices/me/telemetry", payload.c_str());
  client.loop();

  delay(500);
}