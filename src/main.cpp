#include <Arduino.h>
#include "SoftwareSerial.h"
#include "DS18B20.h"
#include "SPIFFS.h"

#define DALLAS_PIN  25
#define MT3608_PIN  33
#define VPP_SP_PIN  39
#define VCCBAT_PIN  36

#define SIM_PWR_PIN 5
#define SIM_RX_PIN  18
#define SIM_TX_PIN  19

#define RS485_RX_PIN  26
#define RS485_TX_PIN  27

#define uS_TO_S_FACTOR  1000000ULL

OneWire oneWire(DALLAS_PIN);
DS18B20 sensor(&oneWire);
float getTemperature();

void setup() {
  pinMode(SIM_PWR_PIN, OUTPUT);
  digitalWrite(SIM_PWR_PIN, 1);
  delay(3000);
  digitalWrite(SIM_PWR_PIN, 0);
  Serial.begin(115200);
  sensor.begin();
  sensor.setResolution(10);
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    delay(5000);
    esp_restart();
  }
  File file = SPIFFS.open("/sensor.temp", "r", true);
  
}
uint32_t last_time = 0, time_span = 1000;

void loop() {

  Serial.printf("Temperature: %.2f\r\n", getTemperature());
  esp_deep_sleep(10 * uS_TO_S_FACTOR);
}

float getTemperature() {
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete()) {
    delay(10);
  }
  return sensor.getTempC();
}