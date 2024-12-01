#include <Arduino.h>
#include "SoftwareSerial.h"
#include "DS18B20.h"

#define DALLAS_PIN  25
#define MT3608_PIN  33
#define VPP_SP_PIN  39
#define VCCBAT_PIN  36

#define SIM_PWR_PIN 5
#define SIM_RX_PIN  18
#define SIM_TX_PIN  19

#define RS485_RX_PIN  26
#define RS485_TX_PIN  27

OneWire oneWire(DALLAS_PIN);
DS18B20 sensor(&oneWire);

void setup() {
  sensor.begin();
  sensor.setResolution(10);
}

void loop() {

}

float getTemperature() {
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete()) {
    delay(10);
  }
  return sensor.getTempC();
}