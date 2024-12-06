#include <Arduino.h>
#include "SoftwareSerial.h"
#include "DS18B20.h"
#include "time.h"

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

#define SENSOR_EN_PIN 23
#define SerialUS Serial2

OneWire oneWire(DALLAS_PIN);
DS18B20 sensor(&oneWire);
SoftwareSerial SerialGSM(SIM_RX_PIN, SIM_TX_PIN);
RTC_DATA_ATTR int dist_mm = 0;
RTC_DATA_ATTR int bootCount = 0;

const String url_server = "gsm.mvahi.uz";

String imei = "";
int batt_mV = 0;
int rssi_db = 0;
float distance = 0.0;
float temperature = 0.0;
bool getHttpState = false;

bool powerDownSimCom() {
  SerialGSM.println("AT+CPOWD=0");
  uint32_t cur_time = millis();
  while (millis() - cur_time < 5000) {
    if (SerialGSM.available()) {
      Serial.println(SerialGSM.readStringUntil('\n'));
      break;
    }
  }
  Serial.println("GSM module is power down!");
  return true;
}

String getResponseSimCom(String cmd) {
  SerialGSM.println(cmd);
  delay(500);
  String resp = "Error";
  uint32_t cur_time = millis();
  while (millis() - cur_time < 2000) {
    if (SerialGSM.available()) {
      String s = SerialGSM.readStringUntil('\n');
      Serial.println(s);
      if (s.indexOf("OK") >= 0 || s.indexOf("ERROR") >= 0) break;
      if (s.length() > 1) resp = s;
    }
  }
  return resp;
}

bool powerUpSimCom() {
  digitalWrite(SIM_PWR_PIN, 1);
  delay(500);
  digitalWrite(SIM_PWR_PIN, 0);
  uint32_t cur_time = millis();
  int at_time = 0, send_time = 0;
  while (1) {
    if (SerialGSM.available()) {
      String data = SerialGSM.readStringUntil('\n');
      Serial.println(data);
      if (data.indexOf("OK") >= 0) {
        at_time += 1;
      }
    }
    if (at_time >= 3) break;
    if (millis() - cur_time > 1000) {
      SerialGSM.println("AT");
      send_time += 1;
      if (send_time >= 3) return powerUpSimCom();
      Serial.println("Send: AT");
      cur_time = millis();
    }
  }
  if (at_time >= 3) return true;
  Serial.println("GSM module is power on!");
  return false;
}

String getIPAddress() {
  return getResponseSimCom(F("AT+CNACT=0,1"));
}

int httpGET(String cmd) {
  SerialGSM.println(cmd);
  delay(500);
  Serial.println(cmd);
  delay(500);
  int len = 0;
  uint32_t cur_time = millis();
  while (millis() - cur_time < 30000) {
    if (SerialGSM.available()) {
      String s = SerialGSM.readStringUntil('\n');
      Serial.println(s);
      if (s.indexOf("+SHREQ: \"GET\",200") >= 0) {
          sscanf(s.c_str(), "+SHREQ: \"GET\",200,%d", &len);
          break;
      }
    }
  }
  return len;
}

String getDateTime() {
  getResponseSimCom(F("AT+CNACT=0,1"));
  delay(1000);
  getResponseSimCom(F("AT+CNTP=\"3.uz.pool.ntp.org\",20,0,0"));
  delay(1000);
  getResponseSimCom(F("AT+CNTP"));
  delay(1000);
  
  String s = getResponseSimCom(F("AT+CCLK?"));
  char result[21] = {0}; // Massivni to'ldirish va null-terminator qo'yish
  
  // +CCLK javobini tahlil qilish
  if (s.indexOf(F("+CCLK")) >= 0) {
    int n = sscanf(s.c_str(), "+CCLK: \"%20s\"", result);  // 24 ta belgini o'qish (25 ta bo'sh joyga nisbatan)
    if (n != 1) {
      Serial.println(F("Error parsing date-time!"));
      return String(""); // Agar noto'g'ri format bo'lsa, bo'sh string qaytaramiz
    }
  }

  // Natijani `String`ga aylantirish va qaytarish
  return String(result);
}

int getRSSI() {
  String resp = getResponseSimCom("AT+CSQ");
  int rssi = 0, k;
  if (resp.indexOf("+CSQ") >= 0) {
    sscanf(resp.c_str(), "+CSQ: %d,%d", &rssi, &k);
  }
  return rssi;
}

int getBattery() {
  String resp = getResponseSimCom("AT+CBC");
  int percent = 0, voltage = 0, k = 0;
  if (resp.indexOf("+CBC") >= 0) {
    sscanf(resp.c_str(), "+CBC: %d,%d,%d", &k, &percent, &voltage);
  }
  return voltage;
}

bool isValidIMEI(String imei) {
  for (int i = 0; i < imei.length(); i++) {
    if (!isdigit(imei[i])) {
      return false;  // Agar birorta belgi raqam bo'lmasa, noto'g'ri IMEI
    }
  }
  return true;  // Agar barcha belgilar raqam bo'lsa, to'g'ri IMEI
}

String getIMEI() {
  String imei = getResponseSimCom(F("AT+GSN")); // IMEI ni olish
  imei.trim(); // Kiritilgan bo'sh joylarni olib tashlash
  
  // IMEI ni tekshirish
  if (isValidIMEI(imei)) {
    return imei;  // Agar IMEI faqat raqamlardan iborat bo'lsa, uni qaytarish
  } else {
    Serial.println(F("Invalid IMEI!"));
    return String("");  // Noto'g'ri IMEI holatida bo'sh string qaytarish
  }
}

float getTemperature() {
  sensor.requestTemperatures();
  while (!sensor.isConversionComplete()) {
    delay(10);
  }
  return sensor.getTempC();
}

uint16_t getDistance() {
  digitalWrite(SENSOR_EN_PIN, 0);
  delay(1000);
  uint16_t distance_mm = 0;
  uint8_t readReg[4];
  uint32_t cur_time = millis();
  while (millis() - cur_time < 10000) {
    if (SerialUS.available()) {
      SerialUS.readBytesUntil(0xFF, readReg, 3);
      distance_mm = (distance_mm + ((readReg[0] << 8) | readReg[1])) / 2;
    }
  }
  digitalWrite(SENSOR_EN_PIN, 1);
  return distance_mm;
}

String createUrl(String imei, int dist, float temp, String datetime, int batt_mV, int rssi) {
  // Format datetime to ISO 8601
  datetime.replace('/', '-');
  imei.trim();
  // Buffer hajmini oshirish
  const size_t BUFFER_SIZE = 512; // Yetarli katta bo'lishi kerak
  char url[BUFFER_SIZE];

  // Snprintf bilan xavfsiz URL yaratish
  int length = snprintf(
    url,
    BUFFER_SIZE,
    "/api/variable?imei=%s&distance=%d&temperature=%.2f&datetime=%s&battery=%d&rssi=%d",
    imei.c_str(),
    dist,
    temp,
    datetime.c_str(),
    batt_mV,
    rssi
  );
  Serial.println(length);
  // Agar uzunlik buferdan oshsa, xato qaytariladi
  if (length < 0 || length >= BUFFER_SIZE) {
    Serial.println("Error: URL buffer size is too small!");
    return "";
  }

  return String(url);
}

void setup() {
  pinMode(SIM_PWR_PIN, OUTPUT);
  pinMode(SENSOR_EN_PIN, OUTPUT);
  digitalWrite(SENSOR_EN_PIN, 1);
  digitalWrite(SIM_PWR_PIN, 0);
  Serial.begin(115200);
  SerialGSM.begin(19200);
  SerialUS.begin(9600);
  sensor.begin();
  sensor.setResolution(10);
  ++bootCount;
  Serial.printf("Boot count: %d\n", bootCount);
  temperature = getTemperature();
  uint16_t dist = getDistance();
  Serial.printf("Distance: %d\n", dist);
  if (abs(dist_mm - dist) > 5) {
    Serial.print("Old dist: "); Serial.print(dist_mm);
    Serial.print("Current dist: "); Serial.println(dist);
    dist_mm = dist;
    
    uint8_t pwr_counter = 0;
    while (!powerUpSimCom()) {
      if (pwr_counter ++ >= 3) break;
      delay(3000);
    }

    while (imei.length() < 10) {
      imei = getIMEI();
      delay(1000);
    }
    for (int x = 0; x < 3; x++) {
      batt_mV = getBattery();
      if (batt_mV > 1000 && batt_mV < 4400) break;
    }
    for (int x = 0; x < 3; x++) {
      rssi_db = getRSSI();
      if (rssi_db > 0 && rssi_db <= 31) break;
    }
    String datetime = getDateTime();
    datetime.trim();
    int datetime_count = 0;
    while (datetime.length() < 19) {
      datetime = getDateTime();
      datetime_count ++;
      if (datetime_count >= 3) break;
    }
    Serial.println("IMEI: " + imei);
    Serial.println("Battery: " + String(batt_mV));
    Serial.println("RSSI: " + String(rssi_db));
    Serial.printf("DateTime: %20s\n", getDateTime().c_str());
    delay(1000);
    String url_get = createUrl(imei, dist, temperature, datetime, batt_mV, rssi_db);
    Serial.println(url_get);
    delay(1000);
    getResponseSimCom("at+shconf=\"url\",\"" + url_server + "\"");
    delay(1000);
    getResponseSimCom("at+shconf=\"bodylen\",1024");
    delay(1000);
    getResponseSimCom("at+shconf=\"headerlen\",350");
    delay(1000);
    getResponseSimCom("at+shconf=\"timeout\",10");
    delay(1000);
    getResponseSimCom("at+shconn");
    delay(1000);
    int len = httpGET("at+shreq=\"" + String(url_get) + "\",1");
    delay(1000);
    getResponseSimCom("at+shread=0," + String(len));
    delay(1000);
    getResponseSimCom("at+shdisc");
  }
}

void loop() {
  powerDownSimCom();
  esp_deep_sleep(30 * uS_TO_S_FACTOR);
}

