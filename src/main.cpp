#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <MQUnifiedsensor.h> // Thêm thư viện MQUnifiedsensor
#include <ArduinoJson.h>
#include <U8g2lib.h>

// 13, 12, 14, 34, 35, 16, 17, 18, 19, 21, 22
// Định nghĩa chân
#define DS18B20_PIN 14
#define RELAY_PIN 13
#define HUMIDITY_SENSOR_PIN 34
#define SIM_RX 16
#define SIM_TX 17
#define SERVO_PIN 12
#define MQ135_PIN 35 // Chân GPIO cho MQ135 (analog)
#define ZH03B_RX 18  // Chân RX cho ZH03B
#define ZH03B_TX 19  // Chân TX cho ZH03B
#define LED1_PIN 2   // relay for led 2

#define HOST "mmsso.com"
#define SERVO_PERIOD 5000

// Khởi tạo đối tượng
Adafruit_AHTX0 aht;
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
HardwareSerial simSerial(2);
HardwareSerial zh03bSerial(1);
Servo servo;
MQUnifiedsensor MQ135("ESP32", 3.3, 12, MQ135_PIN, "MQ-135");
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// Biến toàn cục
float ahtTemp = 0.0, ahtHumidity = 0.0, dsTemp = 0.0, capHumidity = 0.0, co2PPM = 0.0, pm25 = 0.0, pm10 = 0.0;
int led1State = 0;
SemaphoreHandle_t dataMutex;

// Task đọc cảm biến
void sensorTask(void *pvParameters)
{
  while (1)
  {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    ds18b20.requestTemperatures();
    float dsTempLocal = ds18b20.getTempCByIndex(0);
    int rawHumidity = analogRead(HUMIDITY_SENSOR_PIN);
    float capHumidityLocal = map(rawHumidity, 0, 4095, 0, 100);
    uint16_t pm25Raw = 0, pm10Raw = 0;

    MQ135.update();
    float co2PPM_local = MQ135.readSensor();

    if (zh03bSerial.available() >= 9)
    { // ZH03B gửi gói 9 byte
      uint8_t buffer[9];
      zh03bSerial.readBytes(buffer, 9);
      if (buffer[0] == 0xFF) // Kiểm tra byte đầu
      {
        pm25Raw = (buffer[2] << 8) | buffer[3]; // PM2.5 (µg/m³)
        pm10Raw = (buffer[6] << 8) | buffer[7]; // PM10 (µg/m³)
      }
    }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      ahtTemp = temp.temperature;
      ahtHumidity = humidity.relative_humidity;
      dsTemp = dsTempLocal;
      capHumidity = capHumidityLocal;
      co2PPM = co2PPM_local < 0 ? 0 : co2PPM_local;
      pm25 = pm25Raw / 10.0; // Chuyển sang µg/m³
      pm10 = pm10Raw / 10.0; // Chuyển sang µg/m³
      xSemaphoreGive(dataMutex);
    }

    // Hiển thị lên OLED
    char buffer[32];
    u8g2.clearBuffer(); // Xóa buffer

    snprintf(buffer, sizeof(buffer), "T1:%.1fC H1:%.1f%%", ahtTemp, ahtHumidity);
    u8g2.drawStr(0, 10, buffer); // Dòng 1: Nhiệt độ AHT20, độ ẩm

    snprintf(buffer, sizeof(buffer), "T2:%.1fC H2:%.0f", dsTemp, capHumidity);
    u8g2.drawStr(0, 20, buffer); // Dòng 2: Nhiệt độ DS18B20, CO2

    snprintf(buffer, sizeof(buffer), "PM2.5:%.0f PM10:%.0f", pm25, pm10);
    u8g2.drawStr(0, 30, buffer); // Dòng 3: PM2.5, PM10

    u8g2.sendBuffer(); // Gửi dữ liệu lên OLED

    Serial.printf("Nhiệt độ AHT20: %.2f °C, Độ ẩm AHT20: %.2f %%\n", ahtTemp, ahtHumidity);
    Serial.printf("Nhiệt độ DS18B20: %.2f °C, Độ ẩm điện dung: %.2f %%\n", dsTemp, capHumidity);
    Serial.printf("CO2 (MQ135): %.2f PPM\n", co2PPM);
    Serial.printf("PM2.5 (ZH03B): %.2f µg/m³, PM10 (ZH03B): %.2f µg/m³\n", pm25, pm10);

    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Task điều khiển relay
void relayTask(void *pvParameters)
{
  while (1)
  {
    float localTemp;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      localTemp = ahtTemp;
      xSemaphoreGive(dataMutex);
    }

    if (localTemp > 30.0)
    {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay ON - Động cơ chạy");

      for (int pos = 45; pos <= 135; pos++)
      {
        servo.write(pos);
        vTaskDelay(pdMS_TO_TICKS(SERVO_PERIOD / 90));
      }
      for (int pos = 135; pos >= 45; pos--)
      {
        servo.write(pos);
        vTaskDelay(pdMS_TO_TICKS(SERVO_PERIOD / 90));
      }
    }
    else
    {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay OFF - Động cơ dừng");
      servo.write(90);
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// Task gửi dữ liệu qua 4G
void simTask(void *pvParameters)
{
  while (1)
  {
    float temp1, humid1, temp2, humid2, co2, pm25Val, pm10Val;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      temp1 = ahtTemp;
      humid1 = ahtHumidity;
      temp2 = dsTemp;
      humid2 = capHumidity;
      co2 = co2PPM;
      pm25Val = pm25;
      pm10Val = pm10;
      xSemaphoreGive(dataMutex);
    }

    String data = "GET /update?temp1=" + String(temp1) +
                  "&humid1=" + String(humid1) +
                  "&temp2=" + String(temp2) +
                  "&humid2=" + String(humid2) +
                  "&co2=" + String(co2) +
                  "&pm25=" + String(pm25Val) +
                  "&pm10=" + String(pm10Val) +
                  " HTTP/1.1\r\nHost: " HOST "\r\n\r\n";

    simSerial.println("AT+HTTPINIT");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPPARA=\"URL\",\"" HOST "\"");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPDATA=" + String(data.length()) + ",10000");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.print(data);
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPACTION=0");
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (simSerial.available())
    {
      Serial.println("Phản hồi từ server: " + simSerial.readString());
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

// Task read data from server and control led
void serverReadTask(void *pvParameters)
{
  while (1)
  {
    // Chuẩn bị yêu cầu GET cho /led-states
    String request = "GET /led-states HTTP/1.1\r\nHost: " HOST "\r\n\r\n";

    // Gửi yêu cầu qua SIM
    simSerial.println("AT+HTTPINIT");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPPARA=\"URL\",\"" HOST "/led-states\"");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPDATA=" + String(request.length()) + ",10000");
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.print(request);
    vTaskDelay(pdMS_TO_TICKS(1000));
    simSerial.println("AT+HTTPACTION=0");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Đọc phản hồi từ server
    String response = "";
    while (simSerial.available())
    {
      response += simSerial.readString();
    }

    // Xử lý JSON
    if (response.length() > 0)
    {
      // Tìm phần body JSON trong phản hồi
      int jsonStart = response.indexOf("{");
      if (jsonStart != -1)
      {
        String jsonData = response.substring(jsonStart);

        JsonDocument doc; // Buffer 200 byte cho JSON
        DeserializationError error = deserializeJson(doc, jsonData);

        if (!error)
        {
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
          {
            led1State = doc["LED1"];
            xSemaphoreGive(dataMutex);
          }
          Serial.printf("LED States - LED1: %d, LED2: %d, LED3: %d\n", led1State);
          digitalWrite(LED1_PIN, led1State);
        }
        else
        {
          Serial.println("Lỗi phân tích JSON: " + String(error.c_str()));
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(15000)); // Đọc từ server mỗi 15 giây
  }
}

// Hàm khởi tạo module SIM
void initSIMModule()
{
  simSerial.println("AT");
  vTaskDelay(pdMS_TO_TICKS(1000));
  if (simSerial.available())
  {
    Serial.println("Module SIM đã phản hồi: " + simSerial.readString());
  }
  simSerial.println("AT+CPIN?");
  vTaskDelay(pdMS_TO_TICKS(1000));
  if (simSerial.available())
  {
    Serial.println("Trạng thái SIM: " + simSerial.readString());
  }
}

void setup()
{
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  zh03bSerial.begin(9600, SERIAL_8N1, ZH03B_RX, ZH03B_TX);
  Wire.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);

  if (!aht.begin())
  {
    Serial.println("Không tìm thấy AHT20!");
  }
  ds18b20.begin();
  initSIMModule();

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);            // Standard 50hz servo
  servo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
                                       // using SG90 servo min/max of 500us and 2400us
                                       // for MG995 large servo, use 1000us and 2000us,
                                       // which are the defaults, so this line could be
                                       // "myservo.attach(servoPin);"

  MQ135.setRegressionMethod(1); // Sử dụng phương pháp hồi quy tuyến tính
  MQ135.setA(110.47);           // Hệ số A cho CO2 (theo datasheet)
  MQ135.setB(-2.862);           // Hệ số B cho CO2 (theo datasheet)
  MQ135.init();                 // Khởi tạo cảm biến

  // Khởi tạo OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Chọn font chữ nhỏ
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "Starting...");
  u8g2.sendBuffer();

  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(relayTask, "RelayTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(simTask, "SIMTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(serverReadTask, "ServerReadTask", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000)); // Không cần logic trong loop
}