#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>

// Định nghĩa chân
#define DS18B20_PIN 22
#define RELAY_PIN 23
#define HUMIDITY_SENSOR_PIN 34
#define SIM_RX 26
#define SIM_TX 27

#define HOST "example.com"

// Khởi tạo đối tượng
Adafruit_AHTX0 aht;
OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
HardwareSerial simSerial(2);

// Biến toàn cục
float ahtTemp = 0.0, ahtHumidity = 0.0, dsTemp = 0.0, capHumidity = 0.0;
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

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      ahtTemp = temp.temperature;
      ahtHumidity = humidity.relative_humidity;
      dsTemp = dsTempLocal;
      capHumidity = capHumidityLocal;
      xSemaphoreGive(dataMutex);
    }

    Serial.printf("Nhiệt độ AHT20: %.2f °C, Độ ẩm AHT20: %.2f %%\n", ahtTemp, ahtHumidity);
    Serial.printf("Nhiệt độ DS18B20: %.2f °C, Độ ẩm điện dung: %.2f %%\n", dsTemp, capHumidity);

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
    }
    else
    {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay OFF - Động cơ dừng");
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// Task gửi dữ liệu qua 4G
void simTask(void *pvParameters)
{
  while (1)
  {
    float temp1, humid1, temp2, humid2;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      temp1 = ahtTemp;
      humid1 = ahtHumidity;
      temp2 = dsTemp;
      humid2 = capHumidity;
      xSemaphoreGive(dataMutex);
    }

    String data = "GET /update?temp1=" + String(temp1) +
                  "&humid1=" + String(humid1) +
                  "&temp2=" + String(temp2) +
                  "&humid2=" + String(humid2) +
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
  Wire.begin();
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  if (!aht.begin())
  {
    Serial.println("Không tìm thấy AHT20!");
    while (1)
      ;
  }
  ds18b20.begin();
  initSIMModule();

  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(relayTask, "RelayTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(simTask, "SIMTask", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000)); // Không cần logic trong loop
}