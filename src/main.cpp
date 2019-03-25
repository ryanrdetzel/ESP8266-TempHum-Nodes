#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_SHT31.h"
#include <DHTesp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "secrets.h"

//#define DHT
//#define SHT31
#define DS18b20

#ifdef DHT
DHTesp dht;
#endif

#ifdef SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();
#endif

#ifdef DS18b20
#define ONE_WIRE_BUS 4  // D2
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
#endif

WiFiClient espClient;
PubSubClient client(espClient);

const char* host = "temp_sensor_04";
const char* mqtt_clientid = host;
const char* topic = "sensor/temp_sensor_04";
const char* temperature_topic = "sensor/temp_sensor_04/temperature";
const char* humidity_topic = "sensor/temp_sensor_04/humidity";

const int wifi_retry_ms = 500;
const int mqtt_retry_ms = 150;
const int wait_between_messages = 30000; // milliseconds
const int wait_between_readings = 2000;

int time_since_last_read = 0;
int time_since_last_send = 0;

// Storage for each individual reading.
const int number_of_readings = wait_between_messages/wait_between_readings;
float temperature_readings[number_of_readings];
int temperature_readings_count = 0;

void setup_wifi() {
  Serial.println();
  Serial.println("Connecting to wifi");

  WiFi.hostname(host);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(wifi_retry_ms);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_mqtt() {
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqtt_clientid, mqtt_user, mqtt_password)) {
      Serial.println("MQTT connected");
      client.publish(topic, "online");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(mqtt_retry_ms);
    }
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  setup_wifi();
  setup_mqtt();

  #ifdef DHT
    dht.setup(13, DHTesp::DHT22);
  #endif

  #ifdef SHT31
  if (!sht31.begin(0x44)) {
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
  #endif

  #ifdef DS18b20
    sensors.begin();
  #endif

  for (int x=0;x<number_of_readings;x++)
    temperature_readings[x] = 0;
}

float c2f(float temperature){
  return temperature * 1.8 + 32;
}

void get_sensor_temperature(){
  #ifdef DHT
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  float temperature_f = c2f(temperature);
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  #endif

  #ifdef SHT31
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();
  float temperature_f = c2f(temperature);
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from SHT31 sensor!");
    return;
  }
  #endif

  #ifdef DS18b20
    sensors.requestTemperatures();
    float temperature = sensors.getTempCByIndex(0);
    float temperature_f = c2f(temperature);
  #endif

  #if defined DHT || defined SHT31 || defined DS18b20
  temperature_readings[temperature_readings_count++] = temperature_f;
  if (temperature_readings_count >= number_of_readings){
    temperature_readings_count = 0;
    for (int x=0;x<number_of_readings;x++)
      temperature_readings[x] = 0;
  }
  #endif

  // #if defined DHT || defined SHT31
  // Serial.print("humidity: "); Serial.println(humidity);
  // char humidity_str[16];
  // snprintf(humidity_str, sizeof(humidity_str), "%.2f", humidity);
  // client.publish(humidity_topic, humidity_str);
  // #endif
}


void loop() {

  if (time_since_last_read > wait_between_readings){
    get_sensor_temperature();
    time_since_last_read = 0;
  }

  if(time_since_last_send > wait_between_messages) {
    // Cacluate average from last sample set.
    #if defined DHT || defined SHT31 || defined DS18b20
      float temperature_total = 0;
      int count = 0;

      for (int x=0;x<number_of_readings;x++){
        if (temperature_readings[x] > 0){
          temperature_total += temperature_readings[x];
          count++;
        }
      }
      float temperature_f = temperature_total / count;

      Serial.print("Temp: "); Serial.println(temperature_f);
      char temperature_str[16];
      snprintf(temperature_str, sizeof(temperature_str), "%.2f", temperature_f);
      client.publish(temperature_topic, temperature_str);
    #endif

    time_since_last_send = 0;
  }

  delay(100); // 100 Const?

  time_since_last_read += 100;
  time_since_last_send += 100;

  if (!client.connected()) {
    setup_mqtt();
    delay(5000);
  }
  client.loop();
}