#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_SHT31.h"
#include <DHTesp.h>

#include "secrets.h"

//#define DHT
#define SHT31

#ifdef DHT
DHTesp dht; 
#endif

#ifdef SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();
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
const int wait_between_messages = 60000; // milliseconds

int time_since_last_read = wait_between_messages;

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

  #if defined DHT || defined SHT31
  Serial.print("Temp: "); Serial.println(temperature);

  char temperature_str[16];
  snprintf(temperature_str, sizeof(temperature_str), "%.2f", temperature_f);
  client.publish(temperature_topic, temperature_str);
  #endif

  #if defined DHT || defined SHT31
  Serial.print("humidity: "); Serial.println(humidity);
  char humidity_str[16];
  snprintf(humidity_str, sizeof(humidity_str), "%.2f", humidity);
  client.publish(humidity_topic, humidity_str);
  #endif
}


void loop() {
  if(time_since_last_read > wait_between_messages) {
    get_sensor_temperature();
    time_since_last_read = 0;
  }

  delay(100);
  time_since_last_read += 100;

  if (!client.connected()) {
    setup_mqtt();
    delay(5000);
  }
  client.loop();
}