#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <Tone32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
//  Anemometer demo
//  The example demonstrates the use of anemometer.
//  When the wind speed transmitter makes one round in one second,
//  the transmitter outputs 20 pulses, which means that the wind speed
//  is 1.75m/S.
//
const char *ssid = "Cempaka_218B@unifi";
const char *password = "Unisel@218B";
const char *mqtt_server = "192.168.1.109";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
// #define WIFI_SSID "4GMIFI_1022"                     // Put your WifiSSID here
// #define WIFI_PASS "1234567890"                      // Put your wifi password here
// #define TOKEN "BBFF-ovHhJ8LazCmYHu8RKJhPI7ysL4UtE6" // Put your Ubidots' TOKEN
// #define DEVICE_LABEL "ESP32"                        // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string;

// #define VARIABLE_LABEL1 "Wind"        // Assing the variable label
// #define VARIABLE_LABEL2 "Temperature" // Assing the variable label
// #define VARIABLE_LABEL3 "Humidity"    // Assing the variable label

#define DHTPIN 4
#define DHTTYPE DHT11

#define BUZZER_PIN 16
#define BUZZER_CHANNEL 0

#define SEALEVELPRESSURE_HPA (1013.25)
#define RAIN_SENSOR 19

int lcdColumns = 16;
int lcdRows = 2;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP280 bmp; // I2C

float t, h, p, alt, f, wnd;
unsigned int rainfall;
//// the following variables are unsigned longs because the time, measured in
//// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 2000; // the debounce time; increase if the output flickers

int pinInterrupt = 13; // esp8266 D7 pin
float speed_value = 0;

int Count = 0;
void publishData(float t_c, float t_f, float h, float p, float alt, float w, int r);

ICACHE_RAM_ATTR void onChange()
{
  if (digitalRead(pinInterrupt) == LOW)
    Count++;
}

// setup wifi
void setup_wifi()
{
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// mqtt callback
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// mqtt reconnect
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attemping MQTT connection...");
    //    Create random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    //    Attemp to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // subscribe topic
      client.subscribe("ESP32Weather/temperature/celcius");
      client.subscribe("ESP32Weather/temperature/fahrenheit");
      client.subscribe("ESP32Weather/humidity");
      client.subscribe("ESP32Weather/pressure");
      client.subscribe("ESP32Weather/altitude");
      client.subscribe("ESP32Weather/wind");
      client.subscribe("ESP32Weather/rain");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void setup()
{
  pinMode(RAIN_SENSOR, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  lcd.init();
  //  lcd.begin();
  // turn on LCD backlight
  lcd.backlight();
  Serial.begin(115200); // Initialize serial port
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  // subscribe topic
  client.subscribe("ESP32Weather/temperature/celcius");
  client.subscribe("ESP32Weather/temperature/fahrenheit");
  client.subscribe("ESP32Weather/humidity");
  client.subscribe("ESP32Weather/pressure");
  client.subscribe("ESP32Weather/altitude");
  client.subscribe("ESP32Weather/wind");
  client.subscribe("ESP32Weather/rain");
  Serial.println(F("BMP280 test"));
  unsigned status;
  // status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println();
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(pinInterrupt, INPUT_PULLUP); // set the interrupt pin
  dht.begin();

  // Enable
  attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, CHANGE);
}

void loop()
{

  //  lastDebounceTime = millis();
  //        //0.1m/s
  //        s_v.speed = (int)(Count * 8.75 * 0.01 * 10);
  //        // SerialUSB.print(s_v.speed);
  //        Count = 0;
  //        // SerialUSB.println("m/s");
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    lastDebounceTime = millis();
    //    rainfall = map(analogRead(RAIN_SENSOR), 780, 0, 0, 100);
    rainfall = digitalRead(RAIN_SENSOR);
    speed_value = Count * 8.75 * 0.01 * 10;
    //    Serial.print(speed_value);
    //    printValues();
    Count = 0;
    //    Serial.println(" m/s");
    // set cursor to first column, first row

    t = dht.readTemperature();
    f = dht.readTemperature(true); // convert to fahrenheit
    h = dht.readHumidity();
    p = bmp.readPressure() / 100.0F;
    alt = bmp.readAltitude(1013.25);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("Wind: " + (String)speed_value + " m/s  ");

    delay(1000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("H: " + (String)h + " %");
    lcd.setCursor(0, 1);
    lcd.print("T: " + (String)t + " C");

    publishData(t, f, h, p, alt, speed_value, rainfall);

    if (speed_value > 20.00)
    {
      tone(BUZZER_PIN, NOTE_A4, 1000, BUZZER_CHANNEL);
      noTone(BUZZER_PIN, BUZZER_CHANNEL);
      //      tone(BUZZER, 1000); // Send 1KHz sound signal...
      //      delay(500);        // ...for 1 sec
      //      noTone(BUZZER);     // Stop sound...
      //      delay(1000);
    }
    else
    {
    }
    //    delay(3000);
  }

  delay(1);
}
void publishData(float t_c, float t_f, float h, float p, float alt, float w, int r)
{
  StaticJsonDocument<32> doc;
  char output[55];
  //  assign variable
  String rf;
  static char temperatureC[7];
  static char temperatureF[7];
  static char humid[8];
  static char pressure[7];
  static char altitude[7];
  static char wind[7];
  static char rain[10];
  // Convert Float values to String (in Character Array format)
  dtostrf(t_c, 6, 2, temperatureC);
  dtostrf(t_f, 6, 2, temperatureF);
  dtostrf(h, 6, 2, humid);
  dtostrf(p, 6, 2, pressure);
  dtostrf(alt, 6, 2, altitude);
  dtostrf(w, 6, 2, wind);
  // convert rainfall
  if (r == 1)
  {
    rf = "No Rain";
  }
  else
  {
    rf = "Rain";
  }
  // Publish the sensor data on their particular MQTT topics
  client.publish("ESP32Weather/temperature/celcius", temperatureC);
  client.publish("ESP32Weather/temperature/fahrenheit", temperatureF);
  client.publish("ESP32Weather/humidity", String(h).c_str());
  client.publish("ESP32Weather/pressure", pressure);
  client.publish("ESP32Weather/altitude", altitude);
  client.publish("ESP32Weather/wind", wind);
  client.publish("ESP32Weather/rain", rf.c_str());
  // Publish json data to mqtt
  doc["t"] = temperatureC;
  doc["p"] = pressure;
  doc["h"] = h;
  doc["a"] = altitude;
  doc["w"] = wind;
  doc["r"] = rf;

  serializeJson(doc, output);
  Serial.println(output);
  client.publish("ESP32Weather/weather_sensors", output);
  // Printing the Sensor Values on Serial Monitor
  Serial.println("--------------------------------------------------------");
  Serial.println("Temperature: " + (String)t_c);
  Serial.println("Humidity: " + (String)h);
  Serial.println("Rain: " + rf);
  Serial.println("Pressure: " + (String)p);
  Serial.println("Altitude: " + (String)alt);
  Serial.println("Wind Speed: " + (String)w);
  Serial.println("--------------------------------------------------------");
}
