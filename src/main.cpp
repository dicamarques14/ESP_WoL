#include <Arduino.h>
#include <WiFiUdp.h>
#include <WakeOnLan.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include "htmls.h"

#ifdef ESP8266
#include "ESP8266WiFi.h"
#include <ESP8266HTTPClient.h>
#include <ESPAsyncTCP.h>
#define getid ESP.getChipId()

#elif defined(ESP32)
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <analogWrite.h>
#include <ESP32Ping.h>
#define getid ESP.getEfuseMac()
#else
#error "ESP NOT DEFINED"
#endif

#include <jled.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <PubSubClient.h>

WiFiUDP UDP;
WakeOnLan WOL(UDP); // Pass WiFiUDP class
const char *ssid = "-";
const char *password = "-";
char hst[40] = {0};

const char *mqtt_server = "m20.cloudmqtt.com";
#define DHTPIN 5     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22 // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);
float t = 0.0;
float h = 0.0;
unsigned long previousMillis_publish_reading = 0; // will store last time DHT was updated
// Updates DHT readings every 10 seconds
const long interval_publish_reading = 15000;
#define MSG_BUFFER_SIZE (50)
char msg_mqtt[MSG_BUFFER_SIZE];

auto led = JLed(LED_BUILTIN).Breathe(5000).Forever();

// Replaces placeholder with DHT values
String processor(const String& var) {
    //Serial.println(var);
    if (var == "TEMPERATURE") {
        return String(t);
    }
    else if (var == "HUMIDITY") {
        return String(h);
    }
    return String();
}


void setup_ota()
{
  ArduinoOTA.setHostname(hst);
  ArduinoOTA.setPassword("dg");
  ArduinoOTA.onStart([]()
                     {
                       String type;
                       if (ArduinoOTA.getCommand() == U_FLASH)
                         type = "sketch";
                       else // U_SPIFFS
                         type = "filesystem";

                       // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                       Serial.println("Start updating " + type);
                     });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       Serial.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                     });
  ArduinoOTA.begin();
}


void sendPacket(String message)
{
  if (message == "diogo")
  {
    Serial.println("-----SendPacket-----");
    const char *MACAddress = "40:B0:76:44:F4:C0";
    WOL.sendMagicPacket(MACAddress);
  }
}

bool doPing(String message)
{
  bool ret = false;
  if (message == "diogo")
  {
    IPAddress ip(192, 168, 1, 150); // The remote ip to ping
    Serial.println("-----DoPing-----");
    ret = Ping.ping(ip, 2);
  }
  if (message == "fake")
  {
    IPAddress ip(192, 168, 1, 253); // The remote ip to ping
    Serial.println("-----DoPing-----");
    ret = Ping.ping(ip, 2);
    Serial.println("-----PingDo-----");
  }
  return ret;
}


void toggleLed(uint8_t ledState)
{
  digitalWrite(LED_BUILTIN, ledState);
}


void reconnectWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("====reconnectWifi====");
    sprintf(hst, "DG-WoL-%d", getid);
#ifdef ESP8266
    WiFi.hostname(hst);
#else

    IPAddress local_IP(192, 168, 1, 169);
    // Set your Gateway IP address
    IPAddress gateway(192, 168, 1, 254);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress primaryDNS(8, 8, 8, 8);   //optional
    IPAddress secondaryDNS(8, 8, 4, 4); //optional
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
      Serial.println("STA Failed to configure");
    }
    WiFi.setHostname(hst);
#endif
    Serial.print("HostName:");
    Serial.println(hst);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    uint8_t ledState = 1;
    while (WiFi.status() != WL_CONNECTED)
    {
      toggleLed(ledState);
      Serial.print(".");
      if (ledState)
        ledState = 0;
      else
        ledState = 1;
      delay(250);
    }
    toggleLed(0);
    Serial.println(".");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void notFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "Not found");
}

AsyncWebServer server(80);
void setupHttp()
{

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", "Hello, world"); });

  // Send a GET request to <IP>/wol?key=<message>
  server.on("/wol", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String message;
              if (request->hasParam("key"))
              {
                message = request->getParam("key")->value();
                sendPacket(message);
              }
              else
              {
                message = "No message sent";
              }
              request->send(200, "text/plain", "Hello, Waking UP: " + message);
            });

  // Send a GET request to <IP>/wol?key=<message>
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String message;
              bool success = false;
              if (request->hasParam("key"))
              {
                message = request->getParam("key")->value();
                success = doPing(message);
              }
              if (success)
              {
                request->send(200, "text/plain", "Ping OK");
              }
              else
              {
                request->send(200, "text/plain", "Ping NOK");
              }
            });

  server.on("/weather", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", temp_html, processor); });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", String(t).c_str()); });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/plain", String(h).c_str()); });
  server.onNotFound(notFound);

  server.begin();
}

void callback(char *topic, byte *payload, unsigned int length)
{
}

WiFiClient espClient;
PubSubClient mqclient(mqtt_server, 19265, callback, espClient);
void reconnectMqtt()
{
  // Loop until we're reconnected
  while (!mqclient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqclient.connect("esp32Viseu", "temp", "temp"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqclient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqclient.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttLoop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_publish_reading >= interval_publish_reading)
  {
    led.Stop();
    // save the last time you updated the DHT values
    previousMillis_publish_reading = currentMillis;
    // Read temperature as Celsius (the default)
    float newT = dht.readTemperature();
    // if temperature read failed, don't change t value
    if (isnan(newT))
    {
      Serial.println("Failed to read from DHT sensor!");
    }
    else
    {
      t = newT;
      Serial.println(t);
    }
    // Read Humidity
    float newH = dht.readHumidity();
    // if humidity read failed, don't change h value
    if (isnan(newH))
    {
      Serial.println("Failed to read from DHT sensor!");
    }
    else
    {
      h = newH;
      Serial.println(h);
    }
    if( !(isnan(newT)&&isnan(newH)) ){
      snprintf(msg_mqtt, MSG_BUFFER_SIZE, "Temp:%f, Humd:%f", t, h);
      Serial.print("Publish message: ");
      Serial.println(msg_mqtt);
      mqclient.publish("/sensor/viseu/", msg_mqtt);
    }
    led.Reset();
  }

  if (!mqclient.connected())
  {
    reconnectMqtt();
  }
  mqclient.loop();
}

void setup()
{
  Serial.begin(115200);
  delay(1000); // power-up safety delay
  Serial.println("\n\n====SETUP START====");
  pinMode(LED_BUILTIN, OUTPUT);

  reconnectWifi();
  setupHttp();
  WOL.setRepeat(3, 100); // Optional, repeat the packet three times with 100ms between. WARNING delay() is used between send packet function.

  //analogWriteResolution(LED_BUILTIN, 12);
  Serial.println("====SETUP DONE====\n");
}

int brightStep = 1;
int brightness = 0;
void loop()
{
  ArduinoOTA.handle();
  led.Update();
  reconnectWifi();
  mqttLoop();
  delay(1);
}
