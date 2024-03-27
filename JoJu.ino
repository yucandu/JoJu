#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "Adafruit_SHT31.h"
#include <Adafruit_ADS1X15.h>
#include <FastLED.h>

#define BUTTON_PIN 0
#define LED_PIN 2  

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

Adafruit_ADS1115 ads;

Adafruit_SHT31 sht31 = Adafruit_SHT31();

const char* ssid = "mikesnet";
const char* password = "springchicken";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -14400;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;
int zebraR, zebraG, zebraB;

float tempSHT, humSHT;
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;

bool buttonstart = false;
bool ledon = false;

char auth[] = "fEZlZOio7CS1nBXNm3A8HR5DysrzIoYW";

AsyncWebServer server(80);

WidgetTerminal terminal(V10);

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

BLYNK_WRITE(V10) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("reset");
    terminal.println("ledon");
    terminal.println("ledoff");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
  if (String("reset") == param.asStr()) {
    terminal.println("Restarting...");
    terminal.flush();
    ESP.restart();
  }
  if (String("ledon") == param.asStr()) {
    terminal.println("Turning LED on.");
    terminal.flush();
    ledon = true;
  }
  if (String("ledoff") == param.asStr()) {
    terminal.println("Turning LED on.");
    terminal.flush();
    ledon = false;
  }
}

BLYNK_WRITE(V18)
{
     zebraR = param[0].asInt();
     zebraG = param[1].asInt();
     zebraB = param[2].asInt();
}

BLYNK_WRITE(V11)
{
  if (param.asInt() == 1) {buttonstart = true;}
  if (param.asInt() == 0) {buttonstart = false;}
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V11);
}

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}

void setup(void) {
  setCpuFrequencyMhz(80);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP); //BUTTON PIN
  delay(2);
  if (digitalRead(BUTTON_PIN) == LOW){ buttonstart = true;}
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  if (buttonstart){
    while (WiFi.status() != WL_CONNECTED) {
      leds[0] = CRGB(3, 3, 0);
      FastLED.show();
      delay(250);
      leds[0] = CRGB(0, 0, 0);
      FastLED.show();
      delay(250);
      Serial.print(".");
    }
  }
  else {
      while (WiFi.status() != WL_CONNECTED) {
    }
  }
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  while (!Blynk.connected()){}

  

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (buttonstart) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  getLocalTime(&timeinfo);
  hours = timeinfo.tm_hour;
  mins = timeinfo.tm_min;
  secs = timeinfo.tm_sec;
  sht31.begin(0x44);
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();




  tempSHT = sht31.readTemperature();
  humSHT = sht31.readHumidity();
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  volts2 = ads.computeVolts(adc2)*2.0;
  volts3 = ads.computeVolts(adc3)*2.0;
  Blynk.virtualWrite(V1, tempSHT);
  Blynk.virtualWrite(V2, humSHT);
  Blynk.virtualWrite(V3, volts2);
  Blynk.virtualWrite(V4, volts3);
  Blynk.virtualWrite(V5, WiFi.RSSI());
  if (buttonstart){
    terminal.println("***JoJu 1.1 STARTED***");
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    printLocalTime();
    terminal.println(volts3,3);
    terminal.flush();
  }
  Blynk.run();

  if (!buttonstart){
    esp_deep_sleep_enable_gpio_wakeup(1, ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_sleep_enable_timer_wakeup(55000000); // 55 sec
    esp_deep_sleep_start(); 
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  every(10000){
    tempSHT = sht31.readTemperature();
    humSHT = sht31.readHumidity();
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);
    volts2 = ads.computeVolts(adc2)*2.0;
    volts3 = ads.computeVolts(adc3)*2.0;
    Blynk.virtualWrite(V1, tempSHT);
    Blynk.virtualWrite(V2, humSHT);
    Blynk.virtualWrite(V3, volts2);
    Blynk.virtualWrite(V4, volts3);
    Blynk.virtualWrite(V5, WiFi.RSSI());
    
  }

  if (ledon){
    every(10){
      leds[0] = CRGB(zebraR, zebraG, zebraB);
      FastLED.show();
    }
  }

}
