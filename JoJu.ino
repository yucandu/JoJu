#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include <Adafruit_ADS1X15.h>
#include <FastLED.h>
#include <Preferences.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA219.h>

#define ONE_WIRE_BUS 1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);



Adafruit_INA219 ina219;

Preferences preferences;

#define LED_PIN 2

#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

Adafruit_ADS1115 ads;


const char* ssid = "mikesnet";
const char* password = "springchicken";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -14400;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (secs)
int hours, mins, secs;
int zebraR, zebraG, zebraB, menuValue;
int sliderValue = 255;

float tempSHT, humSHT;
int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;
float wifi;

bool buttonstart = false;
bool ledon = false;
bool needssaving = false;

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

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


BLYNK_WRITE(V12)
{
   menuValue = param.asInt(); // assigning incoming value from pin V1 to a variable
}

BLYNK_WRITE(V13)
{
   sliderValue = param.asInt(); // assigning incoming value from pin V1 to a variable
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

void goToSleep(){
    //esp_deep_sleep_enable_gpio_wakeup(1, ESP_GPIO_WAKEUP_GPIO_LOW);
    //WiFi.disconnect();
    //delay(1);
    esp_sleep_enable_timer_wakeup(55000000); // 50 sec
    esp_deep_sleep_start(); 
    delay(1000);
}

void setup(void) {
  setCpuFrequencyMhz(80);
  delay(2);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  sensors.begin();
  sensors.requestTemperatures(); 
  tempSHT = sensors.getTempCByIndex(0);
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();
  adc2 = ads.readADC_SingleEnded(0);
  adc3 = ads.readADC_SingleEnded(1);
  volts2 = ads.computeVolts(adc2)*2.0;
  volts3 = ads.computeVolts(adc3)*2.0;
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);


    
  while ((WiFi.status() != WL_CONNECTED) && (millis() < 15000)) {
      delay(250);
  }
  wifi = WiFi.RSSI();
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  while ((!Blynk.connected()) && (millis() < 15000)){delay(250);}
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  


  Blynk.virtualWrite(V1, tempSHT);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  //Blynk.virtualWrite(V2, humSHT);
  //if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V3, volts2);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V4, volts3);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V5, wifi);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V21, shuntvoltage);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V22, busvoltage);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V23, current_mA);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V24, power_mW);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V25, loadvoltage);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  Blynk.virtualWrite(V25, loadvoltage);
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();} 
   
  
  
  
  if (buttonstart) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    terminal.println("***JoJu 1.3 STARTED***");
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    printLocalTime();
    terminal.println(volts3,3);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    terminal.println("HTTP server started");
    terminal.flush();
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  }
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  //delay(500);
  if (!buttonstart){
    pinMode(LED_PIN, INPUT);
    goToSleep();

  }




}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}

  every(5000){
    sensors.requestTemperatures(); 
    tempSHT = sensors.getTempCByIndex(0);
    adc2 = ads.readADC_SingleEnded(0);
    adc3 = ads.readADC_SingleEnded(1);
    volts2 = ads.computeVolts(adc2)*2.0;
    volts3 = ads.computeVolts(adc3)*2.0;
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    Blynk.virtualWrite(V1, tempSHT);
    //Blynk.virtualWrite(V2, humSHT);
    Blynk.virtualWrite(V3, volts2);
    Blynk.virtualWrite(V4, volts3);
    Blynk.virtualWrite(V5, WiFi.RSSI());
    Blynk.virtualWrite(V21, shuntvoltage);
    Blynk.virtualWrite(V22, busvoltage);
    Blynk.virtualWrite(V23, current_mA);
    Blynk.virtualWrite(V24, power_mW);
    Blynk.virtualWrite(V25, loadvoltage);
    
  }


    every(10){
      leds[0] = CRGB(zebraR, zebraG, zebraB);
      FastLED.setBrightness(sliderValue);
      FastLED.show();
    }

}
