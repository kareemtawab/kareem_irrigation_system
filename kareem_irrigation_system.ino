//#define BLYNK_PRINT Serial
#define DHTPIN 2          // What digital pin we're connected to
#define MOISTPIN A0
#define MOIST_ENABLE_PIN 10
#define R1 0
#define R2 16
// Uncomment whatever type you're using!
#define DHTTYPE DHT11     // DHT 11
//#define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
//#define DHTTYPE DHT21   // DHT 21, AM2301
#define CE_PIN 3 // for nrf24l01
#define CSN_PIN 15 // for nrf24l01

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
extern "C" {
#include <user_interface.h>  // https://github.com/esp8266/Arduino
#include <ping.h>            // https://github.com/esp8266/Arduino
}
#include <SPI.h>
#include "RF24.h"
#include "PL1167_nRF24.h"
#include "MiLightRadio.h"
#include "Timer.h"

// For the BH1750 sensor,
// VCC – Nodemcu 3.3v
// GND – Nodemcu Gnd
// SCL – Nodemcu D1
// SDA – Nodemcu D2

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "907634cb2a944ef7a895b8c902e1a350";

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "HK Xperia Z5 Dual";
//char pass[] = "07081989";
float L = 0.0f;
float M = 0.0f;
int H = 0.0f;
int T = 0.0f;
long S;
long uptime;
int ping_time;
boolean R1state;
boolean R2state;

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

LiquidCrystal_I2C lcd(0x27, 20, 4);

DHT dht(DHTPIN, DHTTYPE, 2);

BH1750 lightMeter;

//BlynkTimer timer;
Timer t;

WiFiManager wifiManager;

RF24 radio(CE_PIN, CSN_PIN);
PL1167_nRF24 prf(radio);
MiLightRadio mlr(prf);
int ControlIDone = 0xB0;
int ControlIDtwo = 0xA6;
int ControlIDthree = 0xE2;
int sequence = 0xEC;
uint8_t outgoingPacket [7] = { ControlIDone, ControlIDtwo, ControlIDthree, 0x00, 0x90, 0x0F, sequence++};
int color;
int brightness = 0x90;
int r;
int g;
int b;

struct ping_option pingOpt, pOpt;

void pingRecv(void *arg, void *pdata) {
  struct ping_resp *pingResp = (struct  ping_resp *)pdata;
  if (pingResp->ping_err == -1) {
    Serial.println("No Pong (device offline)");
  }
  else {
    Serial.print("ping recv: bytes = ");
    Serial.print(pingResp->bytes);
    Serial.print(", time = ");
    ping_time = (*pingResp).resp_time;
    Serial.print(ping_time);
    Serial.println("ms");
  }
}

void doPing(const char *targetIpAddress) {   // init and start Ping
  const uint8_t pingCount = 1;     // number off Ping repetition
  const uint8_t pingInterval = 1;  // Ping repetition every n sec
  struct ping_option *pingOpt = &pOpt;
  pingOpt->count = pingCount;
  pingOpt->coarse_time = pingInterval;
  pingOpt->ip = ipaddr_addr(targetIpAddress);
  ping_regist_recv(pingOpt, pingRecv);  // Pong callback function 'pingRecv'
  ping_start(pingOpt);  // start Ping
}

uint8_t rgb2milight(uint8_t red, uint8_t green, uint8_t blue) {
  float rd = (float) red / 255;
  float gd = (float) green / 255;
  float bd = (float) blue / 255;
  float maxi = _max(rd, _max(gd, bd));
  float mini = _min(rd, _min(gd, bd));
  float h = maxi;

  float d = maxi - mini;
  if (maxi == mini) {
    h = 0; // achromatic
  } else {
    if (maxi == rd) {
      h = (gd - bd) / d + (gd < bd ? 6 : 0);
    } else if (maxi == gd) {
      h = (bd - rd) / d + 2;
    } else if (maxi == bd) {
      h = (rd - gd) / d + 4;
    }
    h /= 6;
  }
  return (uint8_t) (int) ((h) * 256) + 26 % 256;
}

void set_color(uint8_t red, uint8_t green, uint8_t blue) {
  color = rgb2milight(red, green, blue);
  outgoingPacket[3] = color;
  outgoingPacket[4] = brightness;
  outgoingPacket[5] = 0x0F;
  send_milight();
}

void set_brightness(int b) {
  int brightnessi = map(b, 0, 100, 1, 28);
  int counter = 0;
  brightness = 0x90;
  counter++;
  if (brightnessi >= 1 && brightnessi <= 19) {
    brightness = 0x88 - (brightnessi * 0x08);
  }

  if (brightnessi > 19 && brightnessi <= 28) {
    brightness = 0xF8 - ((brightnessi - 20) * 0x08);
  }

  outgoingPacket[3] = color;
  outgoingPacket[4] = brightness;
  outgoingPacket[5] = 0x0E;
  send_milight();
}

void set_maxwhite() {
  outgoingPacket[3] = 0x00;
  outgoingPacket[4] = brightness;
  outgoingPacket[5] = 0x19;
  send_milight();
}

void send_milight() {
  delay(20);
  mlr.write(outgoingPacket, sizeof(outgoingPacket));
  for (int resendcounter = 0; resendcounter < 50; resendcounter++)
  {
    mlr.resend();
    delay(1);
  }
}

void sendSensor() {
  // set_brightness(brightness);

  doPing("8.8.8.8"); // ping www.google.com

  S = WiFi.RSSI();

  uptime = millis() / 1000;

  L = lightMeter.readLightLevel();
  L = L / 40000 * 100;

  digitalWrite (MOIST_ENABLE_PIN, HIGH);
  delay(200);
  int n = 10;
  for (int i = 0; i < n; i++) // read sensor "N" times and get the average
  {
    M += analogRead(MOISTPIN);
    delay(5);
  }
  digitalWrite (MOIST_ENABLE_PIN, LOW);
  M = M / n;
  M = mapfloat(M, 1024, 0, 0, 100);

  H = dht.readHumidity();
  //delay(50);
  T = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
  if (isnan(H) || isnan(T) || H > 99 || T > 99) {
    Serial.println("Failed to read from DHT sensor!");
    T = 0;
    H = 0;
  }
  else {
    Blynk.virtualWrite(V1, H);
    Blynk.virtualWrite(V0, T);
  }
  Blynk.virtualWrite(V2, L);
  Blynk.virtualWrite(V3, M);
  Blynk.virtualWrite(V4, ping_time);
  Blynk.virtualWrite(V5, S);
  Blynk.virtualWrite(V6, uptime);
  WidgetLED R1LED(V7);
  WidgetLED R2LED(V8);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.print(T);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Humi:");
  lcd.print(H);
  lcd.print("%");
  lcd.setCursor(0, 2);
  lcd.print("Moist:");
  lcd.print(M, 1);
  lcd.print("%");
  lcd.setCursor(9, 0);
  lcd.print("Ping:");
  lcd.print(ping_time);
  lcd.print("ms");
  lcd.setCursor(9, 1);
  lcd.print("Light:");
  lcd.print(L, 1);
  lcd.print("%");

  if (L < 50 && M < 35) {
    lcd.noBacklight();
    delay(250);
    lcd.backlight();
    lcd.setCursor(12, 2);
    lcd.print("<--LOW!");
  }
  if (L < 50 && M > 35) {
    lcd.backlight();
  }
  if (L > 50) {
    lcd.noBacklight();
  }

  if (R1state) {
    digitalWrite(R1, LOW);
    lcd.setCursor(16, 3);
    lcd.print("R1");
    R1LED.on();
  }
  else {
    digitalWrite(R1, HIGH);
    lcd.setCursor(16, 3);
    lcd.print("  ");
    R1LED.off();
  }
  if (R2state) {
    digitalWrite(R2, LOW);
    lcd.setCursor(18, 3);
    lcd.print("R2");
    R2LED.on();
  }
  else {
    digitalWrite(R2, HIGH);
    lcd.setCursor(18, 3);
    lcd.print("  ");
    R2LED.off();
  }
}

BLYNK_WRITE(V20)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  if (pinValue == 1) {
    ESP.restart();
  }
}

BLYNK_WRITE(V9)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  if (pinValue == 1) {
    R1state = true;
  }
  else {
    R1state = false;
  }
}

BLYNK_WRITE(V10)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  if (pinValue == 1) {
    R2state = true;
  }
  else {
    R2state = false;
  }
}

BLYNK_WRITE(V11)
{
  r = param[0].asInt(); // assigning incoming value from pin V1 to a variable
  g = param[1].asInt(); // assigning incoming value from pin V1 to a variable
  b = param[2].asInt(); // assigning incoming value from pin V1 to a variable
  if (r == 255 && b == 255 && g == 255) {
    set_maxwhite();
    delay(5);
    set_brightness(brightness);
  }
  else {
    set_color(r, g, b);
  }
}

BLYNK_WRITE(V12)
{
  brightness = param.asInt(); // assigning incoming value from pin V1 to a variable
  set_brightness(brightness);
}

void setup()
{
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(MOIST_ENABLE_PIN, OUTPUT);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, HIGH);
  digitalWrite(MOIST_ENABLE_PIN, LOW);
  EEPROM.begin(512);
  Serial.begin(115200);
  Serial.println();
  Serial.println("HK NodeMCU Weather Station, Irrigation and RGB Milight Controller");
  lcd.init();
  lcd.backlight();
  lcd.clear();
  dht.begin();
  lightMeter.begin();
  mlr.begin();

  // Setup a function to be called every second
  //timer.setInterval(2000L, sendSensor);
  t.every(2000, sendSensor);

  lcd.setCursor(5, 0);
  lcd.print("HK NodeMCU");
  lcd.setCursor(2, 1);
  lcd.print("Weather Station,");
  lcd.setCursor(1, 2);
  lcd.print("Irrigation and RGB");
  lcd.setCursor(1, 3);
  lcd.print("Milight Controller");
  //wifiManager.resetSettings();    //Uncomment this to wipe WiFi settings from EEPROM on boot.  Comment out and recompile/upload after 1 boot cycle.
  delay(10000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting");
  lcd.setCursor(0, 1);
  lcd.print("to pre-configured AP");
  lcd.setCursor(0, 2);
  lcd.print(WiFi.SSID().c_str());
  lcd.setCursor(0, 3);
  lcd.print("(1 minute timeout)");
  wifiManager.setConnectTimeout(60);
  delay(60 * 1000);
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Timeout! Failed to");
    lcd.setCursor(0, 1);
    lcd.print("connect to");
    lcd.setCursor(0, 2);
    lcd.print("pre-configured AP");
    lcd.setCursor(0, 3);
    lcd.print(WiFi.SSID().c_str());
    delay(5000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Configuration mode");
    lcd.setCursor(0, 1);
    lcd.print("SSID: HK_NodeMCU");
    lcd.setCursor(0, 2);
    lcd.print("IP: 192.168.4.1");
    lcd.setCursor(0, 3);
    lcd.print("(3 minutes timeout)");
    wifiManager.setConfigPortalTimeout(180);
    wifiManager.startConfigPortal("HK_NodeMCU");
  }
  if (WiFi.status() == WL_CONNECTED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Connected to");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.SSID().c_str());
    delay(5000);
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Configuration mode");
    lcd.setCursor(0, 1);
    lcd.print("timeout.....");
    lcd.setCursor(0, 3);
    lcd.print("System is rebooting");
    delay(10000);
    ESP.restart();
  }
  //if you get here you have connected to the WiFi
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print("Blynk server.....");

  Blynk.config(auth);
  Blynk.connect();
  Blynk.notify("Device started!");
}

void loop() {
  //timer.run();
    t.update();
  if (WiFi.status() != WL_CONNECTED || (WiFi.status() == WL_CONNECTED && Blynk.connected() != 1)) {
    R1state = false;
    if (WiFi.status() != WL_CONNECTED) {
      lcd.setCursor(0, 3);
      lcd.print("RECONNECTING TO AP! ");
    }
    else {
      lcd.setCursor(0, 3);
      lcd.print("BLYNK DISCONNECTED! ");
      Blynk.connect();
      Blynk.notify("Device reconnected to Blynk!");
    }
  }
  else {
    Blynk.run();
    lcd.setCursor(0, 3);
    lcd.print("BLYNK CONNECTED OK! ");
  }
}


