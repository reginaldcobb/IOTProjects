#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>

#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>

#include "Adafruit_GFX.h"
#include <Adafruit_SSD1306.h>

#include "Adafruit_SHT4x.h"
#include <Adafruit_Sensor.h>

#include "Adafruit_TSL2591.h" // header file for Light Sensor

#include "init_mqtt.h"

#include "init_oled.h"

#include "ESP.h"

///////////   DEBUG SECTION  //////////

// #define DEBUG false
#define DEBUG true

unsigned long int loopCount = 0;

///////////   DEBUG SECTION  //////////

///////////   JSON SECTION  //////////

StaticJsonDocument<200> doc;

///////////   JSON SECTION  //////////

///////////   HISTORICAL VARIABLE SECTION  //////////

// volatile long old_tempF;
// volatile long old_tempC;
// volatile long old_hum;
// volatile int old_pirState;
// volatile uint16_t old_ir;
// volatile uint16_t old_full;
// volatile uint32_t old_lum;
// volatile int old_sensorValue;
// volatile int old_ambientValue;
// volatile int old_delta;
// volatile int old_maxSound;
// volatile int old_minSound;

///////////    HISTORICAL VARIABLE SECTION   //////////

///////////   WIFI INITIALIZATION SECTION  //////////

char message_buff[100];

// long lastMsg = 0;
// long lastRecu = 0;
// bool debug = false; //Display log message if True

///////////   WIFI INITIALIZATION SECTION  //////////

///////////   Temp/Hum Init SECTION  //////////

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

///////////   Temp/Hum Init SECTION  //////////

///////////   MQTT INITIALIZATION SECTION  //////////

WiFiClient espClient;
PubSubClient client(espClient);

unsigned int localPort = 2390; // local port to listen for UDP packets

IPAddress timeServerIP; // time.nist.gov NTP server address
const char *ntpServerName = "time.nist.gov";

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

///////////   MQTT INITIALIZATION SECTION  //////////

///////////   INIT FOR OLED  //////////

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
///////////   INIT FOR OLED  //////////

///////////   ESP32-CAM SECTION  //////////

int cameraPin = 14; // set pin to 14 to match D5 for the D1 mini Pro

///////////   ESP32-CAM SECTION  //////////

///////////   PIR Motion Init SECTION  //////////

int inputPin = 16; // set pin to 16 to match D0 for the D1 mini Pro
// int inputPin = 8;            // choose the input pin (for PIR sensor)
volatile int pirState = LOW; // we start, assuming no motion detected

volatile int val = 0; // variable for reading the pin status


///////////   PIR Motion Init SECTION  //////////

///////////   Light Sensor Init SECTION  //////////

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  if (DEBUG)
  {
    Serial.println(F("------------------------------------"));
    Serial.print(F("Sensor:       "));
    Serial.println(sensor.name);
    Serial.print(F("Driver Ver:   "));
    Serial.println(sensor.version);
    Serial.print(F("Unique ID:    "));
    Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:    "));
    Serial.print(sensor.max_value);
    Serial.println(F(" lux"));
    Serial.print(F("Min Value:    "));
    Serial.print(sensor.min_value);
    Serial.println(F(" lux"));
    Serial.print(F("Resolution:   "));
    Serial.print(sensor.resolution, 4);
    Serial.println(F(" lux"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
  }
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl.setGain(TSL2591_GAIN_MED); // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */
  if (DEBUG)
  {
    Serial.println(F("------------------------------------"));
    Serial.print(F("Gain:         "));
  }
  tsl2591Gain_t gain = tsl.getGain();
  switch (gain)
  {
  case TSL2591_GAIN_LOW:
    if (DEBUG)
    {
      Serial.println(F("1x (Low)"));
    }
    break;
  case TSL2591_GAIN_MED:
    if (DEBUG)
    {
      Serial.println(F("25x (Medium)"));
    }
    break;
  case TSL2591_GAIN_HIGH:
    if (DEBUG)
    {
      Serial.println(F("428x (High)"));
    }
    break;
  case TSL2591_GAIN_MAX:
    if (DEBUG)
    {
      Serial.println(F("9876x (Max)"));
    }
    break;
  }
  if (DEBUG)
  {
    Serial.print(F("Timing:       "));
    Serial.print((tsl.getTiming() + 1) * 100, DEC);
    Serial.println(F(" ms"));
    Serial.println(F("------------------------------------"));
    Serial.println(F(""));
  }
}

uint32_t lum;
uint16_t ir, full;

///////////   Light Sensor Init SECTION  //////////

////////// Initialize for MEMS Sound chip //////////
const int SAMPLE_TIME = 10;
unsigned long millisCurrent;
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;
const int SOUND_PIN = A0; //set to A0 for the D1 mini Pro
// const int SOUND_PIN = A1;
int ambientValue = 0;
int minSound = 1023;
int maxSound = 0;
int delta = 0;
////////// Initialize for MEMS Sound chip //////////

//////////  MEMS Sound chip Functions  //////////
int read_sound_MEMS()
{
  // read the input on analog pin :
  int sensorValue = analogRead(SOUND_PIN);
  delay(10); // delay in between reads for stability
  return (sensorValue);
}

int get_ambient_MEMS(int samples)
{

  float sound_total = 0;
  int i;

  for (i = 0; i < samples; i++)
  {
    sound_total += analogRead(SOUND_PIN);
    delay(10); // delay in between reads for stability
  }
  return (sound_total / i); // return average value
}
//////////  MEMS Sound chip Functions  //////////

//////////   Procedures for Wifi     //////////

void setup_wifi()
{
  delay(10);
  if (DEBUG)
  {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(wifi_ssid);
  }
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (DEBUG)
    {
      Serial.print(".");
    }
  }
  if (DEBUG)
  {
    Serial.println("");
    Serial.println("WiFi OK ");
    Serial.print("=> ESP8266 IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void reconnect()
{

  while (!client.connected())
  {
    if (DEBUG)
    {
      Serial.print("Connecting to MQTT broker ...");
    }
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password))
    {
      if (DEBUG)
      {
        Serial.println("OK");
      }
    }
    else
    {
      if (DEBUG)
      {
        Serial.print("Error : ");
        Serial.print(client.state());
        Serial.println(" Wait 5 seconds before retry");
      }
      delay(5000);
    }
  }
}
//////////   Procedures for Wifi     /////////

//////////   Procedures for NTP    /////////

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  if (DEBUG)
  {
    Serial.println("sending NTP packet...");
  }
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0;          // Stratum, or type of clock
  packetBuffer[2] = 6;          // Polling Interval
  packetBuffer[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

//////////   Procedures for NTP    /////////

///////////   SETUP SECTION  //////////
void setup()
{
  // put your setup code here, to run once:
  if (DEBUG)
  {
    Serial.begin(115200);
    Serial.println("Starting up...");
    Serial.println();
    Serial.println();
  }

  setup_wifi();

  if (DEBUG)
  {
    Serial.println("Starting UDP");
  }
  udp.begin(localPort);
  if (DEBUG)
  {
    Serial.print("Local port: ");
    Serial.println(udp.localPort());
  }

  client.setServer(mqtt_server, 1883);

  ///////////   SETUP FOR ADAFRUIT OLED //////////

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    if (DEBUG)
    {
      Serial.println(F("SSD1306 allocation failed"));
    }
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  ///////////   SETUP FOR ADAFRUIT OLED //////////

  ////////// PIR Motion Sesnor Setup //////////

  pinMode(inputPin, INPUT);   // declare sensor as input
  pinMode(cameraPin, OUTPUT); // setup pin to trigger camera

  ////////// PIR Motion Sesnor Setup //////////

  ////////// Light Sesnor Setup //////////
  if (DEBUG)
  {
    Serial.println(F("Starting Adafruit TSL2591 Test!"));
  }

  if (tsl.begin())
  {
    if (DEBUG)
    {
      Serial.println(F("Found a TSL2591 sensor"));
    }
  }
  else
  {
    if (DEBUG)
    {
      Serial.println(F("No sensor found ... check your wiring?"));
    }
    while (1)
      ;
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Configure the sensor */
  configureSensor();

  ////////// Light Sesnor Setup //////////

  ////////// Get Ambient level for MEMS Sound chip //////////

  ambientValue = get_ambient_MEMS(100); //initialize ambient value of sound to get baseline by averaging the first X values

  ////////// Get Ambient level for MEMS Sound chip  //////////

  ////////// Temp/Hum Sesnor Setup //////////
  if (DEBUG)
  {

    Serial.println("Adafruit SHT4x test");
  }

  if (!sht4.begin())
  {
    if (DEBUG)
    {
      Serial.println("Couldn't find SHT4x");
    }
    while (1)
      delay(1);
  }
  if (DEBUG)
  {

    Serial.println("Found SHT4x sensor");
    Serial.print("Serial number 0x");
    Serial.println(sht4.readSerial(), HEX);
  }

  // You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision())
  {
  case SHT4X_HIGH_PRECISION:
    if (DEBUG)
    {
      Serial.println("High precision");
    }
    break;
  case SHT4X_MED_PRECISION:
    if (DEBUG)
    {
      Serial.println("Med precision");
    }
    break;
  case SHT4X_LOW_PRECISION:
    if (DEBUG)
    {
      Serial.println("Low precision");
    }
    break;
  }

  ////////// Temp/Hum Sesnor Setup //////////
}

//
////////////////////// LOOP ////////////////////////////
//
void loop()
{
  // put your main code here, to run repeatedly:
  loopCount++;

  ////////// Wifi  code //////////
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  ////////// Wifi  code //////////

  val = digitalRead(inputPin); // read input value

  ////////// PIR motion Sensor code //////////
  digitalWrite(cameraPin, val); // trigger the camera based on the PiR readings, e.g. motion

  ////////// PIR motion Sensor code //////////

  if (val == HIGH)
  { // check if the input is HIGH
    if (pirState == LOW)
    {
      // we have just turned on
      if (DEBUG)
      {
        Serial.println("Motion detected!");
      }
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  }
  else
  {
    if (pirState == HIGH)
    {
      // we have just turned of
      if (DEBUG)
      {
        Serial.println("Motion ended!");
      }
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
  ////////// PIR motion Sensor code //////////

  ////////// Light Sensor code //////////

  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  // lum = tsl.getFullLuminosity();
  // ir, full;
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  if (DEBUG)
  {
    Serial.print(F("[ "));
    Serial.print(millis());
    Serial.print(F(" ms ] "));
    Serial.print(F("IR: "));
    Serial.print(ir);
    Serial.print(F("  "));
    Serial.print(F("Full: "));
    Serial.print(full);
    Serial.print(F("  "));
    Serial.print(F("Visible: "));
    Serial.print(full - ir);
    Serial.print(F("  "));
    Serial.print(F("Lux: "));
    Serial.println(tsl.calculateLux(full, ir), 6);
  }

  // Serial.print("ir1: ");
  // Serial.println(ir);

  // delay(500);

  ////////// Light Sensor code //////////

  ////////// MEMS Sound chip code //////////

  int sensorValue = read_sound_MEMS();
  delta = abs(ambientValue - sensorValue);
  if (DEBUG)
  {
    // print out the value you read:

    Serial.print("ambient:");
    Serial.print(ambientValue);
    Serial.print("  sensor:");
    Serial.print(sensorValue);
    Serial.print("   delta:");
    Serial.print(delta);
    Serial.print("   min:");
    Serial.print(minSound);
    Serial.print("   max:");
    Serial.println(maxSound);
  }
  if (delta > maxSound)
  {
    maxSound = delta;
  }
  if (delta < minSound)
  {
    minSound = delta;
  }

  ////////// MEMS Sound chip code //////////

  ////////// Temp/Hum Sesnor code //////////

  sensors_event_t humidity, temp;

  //  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
  //  timestamp = millis() - timestamp;

  long tempC = temp.temperature;
  long tempF = ((tempC / 5.0) * 9.0) + 32.0;
  long hum = humidity.relative_humidity;
  if (DEBUG)
  {

    Serial.print("Temp: ");
    Serial.print(tempC);
    Serial.print(" C  ");
    Serial.print("Temperature: ");
    Serial.print(tempF);
    Serial.print(" F");
    Serial.print("  Hum: ");
    Serial.print(hum);
    Serial.println("% rH");
  }
  ////////// Temp/Hum Sesnor code //////////

  ///////////   CODE FOR ADAFRUIT OLED //////////

  // Clear the display
  display.clearDisplay();
  //Set the color - always use white despite actual display color
  display.setTextColor(WHITE);
  //Set the font size
  display.setTextSize(1);
  //Set the cursor coordinates
  display.setCursor(0, 0);
  display.print(loopCount);
  //   display.setCursor(0, 10);
  // display.print("C:");
  // display.print(tempC);
  display.print(" F:");
  display.print(tempF);
  display.print(" H:");
  display.print(hum);
  display.print(" M:");
  display.print(pirState);

  display.setCursor(0, 10);
  display.print("I:");
  display.print(ir);
  display.print(" V:");
  display.print((full - ir));

  display.setCursor(0, 20);

  display.print("S:");
  display.print(sensorValue);
  display.print(" A:");
  display.print(ambientValue);

  display.setCursor(0, 30);

  display.print("D:");
  display.print(delta);
  display.print(" X:");
  display.print(maxSound);
  display.print(" N:");
  display.print(minSound);

  delay(150);
  display.display();

  ///////////   CODE FOR ADAFRUIT OLED //////////

  ///////////   CODE FOR NTP Time //////////

  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  sendNTPpacket(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb)
  {
    if (DEBUG)
    {
      Serial.println("no packet yet");
    }
  }
  else
  {
    if (DEBUG)
    {
      Serial.print("packet received, length=");
      Serial.println(cb);
    }
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    if (DEBUG)
    {
      Serial.print("Seconds since Jan 1 1900 = ");
      Serial.println(secsSince1900);

      // now convert NTP time into everyday time:
      Serial.print("Unix time = ");
    }
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    //    epoch = secsSince1900 - seventyYears;
    client.publish(unix_date_topic, ("Unix Date=" + String(epoch)).c_str(), true);
    //    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    if (DEBUG)
    {
      Serial.println(epoch);
    }
  }
  ///////////   CODE FOR NTP Time //////////

  ///////////   JSON SECTION  //////////

  StaticJsonDocument<200> doc;

  // Add values in the document
  //
  // doc["sensor"] = "gps";
  // doc["time"] = 1351824120;

  // JsonArray data = doc.createNestedArray("data");
  // data.add(48.756080);
  // data.add(2.302038);

  // Generate the minified JSON and send it to the Serial port.
  //
  // serializeJson(doc, Serial);
  // The above line prints:
  // {"sensor":"gps","time":1351824120,"data":[48.756080,2.302038]}

  // Generate the prettified JSON and send it to the Serial port.
  //
  // serializeJsonPretty(doc, Serial);
  String serializedJSON;

  uint32_t chipId = 0;
  chipId = ESP.getChipId();

  if (DEBUG)
  {
    Serial.print("Chip ID: ");
    Serial.println(chipId);
    Serial.printf("%X\n", chipId);

    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
  }

  doc["chipID"] = chipId;
  doc["tempF"] = tempF;
  doc["tempC"] = tempC;
  doc["hum"] = hum;
  doc["pirState"] = pirState;
  doc["ir"] = ir;
  doc["full"] = full;
  doc["full - ir)"] = (full - ir);
  doc["tempF"] = tempF;
  doc["sensorValue"] = sensorValue;
  doc["ambientValue"] = ambientValue;
  doc["delta"] = delta;
  doc["maxSound"] = maxSound;
  doc["minSound"] = minSound;
  doc["loopCount"] = loopCount;

  serializeJsonPretty(doc, serializedJSON);
  Serial.println(serializedJSON);

  ///////////   JSON SECTION  //////////

  ///////////   Publish MQTT data  //////////

  // Write MQTT data
  //  client.publish(temperatureF_topic, String(tempF).c_str(), true);   // Publish temperature
  //  client.publish(temperatureC_topic, String(tempF).c_str(), true);   // Publish temperature
  //  client.publish(humidity_topic, String(hum).c_str(), true);      // Publish humidity

  client.publish(temperatureF_topic, ("F=" + String(tempF)).c_str(), true);
  client.publish(temperatureC_topic, ("C=" + String(tempC)).c_str(), true);
  client.publish(humidity_topic, ("Hum=" + String(hum)).c_str(), true);
  client.publish(motion_topic, ("Motion=" + String(pirState)).c_str(), true);
  client.publish(ir_topic, ("IR=" + String(ir)).c_str(), true);
  client.publish(full_topic, ("Full=" + String(full)).c_str(), true);
  client.publish(visible_topic, ("Visible=" + String((full - ir))).c_str(), true);
  // client.publish(lux_topic, ("Lux=" + String(tempF)).c_str(), true);
  client.publish(sound_topic, ("Sound=" + String(sensorValue)).c_str(), true);
  client.publish(ambient_topic, ("Ambient=" + String(ambientValue)).c_str(), true);
  client.publish(delta_topic, ("Delta=" + String(delta)).c_str(), true);
  client.publish(maxsound_topic, ("Max Sound=" + String(maxSound)).c_str(), true);
  client.publish(minsound_topic, ("Min Sound=" + String(minSound)).c_str(), true);
  client.publish(loopcount_topic, ("loopCount=" + String(loopCount)).c_str(), true);
  client.publish(doc_topic, ("doc=" + String(serializedJSON)).c_str(), true);
  //  client.publish(unix_date_topic, ("Unix Date=" +  String(epoch)).c_str(), true);

  // delay(2000);

  ///////////   Publish MQTT data  //////////
}

////////////////////// LOOP ////////////////////////////

void macAddrToString(byte *mac, char *str)
{
  for (int i = 0; i < 6; i++)
  {
    byte digit;
    digit = (*mac >> 8) & 0xF;
    *str++ = (digit < 10 ? '0' : 'A' - 10) + digit;
    digit = (*mac) & 0xF;
    *str++ = (digit < 10 ? '0' : 'A' - 10) + digit;
    *str++ = ':';
    mac++;
  }
  // replace the final colon with a nul terminator
  str[-1] = '\0';
}