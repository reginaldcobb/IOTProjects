#include <Arduino.h>

#include <LowPower.h>

#include <U8g2lib.h>

#include <Adafruit_SSD1306.h>
#include "Adafruit_GFX.h"

// Reset pin not used but needed for library
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include "Adafruit_SHT4x.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

///////////   DEBUG SECTION  //////////

// #define DEBUG false
#define DEBUG true

unsigned long int loopCount = 0;
// unsigned long int loopCount = 0;

///////////   DEBUG SECTION  //////////
// Comment this out for D1 mini Pro
// #define LED_BUILTIN 9

///////////   OLED Init Section  //////////

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // pin remapping with ESP8266 HW I2C

///////////   OLED Init Section  //////////

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
  // delay(500);
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

/**************************************************************************/
/*
    Shows how to perform a basic read on visible, full spectrum or
    infrared light (returns raw 16-bit ADC values)
*/
/**************************************************************************/
void simpleRead(void)
{
  // Simple data read example. Just read the infrared, fullspecrtrum diode
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  if (DEBUG)
  {
    Serial.print(F("[ "));
    Serial.print(millis());
    Serial.print(F(" ms ] "));
    Serial.print(F("Luminosity: "));
    Serial.println(x, DEC);
  }
}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
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
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);

  /* Display the results (light is measured in lux) */
  if (DEBUG)
  {
    Serial.print(F("[ "));
    Serial.print(event.timestamp);
    Serial.print(F(" ms ] "));
  }
  if ((event.light == 0) |
      (event.light > 4294966000.0) |
      (event.light < -4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    if (DEBUG)
    {
      Serial.println(F("Invalid data (adjust gain or timing)"));
    }
  }
  else
  {
    if (DEBUG)
    {
      Serial.print(event.light);
      Serial.print(F(" lux"));
    }
  }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

///////////   Light Sensor Init SECTION  //////////

////////// Initialize for MEMS Sound chip //////////
const int SAMPLE_TIME = 10;
unsigned long millisCurrent;
unsigned long millisLast = 0;
unsigned long millisElapsed = 0;
const int SOUND_PIN = A0; //set to A0 for the D1 mini Pro
// const int SOUND_PIN = A1;
int ambientValue = 0;
int min = 1023;
int max = 0;
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

///////////   Temp/Hum Init SECTION  //////////

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

///////////   Temp/Hum Init SECTION  //////////

///////////   PIR Motion Init SECTION  //////////

int inputPin = 16; // set pin to 16 to match D0 for the D1 mini Pro
// int inputPin = 8;            // choose the input pin (for PIR sensor)
volatile int pirState = LOW; // we start, assuming no motion detected
volatile int val = 0;        // variable for reading the pin status

///////////   PIR Motion Init SECTION  //////////

///////////   Light Sensor Init SECTION  //////////

uint32_t lum;
uint16_t ir, full;

///////////   Light Sensor Init SECTION  //////////

void setup()
{

  // Setup Adafruit display

  // initialize OLED with I2C addr 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  //

  if (DEBUG)
  {
    Serial.begin(9600);
  }
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  ////////// Setup for OLED //////////

  u8g2.begin();

  ////////// Setup for OLED //////////

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

  ////////// PIR Motion Sesnor Setup //////////

  pinMode(inputPin, INPUT); // declare sensor as input

  ////////// PIR Motion Sesnor Setup //////////
}

//////////////////////////// the loop function runs over and over again forever ///////////////////////////////////////
void loop()
{

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
    //    Serial.print(F("[ "));
    //    Serial.print(millis());
    //    Serial.print(F(" ms ] "));
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
    if (DEBUG)
    {

      Serial.print("ambient:");
      Serial.print(ambientValue);
      Serial.print("  sensor:");
      Serial.print(sensorValue);
      Serial.print("   delta:");
      Serial.print(delta);
      Serial.print("   min:");
      Serial.print(min);
      Serial.print("   max:");
      Serial.println(max);
    }
  }
  if (delta > max)
  {
    max = delta;
  }
  if (delta < min)
  {
    min = delta;
  }

  ////////// MEMS Sound chip code //////////

  // Serial.print("ir2: ");
  // Serial.println(ir);

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
    // Serial.print("Temperature: ");
    Serial.print(tempF);
    Serial.print(" F");
    Serial.print("  Hum: ");
    Serial.print(hum);
    Serial.println("% rH");
  }
  ////////// Temp/Hum Sesnor code //////////

  // Serial.print("ir3: ");
  // Serial.println(ir);

  ////////// PIR motion Sensor code //////////
  val = digitalRead(inputPin); // read input value

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

  // Serial.print("ir4: ");
  // Serial.println(ir);

  ////////// OLED code //////////
  if (DEBUG)
  {

    Serial.print("ir: ");
    Serial.print(ir, DEC);

    // Serial.print("  full: ");
    // Serial.print(full, DEC);

    Serial.print("  visible: ");
    Serial.print((full - ir), DEC);

    // Serial.print("  lux: ");
    // Serial.print((tsl.calculateLux(full, ir)), HEX);

    Serial.print("Loopcount ");
    Serial.println(loopCount);
  }
  /// Display to the OLED  ////////

  // u8g2.firstPage();
  // do
  // {
  //   // u8g2.clear() ;
  //   // u8g2.setFont(u8g2_font_6x10_tr);
  //   // u8g2.setFont(u8g2_font_5x7_tr);

  //   u8g2.setCursor(0, 15);
  //   // u8g2.print(loopCount); // Print Loop Count
  //   // u8g2.print(u8x8_u8toa(loopCount, 3)); // Print Loop Count

  //   //    Print Temp
  //   u8g2.setCursor(62, 15);
  //   // u8g2.setCursor(0, 30);
  //   // u8g2.print("F:"); // Print temp label
  //   u8g2.setCursor(73, 15);
  //   // u8g2.print(u8x8_u8toa(tempC, 3)); // Print temp C
  //   // u8g2.print(u8x8_u8toa(tempF, 3)); // Print temp F

  //   u8g2.setCursor(95, 15);
  //   // u8g2.print("H:"); // Print humidity label
  //   u8g2.setCursor(106, 15);
  //   // u8g2.print(u8x8_u8toa(delta, 3)); // Print temp F
  //   // u8g2.print(u8x8_u8toa(hum, 3)); // Print Humidity

  //   u8g2.setCursor(0, 30);
  //   // u8g2.print("M:"); // Print  label
  //   u8g2.setCursor(11, 30);
  //   // u8g2.print(pirState); // Print motion

  //   // //    Print Light
  //   // u8g2.setCursor(18, 30);
  //   // u8g2.print("I:"); // Print  label
  //   // u8g2.setCursor(29, 30);
  //   // u8g2.print(u8x8_u16toa(ir, 4)); // Print

  //   u8g2.setCursor(18, 30);
  //   // u8g2.setCursor(56, 30);
  //   // u8g2.print("L:"); // Print  label for Light
  //   u8g2.setCursor(30, 30);
  //   // u8g2.print(u8x8_u16toa((full - ir), 5)); // Print

  //   u8g2.setCursor(64, 30);
  //   // u8g2.print("S:"); // Print  label
  //   u8g2.setCursor(75, 30);
  //   // u8g2.print(sensorValue); // Print temp F

  //   u8g2.setCursor(96, 30);
  //   // u8g2.print("D:"); // Print temp label
  //   u8g2.setCursor(107, 30);
  //   // u8g2.print(u8x8_u8toa(delta, 3)); // Print temp F

  // } while (u8g2.nextPage());

  /// Write to Oled with Adafruit libraries
  // Clear the display
  display.clearDisplay();
  //Set the color - always use white despite actual display color
  display.setTextColor(WHITE);
  //Set the font size
  display.setTextSize(1);
  //Set the cursor coordinates
  display.setCursor(0, 0);
  display.print(loopCount);
  // display.setCursor(0, 10);
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
  display.print(" L:");
  display.print((full - ir));

  display.setCursor(0, 20);

  display.print("S:");
  display.print(sensorValue);
  // display.print(" A:");
  // display.print(ambientValue);
  display.print(" D:");
  display.print(delta);
  display.print(" X:");
  display.print(max);
  display.print(" N:");
  display.print(min);

  delay(150);
  display.display();
  /// Write to Oled with Adafruit libraries

  ////////// OLED code //////////

  loopCount++;

  //
  // Time to Sleep
  //

  if (!(loopCount % 10))
  // for (int i = 0; i <= 2; i++)
  {
    if (DEBUG)
    {
      Serial.println("Going to sleep...");
    }
    delay(100);
    // LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
    delay(1000);
    if (DEBUG)
    {
      Serial.println("Arduino: Hey I just Woke up");
      Serial.println("");
    }
  }

  //
  // Flash the Led
  //

  if (DEBUG)
  {
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(250);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // turn the LED off by making the voltage LOW

    Serial.println("End of Loop");
  }
  // delay(2500); // wait for a second
}
