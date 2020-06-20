/**************************************************************************
  This is a library for several Adafruit displays based on ST77* drivers.

  Works with the Adafruit 1.8" TFT Breakout w/SD card
    ----> http://www.adafruit.com/products/358
  The 1.8" TFT shield
    ----> https://www.adafruit.com/product/802
  The 1.44" TFT breakout
    ----> https://www.adafruit.com/product/2088
  The 1.14" TFT breakout
  ----> https://www.adafruit.com/product/4383
  The 1.3" TFT breakout
  ----> https://www.adafruit.com/product/4313
  The 1.54" TFT breakout
    ----> https://www.adafruit.com/product/3787
  The 2.0" TFT breakout
    ----> https://www.adafruit.com/product/4311
  as well as Adafruit raw 1.8" TFT display
    ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams.
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional).

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

// 需要 Adafruit_AMG88xx 库和 ST7735 库
// 这个文件修改自 ST7735中的 testgraphics 例子

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "AMG.h"
#include <float.h>

#if defined(ARDUINO_FEATHER_ESP32) // Feather Huzzah32
#define TFT_CS 14
#define TFT_RST 15
#define TFT_DC 32

#elif defined(ESP8266)
#define TFT_CS 16
#define TFT_RST 16
#define TFT_DC 2

#else
// For the breakout board, you can use any 2 or 3 pins.
// These pins will also work for the 1.8" TFT shield.
#define TFT_CS 10
#define TFT_RST 9 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 8
#endif

// OPTION 1 (recommended) is to use the HARDWARE SPI pins, which are unique
// to each board and not reassignable. For Arduino Uno: MOSI = pin 11 and
// SCLK = pin 13. This is the fastest mode of operation and is required if
// using the breakout board's microSD card.

// For 1.44" and 1.8" TFT with ST7735 use:
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// For 1.14", 1.3", 1.54", and 2.0" TFT with ST7789:
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// OPTION 2 lets you interface the display using ANY TWO or THREE PINS,
// tradeoff being that performance is not as fast as hardware SPI above.
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out

// For ST7735-based displays, we will use this call
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// OR for the ST7789-based displays, we will use this call
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

//float p = 3.1415926;

#define SDA_PIN 4
#define SCL_PIN 5

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);

void setup(void)
{
  delay(500);
  Serial.begin(115200);
  Serial.print(F("Hello! ST77xx TFT Test"));
  Serial.println("\n\nAMG88xx Interpolated Thermal Camera!");

  // Use this initializer if using a 1.8" TFT screen:
  //tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  // OR use this initializer if using a 1.8" TFT screen with offset such as WaveShare:
  // tft.initR(INITR_GREENTAB);      // Init ST7735S chip, green tab

  // OR use this initializer (uncomment) if using a 1.44" TFT:
  tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab

  // OR use this initializer (uncomment) if using a 0.96" 160x80 TFT:
  //tft.initR(INITR_MINI160x80);  // Init ST7735S mini display

  // OR use this initializer (uncomment) if using a 1.3" or 1.54" 240x240 TFT:
  //tft.init(240, 240);           // Init ST7789 240x240

  // OR use this initializer (uncomment) if using a 2.0" 320x240 TFT:
  //tft.init(240, 320);           // Init ST7789 320x240

  // OR use this initializer (uncomment) if using a 1.14" 240x135 TFT:
  //tft.init(135, 240);           // Init ST7789 240x135


  Wire.pins(SDA_PIN, SCL_PIN);

  // default settings
  if (!amg.begin())
  {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1)
    {
      delay(1);
    }
  }

  Serial.println("-- Thermal Camera Test --");
  Serial.printf("width:%d, height:%d\n", tft.width(), tft.height());
  // tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
}

float min(float a, float b)
{
  return a < b ? a : b;
}

float max(float a, float b)
{
  return a > b ? a : b;
}

void loop()
{
  //  tft.invertDisplay(true);
  //  delay(500);
  //  tft.invertDisplay(false);
  //  delay(500);
  //read all the pixels
  amg.readPixels(pixels);

  Serial.print("[");
  float maxTemp = FLT_MIN;
  float minTemp = FLT_MAX;

  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {
    Serial.print(pixels[i - 1]);
    Serial.print(", ");
    if (i % 8 == 0)
      Serial.println();

    maxTemp = max(maxTemp, pixels[i - 1]);
    minTemp = min(minTemp, pixels[i - 1]);
  }
  Serial.println("]");
  Serial.println();

  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

  int32_t t = millis();
  interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
  int32_t delta = millis() - t;
  Serial.print("Interpolation took ");
  Serial.print(delta);
  Serial.println(" ms");

  uint16_t boxsize = 5; // min(tft.width() / INTERPOLATED_COLS, tft.height() / INTERPOLATED_COLS);

  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, false);

  tft.fillRect(0, tft.height() - 8, tft.width(), 8, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, tft.height() - 8);
  tft.printf("min:%.2f, max:%.2f", minTemp, maxTemp);

  if(delta < 100)
  {
    delay(100 - delta);
  }
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal)
{
  int colorTemp;
  for (int y = 0; y < rows; y++)
  {
    for (int x = 0; x < cols; x++)
    {
      float val = get_point(p, rows, cols, x, y);
      if (val >= MAXTEMP)
        colorTemp = MAXTEMP;
      else if (val <= MINTEMP)
        colorTemp = MINTEMP;
      else
        colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      uint16_t color;
      color = val * 2;
      tft.fillRect(boxWidth * x, boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);

      if (showVal)
      {
        tft.setCursor(boxWidth * y + boxWidth / 2 - 12, 40 + boxHeight * x + boxHeight / 2 - 4);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);
        tft.print(val, 1);
      }
    }
  }
}
