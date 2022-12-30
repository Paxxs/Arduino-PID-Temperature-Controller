// cspell:words OLED datasheet Adafruit SWITCHCAPVCC
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "logo.h"
#include <Adafruit_SSD1306.h>
// k type thermocouple driver
#include <GyverMAX6675.h>

#define OLED_REST -1
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address;

Adafruit_SSD1306 display0(SCREEN_WIDTH, SCREEN_HEIGHT);

#define INTERVAL_LED 2000
#define INTERVAL_K_TYPE 1000
// Arduino pin
#define LED_PIN 2
#define CLK_PIN 9  // MAX6675 SCK
#define CS_PIN 8   // MAX6675 CS
#define DATA_PIN 7 // MAX6675 SO

GyverMAX6675<CLK_PIN, DATA_PIN, CS_PIN> k_type_sensor;

// led time and state
unsigned long time_led = 0;
unsigned long time_k_type = 0;
int led_1_state = LOW;
float k_type_temp = 0;
bool k_type_err = true;

void setup()
{
  Serial.begin(9600);
  delay(500);
  pinMode(LED_PIN, OUTPUT);

  if (!display0.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    for (;;)
    {
      Serial.println(F("SSD1306 allocation failed"));
      delay(5000); // Don't proceed, loop forever
    }
  }

  display0.clearDisplay();
  display0.drawBitmap((display0.width() - logo_width) / 2,
                      (display0.height() - logo_height) / 2,
                      logo_data, logo_width, logo_height, 1);
  display0.display();
  delay(4000); // Pause for 2 seconds

  TestAnimate(logo16_data, logo16_width, logo16_height);
  // delay(500);
}

void loop()
{
  // put your main code here, to run repeatedly:
  DelayRun(&time_led, INTERVAL_LED, []() -> void
           {
    led_1_state = led_1_state == LOW ? HIGH : LOW;
    digitalWrite(LED_PIN, led_1_state); 
    PrintTime(time_led); });

  // refresh temp
  DelayRun(&time_k_type, INTERVAL_K_TYPE, &UpdateTemp);

  // display
  DisplayInfo();
}

/**
 * @brief 间隔运行某个函数
 *
 * @param time 计时器
 * @param interval 间隔 ms
 * @param callback 回调函数
 */
void DelayRun(unsigned long *time, const int interval, void (*callback)())
{
  if (millis() > *time + interval)
  {
    // PrintTime(*time);
    callback();
    *time = millis();
  }
}

/** 更新温度信息 */
void UpdateTemp()
{
  if (k_type_err = k_type_sensor.readTemp()) // 读取温度
    k_type_temp = k_type_sensor.getTemp();
  // Serial.print(sens.getTempInt());   // 或 getTempInt - 整数（无浮点数
}

// 控制台输出文字
void PrintTime(unsigned long time_millis)
{
  Serial.print("Time: ");
  Serial.print(time_millis / 1000);
  Serial.print("s - ");

  Serial.print("Temp: ");
  if (k_type_err)
  {
    Serial.print(k_type_temp);
    Serial.println(" *C");
  }
  else
    Serial.println("Error");
}

// 更新显示器
void DisplayInfo()
{
  display0.clearDisplay();

  display0.setTextSize(1);              // Normal 1:1 pixel scale
  display0.setTextColor(SSD1306_WHITE); // Draw white text
  display0.setCursor(0, 0);             // Start at top-left corner
  display0.print(F("LED: "));
  display0.print(time_led / 1000);
  display0.print(F("s - Tp: "));
  display0.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  if (k_type_err)                                      // 读取温度
  {
    display0.println(k_type_temp);
  }
  else
    display0.println("Error");

  display0.setTextSize(2); // Draw 2X-scale text
  display0.setTextColor(SSD1306_WHITE);
  display0.print("Baby");
  display0.println(F(" Yang"));

  display0.setTextSize(1); // Draw 1X-scale text
  display0.setTextColor(SSD1306_WHITE);
  display0.print("Tp Refresh:");
  display0.println(time_k_type / 1000);

  display0.display();
}

#define NUMFLAKES 10 // Number of snowflakes in the animation example
#define XPOS 0       // Indexes into the 'icons' array in function below
#define YPOS 1
#define DELTAY 2

void TestAnimate(const uint8_t *bitmap, uint8_t w, uint8_t h)
{
  int8_t f, icons[NUMFLAKES][3];

  // Initialize 'snowflake' positions
  for (f = 0; f < NUMFLAKES; f++)
  {
    icons[f][XPOS] = random(1 - logo16_width, display0.width());
    icons[f][YPOS] = -logo16_height;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
  }

  for (unsigned int a = 0; a < 25; a++)
  {                          // Loop forever...
    display0.clearDisplay(); // Clear the display buffer

    // Draw each snowflake:
    for (f = 0; f < NUMFLAKES; f++)
    {
      display0.drawBitmap(icons[f][XPOS], icons[f][YPOS], bitmap, w, h, SSD1306_WHITE);
    }

    display0.display(); // Show the display buffer on the screen
    delay(200);         // Pause for 1/10 second

    // Then update coordinates of each flake...
    for (f = 0; f < NUMFLAKES; f++)
    {
      icons[f][YPOS] += icons[f][DELTAY];
      // If snowflake is off the bottom of the screen...
      if (icons[f][YPOS] >= display0.height())
      {
        // Reinitialize to a random position, just off the top
        icons[f][XPOS] = random(1 - logo16_width, display0.width());
        icons[f][YPOS] = -logo16_height;
        icons[f][DELTAY] = random(1, 6);
      }
    }
  }
}
