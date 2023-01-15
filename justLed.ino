// cspell:words OLED datasheet Adafruit SWITCHCAPVCC
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "logo.h"
#include <Adafruit_SSD1306.h>

// k type thermocouple driver
#include <max6675.h>

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

MAX6675 g_k_thermocouple(CLK_PIN, CS_PIN, DATA_PIN);

// push Button
#define BTN_DOWN_PIN 3
#define BTN_MENU_PIN 4
#define BTN_UP_PIN 5
#define BTN_BACK_PIN 6

// SSR
#define SSR_PIN A0

// led time and state
unsigned long g_time_led = 0;
unsigned long g_time_k_type = 0;

// btn state
volatile bool g_down_btn = false;
volatile bool g_up_btn = false;
volatile bool g_menu_btn = false;
volatile bool g_back_btn = false;

// btn 抖动计算
#define debounce 250 // time to wait in milli secs
unsigned long g_pre_time_down_btn;
unsigned long g_pre_time_menu_btn;
unsigned long g_pre_time_up_btn;
unsigned long g_pre_time_back_btn;

int led_1_state = LOW;
float g_k_type_temp = 0;
bool g_k_type_err = true;

void setup()
{
  noInterrupts();

  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_MENU_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN, INPUT_PULLUP);

  // Bit2 = 1 -> "PCIE2" enabled (PIND)
  PCICR |= B00000100;
  PCMSK2 |= B00111000; // 3 4 5 中断

  interrupts();

  if (!display0.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    for (;;)
    {
      Serial.println(F("SSD1306 allocation failed"));
      delay(5000); // Don't proceed, loop forever
    }
  }

  DisplayLogo();
  delay(2000); // Pause for 2 seconds

  // TestAnimate(logo16_data, logo16_width, logo16_height);
  // delay(500);
}

void loop()
{
  // put your main code here, to run repeatedly:
  DelayRun(&g_time_led, INTERVAL_LED, []() -> void
           { led_1_state = led_1_state == LOW ? HIGH : LOW;
    digitalWrite(LED_PIN, led_1_state);
    PrintTime(g_time_led); });

  // refresh temp
  DelayRun(&g_time_k_type, INTERVAL_K_TYPE, &UpdateTemp);

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

void UpdateTemp()
{
  g_k_type_temp = g_k_thermocouple.readCelsius();
  g_k_type_err = (g_k_type_temp == NAN);
}

// 控制台输出文字
void PrintTime(unsigned long time_millis)
{
  Serial.print(F("Time: "));
  Serial.print(time_millis / 1000);
  Serial.print(F("s - "));

  Serial.print(F("Temp: "));

  Serial.print(g_k_type_temp);
  Serial.print(F(" *C"));
  Serial.print(F(" k_type_err: "));
  Serial.print(g_k_type_err);

  Serial.print(F(" SRAM Left: "));
  Serial.println(GetFreeRam());

  Serial.print(g_pre_time_down_btn);
  Serial.print(F(" "));
  Serial.print(g_pre_time_menu_btn);
  Serial.print(F(" "));
  Serial.println(g_pre_time_up_btn);
}

// 显示 logo
void DisplayLogo()
{
  display0.clearDisplay();
  display0.drawBitmap((display0.width() - logo_width) / 2,
                      (display0.height() - logo_height) / 2,
                      logo_data, logo_width, logo_height, 1);
  display0.display();
}

// 主信息显示界面
void DisplayInfo()
{
#define FONT_X1_W 6
#define FONT_X1_H 8

  display0.clearDisplay();

  display0.setCursor(FONT_X1_W * 3, 0); // 先空三个空格输出设定值
  display0.setTextSize(1);              // Normal 1:1 pixel scale
  display0.setTextColor(SSD1306_WHITE); // Draw white text
  display0.print(F("SetPoint: "));
  display0.println(100.00);
#define FONT_X1_GAP 2
  display0.setCursor(0, FONT_X1_H * 1 + FONT_X1_GAP); // 换行加个间隙
  display0.setTextSize(2);
  display0.print(g_k_type_temp); // MAX: 000.00
  display0.setTextSize(1);
  display0.print(".C");

  // 绘制温度侧边
#define FONT_X2_W 12
#define FONT_X2_H 16
  display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 1 + FONT_X1_GAP); // 保持 2 间隙
  display0.print("PID");

  display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 2 + FONT_X1_GAP); // 1x 换行
  display0.print(255);

  // 绘制 PID 参数框
  // x: 2       y: + 1x 换行
  // w: 9个1x   h: 3个1x
  display0.drawRect(2, FONT_X1_H * 3 + FONT_X1_GAP - 1, FONT_X1_W * 9 + FONT_X1_GAP, FONT_X1_H * 3 + FONT_X1_GAP, SSD1306_WHITE);

  // 绘制 三个 PID 参数列表
  display0.setCursor(4, FONT_X1_H * 3 + FONT_X1_GAP);
  display0.print(F("Kp: "));
  display0.print(12.0000);
  display0.setCursor(4, FONT_X1_H * 4 + FONT_X1_GAP);
  display0.print(F("Ki: "));
  display0.print(1.20);
  display0.setCursor(4, FONT_X1_H * 5 + FONT_X1_GAP);
  display0.print(F("Kd: "));
  display0.print(0.70);

  // 绘制 PID 参数列表侧边系统运行时间
  // 时间文本 x: 9个1x 加4个空格    y: 和 Ki 一行
  // 时间数字 x: 9个1x 加3个空格    y: 和 Kd 一行
  display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 4 + FONT_X1_GAP);
  display0.print(F("Timer"));
  display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 5 + FONT_X1_GAP);
  display0.print(g_time_k_type / 1000);
  // display0.print(5097600);
  display0.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display0.print(F("s"));

  // 绘制 dock 栏
  // x: 0           y: Kd 换行 + 间隔 3
  // w: 屏幕大小     h: 1x字体大小 + gap
  display0.fillRoundRect(0, FONT_X1_H * 6 + FONT_X1_GAP + 3, SCREEN_WIDTH, FONT_X1_H + FONT_X1_GAP, 2, SSD1306_WHITE);

  // 绘制 dock 栏文本
  // x: 2             y: Kd 换行 + 间隔 4
  display0.setTextColor(SSD1306_BLACK);
  display0.setCursor(2, FONT_X1_H * 6 + FONT_X1_GAP + 4);
  display0.print(F("HOT")); // PID HOT COL
  display0.print(F(" RAM:"));
  display0.print(GetFreeRam());
  display0.print(F(" HUM:"));
  display0.print(15.23);

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

/**
 * @brief Get the Free Ram object
 *
 * @return int current Ram
 */
int GetFreeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0
                        ? (int)&__heap_start
                        : (int)__brkval);
}

ISR(PCINT2_vect)
{
  if (digitalRead(BTN_DOWN_PIN) == LOW && millis() - g_pre_time_down_btn > debounce)
  {
    g_down_btn = !g_down_btn;
    g_pre_time_down_btn = millis();
  }
  if (digitalRead(BTN_MENU_PIN) == LOW && millis() - g_pre_time_menu_btn > debounce)
  {
    g_menu_btn = !g_menu_btn;
    g_pre_time_menu_btn = millis();
  }
  if (digitalRead(BTN_BACK_PIN) == LOW && millis() - g_pre_time_back_btn > debounce)
  {
    g_back_btn = !g_back_btn;
    g_pre_time_back_btn = millis();
  }
  if (digitalRead(BTN_UP_PIN) == LOW && millis() - g_pre_time_up_btn > debounce)
  {
    g_up_btn = !g_up_btn;
    g_pre_time_up_btn = millis();
  }
}