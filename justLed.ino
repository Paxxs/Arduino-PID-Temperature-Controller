// cspell:words OLED datasheet Adafruit SWITCHCAPVCC PCICR PCMSK

// ############### Display ###############
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include "logo.h"
// #include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include "logo_xbm.h"

// k type thermocouple driver
#include <max6675.h>
#include "DHT.h"

// ############### DHT ###############
// Uncomment whatever type you're using!
#define DHT_TYPE DHT11 // DHT 11
// #define DHT_TYPE DHT22 // DHT 22  (AM2302), AM2321
// #define DHT_TYPE DHT21   // DHT 21 (AM2301)

// PID control
#include <PID_v1.h>

// Menu System
#include <MenuSystem.h>
#include "CustomRender.h"
#include "ToggleMenuItem.h"

// ############### SSD1306 display ###############
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_RESET -1
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address;

// Adafruit_SSD1306 display0(SCREEN_WIDTH, SCREEN_HEIGHT);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C g_display(U8G2_R0, OLED_RESET);

// Arduino pin
#define LED_PIN 2

#define CLK_PIN 9  // MAX6675 SCK
#define CS_PIN 8   // MAX6675 CS
#define DATA_PIN 7 // MAX6675 SO

#define DHT_PIN 10 // DHT22

// push Button
#define BTN_DOWN_PIN 3
#define BTN_MENU_PIN 4
#define BTN_UP_PIN 5
#define BTN_BACK_PIN 6

// SSR(PWM)
#define SSR_PIN 11

// ############### 传感器 ###############
MAX6675 g_k_thermocouple(CLK_PIN, CS_PIN, DATA_PIN);
DHT dht(DHT_PIN, DHT_TYPE);

// ############### Delay Interval ###############
// #define INTERVAL_LED 1000
#define INTERVAL_K_TYPE 1000

// unsigned long g_time_led = 0;
unsigned long g_timer_k_type;

// ############### 按钮抖动去除计算 ###############
#define debounce 250 // time to wait in milli secs
unsigned long g_pre_time_down_btn;
unsigned long g_pre_time_menu_btn;
unsigned long g_pre_time_up_btn;
// unsigned long g_pre_time_back_btn;

// ############### PID ###############
double g_kp = 2;
double g_ki = 5;
double g_kd = 1;
double g_pid_setpoint, g_pid_input, g_pid_output;
PID myPID(&g_pid_input, &g_pid_output, &g_pid_setpoint, g_kp, g_ki, g_kd, DIRECT);

// ############### 指示状态 ###############
int led_1_state = LOW;
// g_pid_input 温度
float g_dht_temp;
float g_dht_humidity;

bool g_dht_humidity_err = true;      // 温度传感器错误状态
bool g_k_type_err = true;            // 热电偶错误状态
bool g_pid_state = false;            // PID 开启状态
bool g_normal_heating_state = false; // 普通恒温开启状态

// btn state
volatile bool g_down_btn = false;
volatile bool g_up_btn = false;
volatile bool g_menu_btn = false;
volatile bool g_back_btn = false;

// 显示页面
enum DisplayState
{
  e_DisplayInfo,
  e_DisplayMenu
};

DisplayState display_state = e_DisplayInfo;

// ############Menu#########
// const uint16_t updateMenuInterval = 200;

const char menu_main[] PROGMEM = "Back Home Screen";
const char menu_back[] PROGMEM = "Back";

const char menu_change_state[] PROGMEM = "Change State";
const char menu_off_str[] PROGMEM = "OFF";
const char menu_on_str[] PROGMEM = "ON";

const char menu_pid[] PROGMEM = "PID Editor";
const char menu_pid_kp[] PROGMEM = "Kp";
const char menu_pid_ki[] PROGMEM = "Ki";
const char menu_pid_kd[] PROGMEM = "Kd";

const char menu_normal_name[] PROGMEM = "Normal Heating";
const char menu_normal_heating_value[] PROGMEM = "Stop Temp";
const char menu_normal_cooling_value[] PROGMEM = "Start Temp";

CustomRender my_render(&g_display, 4);
MenuSystem ms(my_render);

BackMenuItem back_main_screen(menu_main, &BackMainScreen, &ms);

Menu pid_menu(menu_pid, nullptr);
BackMenuItem pid_back_item(menu_back, nullptr, &ms);
NumericMenuItem pid_kp_item(menu_pid_kp, nullptr, 0, 0, 255.0, 10);
NumericMenuItem pid_ki_item(menu_pid_ki, nullptr, 0, 0, 255.0, 10);
NumericMenuItem pid_kd_item(menu_pid_kd, nullptr, 0, 0, 255.0, 10);
ToggleMenuItem pid_on_off_item(menu_change_state, nullptr, menu_on_str, menu_off_str, &g_pid_state);

Menu simple_hot_menu(menu_normal_name);
BackMenuItem simple_hot_back_item(menu_back, nullptr, &ms);
NumericMenuItem simple_hot_setpoint_hot(menu_normal_heating_value, nullptr, 0.0, 0, 255.0, 10);
NumericMenuItem simple_hot_setpoint_cold(menu_normal_cooling_value, nullptr, 0.0, 0, 255.0, 10);
ToggleMenuItem simple_hot_on_off_item(menu_change_state, nullptr, menu_on_str, menu_off_str, &g_normal_heating_state);

void setup()
{
  g_display.begin(); // always return 1(true)
  DisplayLogo();

  // noInterrupts();

  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_MENU_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN, INPUT_PULLUP);

  dht.begin();

  InitializeMenu();

  g_pid_setpoint = 70;
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(INTERVAL_K_TYPE + 50);
  myPID.SetMode(MANUAL);

  // Bit2 = 1 -> "PCIE2" enabled (PIND)
  PCICR |= B00000100;
  PCMSK2 |= B00111000; // 3 4 5 中断

  // interrupts();

  // if (!display0.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  // {
  //   for (;;)
  //   {
  //     Serial.println(F("SSD1306 allocation failed"));
  //     delay(5000); // Don't proceed, loop forever
  //   }
  // }

  delay(1000); // Pause for 2 seconds
}

void loop()
{
  // put your main code here, to run repeatedly:
  // DelayRun(&g_time_led, INTERVAL_LED, []() -> void
  //          { led_1_state = led_1_state == LOW ? HIGH : LOW;
  //   digitalWrite(LED_PIN, led_1_state);
  //   PrintTime(g_time_led); });

  // refresh temp
  DelayRun(&g_timer_k_type, INTERVAL_K_TYPE, &UpdateTemp);

  // DelayRun(&g_time_key, INTERVAL_KEY_DE, DealKeyPress);

  DealKeyPress();

  myPID.Compute();
  analogWrite(SSR_PIN, g_pid_output);

  switch (display_state)
  {
  case e_DisplayInfo:
    DisplayInfo();
    break;
  case e_DisplayMenu:
    ms.display();
    break;
  default:
    break;
  }
  // display
  // DisplayInfo();
}

/**
 * @brief 间隔运行某个函数
 *
 * @param time 计时器
 * @param interval 间隔 ms
 * @param callback 回调函数
 */
void DelayRun(unsigned long *time, uint32_t interval, void (*callback)())
{
  if (millis() > *time + interval)
  {
    // PrintTime(*time);
    callback();
    *time = millis();
  }
}

void InitializeMenu()
{
  ms.get_root_menu().add_item(&back_main_screen);

  ms.get_root_menu().add_menu(&pid_menu);
  pid_menu.add_item(&pid_back_item);
  pid_menu.add_item(&pid_kp_item);
  pid_menu.add_item(&pid_ki_item);
  pid_menu.add_item(&pid_kd_item);
  pid_menu.add_item(&pid_on_off_item);

  ms.get_root_menu().add_menu(&simple_hot_menu);
  simple_hot_menu.add_item(&simple_hot_back_item);
  simple_hot_menu.add_item(&simple_hot_on_off_item);
  simple_hot_menu.add_item(&simple_hot_setpoint_hot);
  simple_hot_menu.add_item(&simple_hot_setpoint_cold);
}

void BackMainScreen(MenuComponent *menu_component)
{
  // 因为是最低一级了，没东西back，无需 ms.back()
  display_state = e_DisplayInfo;
}

// void UpdatePidTurns()
// {
//   g_ki = pid_ki_item.get_value();
//   g_kp = pid_kp_item.get_value();
//   g_kd = pid_kd_item.get_value();
// }

void DealKeyPress()
{
  if (BtnPressed(&g_menu_btn))
  {
    if (display_state != e_DisplayMenu)
    {
      display_state = e_DisplayMenu;
      // Serial.println(F("切换显示！"));
    }
    else
    {
      ms.select();
      // Serial.println(F("menu"));
    }
  }
  if (BtnPressed(&g_down_btn))
  {
    // Serial.println(F("down"));
    ms.next();
  }
  if (BtnPressed(&g_up_btn))
  {
    // Serial.println(F("up"));
    ms.prev();
  }
}

bool BtnPressed(volatile bool *btn_state)
{
  if (*btn_state)
  {
    noInterrupts();
    *btn_state = false;
    interrupts();
    return true;
  }
  return false;
}

void UpdateTemp()
{
  g_k_type_err = (NAN == g_k_thermocouple.readCelsius());

  if (g_pid_state == false || g_k_type_err)
  {
    myPID.SetMode(MANUAL);
  }
  else
  {
    myPID.SetMode(AUTOMATIC);
  }

  g_pid_input = g_k_thermocouple.readCelsius();
  // g_pid_input = dht.readTemperature();
  g_dht_temp = dht.readTemperature();

  // 湿度
  g_dht_humidity = dht.readHumidity();
  g_dht_humidity_err = isnan(g_dht_humidity);
  // PrintTime(g_timer_k_type);
}

// 控制台输出文字
void PrintTime(unsigned long time_millis)
{
  Serial.print(F("Time:"));
  Serial.print(time_millis / 1000);
  Serial.print(F("-"));

  Serial.print(F("Temp: "));

  Serial.print(g_pid_input);
  Serial.print(F(""));
  Serial.print(F("-k_type_err:"));
  Serial.println(g_k_type_err ? F("ERROR") : F("NO"));
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
  // display0.clearDisplay();
  // display0.drawBitmap((display0.width() - logo_width) / 2,
  //                     (display0.height() - logo_height) / 2,
  //                     logo_data, logo_width, logo_height, 1);
  // display0.display();
  g_display.firstPage();
  do
  {
    // g_display.setDrawColor(1);
    // g_display.setBitmapMode(1);
    g_display.drawXBMP((SCREEN_WIDTH - project_logo_width) / 2, 0, project_logo_width, project_logo_height, project_logo_bits);
  } while (g_display.nextPage());
}

#define FONT_X1_W 6
#define FONT_X1_H 8
#define FONT_X1_GAP 2
#define FONT_X2_W 12
#define FONT_X2_H 16

// 主信息显示界面
void DisplayInfo()
{
  // cspell:words ncen profont
  // display0.clearDisplay();
  g_display.firstPage();
  do
  {
    // #### SetPoint
    // display0.setCursor(FONT_X1_W * 3, 0); // 先空三个空格输出设定值
    // display0.setTextSize(1);              // Normal 1:1 pixel scale
    // display0.setTextColor(SSD1306_WHITE); // Draw white text
    // display0.print(F("SetPoint: "));
    // display0.println(g_pid_setpoint);
    g_display.setFont(u8g2_font_t0_11b_mr);
    g_display.setFontPosTop(); // 顶部对齐
    g_display.drawStr(FONT_X1_W * 3, 0, "SetPoint");
    g_display.setFont(u8g2_font_t0_11_mr);
    g_display.setCursor(FONT_X1_W * (3 + 9), 0);
    g_display.print(g_pid_setpoint);

    // #### 当前温度
    // display0.setCursor(0, FONT_X1_H * 1 + FONT_X1_GAP); // 换行加个间隙
    // display0.setTextSize(2);
    // display0.print(g_pid_input); // MAX: 000.00
    // display0.setTextSize(1);
    // display0.print(F(".C"));
    g_display.setFont(u8g2_font_VCR_OSD_mn); // 只有数字 num 15Px
    g_display.setCursor(0, FONT_X1_H + 1);
    g_display.print(g_pid_input);

    g_display.setFont(u8g2_font_t0_11b_mr);
    g_display.print(F(".C"));

    // #### 绘制温度侧边
    // display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 1 + FONT_X1_GAP); // 保持 2 间隙
    // display0.print(F("PID"));
    // display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 2 + FONT_X1_GAP); // 1x 换行
    // display0.print((int)g_pid_output);

    g_display.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 1 + FONT_X1_GAP); // 保持 2 间隙
    g_display.print(F("PID"));
    g_display.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 2 + FONT_X1_GAP); // 1x 换行
    g_display.setFont(u8g2_font_t0_11_mr);
    g_display.print((int)g_pid_output);

    // #### 绘制 PID 参数框
    // // x: 2       y: + 1x 换行
    // // w: 9个1x   h: 3个1x
    // display0.drawRect(2, FONT_X1_H * 3 + FONT_X1_GAP - 1, FONT_X1_W * 9 + FONT_X1_GAP, FONT_X1_H * 3 + FONT_X1_GAP, SSD1306_WHITE);
    g_display.drawFrame(2, FONT_X1_H * 3 + FONT_X1_GAP - 1, FONT_X1_W * 9 + FONT_X1_GAP, FONT_X1_H * 3 + FONT_X1_GAP * 1);

    // #### 绘制 三个 PID 参数列表
    // display0.setCursor(4, FONT_X1_H * 3 + FONT_X1_GAP);
    // display0.print(F("Kp: "));
    // display0.print(g_kp);
    // display0.setCursor(4, FONT_X1_H * 4 + FONT_X1_GAP);
    // display0.print(F("Ki: "));
    // display0.print(g_ki);
    // display0.setCursor(4, FONT_X1_H * 5 + FONT_X1_GAP);
    // display0.print(F("Kd: "));
    // display0.print(g_kd);
    g_display.setCursor(4, FONT_X1_H * 3 + FONT_X1_GAP);
    g_display.setFont(u8g2_font_5x7_mr); // 6px hight
    g_display.print(F("Kp: "));
    g_display.print(g_kp);
    g_display.setCursor(4, FONT_X1_H * 4 + FONT_X1_GAP);
    g_display.print(F("Ki: "));
    g_display.print(g_ki);
    g_display.setCursor(4, FONT_X1_H * 5 + FONT_X1_GAP);
    g_display.print(F("Kd: "));
    g_display.print(g_kd);

    // #### 绘制 PID 参数列表侧边系统运行时间
    // 时间文本 x: 9个1x 加4个空格    y: 和 Ki 一行
    // 时间数字 x: 9个1x 加3个空格    y: 和 Kd 一行
    // display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 4 + FONT_X1_GAP);
    // display0.print(F("Timer"));
    // display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 5 + FONT_X1_GAP);
    // display0.print(g_timer_k_type / 1000);
    // // display0.print(5097600);
    // display0.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    // display0.print(F("s"));

    // g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 4);
    // g_display.setFont(u8g2_font_t0_11b_mr);
    // g_display.print(F("Timer"));
    g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 0, FONT_X1_H * 4);
    g_display.setFont(u8g2_font_t0_11b_mr);
    g_display.print(F("Timer"));
    g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 5 /* Timer*/ + FONT_X1_W * 2 /*空两格*/, FONT_X1_H * 4);
    g_display.print(F("DHT"));
    g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 0, FONT_X1_H * 5 + FONT_X1_GAP);
    g_display.setFont(u8g2_font_5x7_mr); // 6px hight
    g_display.print(g_timer_k_type / 1000);
    // g_display.print(5097600);
    g_display.setCursor(5 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 6, FONT_X1_H * 5 + FONT_X1_GAP);
    // g_display.print(100.00);
    g_display.print(g_dht_temp);

    // #### 绘制 dock 栏
    // x: 0           y: Kd 换行 + 间隔 3
    // w: 屏幕大小     h: 1x字体大小 + gap
    // display0.fillRoundRect(0, FONT_X1_H * 6 + FONT_X1_GAP + 3, SCREEN_WIDTH, FONT_X1_H + FONT_X1_GAP, 2, SSD1306_WHITE);
    g_display.drawRBox(0, FONT_X1_H * 6 + FONT_X1_GAP * 2 + 1, SCREEN_WIDTH, FONT_X1_H + FONT_X1_GAP, 2);

    // #### 绘制 dock 栏文本
    // x: 2             y: Kd 换行 + 间隔 4
    // display0.setTextColor(SSD1306_BLACK);
    // display0.setCursor(2, FONT_X1_H * 6 + FONT_X1_GAP + 4);
    // if (myPID.GetMode() == AUTOMATIC)
    //   display0.print(F("PID")); // PID HOT OFF
    // else
    //   display0.print(F("OFF")); // PID HOT OFF
    g_display.setDrawColor(0); // 只绘制背景

    g_display.setCursor(4, FONT_X1_H * 6 + FONT_X1_GAP * 2 + 1);
    g_display.setFont(u8g2_font_t0_11b_mr);
    if (g_pid_state)
      g_display.print(F("PID")); // PID HOT OFF
    else if (g_normal_heating_state)
      g_display.print(F("HOT")); // PID HOT OFF
    else
      g_display.print(F("OFF")); // PID HOT OFF

    // #### 覆盖 PID状态 末尾凸出来的背景
    // 在 RBox 底部绘制一个方块
    g_display.drawBox(0, FONT_X1_H * 6 + FONT_X1_GAP * 2 + 1 + FONT_X1_H + FONT_X1_GAP, SCREEN_WIDTH / 4, SCREEN_HEIGHT - (FONT_X1_H * 6 + FONT_X1_GAP * 2 + 1 + FONT_X1_H + FONT_X1_GAP));

    // display0.print(F(" RAM:"));
    // display0.print(GetFreeRam());
    // display0.print(F(" HUM:"));
    // if (g_dht_humidity_err)
    //   display0.print(F("ERROR"));
    // else
    //   display0.print(g_dht_humidity);

    g_display.setCursor(FONT_X1_W * 5, FONT_X1_H * 6 + FONT_X1_GAP * 3);
    // g_display.setFont(u8g2_font_t0_11_mr); // 8px
    // g_display.setFont(u8g2_font_6x10_mr); //7px
    g_display.setFont(u8g2_font_profont10_mr); // 7px
    g_display.print(F(" RAM:"));
    g_display.print(GetFreeRam());
    g_display.print(F(" HUM:"));
    if (g_dht_humidity_err)
      g_display.print(F("ERROR"));
    else
      g_display.print(g_dht_humidity);

    g_display.setDrawColor(1); // 重置绘制模式，绘制前景不背景
  } while (g_display.nextPage());

  // display0.display();
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
  // if (digitalRead(BTN_BACK_PIN) == LOW && millis() - g_pre_time_back_btn > debounce)
  // {
  //   g_back_btn = !g_back_btn;
  //   g_pre_time_back_btn = millis();
  // }
  if (digitalRead(BTN_UP_PIN) == LOW && millis() - g_pre_time_up_btn > debounce)
  {
    g_up_btn = !g_up_btn;
    g_pre_time_up_btn = millis();
  }
}