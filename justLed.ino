// cspell:words OLED datasheet Adafruit SWITCHCAPVCC PCICR PCMSK
// ############### 持久化存储 ###############
#include <EEPROM.h>
/**
 * 要持久化的数据
 * pid_state bool (1bit)
 * heating_state bool(1bit)
 * setpoint int
 * setpoint_diff int
 * Kp float
 * Ki float
 * Kd float
 */

// ############### Display ###############
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include "logo.h"
// #include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include "logo_xbm.h"

// k type thermocouple driver
#include <max6675.h>
// #include "DHT.h"
#include "TinyDHT.h"

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
#include "UIntMenuItem.h"

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
struct PERSIS_DATA
{
  bool pid_state /* PID 开启状态 */, heating_state /* 普通恒温开启状态 */;
  uint8_t setpoint, setpoint_diff;
  double Kp, Ki, Kd;
};

PERSIS_DATA g_persist_data;

double g_pid_setpoint, g_pid_input, g_pid_output;
PID myPID(&g_pid_input, &g_pid_output, &g_pid_setpoint, g_persist_data.Kp, g_persist_data.Ki, g_persist_data.Kd, DIRECT);

// ############## 普通加热 ##############
// float g_start_temp; // 开始温度，当低于开始加热 [改为使用差值]
// float g_stop_temp;  // 结束温度，当高于停止加热 [改为使用差值]
// uint32_t g_setpoint_diff = 5; // 与 setpoint 差值

// ############### 指示状态 ###############
// g_pid_input 温度
float g_dht_temp;
float g_dht_humidity;

bool g_dht_humidity_err = true; // 温度传感器错误状态
bool g_k_type_err = true;       // 热电偶错误状态

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

// 加热模式
enum HeatingState
{
  ePid,
  eNormalHeating,
  eOff
};

// 当前屏幕显示内容
DisplayState g_display_state = e_DisplayInfo;
// 当前加热模式
HeatingState g_heating_state = eOff;

// ############Menu#########

const char menu_main_str[] PROGMEM = "Back Home";
const char menu_back_str[] PROGMEM = "Back";

const char menu_change_state_str[] PROGMEM = "State";
const char menu_off_str[] PROGMEM = "OFF";
const char menu_on_str[] PROGMEM = "ON";

const char menu_pid_str[] PROGMEM = "PID";
const char menu_pid_kp_str[] PROGMEM = "Kp";
const char menu_pid_ki_str[] PROGMEM = "Ki";
const char menu_pid_kd_str[] PROGMEM = "Kd";
const char menu_pid_setpoint_str[] PROGMEM = "SetPoint";

const char menu_normal_str[] PROGMEM = "Normal Heating";
const char menu_normal_diff_str[] PROGMEM = "Diff";

const char menu_save_str[] PROGMEM = "Save to EEPROM";
const char menu_read_from_com[] PROGMEM = "Read COM";

const char error_str[] PROGMEM = "ERR";

CustomRender my_render(&g_display, 4);
MenuSystem ms(my_render);

BackMenuItem back_main_screen(menu_main_str, &CallBackMenu, nullptr);        // 返回主页面
BackMenuItem save_main_screen(menu_save_str, &CallBackMenu, nullptr);        // 持久化保存
BackMenuItem read_from_com_item(menu_read_from_com, &CallBackMenu, nullptr); // 持久化保存

Menu pid_menu(menu_pid_str, nullptr);
BackMenuItem menu_back_item(menu_back_str, &CallBackMenu, &ms);
NumericMenuItem pid_kp_item(menu_pid_kp_str, nullptr, 0.0, 0.0, 255.0, 0.1);
NumericMenuItem pid_ki_item(menu_pid_ki_str, nullptr, 0.0, 0.0, 255.0, 0.01);
NumericMenuItem pid_kd_item(menu_pid_kd_str, nullptr, 0.0, 0.0, 255.0, 0.01);

UIntMenuItem pid_setpoint_item(menu_pid_setpoint_str, nullptr, 0, 255);
ToggleMenuItem pid_on_off_item(menu_change_state_str, nullptr, menu_on_str, menu_off_str, &g_persist_data.pid_state);

Menu simple_hot_menu(menu_normal_str);
UIntMenuItem simple_hot_diff(menu_normal_diff_str, nullptr, 0, 255);
ToggleMenuItem simple_hot_on_off_item(menu_change_state_str, nullptr, menu_on_str, menu_off_str, &g_persist_data.heating_state);

void setup()
{
  g_display.begin(); // always return 1(true)
  DisplayLogo();
  // noInterrupts();

  Serial.begin(9600);

  InitializeVariable();

  pinMode(LED_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  // digitalWrite(SSR_PIN, LOW);

  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_MENU_PIN, INPUT_PULLUP);
  pinMode(BTN_UP_PIN, INPUT_PULLUP);

  dht.begin();

  InitializeMenu();

  g_pid_setpoint = 70;
  // myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(INTERVAL_K_TYPE + 500);
  // myPID.SetMode(MANUAL);

  // Bit2 = 1 -> "PCIE2" enabled (PIND)
  PCICR |= B00000100;
  PCMSK2 |= B00111000; // 3 4 5 中断

  // interrupts();

  delay(1000); // Pause for 2 seconds
}

void loop()
{
  // refresh temp
  DelayRun(&g_timer_k_type, INTERVAL_K_TYPE, &UpdateTemp);

  // DelayRun(&g_time_key, INTERVAL_KEY_DE, DealKeyPress);

  DealKeyPress();
  UpdateDisplayContent();

  UpdateHeatingMode();
  // OperateHeating();
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
    // PrintInfo(*time);
    callback();
    *time = millis();
  }
}

void InitializeVariable()
{
  // TODO： 读取持久化存储中的变量
  // PERSIS_DATA data_read;
  // byte tmp;
  // EEPROM.get(0 + sizeof(bool) * 2 + sizeof(uint8_t) * 2, tmp);
  // Serial.println(tmp);
  if (EEPROM.read(sizeof(bool) * 2 + sizeof(uint8_t) * 2) != 255 &&
      EEPROM.read(sizeof(bool) * 2 + sizeof(uint8_t) * 2 + sizeof(float) * 2) != 255)
  {
    EEPROM.get(0, g_persist_data);
  }
}

void InitializeMenu()
{
  ms.get_root_menu().add_item(&back_main_screen);

  ms.get_root_menu().add_menu(&pid_menu);
  pid_menu.add_item(&menu_back_item);
  pid_menu.add_item(&pid_kp_item);
  pid_menu.add_item(&pid_ki_item);
  pid_menu.add_item(&pid_kd_item);
  pid_menu.add_item(&pid_setpoint_item);
  pid_menu.add_item(&read_from_com_item);
  pid_menu.add_item(&pid_on_off_item); // 指针无需下方初始化值

  ms.get_root_menu().add_menu(&simple_hot_menu);
  simple_hot_menu.add_item(&menu_back_item);
  simple_hot_menu.add_item(&simple_hot_on_off_item); // 指针无需下方初始化值
  simple_hot_menu.add_item(&pid_setpoint_item);
  simple_hot_menu.add_item(&simple_hot_diff);

  ms.get_root_menu().add_item(&save_main_screen);

  UpdateMenuValue();
}

// 更新默认值
void UpdateMenuValue()
{
  pid_kp_item.set_value(g_persist_data.Kp);
  pid_ki_item.set_value(g_persist_data.Ki);
  pid_kd_item.set_value(g_persist_data.Kd);
  myPID.SetTunings(g_persist_data.Kp,
                   g_persist_data.Ki,
                   g_persist_data.Kd);

  pid_setpoint_item.set_value(g_persist_data.setpoint);
  g_pid_setpoint = g_persist_data.setpoint;

  simple_hot_diff.set_value(g_persist_data.setpoint_diff);
}

// 从 Serial 中读取 float 值
void ReadFromCOM(double *data, char mode)
{
  Serial.print("Input ");
  if (mode == 1)
    Serial.println((const __FlashStringHelper *)menu_pid_kp_str);
  else if (mode == 2)
    Serial.println((const __FlashStringHelper *)menu_pid_ki_str);
  else
    Serial.println((const __FlashStringHelper *)menu_pid_kd_str);

  while (Serial.available() == 0)
  {
  }
  *data = Serial.parseFloat(SKIP_ALL, '\n');
}

// 保存至 EEPROM
void CallBackMenu(MenuComponent *menu_component)
{
  const char *tmp = menu_component->get_name();
  if (tmp == menu_save_str)
  {
    // Serial.println("woc");
    EEPROM.put(0, g_persist_data);
  }
  else if (tmp == menu_main_str)
  {
    // 返回主页
    g_display_state = e_DisplayInfo;
  }
  else if (tmp == menu_read_from_com)
  {
    // read from com
    ReadFromCOM(&g_persist_data.Kp, 1);
    ReadFromCOM(&g_persist_data.Ki, 2);
    ReadFromCOM(&g_persist_data.Kd, 3);
  }
  else
  {
    // 保存数据
    // Serial.println("Save");
    g_persist_data.Kp = pid_kp_item.get_value();
    g_persist_data.Ki = pid_ki_item.get_value();
    g_persist_data.Kd = pid_kd_item.get_value();

    myPID.SetTunings(g_persist_data.Kp,
                     g_persist_data.Ki,
                     g_persist_data.Kd);

    g_pid_setpoint = g_persist_data.setpoint = pid_setpoint_item.get_value();

    g_persist_data.setpoint_diff = simple_hot_diff.get_value();
  }
  UpdateMenuValue();
}

// 更新加热模式
void UpdateHeatingMode()
{
  // PID 优先
  if (g_persist_data.pid_state && g_k_type_err == false)
  {
    g_heating_state = ePid;
    bitSet(PORTD, LED_PIN);
    myPID.SetMode(AUTOMATIC);
    myPID.Compute();
    analogWrite(SSR_PIN, g_pid_output);
    return;
  }
  else if (g_persist_data.heating_state && g_k_type_err == false)
  {
    g_heating_state = eNormalHeating;
    if (g_pid_input >= g_pid_setpoint) // 达到停止温度就停止
    {
      // digitalWrite(SSR_PIN, LOW);
      // digitalWrite(LED_PIN, LOW);
      bitClear(PORTB, 3); // SSR
      bitClear(PORTD, LED_PIN);
    }
    else if (g_pid_input < (g_persist_data.setpoint - g_persist_data.setpoint_diff)) // 低于开始温度就开始
    {
      // digitalWrite(SSR_PIN, HIGH);
      // digitalWrite(LED_PIN, HIGH);
      bitSet(PORTB, 3); // SSR
      bitSet(PORTD, LED_PIN);
    }
  }
  else
  {
    bitClear(PORTB, 3); // SSR
    bitClear(PORTD, LED_PIN);
    g_heating_state = eOff;
  }
  myPID.SetMode(MANUAL);
}

// 更新界面内容
void UpdateDisplayContent()
{
  // switch (g_display_state)
  // {
  // case e_DisplayInfo:
  //   DisplayInfo();
  //   break;
  // case e_DisplayMenu:
  //   ms.display();
  //   break;
  // }
  if (g_display_state == e_DisplayMenu)
  {
    ms.display();
    return;
  }
  DisplayInfo();
}
// void UpdatePidTurns()
// {
//   g_ki = pid_ki_item.get_value();
//   g_kp = pid_kp_item.get_value();
//   g_kd = pid_kd_item.get_value();
// }

// 处理按键
void DealKeyPress()
{
  if (BtnPressed(&g_menu_btn))
  {
    if (g_display_state != e_DisplayMenu)
    {
      g_display_state = e_DisplayMenu;
      // Serial.println(F("切换显示！"));
      return;
    }
    ms.select();
  }
  if (BtnPressed(&g_down_btn))
  {
    // Serial.println(F("down"));
    if (g_display_state == e_DisplayMenu)
    {
      ms.next(true);
      return;
    }
  }
  if (BtnPressed(&g_up_btn))
  {
    // Serial.println(F("up"));
    if (g_display_state == e_DisplayMenu)
    {
      ms.prev(true);
      return;
    }
  }
}

/**
 * @brief 将按钮状态设置为按下（重置）
 *
 * @param btn_state 按钮当前状态变量指针
 * @return true 按钮被用户按下
 * @return false 按钮未按下
 */
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
  float tmp = g_k_thermocouple.readCelsius();
  if (!(g_k_type_err = isnan(tmp))) // 如果没有错误，温度更新，避免 nan 传入 pid
    g_pid_input = tmp;

  g_dht_temp = dht.readTemperature();
  // 湿度
  g_dht_humidity = dht.readHumidity();
  g_dht_humidity_err = (g_dht_humidity == BAD_HUM || g_dht_temp == BAD_TEMP);

  PrintInfo(&g_timer_k_type);
}

// 控制台输出文字
void PrintInfo(unsigned long *time_millis)
{
  // Serial.print(F("Time:"));
  Serial.print(*time_millis / 1000);

  Serial.print(F("-"));
  // Serial.print(F("Temp:"));
  Serial.print(g_pid_input);

  // Serial.print(F(" "));
  Serial.print(F("-"));
  Serial.print(g_pid_output);

  Serial.print(F("-"));
  Serial.println(g_heating_state);
}

// 显示 logo
void DisplayLogo()
{

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

// 绘制信息显示界面（MainScreen)
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
    g_display.setCursor(FONT_X1_W * 4, 0);
    g_display.print((const __FlashStringHelper *)menu_pid_setpoint_str);
    // g_display.drawStr(FONT_X1_GAP, 0, "SetPoint:");
    // g_display.drawStr(FONT_X1_W * (1 + 9 + 3) + FONT_X1_GAP, 0, "+/-");

    g_display.setFont(u8g2_font_t0_11_mr);
    g_display.setCursor(FONT_X1_W * (4 + 9), 0);
    // g_display.print(100);
    g_display.print(g_persist_data.setpoint);

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
    g_display.print(".C");

    // #### 绘制温度侧边
    // display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 1 + FONT_X1_GAP); // 保持 2 间隙
    // display0.print(F("PID"));
    // display0.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 2 + FONT_X1_GAP); // 1x 换行
    // display0.print((int)g_pid_output);

    g_display.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 1 + FONT_X1_GAP); // 保持 2 间隙
    g_display.print((const __FlashStringHelper *)menu_pid_str);

    // #### PID 输出（温度侧边）底部的系统运行时间标题
    // g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 4);
    // g_display.setFont(u8g2_font_t0_11b_mr);
    // g_display.print(F("Timer"));
    g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 0, FONT_X1_H * 4);
    // g_display.setFont(u8g2_font_t0_11b_mr);
    g_display.print("Timer");
    g_display.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 5 /* Timer*/ + FONT_X1_W * 2 /*空两格*/, FONT_X1_H * 4);
    g_display.print("DHT");

    g_display.setCursor(FONT_X2_W * 6 + FONT_X1_W * 4 + 1, FONT_X1_H * 2 + FONT_X1_GAP); // 1x 换行
    g_display.setFont(u8g2_font_t0_11_mr);
    if (g_heating_state == ePid)
      g_display.print((int)g_pid_output);
    else
      g_display.print(g_persist_data.setpoint_diff);

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
    g_display.print("Kp: ");
    // g_display.print(F("Kp: "));
    g_display.print(g_persist_data.Kp);
    g_display.setCursor(4, FONT_X1_H * 4 + FONT_X1_GAP);
    g_display.print("Ki: ");
    // g_display.print(F("Ki: "));
    g_display.print(g_persist_data.Ki);
    g_display.setCursor(4, FONT_X1_H * 5 + FONT_X1_GAP);
    g_display.print("Kd: ");
    // g_display.print(F("Kd: "));
    g_display.print(g_persist_data.Kd);

    // #### 绘制 PID 参数列表侧边的系统运行时间
    // 时间文本 x: 9个1x 加4个空格    y: 和 Ki 一行
    // 时间数字 x: 9个1x 加3个空格    y: 和 Kd 一行
    // display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 4 + FONT_X1_GAP);
    // display0.print(F("Timer"));
    // display0.setCursor(4 + FONT_X1_W * 9 + FONT_X1_GAP + FONT_X1_W * 3, FONT_X1_H * 5 + FONT_X1_GAP);
    // display0.print(g_timer_k_type / 1000);
    // // display0.print(5097600);
    // display0.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    // display0.print(F("s"));
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
    if (g_heating_state == ePid) // PID HOT OFF
      g_display.print((const __FlashStringHelper *)menu_pid_str);
    else if (g_heating_state == eNormalHeating)
      g_display.print("HOT");
    else if (g_k_type_err)
      g_display.print((const __FlashStringHelper *)error_str);
    else
      g_display.print((const __FlashStringHelper *)menu_off_str);

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
    // g_display.print(F(" RAM:"));
    g_display.print(" RAM:");
    g_display.print(GetFreeRam());
    // g_display.print(F(" HUM:"));
    g_display.print(" HUM:");
    if (g_dht_humidity_err)
      g_display.print((const __FlashStringHelper *)error_str);
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
  // if (digitalRead(BTN_DOWN_PIN) == LOW && millis() - g_pre_time_down_btn > debounce)
  if (bitRead(PIND, BTN_DOWN_PIN) == 0 && millis() - g_pre_time_down_btn > debounce)
  {
    g_down_btn = !g_down_btn;
    g_pre_time_down_btn = millis();
  }
  if (bitRead(PIND, BTN_MENU_PIN) == 0 && millis() - g_pre_time_menu_btn > debounce)
  {
    g_menu_btn = !g_menu_btn;
    g_pre_time_menu_btn = millis();
  }
  // if (digitalRead(BTN_BACK_PIN) == LOW && millis() - g_pre_time_back_btn > debounce)
  // {
  //   g_back_btn = !g_back_btn;
  //   g_pre_time_back_btn = millis();
  // }
  if (bitRead(PIND, BTN_UP_PIN) == 0 && millis() - g_pre_time_up_btn > debounce)
  {
    g_up_btn = !g_up_btn;
    g_pre_time_up_btn = millis();
  }
}