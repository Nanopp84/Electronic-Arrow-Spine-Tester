#include <limits.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <HX711.h>
#include <U8g2lib.h>          
#include "u8g2_font_9d78f479cc9b46edd339660b427de7a6.h"

// -------------------------- 1. 全局宏定义 --------------------------
#define MAX_INPUT_LEN 9
#define DECIMAL_MAX 1
#define IN_TO_CM 2.54
#define CM_TO_IN (1.0/2.54)
#define G_TO_GR 15.4324
#define GR_TO_G (1.0/15.4324)
#define MAX_WEIGHT_DIGITS 6
#define LEN_MAX_DIGITS 6
#define INIT_DELAY 1000
#define SWITCH_DELAY 300
#define EXIT_DELAY 200
#define KEY_DELAY 50
#define SENSOR_DIST 355.6f
#define CMD_TIMEOUT 5000
#define KEY_TEST_MODE 0
#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define GLOBAL_DELAY 400
#define LONG_PRESS_TIME 800
#define MENU_AUTO_SWITCH_MS 1500

// OLED 参数
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C      

// -------------------------- 2. 硬件配置 --------------------------
const int HX711_1_DOUT = 12; // HX711-1＝右側感測器（D8 已讓給鍵盤，改到 D12）
const int HX711_1_SCK = 13;  // 右側 SCK（D13 有板載 LED 會閃，屬正常）
const int HX711_2_DOUT = 10; // HX711-2＝左側感測器
const int HX711_2_SCK = 11;  // 左側 SCK
const int BUZZER_PIN = 9;    // 蜂鳴器（D9，與鍵盤/HX711/OLED 無衝突）

const float CALIB_FACTOR_1 = 1073.1;  // HX711-1（右側）校準係數：722g法碼實測1913g換算(405*1913/722≈1073.1)
const float CALIB_FACTOR_2 = 1091.0;  // HX711-2（左側）校準係數：722g法碼實測1994g換算(395.05*1994/722≈1091)
const float WEIGHT_LIMIT = 10000.0;

// ==========================
// 鍵盤腳位：依實際測試結果（keypad_test）定義
// ==========================
const int KEYU_ROW = 6,  KEYU_COL = 5;    // u: 6+5
const int KEY0_ROW = 5,  KEY0_COL = 8;    // 0: 5+8
const int KEYB_ROW = 4,  KEYB_COL = 5;    // b: 4+5

const int KEY7_ROW = 3,  KEY7_COL = 6;    // 7: 3+6
const int KEY8_ROW = 3,  KEY8_COL = 8;    // 8: 3+8
const int KEY9_ROW = 4,  KEY9_COL = 3;    // 9: 4+3

const int KEY4_ROW = 2,  KEY4_COL = 6;    // 4: 2+6
const int KEY5_ROW = 8,  KEY5_COL = 2;    // 5: 8+2
const int KEY6_ROW = 2,  KEY6_COL = 4;    // 6: 2+4

const int KEY1_ROW = 6,  KEY1_COL = 7;    // 1: 6+7
const int KEY2_ROW = 7,  KEY2_COL = 8;    // 2: 7+8
const int KEY3_ROW = 4,  KEY3_COL = 7;    // 3: 4+7

// 全局对象 - U8g2 构造器 (I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
HX711 scale1;
HX711 scale2;

// 显示缓存（25 bytes：容納 UTF-8 中文最長選單字串「1.撓度ASTM 自由」20 bytes + \0，防溢出）
char lastLine0[25] = "";
char lastLine1[25] = "";
char lastLine2[25] = "";
char line[25];

// -------------------------- 3. 全局变量 --------------------------
int menuSel = 0;
const char* menuItems[] = {
  "1.ASTM 自由",
  "2.ASTM 4 點",
  "3.撓度AMO",
  "4.FOC",
  "5.電子秤",
  "6.重量換算",
  "7.長度換算"
};
const int MENU_CNT = sizeof(menuItems)/sizeof(menuItems[0]);
unsigned long lastKeyPrintTime = 0;
unsigned long lastKeyTime = 0;
char detectedKey = 0;
bool uKeyHeld = false;
unsigned long uKeyHoldStartTime = 0;
unsigned long lastMenuSwitchTime = 0;

// -------------------------- 4. 函数声明 --------------------------
char scanKeypad();
bool waitForKeyRelease(char key);
float getNumericInput(const char* prompt, float minVal, float maxVal, int maxDigits, bool allowDecimal);
bool checkKey(int rowPin, int colPin);
bool anyMenuKeyDown();
void updateLCD(int row, const char* text);
void displayMenu();
void execMenu(int item);
float readWeight(HX711& scale, const char* name, float calibFactor);
void cleanSensors();
void displayMsg(const char* msg, unsigned int delayT);
void showErr(const char* name, const char* msg);
float getLenInput(const char* prompt, float def);
void keyTestLoop();
void lengthConversion();
void weightConversion();
bool isInputValid(const char* input);
int strLen(const char* str);
void strCpy(char* dest, const char* src);
void strCat(char* dest, const char* src);
void safeFloatPrint(char* buffer, int bufferSize, float value, int decimalPlaces);

// -------------------------- 5. 鍵盤處理 已修復防抖 + 衝突 --------------------------
bool checkKey(int rowPin, int colPin) {
  pinMode(rowPin, OUTPUT);
  digitalWrite(rowPin, HIGH);
  pinMode(colPin, INPUT_PULLUP);
  digitalWrite(rowPin, LOW);
  delay(1); // 防抖
  bool isPressed = (digitalRead(colPin) == LOW);
  digitalWrite(rowPin, HIGH);
  pinMode(rowPin, INPUT);
  pinMode(colPin, INPUT);
  return isPressed;
}

// 偵測任一選單數字鍵(1~7)是否被實體按下（含防抖鎖定期間）
// 用途：按下按鍵的瞬間暫停選單自動輪播，避免「按4卻閃一下5.電子秤」的跳屏
bool anyMenuKeyDown() {
  return checkKey(KEY1_ROW, KEY1_COL) || checkKey(KEY2_ROW, KEY2_COL)
      || checkKey(KEY3_ROW, KEY3_COL) || checkKey(KEY4_ROW, KEY4_COL)
      || checkKey(KEY5_ROW, KEY5_COL) || checkKey(KEY6_ROW, KEY6_COL)
      || checkKey(KEY7_ROW, KEY7_COL);
}

char scanKeypad() {
  detectedKey = 0;
  uKeyHeld = checkKey(KEYU_ROW, KEYU_COL);
  
  if (checkKey(KEY1_ROW, KEY1_COL)) detectedKey = '1';
  else if (checkKey(KEY2_ROW, KEY2_COL)) detectedKey = '2';
  else if (checkKey(KEY3_ROW, KEY3_COL)) detectedKey = '3';
  else if (checkKey(KEY4_ROW, KEY4_COL)) detectedKey = '4';
  else if (checkKey(KEY5_ROW, KEY5_COL)) detectedKey = '5';
  else if (checkKey(KEY6_ROW, KEY6_COL)) detectedKey = '6';
  else if (checkKey(KEY7_ROW, KEY7_COL)) detectedKey = '7';
  else if (checkKey(KEY8_ROW, KEY8_COL)) detectedKey = '8';
  else if (checkKey(KEY9_ROW, KEY9_COL)) detectedKey = '9';
  else if (checkKey(KEY0_ROW, KEY0_COL)) detectedKey = '0';
  else if (checkKey(KEYU_ROW, KEYU_COL)) detectedKey = 'U';
  else if (checkKey(KEYB_ROW, KEYB_COL)) detectedKey = 'B';

  if (detectedKey != 0) {
    unsigned long now = millis();
    if (now - lastKeyTime > GLOBAL_DELAY) {
      lastKeyTime = now;
      return detectedKey;
    }
  }
  return '\0';
}

bool waitForKeyRelease(char key) {
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
    char currentKey = 0;
    if(checkKey(KEY1_ROW, KEY1_COL) && key == '1') currentKey = '1';
    else if(checkKey(KEY2_ROW, KEY2_COL) && key == '2') currentKey = '2';
    else if(checkKey(KEY3_ROW, KEY3_COL) && key == '3') currentKey = '3';
    else if(checkKey(KEY4_ROW, KEY4_COL) && key == '4') currentKey = '4';
    else if(checkKey(KEY5_ROW, KEY5_COL) && key == '5') currentKey = '5';
    else if(checkKey(KEY6_ROW, KEY6_COL) && key == '6') currentKey = '6';
    else if(checkKey(KEY7_ROW, KEY7_COL) && key == '7') currentKey = '7';
    else if(checkKey(KEY8_ROW, KEY8_COL) && key == '8') currentKey = '8';
    else if(checkKey(KEY9_ROW, KEY9_COL) && key == '9') currentKey = '9';
    else if(checkKey(KEY0_ROW, KEY0_COL) && key == '0') currentKey = '0';
    else if(checkKey(KEYU_ROW, KEYU_COL) && key == 'U') currentKey = 'U';
    else if(checkKey(KEYB_ROW, KEYB_COL) && key == 'B') currentKey = 'B';

    if (currentKey != key) {
      delay(20);
      return true;
    }
    delay(10);
  }
  return false;
}

float getNumericInput(const char* prompt, float minVal, float maxVal, int maxDigits, bool allowDecimal) {
  char inputBuf[MAX_INPUT_LEN + 1] = "0";
  bool inputComplete = false;
  updateLCD(0, prompt);
  snprintf(line, 17, "輸入: %s", inputBuf);
  updateLCD(1, line);
  unsigned long inputStartTime = millis();
  const unsigned long INPUT_TIMEOUT = 60000;
  unsigned long uKeyPressStartTime = 0;
  bool uKeyIsPressed = false;
  
  while (!inputComplete) {
    if (millis() - inputStartTime > INPUT_TIMEOUT) {
      displayMsg("逾時", 1000);
      return -1;
    }
    char key = scanKeypad();
    if (key == 'U' && !inputComplete) {
      uKeyPressStartTime = millis();
      uKeyIsPressed = true;
      while (checkKey(KEYU_ROW, KEYU_COL)) {
        if (millis() - uKeyPressStartTime >= LONG_PRESS_TIME) {
          inputComplete = true;
          displayMsg("確認", 300);
          lastKeyTime = millis();
          uKeyIsPressed = false;
          break;
        }
        delay(10);
      }
      if (uKeyIsPressed) {
        waitForKeyRelease('U');
        int bufLen = strLen(inputBuf);
        if (bufLen > 1) {
          inputBuf[bufLen - 1] = '\0';
        } else {
          strCpy(inputBuf, "0");
        }
        snprintf(line, 17, "輸入: %s", inputBuf);
        updateLCD(1, line);
      }
      uKeyPressStartTime = 0;
    }
    if (key == 'B') {
      waitForKeyRelease('B');
      return -1;
    }
    if (key >= '0' && key <= '9') {
      waitForKeyRelease(key);
      int bufLen = strLen(inputBuf);
      if (bufLen < maxDigits) {
        if (bufLen == 1 && inputBuf[0] == '0' && key != '0') {
          inputBuf[0] = key;
        } else if (!(bufLen == 1 && inputBuf[0] == '0')) {
          inputBuf[bufLen] = key;
          inputBuf[bufLen + 1] = '\0';
        }
      } else {
        displayMsg("超出位數", 500);
      }
      snprintf(line, 17, "輸入: %s", inputBuf);
      updateLCD(1, line);
      inputStartTime = millis();
    }
    delay(10);
  }
  float value = atof(inputBuf);
  if (value >= minVal && value <= maxVal) {
    return value;
  } else {
    displayMsg("數值無效", 1000);
    return -1;
  }
}

// -------------------------- 6. 核心工具函数 --------------------------
void safeFloatPrint(char* buffer, int bufferSize, float value, int decimalPlaces) {
  if (isnan(value) || isinf(value)) {
    snprintf(buffer, bufferSize, "Err");
    return;
  }
  if (value < 0) value = 0;
  if (value > 999999.9) {
    snprintf(buffer, bufferSize, "999999.9");
    return;
  }
  dtostrf(value, 1, decimalPlaces, buffer);
  buffer[bufferSize - 1] = '\0';
}

// ==========================
// ✅ 三行显示菜单（14/32/50 坐标）
// ==========================
void updateLCD(int row, const char* text) {
  // 安全拷貝：上限 24 bytes + 強制 \0，絕不溢出（最長選單字串 20 bytes 完整容納）
  char* dst;
  if (row == 0) dst = lastLine0;
  else if (row == 1) dst = lastLine1;
  else dst = lastLine2;
  int i = 0;
  while (text[i] != '\0' && i < 24) { dst[i] = text[i]; i++; }
  dst[i] = '\0';

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_myfonts);
  
  u8g2.setCursor(0, 14);
  u8g2.print(lastLine0);
  
  u8g2.setCursor(0, 32);
  u8g2.print(lastLine1);

  u8g2.setCursor(0, 50);
  u8g2.print(lastLine2);
  
  u8g2.sendBuffer();
}

float readWeight(HX711& scale, const char* name, float calibFactor) {
  unsigned long start = millis();
  int retry = 3;
  while (retry > 0 && !scale.is_ready() && millis() - start < CMD_TIMEOUT) {
    delay(100);
    retry--;
  }
  long rawSum = 0;
  for (int i = 0; i < 3; i++) {
    rawSum += scale.read();
    delay(60);
  }
  long avgRaw = rawSum / 3;
  long tareOffset = scale.get_offset();
  float weight = (float)(avgRaw - tareOffset) / calibFactor;
  weight = abs(weight);
  if (weight < 0.1) weight = 0;
  if (weight > WEIGHT_LIMIT) weight = WEIGHT_LIMIT;
  return weight;
}

void cleanSensors() {
  scale1.power_up();
  scale2.power_up();
  delay(50);
}

// ==========================
// ✅ 修复：消息显示也用安全坐标
// ==========================
void displayMsg(const char* msg, unsigned int delayT) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_unifont_myfonts);
  u8g2.setCursor(0, 14);
  u8g2.print(msg);
  u8g2.sendBuffer();
  delay(delayT);
}

void showErr(const char* name, const char* msg) {
  char errLine1[17] = {0}, errLine2[17] = {0};
  snprintf(errLine1, 17, "錯誤:%s", name);
  snprintf(errLine2, 17, "%s", msg);
  updateLCD(0, errLine1);
  updateLCD(1, errLine2);
  delay(3000);
}

int strLen(const char* str) {
  int len = 0;
  while (str[len] != '\0' && len < 255) len++;
  return len;
}

void strCpy(char* dest, const char* src) {
  int i = 0;
  while (src[i] != '\0' && i < 255) {
    dest[i] = src[i];
    i++;
  }
  dest[i] = '\0';
}

void strCat(char* dest, const char* src) {
  int destLen = strLen(dest);
  int i = 0;
  while (src[i] != '\0' && destLen + i < 255) {
    dest[destLen + i] = src[i];
    i++;
  }
  dest[destLen + i] = '\0';
}

bool isInputValid(const char* input) {
  int len = strLen(input);
  if (len == 0) return false;
  if (len == 1 && input[0] == '.') return false;
  int dotCount = 0;
  for (int i=0; i<len; i++) {
    if (input[i] == '.') dotCount++;
    if (dotCount > 1) return false;
  }
  return true;
}

// -------------------------- 7. 键盘测试模式 --------------------------
void keyTestLoop() {
  char key = scanKeypad();
  if (key != '\0' && (millis() - lastKeyPrintTime > 1000)) {
    if (key == 'B') {
      displayMsg("離開測試模式", 2000);
      setup();
      return;
    }
    char lcdLine1[17], lcdLine2[17];
    snprintf(lcdLine1, 17, "按鍵: %c", key);
    snprintf(lcdLine2, 17, "B=離開");
    updateLCD(0, lcdLine1);
    updateLCD(1, lcdLine2);
    lastKeyPrintTime = millis();
  }
  delay(50);
}

// -------------------------- 8. 菜单与功能实现 --------------------------
// ==========================
// ✅ 菜单一次显示 3 个
// ==========================
void displayMenu() {
  int m1 = menuSel % MENU_CNT;
  int m2 = (menuSel + 1) % MENU_CNT;
  int m3 = (menuSel + 2) % MENU_CNT;

  updateLCD(0, menuItems[m1]);
  updateLCD(1, menuItems[m2]);
  updateLCD(2, menuItems[m3]);
}

// ==========================
// 4點模式輔助函式
// ==========================
// 蜂鳴器響 ms 毫秒（D9）
void beep(int ms) {
  tone(BUZZER_PIN, 2000);
  delay(ms);
  noTone(BUZZER_PIN);
}
// 格式化單點 ASTM 值：未記錄=---，最小那組加括號
void formatP(char* out, int val, bool recorded, bool marked) {
  if (!recorded) { strCpy(out, "---"); return; }
  if (marked) snprintf(out, 7, "(%d)", val);
  else snprintf(out, 7, "%d", val);
}
// 更新第2/3行 P1~P4（4點完成時，最小值那組加括號）
void updatePDisplay(int astmVal[4], int pt, bool done) {
  int minIdx = 0;
  if (done) {
    for (int i = 1; i < 4; i++) {
      if (astmVal[i] < astmVal[minIdx]) minIdx = i;
    }
  }
  char p[4][7];
  for (int i = 0; i < 4; i++) {
    formatP(p[i], astmVal[i], i < pt, done && (i == minIdx));
  }
  char l1[21], l2[21];
  snprintf(l1, 21, "P1:%s P2:%s", p[0], p[1]);
  snprintf(l2, 21, "P3:%s P4:%s", p[2], p[3]);
  updateLCD(1, l1);
  updateLCD(2, l2);
}
void execMenu(int item) {
  displayMsg(menuItems[item], 500);
  delay(500);

  switch(item) {
    case 0: {
      cleanSensors();
      updateLCD(2, "");   // 清除主選單殘留在第3行的字
      displayMsg("請勿碰觸", 500);
      updateLCD(1, "歸零中...");
      scale1.tare();
      scale2.tare();
      delay(500);
      static float lastW1 = -1, lastW2 = -1, lastASTM = -1;
      unsigned long lastDisplayTime = 0;
      const unsigned long FORCE_UPDATE_INTERVAL = 1000;
      while (true) {
        char key = scanKeypad();
        float weight1 = readWeight(scale1, "L", CALIB_FACTOR_2);
        float weight2 = readWeight(scale2, "R", CALIB_FACTOR_1);
        float totalWeight = weight1 + weight2;
        float astmValue = 440000.0f / (totalWeight + 0.0001f);
        bool needUpdate = fabs(weight1 - lastW1) > 1 || fabs(weight2 - lastW2) > 1 || fabs(astmValue - lastASTM) > 0.1;
        if (needUpdate || millis() - lastDisplayTime >= FORCE_UPDATE_INTERVAL) {
          char w1Str[8], w2Str[8];
          safeFloatPrint(w1Str, sizeof(w1Str), weight1, 0);
          safeFloatPrint(w2Str, sizeof(w2Str), weight2, 0);
          char line1[17];
          snprintf(line1, sizeof(line1), "左:%sg 右:%sg", w1Str, w2Str);
          updateLCD(0, line1);
          char astmStr[10];
          safeFloatPrint(astmStr, sizeof(astmStr), astmValue, 0);
          char line2[17];
          snprintf(line2, sizeof(line2), "ASTM:%s", astmStr);
          updateLCD(1, line2);
          lastW1 = weight1; lastW2 = weight2; lastASTM = astmValue;
          lastDisplayTime = millis();
        }
        if (key == 'B') { waitForKeyRelease('B'); break; }
        else if (key == 'U') {
          waitForKeyRelease('U');
          updateLCD(0, "歸零中...");
          updateLCD(1, "請稍候");
          scale1.tare(); scale2.tare();
          lastW1 = lastW2 = lastASTM = -1;
          delay(300);
        }
        delay(200);
      }
      cleanSensors();
      break;
    }
    case 1: {  // 撓度ASTM 4點：ASTM<1500 穩定1秒自動記錄，4點完成標最小
      cleanSensors();
      updateLCD(2, "");   // 清除主選單殘留在第3行的字
      updateLCD(0, "ASTM 4點模式");
      updateLCD(1, "歸零中...");
      scale1.tare();
      scale2.tare();
      delay(500);
      const float TRIGGER_LEVEL = 1500.0f;   // 觸發門檻：低於此值才記錄
      int pt = 0;
      int astmVal[4] = {0, 0, 0, 0};
      bool done = false;
      bool recording = false;
      bool requireRelease = false;    // 記錄後需回到1500以上才能觸發新一輪
      bool releasedAfterDone = false; // 4點完成後已回到1500以上
      unsigned long stableStart = 0;
      float stableRef = 0.0f;         // 穩定參考值（±2 內判定）
      // 初始畫面：第1行目前ASTM值，第2/3行 P1~P4
      updateLCD(0, "ASTM:---");
      updateLCD(1, "P1:--- P2:---");
      updateLCD(2, "P3:--- P4:---");
      while (true) {
        char key = scanKeypad();
        if (key == 'B') { waitForKeyRelease('B'); break; }
        if (key == 'U') {
          // 長按U：全部重測
          unsigned long pressStart = millis();
          bool longPress = false;
          while (checkKey(KEYU_ROW, KEYU_COL)) {
            if (millis() - pressStart >= LONG_PRESS_TIME) { longPress = true; break; }
            delay(10);
          }
          if (longPress) {
            waitForKeyRelease('U');
            pt = 0;
            done = false;
            recording = false;
            requireRelease = false;
            releasedAfterDone = false;
            for (int i = 0; i < 4; i++) astmVal[i] = 0;
            updateLCD(0, "重測 歸零中...");
            scale1.tare(); scale2.tare();
            delay(500);
            updateLCD(0, "ASTM:---");
            updateLCD(1, "P1:--- P2:---");
            updateLCD(2, "P3:--- P4:---");
            lastKeyTime = millis();
          }
        }
        // 讀取兩顆感測器 → 目前 ASTM 值
        float w1 = readWeight(scale1, "L", CALIB_FACTOR_2);
        float w2 = readWeight(scale2, "R", CALIB_FACTOR_1);
        float total = w1 + w2;
        float astm = 440000.0f / (total + 0.0001f);
        if (astm > 999999.0f) astm = 999999.0f;
        int astmInt = (int)astm;
        // 第1行：目前ASTM值
        char line0[17];
        snprintf(line0, 17, "ASTM:%d", astmInt);
        updateLCD(0, line0);
        // 觸發：ASTM<1500 且在 ±2 內穩定1秒 → 記錄；回到1500以上才可觸發新一輪
        if (!done) {
          if (astm < TRIGGER_LEVEL && !requireRelease) {
            if (!recording) {
              recording = true;
              stableStart = millis();
              stableRef = astm;
            } else if (fabs(astm - stableRef) <= 2.0f) {
              if (millis() - stableStart >= 1000) {
                astmVal[pt] = astmInt;
                pt++;
                recording = false;
                requireRelease = true;   // 需回到1500以上才可再觸發
                beep(200);               // 蜂鳴器響 0.2 秒
                if (pt >= 4) done = true;
                updatePDisplay(astmVal, pt, done);
              }
            } else {
              // 偏移超過 ±2：重新開始穩定計時
              stableRef = astm;
              stableStart = millis();
            }
          } else {
            recording = false;
            if (astm >= TRIGGER_LEVEL) requireRelease = false;
          }
        } else {
          // 4點完成：回到1500以上後，再次降到1500以下 → 自動清零重來
          if (astm >= TRIGGER_LEVEL) releasedAfterDone = true;
          if (releasedAfterDone && astm < TRIGGER_LEVEL) {
            pt = 0;
            done = false;
            recording = true;          // 這次按下直接開始 P1 的穩定計時
            stableStart = millis();
            stableRef = astm;
            requireRelease = false;
            releasedAfterDone = false;
            for (int i = 0; i < 4; i++) astmVal[i] = 0;
            updatePDisplay(astmVal, pt, done);
          }
        }
        delay(100);
      }
      cleanSensors();
      break;
    }
    case 2: {
      cleanSensors();
      updateLCD(2, "");   // 清除主選單殘留在第3行的字
      displayMsg("AMO 模式", 500);
      updateLCD(1, "歸零中...");
      scale1.tare(); scale2.tare();
      delay(1000);
      float lastSpine = -1.0f, lastBow = -1.0f;
      unsigned long lastDisplayTime = 0;
      const unsigned long FORCE_UPDATE_INTERVAL = 1000;
      bool needTare = false;
      while (true) {
        if (needTare) {
          updateLCD(0, "歸零中...");
          updateLCD(1, "請稍候");
          scale1.tare(); scale2.tare();
          delay(500);
          needTare = false;
          lastSpine = lastBow = -1.0f;
          updateLCD(0, "等待測量");
          updateLCD(1, "U=歸零 B=返回");
        }
        char loopKey = scanKeypad();
        if (loopKey == 'B') { waitForKeyRelease('B'); break; }
        if (loopKey == 'U') { waitForKeyRelease('U'); needTare = true; }
        float weight1 = readWeight(scale1, "L", CALIB_FACTOR_2);
        float weight2 = readWeight(scale2, "R", CALIB_FACTOR_1);
        float totalWeight = weight1 + weight2;
        float spineValue = (totalWeight > 0.1f) ? ((440000.0f / totalWeight) * 0.825f) : 0.0f;
        float bowPound = (spineValue > 0.1f) ? (26.0f / (spineValue / 1000.0f)) : 0.0f;
        bool needUpdate = fabs(spineValue - lastSpine) > 0.1f || fabs(bowPound - lastBow) > 0.1f;
        if (needUpdate || millis() - lastDisplayTime >= FORCE_UPDATE_INTERVAL) {
          char spineStr[8]; safeFloatPrint(spineStr, sizeof(spineStr), spineValue, 1);
          char line1[17]; snprintf(line1, sizeof(line1), "Spine:%s", spineStr);
          updateLCD(0, line1);
          char bowStr[8]; safeFloatPrint(bowStr, sizeof(bowStr), bowPound, 1);
          char line2[17]; snprintf(line2, sizeof(line2), "磅數:%slb", bowStr);
          updateLCD(1, line2);
          lastSpine = spineValue; lastBow = bowPound;
          lastDisplayTime = millis();
        }
        delay(200);
      }
      cleanSensors();
      break;
    }
    case 3: {
      cleanSensors();
      updateLCD(2, "");   // 清除主選單殘留在第3行的字
      float arrowLength = getNumericInput("輸入箭長(mm)", 100, 2000, 4, false);
      if (arrowLength == -1) { displayMsg("已取消", 1000); return; }
      char lenInfo[17]; snprintf(lenInfo, 17, "長度:%dmm", (int)arrowLength);
      updateLCD(0, lenInfo);
      updateLCD(1, "歸零中...");
      scale1.tare(); scale2.tare();
      delay(500);
      while (true) {
        char loopKey = scanKeypad();
        if (loopKey == 'U') {
          waitForKeyRelease('U');
          updateLCD(0, "歸零中...");
          updateLCD(1, "請稍候");
          scale1.tare(); scale2.tare();
          delay(500);
        } else if (loopKey == 'B') { waitForKeyRelease('B'); break; }
        float weight1 = readWeight(scale1, "L", CALIB_FACTOR_2);
        float weight2 = readWeight(scale2, "R", CALIB_FACTOR_1);
        float totalWeight = weight1 + weight2;
        float cog = 0.0f, foc = 0.0f;
        if (totalWeight > 0.1f) {
          cog = ((weight2 - weight1) * SENSOR_DIST) / totalWeight + SENSOR_DIST;
          foc = ((cog - arrowLength / 2) / arrowLength) * 100;
        }
        snprintf(line, 17, "重心:%dmm", (int)cog);
        updateLCD(0, line);
        snprintf(line, 17, "FOC:%d%%", (int)foc);
        updateLCD(1, line);
        delay(200);
      }
      cleanSensors();
      break;
    }
    case 4: {
      cleanSensors();
      updateLCD(2, "");   // 清除主選單殘留在第3行的字
      updateLCD(0, "電子秤模式");
      updateLCD(1, "U=單位/歸零");
      scale1.tare(); scale2.tare();
      delay(2000);
      bool showGrams = true;
      float lastW1 = -1, lastW2 = -1, lastTotal = -1;
      unsigned long lastUpdate = 0;
      while (true) {
        char key = scanKeypad();
        if (key == 'U') {
          unsigned long pressStart = millis();
          bool longPress = false;
          while (checkKey(KEYU_ROW, KEYU_COL)) {
            if (millis() - pressStart >= LONG_PRESS_TIME) {
              longPress = true;
              updateLCD(0, "歸零中...");
              updateLCD(1, "請稍候");
              scale1.tare(); scale2.tare();
              delay(800);
              lastW1 = lastW2 = lastTotal = -1;
              updateLCD(0, "電子秤模式");
              updateLCD(1, "U=單位/歸零");
              break;
            }
            delay(10);
          }
          if (!longPress) {
            waitForKeyRelease('U');
            showGrams = !showGrams;
            displayMsg(showGrams ? "公克(g)" : "格令(gr)", 500);
            lastW1 = lastW2 = lastTotal = -1;
            updateLCD(0, "電子秤模式");
            updateLCD(1, "U=單位/歸零");
          }
        }
        if (key == 'B') { waitForKeyRelease('B'); break; }
        if (millis() - lastUpdate >= 500) {
          lastUpdate = millis();
          float w1 = readWeight(scale1, "L", CALIB_FACTOR_2);
          float w2 = readWeight(scale2, "R", CALIB_FACTOR_1);
          float total = w1 + w2;
          float w1d = showGrams ? w1 : w1 * G_TO_GR;
          float w2d = showGrams ? w2 : w2 * G_TO_GR;
          float totd = showGrams ? total : total * G_TO_GR;
          const char* unit = showGrams ? "g" : "gr";
          if (fabs(w1d - lastW1) > 0.1 || fabs(w2d - lastW2) > 0.1) {
            char s1[6], s2[6];
            safeFloatPrint(s1, 6, w1d, 0); safeFloatPrint(s2, 6, w2d, 0);
            snprintf(line, 17, "左:%s%s 右:%s%s", s1, unit, s2, unit);
            updateLCD(0, line);
            lastW1 = w1d; lastW2 = w2d;
          }
          if (fabs(totd - lastTotal) > 0.1) {
            char st[6]; safeFloatPrint(st, 6, totd, 0);
            snprintf(line, 17, "總重:%s%s", st, unit);
            updateLCD(1, line);
            lastTotal = totd;
          }
        }
        delay(10);
      }
      break;
    }
    case 5: weightConversion(); break;
    case 6: lengthConversion(); break;
  }
  displayMenu();
}

void weightConversion() {
  bool mode = 0;
  char inputBuf[10] = "0";
  const char* unitFrom[2] = {"公克", "格令"};
  const char* unitTo[2] = {"格令", "公克"};
  while (true) {
    char key = scanKeypad();
    if (key == 'U') {
      unsigned long pressStart = millis();
      bool longPress = false;
      while (checkKey(KEYU_ROW, KEYU_COL)) {
        if (millis() - pressStart >= LONG_PRESS_TIME) {
          mode = !mode;
          displayMsg(mode ? "格令→公克" : "公克→格令", 500);
          longPress = true;
          break;
        }
        delay(10);
      }
      if (!longPress) {
        waitForKeyRelease('U');
        int len = strLen(inputBuf);
        if (len > 1) inputBuf[len-1] = '\0';
        else strCpy(inputBuf, "0");
      }
    }
    if (key == 'B') { waitForKeyRelease('B'); break; }
    if (key >= '0' && key <= '9') {
      waitForKeyRelease(key);
      int len = strLen(inputBuf);
      if (len < 5) {
        if (len == 1 && inputBuf[0] == '0' && key != '0') inputBuf[0] = key;
        else if (!(len == 1 && inputBuf[0] == '0')) {
          inputBuf[len] = key; inputBuf[len+1] = '\0';
        }
      } else displayMsg("最多5位", 500);
    }
    float val = atof(inputBuf);
    float result = (mode == 0) ? val * G_TO_GR : val * GR_TO_G;
    long resInt = round(result * 10);

    char line0[17], line1[17], line2[17];
    snprintf(line0, 17, "%s>>>%s", unitFrom[mode], unitTo[mode]);
    snprintf(line1, 17, ":%s", inputBuf);
    snprintf(line2, 17, "=%d.%d%s", (int)(resInt/10), (int)abs(resInt%10), unitTo[mode]);

    updateLCD(0, line0);
    updateLCD(1, line1);
    updateLCD(2, line2);
    delay(KEY_DELAY);
  }
}

void lengthConversion() {
  bool mode = 0;
  char inputBuf[10] = "0";
  const char* unitFrom[2] = {"英吋", "公分"};
  const char* unitTo[2] = {"公分", "英吋"};
  while (true) {
    char key = scanKeypad();
    if (key == 'U') {
      unsigned long pressStart = millis();
      bool longPress = false;
      while (checkKey(KEYU_ROW, KEYU_COL)) {
        if (millis() - pressStart >= LONG_PRESS_TIME) {
          mode = !mode;
          displayMsg(mode ? "公分→英吋" : "英吋→公分", 500);
          longPress = true;
          break;
        }
        delay(10);
      }
      if (!longPress) {
        waitForKeyRelease('U');
        int len = strLen(inputBuf);
        if (len > 1) inputBuf[len-1] = '\0';
        else strCpy(inputBuf, "0");
      }
    }
    if (key == 'B') { waitForKeyRelease('B'); break; }
    if (key >= '0' && key <= '9') {
      waitForKeyRelease(key);
      int len = strLen(inputBuf);
      if (len < 5) {
        if (len == 1 && inputBuf[0] == '0' && key != '0') inputBuf[0] = key;
        else if (!(len == 1 && inputBuf[0] == '0')) {
          inputBuf[len] = key; inputBuf[len+1] = '\0';
        }
      } else displayMsg("最多5位", 500);
    }
    float val = atof(inputBuf);
    float result = (mode == 0) ? val * IN_TO_CM : val * CM_TO_IN;
    long resInt = round(result * 100);

    // 👇 改成 3 行，和你风格完全一致
    char line0[17], line1[17], line2[17];
    snprintf(line0, 17, "%s>>>%s", unitFrom[mode], unitTo[mode]);
    snprintf(line1, 17, ":%s", inputBuf);
    snprintf(line2, 17, "=%d.%02d%s", (int)(resInt/100), (int)abs(resInt%100), unitTo[mode]);

    updateLCD(0, line0);
    updateLCD(1, line1);
    updateLCD(2, line2);

    delay(KEY_DELAY);
  }
}

// -------------------------- 9. 初始化 --------------------------
void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.enableUTF8Print();
  displayMsg("系統啟動中", 1000);
  
  // scale1 = 左側（HX711-2 模組，D10/D11）；scale2 = 右側（HX711-1 模組，D12/D13）
  scale1.begin(HX711_2_DOUT, HX711_2_SCK);
  scale2.begin(HX711_1_DOUT, HX711_1_SCK);
  cleanSensors();
  displayMsg("就緒 按1-7選單", 1000);
  displayMenu();
  lastMenuSwitchTime = millis();
}

// -------------------------- 10. 主循环 --------------------------
void loop() {
  if (KEY_TEST_MODE) {
    keyTestLoop();
  } else {
    // 有選單鍵被按下時暫停自動輪播，避免按鍵瞬間畫面跳屏
    if (!anyMenuKeyDown() && millis() - lastMenuSwitchTime >= MENU_AUTO_SWITCH_MS) {
      menuSel = (menuSel + 1) % MENU_CNT;
      displayMenu();
      lastMenuSwitchTime = millis();
    }
    char key = scanKeypad();
    if (key >= '1' && key <= '7') {
      lastMenuSwitchTime = millis(); // 偵測到有效按鍵：凍結輪播計時
      waitForKeyRelease(key);
      int idx = key - '1';
      if (idx < MENU_CNT) {
        menuSel = idx;
        execMenu(idx);
        lastMenuSwitchTime = millis();
      }
    }
    delay(10);
  }
}