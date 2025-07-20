#include <bluefruit.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <services/BLEHidGamepad.h>
#include <services/BLEHidAdafruit.h>


// --- Info del dispositivo ---
#define VENDOR_ID_SOURCE  0x01
#define VENDOR_ID         0x054c     // Sony
#define PRODUCT_ID        0x0ce6     // DualSense
#define PRODUCT_VERSION   0x0005

// --- OLED config ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDR     0x3C

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Matriz de teclado 5x5 ---
const uint8_t ROWS = 5;
const uint8_t COLS = 5;
uint8_t rowPins[ROWS] = {9, 10, 11, 12, 13};  // Fila 0–4
uint8_t colPins[COLS] = {2, 3, 4, 5, 8};      // Columna 0–4

// --- Mapeo de botones ---
uint8_t buttonMap[ROWS][COLS] = {
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },      // 0=⬜,1=❌,2=🅾️,3=🔺
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },      // 4=L1,5=R1 6y7 serian L2R2,8=share,9=options
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },      // 10=L3(joyswitch),11=R3,12=PS,13=touch,
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },  
  { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } 
};

// --- HID Keyboard mapping for free matrix buttons ---
const uint8_t hidKeys[ROWS][COLS] = {
  { HID_KEY_ESCAPE, HID_KEY_1, HID_KEY_2, HID_KEY_3, HID_KEY_4 },           
  { HID_KEY_CAPS_LOCK,  HID_KEY_Q, HID_KEY_W, HID_KEY_E, HID_KEY_R },          
  { HID_KEY_SHIFT_LEFT, HID_KEY_A, HID_KEY_S, HID_KEY_D, HID_KEY_F },     
  { HID_KEY_CONTROL_LEFT, HID_KEY_Z, HID_KEY_X, HID_KEY_C, HID_KEY_V },
  { 0, 0, 0, 0, HID_KEY_SPACE } 
};

// --- Joystick ---
#define JOY_VRX   A0
#define JOY_VRY   A1
#define JOY_SW    14   // Botón joystick

// --- Batería ---
#define BATTERY_PIN A2 // Pin analógico para lectura de batería

const float R1 = 330000.0; // Ohms
const float R2 = 330000.0; // Ohms
const float VREF = 3.30;   // Voltaje de referencia ADC
const int ADC_BITS = 12;   // Precisión ADC si tu core lo soporta
const int ADC_MAX = 4095;  // 4095 para 12 bits, 1023 para 10 bits

unsigned long lastBatterySample = 0;
float batteryVoltage = 0;
float batteryPercent = 100;

BLEDis bledis;
BLEHidGamepad blegamepad;
BLEHidAdafruit blekeyboard;

bool isConnected = false;

// Flags D-pad
bool dpad_up = false, dpad_down = false, dpad_left = false, dpad_right = false;

// --- Dibujo del icono de batería con barras y parpadeo si <10% ---
void drawBatteryIcon(int x, int y, float percent, bool blink) {
  int w = 32; // ancho batería
  int h = 14; // alto batería
  int capW = 4; // ancho terminal
  int capH = 6; // alto terminal

  oled.drawRect(x, y, w, h, SSD1306_WHITE);
  oled.fillRect(x + w, y + (h-capH)/2, capW, capH, SSD1306_WHITE);

  int bars = 0;
  if (percent > 87) bars = 4;
  else if (percent > 62) bars = 3;
  else if (percent > 37) bars = 2;
  else if (percent > 12) bars = 1;
  else bars = 0;

  if (percent < 10 && blink) {
    return; // solo el marco, barras parpadean
  }

  int pad = 2;
  int barW = 6;
  int barH = h - 4;
  int gap = 2;

  for (int i = 0; i < bars; i++) {
    int bx = x + pad + i * (barW + gap);
    int by = y + 2;
    oled.fillRect(bx, by, barW, barH, SSD1306_WHITE);
  }
}

// --- Lectura de batería promediada ---
float readBatteryVoltageAverage() {
  long sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BATTERY_PIN);
    delayMicroseconds(200);
  }
  float raw = sum / (float)samples;
  float vA2 = (raw * VREF) / ADC_MAX;
  float vBat = vA2 * ((R1 + R2) / R2);
  return vBat;
}

void setup() {
  Serial.begin(115200);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DualSense Wireless Controller");

  bledis.setManufacturer("Sony Interactive Entertainment");
  bledis.setModel("DualSense");

  const uint8_t pnpID[7] = {
    VENDOR_ID_SOURCE,
    (uint8_t)(VENDOR_ID & 0xFF), (uint8_t)(VENDOR_ID >> 8),
    (uint8_t)(PRODUCT_ID & 0xFF), (uint8_t)(PRODUCT_ID >> 8),
    (uint8_t)(PRODUCT_VERSION & 0xFF), (uint8_t)(PRODUCT_VERSION >> 8)
  };
  bledis.setPNPID((const char*)pnpID, sizeof(pnpID));
  bledis.begin();

  blegamepad.begin();
  blekeyboard.begin();
  Bluefruit.Advertising.addService(blekeyboard);
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);

  Bluefruit.Periph.setConnectCallback([](uint16_t){ isConnected = true; });
  Bluefruit.Periph.setDisconnectCallback([](uint16_t, uint8_t){ isConnected = false; });

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
  Bluefruit.Advertising.addService(blegamepad);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oled.clearDisplay();
  oled.display();

  for (uint8_t r = 0; r < ROWS; r++) pinMode(rowPins[r], OUTPUT);
  for (uint8_t c = 0; c < COLS; c++) pinMode(colPins[c], INPUT_PULLUP);

  pinMode(JOY_VRX, INPUT);
  pinMode(JOY_VRY, INPUT);
  pinMode(JOY_SW, INPUT_PULLUP);

  analogReadResolution(ADC_BITS);
  pinMode(BATTERY_PIN, INPUT);
}

uint16_t readMatrixButtons() {
  uint16_t buttons = 0;
  dpad_up = dpad_down = dpad_left = dpad_right = false;

  for (uint8_t r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    for (uint8_t c = 0; c < COLS; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        uint8_t btn = buttonMap[r][c];
        if (btn <= 13) {
          buttons |= (1 << btn);
        } else if (btn == 29) {
          dpad_up = true;
        } else if (btn == 30) {
          dpad_down = true;
        } else if (btn == 31) {
          dpad_left = true;
        } else if (btn == 32) {
          dpad_right = true;
        }
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
  return buttons;
}
void sendMatrixKeysAsKeyboard() {
  uint8_t modifier = 0;
  uint8_t keycode[6] = {0};
  uint8_t count = 0;
  bool anyKeyPressed = false;
  static bool keyPressedPreviously = false;

  for (uint8_t r = 0; r < ROWS; r++) {
    digitalWrite(rowPins[r], LOW);
    for (uint8_t c = 0; c < COLS; c++) {
      uint8_t key = hidKeys[r][c];
      if (key != 0 && digitalRead(colPins[c]) == LOW) {
        if (count < 6) keycode[count++] = key;
        anyKeyPressed = true;
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }

  if (count > 0) {
    blekeyboard.keyboardReport(modifier, keycode);
    keyPressedPreviously = true;
  } else if (keyPressedPreviously) {
    blekeyboard.keyRelease();
    keyPressedPreviously = false;
  }
}


void sendGamepadReport() {
  // Joystick analógico (usado como LX, LY)
  int joyX = analogRead(JOY_VRX);
  delay(1); // Evita arrastre ADC
  int joyY = analogRead(JOY_VRY);
  int8_t lx = map(joyX, 0, ADC_MAX, 127, -127); // ADC_MAX (4095) por 12 bits
  int8_t ly = map(joyY, 0, ADC_MAX, 127, -127);

  uint16_t buttons = readMatrixButtons();

  if (digitalRead(JOY_SW) == LOW) {
    buttons |= (1 << 10); // botón PS
  }

  uint8_t hat = 0;
  if (dpad_up) hat = 1;
  else if (dpad_up && dpad_right) hat = 2;
  else if (dpad_right) hat = 3;
  else if (dpad_down && dpad_right) hat = 4;
  else if (dpad_down) hat = 5;
  else if (dpad_down && dpad_left) hat = 6;
  else if (dpad_left) hat = 7;
  else if (dpad_up && dpad_left) hat = 8;

  if (isConnected) {
    hid_gamepad_report_t report = {0};
    report.buttons = buttons;
    report.x = lx;
    report.y = ly;
    report.hat = hat;

    blegamepad.report(&report);
    delay(5);
  }
}

void loop() {
  // Lectura de batería cada 30 segundos
  if (millis() - lastBatterySample > 30000) {
    lastBatterySample = millis();
    batteryVoltage = readBatteryVoltageAverage();
    batteryPercent = constrain((batteryVoltage - 3.0) * 100.0 / 1.2, 0, 100); // Para LiPo. Ajusta según tu batería.
  }

  // OLED
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  static bool blinkState = true;
  static unsigned long lastBlink = 0;
  if (batteryPercent < 10) {
    if (millis() - lastBlink > 400) {
      blinkState = !blinkState;
      lastBlink = millis();
    }
  } else {
    blinkState = true;
  }
  drawBatteryIcon(0, 0, batteryPercent, !blinkState);

  oled.setCursor(40, 2);
  oled.print((int)batteryPercent);
  oled.print("%");

  oled.setCursor(0, 16);
  oled.print(isConnected ? "Conectado" : "Desconectado");

  // NO mostrar el voltaje

  oled.display();

  sendMatrixKeysAsKeyboard();
  sendGamepadReport();
  delay(10);
}