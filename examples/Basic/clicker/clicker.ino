#define USE_NIMBLE

#include "M5Capsule.h"
#include <FastLED.h>
#include <BleKeyboard.h>
// #include <esp_bt_main.h>
// #include <esp_bt_device.h>
// #include <esp_gap_ble_api.h>
// #include <esp_bt_defs.h>
// #include <esp_bt_main.h>
// Include necessary headers
#include <NimBLEDevice.h>
#include <esp_bt.h>
#include <OneButton.h>

// Test GIT aother

#define HOLD_PIN 46
#define LED_PIN 21  // GPIO21 pin for NeoPixel LED
#define NUM_LEDS 1

CRGB leds[NUM_LEDS];
CRGB colorArray[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Purple, CRGB::Cyan, CRGB::White };
int currentColorIndex = 0;

// Define a static MAC address
uint8_t staticMacAddress[6] = { 0x24, 0x0A, 0xC4, 0x9A, 0xE5, 0x10 };

BleKeyboard bleKeyboard("M5Capsule Clicker");

// Create OneButton object
#define BTN_PIN  42
OneButton button(BTN_PIN, true);

void setup() {
  auto cfg = M5.config();
  M5Capsule.begin(cfg);
  Serial.begin(115200);
  Serial.println("M5Capsule ready. Press BtnA to cycle through colors and send PageDown. Double-click BtnA to send PageUp.");

  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(50);
  leds[0] = CRGB::Black;
  FastLED.show();

  // Set the static MAC address
  esp_base_mac_addr_set(staticMacAddress);

  // Initialize Bluetooth
  bleKeyboard.begin();
  waitForBluetoothConnection();

  // Setup OneButton callbacks
  button.attachClick(handleSingleClick);
  button.attachDoubleClick(handleDoubleClick);
  button.attachLongPressStop(reconnectBluetooth);
}

void loop() {
  M5Capsule.update();
  button.tick();

  if (Serial.available() > 0) {
    String input = Serial.readString();
    input.trim();

    if (input.equalsIgnoreCase("reset")) {
      Serial.println("Resetting the device...");
      esp_restart();
    } else if (input.equalsIgnoreCase("reconnect")) {
      Serial.println("Attempting to reconnect Bluetooth..");
      reconnectBluetooth();
      delay(3000);
      Serial.println("Resetting the device...");
      esp_restart();
    } else {
      Serial.println("Press BtnA to cycle through colors and send PageDown. Double-click BtnA to send PageUp.");
      Serial.println("Type 'reset' to restart the device or 'reconnect' to reinitialize Bluetooth.");
    }
  }
}

void handleSingleClick() {
  Serial.println("Button A pressed. Sending PageDown.");
  if (bleKeyboard.isConnected()) {
    bleKeyboard.write(KEY_PAGE_DOWN);
    delay(50);
    bleKeyboard.releaseAll();
  } else {
    Serial.println("Bluetooth not connected. Attempting to reconnect...");
    reconnectBluetooth();
  }

  currentColorIndex = (currentColorIndex + 1) % (sizeof(colorArray) / sizeof(colorArray[0]));
  leds[0] = colorArray[currentColorIndex];
  FastLED.show();

  Serial.print("LED color changed to: ");
  Serial.println(getColorName(colorArray[currentColorIndex]));
}

void handleDoubleClick() {
  Serial.println("Button A double-clicked. Sending PageUp.");
  if (bleKeyboard.isConnected()) {
    bleKeyboard.write(KEY_PAGE_UP);
    delay(50);
    bleKeyboard.releaseAll();
  } else {
    Serial.println("Bluetooth not connected. Attempting to reconnect...");
    reconnectBluetooth();
  }
  leds[0] = CRGB::Black;
  FastLED.show();
}

// Helper function to wait until Bluetooth is connected
void waitForBluetoothConnection() {
  while (!bleKeyboard.isConnected()) {
    leds[0] = CRGB::Blue;
    FastLED.show();
    Serial.println("Black");
    delay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.println("Blue");
    delay(500);
    Serial.println("Waiting for Bluetooth connection...");
  }
  Serial.println("Bluetooth connected.");
}

// Helper function to reconnect Bluetooth
void reconnectBluetooth() {
  bleKeyboard.end();  // Close the existing BLE session
  for (int i = 0; i < 3; i++) {
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(500);
  }
  bleKeyboard.begin();
  
  // Blink the LED blue while waiting for connection
  waitForBluetoothConnection();
  
  Serial.println("Bluetooth connected.");

  delay(3000);
  Serial.println("Resetting the device...");
  esp_restart();
}

// Helper function to get color name as a string
String getColorName(CRGB color) {
  if (color == CRGB::Red) return "Red";
  if (color == CRGB::Green) return "Green";
  if (color == CRGB::Blue) return "Blue";
  if (color == CRGB::Yellow) return "Yellow";
  if (color == CRGB::Purple) return "Purple";
  if (color == CRGB::Cyan) return "Cyan";
  if (color == CRGB::White) return "White";
  return "Unknown";
}
