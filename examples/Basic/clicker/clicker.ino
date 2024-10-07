#include "M5Capsule.h"
#include <FastLED.h>
#include <BleKeyboard.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_ble_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>

#define HOLD_PIN 46
#define LED_PIN 21  // GPIO21 pin for NeoPixel LED
#define NUM_LEDS 1

CRGB leds[NUM_LEDS];
CRGB colorArray[] = { CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Yellow, CRGB::Purple, CRGB::Cyan, CRGB::White };
int currentColorIndex = 0;

unsigned long lastButtonPress = 0;
const unsigned long doubleClickTime = 300;

// Define a static MAC address
uint8_t staticMacAddress[6] = {0x24, 0x0A, 0xC4, 0x9A, 0xE5, 0x10};

BleKeyboard bleKeyboard("M5Capsule Clicker");

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
  if (btStart()) {
    Serial.println("Bluetooth initialized");
    esp_bluedroid_init();
    if (esp_bluedroid_enable() != ESP_OK) {
      Serial.println("Failed to enable bluedroid");
      return;
    }
  } else {
    Serial.println("Failed to initialize Bluetooth");
    return;
  }

  // Set security parameters for bonding
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

  bleKeyboard.begin();
  waitForBluetoothConnection();
}

void loop() {
  M5Capsule.update();

  if (M5Capsule.BtnA.wasPressed()) {
    unsigned long currentTime = millis();

    if (currentTime - lastButtonPress <= doubleClickTime) {
      // Double-click detected
      Serial.println("Button A double-clicked. Sending PageUp.");
      if (bleKeyboard.isConnected()) {
        bleKeyboard.write(KEY_PAGE_UP);
        delay(50);
        bleKeyboard.write(KEY_PAGE_UP);
        delay(50);
        bleKeyboard.releaseAll();
      } else {
        Serial.println("Bluetooth not connected. Attempting to reconnect...");
        reconnectBluetooth();
      }
      leds[0] = CRGB::Black;
      FastLED.show();
    } else {
      // Single click
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

    lastButtonPress = currentTime;
  }

  if (Serial.available() > 0) {
    String input = Serial.readString();
    input.trim();

    if (input.equalsIgnoreCase("reset")) {
      Serial.println("Resetting the device...");
      esp_restart();
    } else if (input.equalsIgnoreCase("reconnect")) {
      Serial.println("Attempting to reconnect Bluetooth...");
      reconnectBluetooth();
    } else {
      Serial.println("Press BtnA to cycle through colors and send PageDown. Double-click BtnA to send PageUp.");
      Serial.println("Type 'reset' to restart the device or 'reconnect' to reinitialize Bluetooth.");
    }
  }
}

// Helper function to wait until Bluetooth is connected
void waitForBluetoothConnection() {
  while (!bleKeyboard.isConnected()) {
    Serial.println("Waiting for Bluetooth connection...");
    delay(500);  // Wait for half a second before checking again
  }
  Serial.println("Bluetooth connected.");
}

// Helper function to reconnect Bluetooth
void reconnectBluetooth() {
  bleKeyboard.end();  // Close the existing BLE session
  delay(500);         // Wait a bit before trying to reconnect
  bleKeyboard.begin();
  waitForBluetoothConnection();  // Ensure connection before proceeding
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
