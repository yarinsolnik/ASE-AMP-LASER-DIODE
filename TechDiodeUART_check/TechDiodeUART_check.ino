// Nano ESP32 UART Diagnostic for SF6015 and TC1540
// Tests TX/RX lines and displays results on SH1107 OLED and Serial Monitor

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <HardwareSerial.h>

// -------- OLED --------
#define OLED_ADDR 0x3C
Adafruit_SH1107 display(128, 128, &Wire);

// ======== UART SETUP ========

// SF6015 (Laser) UART
HardwareSerial sf(1);
const int SF_RX = 3; // D3 (RX) <- driver TX
const int SF_TX = 2; // D2 (TX) -> driver RX

// TC1540 (TEC) UART
HardwareSerial tc(2);
const int TC_RX = 5; // D9 (RX) <- TC1540 TX
const int TC_TX = 4; // D8 (TX) -> TC1540 RX

// ======== PING FUNCTIONS ========

// Flush buffer, send a read command, and wait for a response
bool testLaserUART() {
  // 1. Clear any leftover garbage in the receive buffer
  while (sf.available()) sf.read(); 
  
  // 2. Send a benign read command (J0300 = Read Iset)
  sf.print("J0300\r");
  
  // 3. Wait up to 100ms for a response
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    if (sf.available()) {
      String response = sf.readStringUntil('\r');
      response.trim();
      // If we got anything starting with 'K' (Maiman standard reply), it works
      if (response.length() > 0) return true; 
    }
  }
  return false; // Timed out
}

bool testTECUART() {
  // 1. Clear buffer
  while (tc.available()) tc.read();
  
  // 2. Send benign read command (J0A10 = Read Tset)
  tc.print("J0A10\r");
  
  // 3. Wait up to 100ms for a response
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    if (tc.available()) {
      String response = tc.readStringUntil('\r');
      response.trim();
      if (response.length() > 0) return true;
    }
  }
  return false; // Timed out
}

// ======== DASHBOARD ========

void updateDisplay(bool laserOK, bool tecOK) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  int y = 0;
  display.setCursor(0, y); display.println("UART DIAGNOSTICS"); 
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);
  y += 20;

  // Laser Status
  display.setTextSize(2);
  display.setCursor(0, y); 
  display.print("LASER:");
  if (laserOK) {
    display.println("YES");
  } else {
    display.println("NO");
  }
  y += 30;

  // TEC Status
  display.setCursor(0, y); 
  display.print("TEC  :");
  if (tecOK) {
    display.println("YES");
  } else {
    display.println("NO");
  }

  display.display();
}

// ======== SETUP & LOOP ========

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to open
  Serial.println("Starting UART Diagnostics...");

  // Init OLED
  Wire.begin(D6, D7);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("OLED not found! Check I2C wiring.");
  }
  display.setRotation(1);
  display.clearDisplay();
  display.display();

  // Init UARTs
  sf.begin(115200, SERIAL_8N1, SF_RX, SF_TX);
  tc.begin(115200, SERIAL_8N1, TC_RX, TC_TX);
}

void loop() {
  // Test both connections
  bool laserStatus = testLaserUART();
  bool tecStatus = testTECUART();

  // Print to Serial Monitor
  Serial.print("Laser UART: "); Serial.print(laserStatus ? "YES" : "NO");
  Serial.print("  |  TEC UART: "); Serial.println(tecStatus ? "YES" : "NO");

  // Update OLED screen
  updateDisplay(laserStatus, tecStatus);

  // Wait 1 second before testing again
  delay(1000);
}