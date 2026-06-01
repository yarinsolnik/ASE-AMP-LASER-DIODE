// Nano ESP32 UART Diagnostic for SF6015 and TC1540
// Tests TX/RX, Errors, and ACTIVELY FIRES THE LASER

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <HardwareSerial.h>

// -------- OLED --------
#define OLED_ADDR 0x3C
Adafruit_SH1107 display(128, 128, &Wire);

// ======== UART SETUP ========

HardwareSerial sf(1);
const int SF_RX = 3; // D3 (RX) <- driver TX
const int SF_TX = 2; // D2 (TX) -> driver RX

HardwareSerial tc(2);
const int TC_RX = 5; // D9 (RX) <- TC1540 TX
const int TC_TX = 4; // D8 (TX) -> TC1540 RX

// ======== LASER COMMAND FUNCTIONS ========

// Helper function to send commands and parse space/comma responses
String queryLaser(String cmd) {
  while (sf.available()) sf.read();
  sf.print(cmd + "\r");
  
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    if (sf.available()) {
      String response = sf.readStringUntil('\r');
      response.trim();
      
      if (response.startsWith("K")) {
        int sepIndex = response.indexOf(',');
        if (sepIndex == -1) sepIndex = response.indexOf(' '); 
        
        if (sepIndex > 0) {
          String val = response.substring(sepIndex + 1);
          val.trim(); 
          return val;
        } else if (response == "K0000") {
          return "0"; 
        }
      }
      return "PARSE_ERR"; 
    }
  }
  return "TIMEOUT";
}

// --- NEW: Active Commands ---

void setLaserCurrent(String amps) {
  Serial.print("Setting Target Current to "); Serial.print(amps); Serial.println(" A...");
  sf.print("J0300," + amps + "\r");
  delay(100); // Give driver time to process
}

void enableLaser(bool turnOn) {
  if (turnOn) {
    Serial.println("COMMANDING LASER ON!");
    sf.print("J0502,1\r");
  } else {
    Serial.println("COMMANDING LASER OFF!");
    sf.print("J0502,0\r");
  }
  delay(100);
}

// --- Diagnostic Reads ---

bool testLaserUART() {
  String ping = queryLaser("J0300");
  return (ping != "TIMEOUT");
}

String getLaserError() {
  String err = queryLaser("J0500");
  if (err == "0" || err == "0000") return "NONE";
  return err;
}

String getLaserTargetCurrent() {
  String cur = queryLaser("J0300");
  if (cur == "PARSE_ERR" || cur == "TIMEOUT") return cur;
  return cur + " A";
}

String getLaserActualCurrent() {
  String cur = queryLaser("J0301");
  if (cur == "PARSE_ERR" || cur == "TIMEOUT") return cur;
  return cur + " A";
}

bool testTECUART() {
  while (tc.available()) tc.read();
  tc.print("J0A10\r"); 
  unsigned long t0 = millis();
  while (millis() - t0 < 100) {
    if (tc.available()) {
      String response = tc.readStringUntil('\r');
      response.trim();
      if (response.length() > 0) return true;
    }
  }
  return false; 
}

// ======== DASHBOARD ========

void updateDisplay(bool laserOK, String laserErr, String tgtCur, String actCur, bool tecOK) {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  int y = 0;
  display.setTextSize(1);
  display.setCursor(0, y); display.println("UART DIAGNOSTICS"); 
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);
  
  // --- Laser Status ---
  y = 15;
  display.setTextSize(2);
  display.setCursor(0, y); 
  display.print("LASER:");
  display.println(laserOK ? "YES" : "NO");

  // --- Laser Diagnostics ---
  display.setTextSize(1);
  
  y = 35; display.setCursor(0, y);
  display.print(" ERR: "); display.println(laserOK ? laserErr : "N/A");

  y = 45; display.setCursor(0, y);
  display.print(" TGT: "); display.println(laserOK ? tgtCur : "N/A");

  y = 55; display.setCursor(0, y);
  display.print(" ACT: "); display.println(laserOK ? actCur : "N/A"); 

  // --- TEC Status ---
  y = 75;
  display.setTextSize(2);
  display.setCursor(0, y); 
  display.print("TEC  :");
  display.println(tecOK ? "YES" : "NO");

  display.display();
}

// ======== SETUP & LOOP ========

void setup() {
  Serial.begin(115200);
  delay(1000); 

  Wire.begin(D6, D7);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("OLED not found!");
  }
  display.setRotation(1);
  display.clearDisplay();
  display.display();

  sf.begin(115200, SERIAL_8N1, SF_RX, SF_TX);
  tc.begin(115200, SERIAL_8N1, TC_RX, TC_TX);

  // --- FIRING SEQUENCE ---
  Serial.println("Starting up... firing laser in 5 seconds!");
  delay(5000); // 5 second safety buffer

  // 1. Set the target current to 0.1 Amps (100mA)
  setLaserCurrent("0.1"); 
  
  // 2. Send the Enable/ON command
  enableLaser(true); 
}

void loop() {
  bool laserStatus = testLaserUART();
  bool tecStatus = testTECUART();
  
  String laserErr = "N/A";
  String tgtCur = "N/A";
  String actCur = "N/A";
  
  if (laserStatus) {
    laserErr = getLaserError();
    tgtCur = getLaserTargetCurrent();
    actCur = getLaserActualCurrent();
  }

  updateDisplay(laserStatus, laserErr, tgtCur, actCur, tecStatus);
  delay(1000);
}