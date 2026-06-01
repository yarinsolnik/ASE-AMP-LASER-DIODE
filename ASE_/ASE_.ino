// Nano ESP32 ↔ SF6015 (Current via potentiometer) + SH1107 128x128 I2C OLED
// Maiman TC1540 TEC over UART (ALWAYS 25.00 °C setpoint; measured temp = external NTC)
// Single OLED page: "ASE Amplifier" with set/meas current, set/meas temp, interlocks
// Serial Monitor: 115200

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <HardwareSerial.h>
#include <math.h>

// -------- OLED --------
#define OLED_ADDR 0x3C
Adafruit_SH1107 display(128, 128, &Wire);

// ======== LASER (SF6015) SECTION ========

// Potentiometer (Laser)
const int POT_PIN = A0;     // wiper to A0, ends to 3V3 and GND
const float I_MAX = 4.0f;   // max set current [A]
const uint8_t ADC_SAMPLES = 8;
const float SEND_THRESHOLD = 0.02f;     // only send if change ≥ 0.02 A
const uint16_t SEND_RATE_MS = 100;      // rate-limit setpoint updates

// SF6015 UART
HardwareSerial sf(1);
const int SF_RX = 3; // D3 (RX)  <- driver TX (via 5V->3.3V divider/level shifter)
const int SF_TX = 2; // D2 (TX)  -> driver RX

// ---- helpers (SF6015) ----
void sendCmdSF(const char* s) { sf.print(s); sf.print('\r'); }

// PATCHED: robust line reader (accepts \r or \n, skips empty lines)
bool readLineSF(String& out, unsigned long timeout_ms = 800) {
  unsigned long t0 = millis();
  out = "";
  while (millis() - t0 < timeout_ms) {
    while (sf.available()) {
      char c = (char)sf.read();
      if (c == '\r' || c == '\n') {
        if (out.length() > 0) { out.trim(); return true; } // return non-empty line
      } else {
        out += c;
      }
    }
    delay(1);
  }
  return false; // timeout
}

uint16_t valFromK(const String& kline) {
  int sp = kline.indexOf(' ');
  if (sp < 0) return 0;
  return (uint16_t)strtoul(kline.substring(sp+1).c_str(), nullptr, 16);
}

// PATCHED: skip echo (e.g. "J0700") and wait for "Kxxxx yyyy"
bool readParamSF(const char* pcode, uint16_t &val) {
  String cmd = String("J") + pcode;
  sendCmdSF(cmd.c_str());

  String s;
  unsigned long t0 = millis();
  const unsigned long timeout_ms = 1000;
  while (millis() - t0 < timeout_ms) {
    if (!readLineSF(s, timeout_ms - (millis() - t0))) break;
    if (s.length() == 0) continue;                 // skip blanks
    if (s.charAt(0) == 'K' || s.charAt(0) == 'k') {
      val = valFromK(s);
      return true;
    }
    // else it's probably the echo "Jxxxx" — ignore and continue
  }
  return false;
}

void setIsetAmps(float amps) {
  float a = amps;
  if (a < 0) a = 0;
  if (a > I_MAX) a = I_MAX;
  uint16_t raw = (uint16_t)roundf(a * 100.0f);   // 0.01 A units
  char buf[24];
  snprintf(buf, sizeof(buf), "P0300 %04X", raw);
  sendCmdSF(buf);
  delay(40);
}

// --- SF6015 NTC interlock helpers (pin 14 of analog connector) ---
static void sf_setNTC_Beta(uint16_t beta) {
  char buf[24];
  snprintf(buf, sizeof(buf), "P0B0E %04X", beta);   // thermistor β (e.g. 3950 => 0x0F6E)
  sendCmdSF(buf); delay(40);
}
static void sf_setNTC_Limits(float lowC, float highC) {
  int16_t low  = (int16_t)roundf(lowC  * 10.0f);   // 0.1 °C units
  int16_t high = (int16_t)roundf(highC * 10.0f);
  char buf[24];
  snprintf(buf, sizeof(buf), "P0A05 %04X", (uint16_t)low);  sendCmdSF(buf); delay(40);
  snprintf(buf, sizeof(buf), "P0A06 %04X", (uint16_t)high); sendCmdSF(buf); delay(40);
}

// Bring-up sequence (START must be last) — single-action P0700 writes
void bringUpAndStartSF(float amps) {
  // Put the driver in a known stopped state first
  sendCmdSF("P0700 0010"); delay(80);   // STOP

  // Configure NTC protection (keep this if you want NTC over-temp shutdown active)
  sf_setNTC_Beta(0x0F94);               // β = 3988
  sf_setNTC_Limits(0.0f, 40.0f);        // adjust high limit if needed

  // Select UART/internal current setpoint
  sendCmdSF("P0700 0020"); delay(80);   // Internal current set

  // Select internal enable (so enable is not taken from the external enable pin)
  sendCmdSF("P0700 0400"); delay(80);   // Internal enable

  // Keep the external interlock ACTIVE as a real on/off switch
  sendCmdSF("P0700 1000"); delay(80);   // Allow interlock

  // Keep external NTC interlock ACTIVE too
  sendCmdSF("P0700 8000"); delay(80);   // Allow NTC interlock

  // Write desired current setpoint
  setIsetAmps(amps);

  // START must be the last 0700 action
  sendCmdSF("P0700 0008"); delay(100);  // START
}

// ADC / pot (Laser)
float readPotAmps() {
  uint32_t acc = 0;
  for (uint8_t i=0;i<ADC_SAMPLES;i++) { acc += analogRead(POT_PIN); delayMicroseconds(200); }
  float avg = acc / (float)ADC_SAMPLES;          // ESP32 ADC: 0..4095 default
  float amps = (avg / 4095.0f) * I_MAX;          // map to 0..I_MAX
  if (amps < 0) amps = 0;
  if (amps > I_MAX) amps = I_MAX;
  return amps;
}

// ======== TEC (TC1540) SECTION (fixed 25.00 °C; measured from external NTC) ========
HardwareSerial tc(2);
const int TC_RX = 5; // D5 (Arduino RX)  <- TC1540 TX
const int TC_TX = 4; // D4 (Arduino TX)  -> TC1540 RX

const float TEC_FIXED_SET_C = 25.00f;
const uint32_t TEC_REASSERT_MS = 3000;

void sendCmdTC(const char* s) { tc.print(s); tc.print('\r'); }

bool readLineTC(String& out, unsigned long timeout_ms = 500) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (tc.available()) { out = tc.readStringUntil('\r'); out.trim(); return true; }
  }
  return false;
}

bool readParamTC(const char* pcode, uint16_t &val) {
  String cmd = String("J") + pcode;
  sendCmdTC(cmd.c_str());
  String s;
  if (!readLineTC(s)) return false;   // expect "Kxxxx yyyy"
  val = valFromK(s);
  return true;
}

// 0A10 uses 0.01 °C units (setpoint)
void setTempC(float degC) {
  float t = degC;
  if (t < -100.0f) t = -100.0f; // sanity clamp
  if (t >  150.0f) t =  150.0f;
  uint16_t raw = (uint16_t)roundf(t * 100.0f);
  char buf[24];
  snprintf(buf, sizeof(buf), "P0A10 %04X", raw);
  sendCmdTC(buf);
  delay(50);
}

// write state to 0A1A (START/STOP, source selects, interlock)
void writeStateTC(uint16_t word) {
  char buf[24];
  snprintf(buf, sizeof(buf), "P0A1A %04X", word);
  sendCmdTC(buf);
  delay(60);
}

// Ensure external NTC is the feedback source (clear "temp source internal" bit)
void setTempSourceExternalNTC() {
  uint16_t state = 0;
  readParamTC("0A1A", state);
  state &= ~0x0002;       // 0 = external sensor (NTC)
  writeStateTC(state);
}

void bringUpAndStartTC(float tempC) {
  writeStateTC(0x0020);   // Internal temperature SET (UART/internal setpoint)
  writeStateTC(0x0400);   // Internal enable
  writeStateTC(0x1000);   // Allow interlock
  setTempSourceExternalNTC();
  setTempC(tempC);
  writeStateTC(0x0008);   // START (must be last)
}

// ======== DASHBOARD DISPLAY (single page) ========
void drawDashboard(const char* interlocksText,
                   float isetA, float iA,
                   float tsetC, float tmeasC) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  int y = 0;
  display.setCursor(0, y); display.println("ASE Amplifier"); y += 12;  // headline
  display.setCursor(0, y); display.print("Moran Photonics"); y += 12;

  display.setCursor(0, y); display.print("Iset: "); display.print(isetA, 2); display.println(" A"); y += 12;
  display.setCursor(0, y); display.print("Imeas: "); display.print(iA,   2); display.println(" A"); y += 12;

  display.setCursor(0, y); display.print("Tset: "); display.print(tsetC, 2); display.println(" C"); y += 12;
  display.setCursor(0, y); display.print("Tmeas: "); display.print(tmeasC, 2); display.println(" C"); y += 12;

  display.setCursor(0, y); display.print("Interlocks:"); y += 12;
  display.setCursor(0, y); display.println(interlocksText); y += 24;

  display.display();
}

// ======== SETUP / LOOP ========
unsigned long lastSendMs = 0;
float lastSentA = -100.0f;
unsigned long lastTECAssertMs = 0;

void setup() {
  Serial.begin(115200);
  delay(400);

  // OLED init (D6 SDA, D7 SCL on Nano ESP32)
  Wire.begin(6, 7);
  if (!display.begin(OLED_ADDR, true)) {
    Serial.println("SH1107 OLED not found at 0x3C (try 0x3D or check wiring).");
  }
  display.setRotation(1);
  display.clearDisplay();
  display.display();

  // LASER UART
  sf.begin(115200, SERIAL_8N1, SF_RX, SF_TX);
  delay(150);
  while (sf.available()) sf.read();

  // TEC UART
  tc.begin(115200, SERIAL_8N1, /*RX=*/TC_RX, /*TX=*/TC_TX);
  delay(150);
  while (tc.available()) tc.read();

  // Bring-up LASER at current pot
  float potA = readPotAmps();
  bringUpAndStartSF(potA);
  lastSentA = potA;
  lastSendMs = millis();

  // Bring-up TEC at FIXED 25.00 °C using EXTERNAL NTC feedback
  bringUpAndStartTC(TEC_FIXED_SET_C);
  lastTECAssertMs = millis();
}

void loop() {
  // ----- LASER: pot -> setpoint -----
  float potA = readPotAmps();
  unsigned long now = millis();
  if ((fabs(potA - lastSentA) >= SEND_THRESHOLD) && (now - lastSendMs >= SEND_RATE_MS)) {
    setIsetAmps(potA);
    lastSentA = potA;
    lastSendMs = now;
  }

  // LASER telemetry
  uint16_t v;
  float isetA = -1, iA = -1, vV = -1;
  bool startedLaser = false;
  uint16_t stateRawLaser = 0;

  if (readParamSF("0300", v)) isetA = v / 100.0f;  // Iset (0.01 A)
  if (readParamSF("0307", v)) iA    = v / 10.0f;   // I (0.1 A)
  if (readParamSF("0407", v)) vV    = v / 10.0f;   // V (0.1 V)
  if (readParamSF("0700", stateRawLaser)) startedLaser = (stateRawLaser & 0x0002);

/*if (isetA > 0.1 && iA < 0.01) {
  uint16_t s0700 = 0, s0800 = 0;
  if (readParamSF("0700", s0700)) Serial.printf("0700 = 0x%04X  ", s0700);
  if (readParamSF("0800", s0800)) Serial.printf("0800 = 0x%04X  ", s0800);
  Serial.printf("Iset=%.2f  Imeas=%.2f\n", isetA, iA);}*/

  // keep-alive: only assert START bit so interlock bits remain intact
  if (!startedLaser) {
    sendCmdSF("P0700 0008"); delay(60);
  }

  // SF6015 interlocks (0800)
  uint16_t k0800 = 0;
  bool have0800 = readParamSF("0800", k0800);
  bool il  = have0800 ? ((k0800 >> 1) & 1) : 0;  // interlock
  bool oc  = have0800 ? ((k0800 >> 3) & 1) : 0;  // overcurrent
  bool otw = have0800 ? ((k0800 >> 4) & 1) : 0;  // overheat warn
  bool ntc = have0800 ? ((k0800 >> 5) & 1) : 0;  // external NTC flag (laser PSU side)

  // NTC temp from SF6015 (0.1 °C units)
  float TntcC = NAN;
  if (readParamSF("0AE4", v)) TntcC = ((int16_t)v) / 10.0f;

  // ----- TEC: ALWAYS 25.00 °C; measured from external NTC -----
  float Tset = NAN, Tmeas = NAN, TI = NAN, TV = NAN;
  uint16_t stateTC = 0;

  if (readParamTC("0A10", v)) Tset   = v / 100.0f;  // setpoint (0.01 °C)
  if (readParamTC("0A15", v)) Tmeas  = v / 100.0f;  // measured temp (now from external NTC)
  if (readParamTC("0A16", v)) TI     = v / 10.0f;   // TEC current (0.1 A)
  if (readParamTC("0A18", v)) TV     = v / 10.0f;   // TEC voltage (0.1 V)
  readParamTC("0A1A", stateTC);

  bool startedTEC = (stateTC & 0x0001);   // started
  bool tempInt    = (stateTC & 0x0002);   // 1=internal temp source, 0=external (we force 0)
  bool enInt      = (stateTC & 0x0010);   // enable source internal
  bool ilDenyTEC  = (stateTC & 0x0080);   // interlock denied

  // Self-heal / reassert
  if (!startedTEC || ilDenyTEC || tempInt || !enInt ||
      (fabs((isnan(Tset)?TEC_FIXED_SET_C:Tset) - TEC_FIXED_SET_C) > 0.05f) ||
      (now - lastTECAssertMs >= TEC_REASSERT_MS)) {
    setTempSourceExternalNTC();
    bringUpAndStartTC(TEC_FIXED_SET_C);
    lastTECAssertMs = now;
  }

  // Interlocks line
  char interlocks[64];
  snprintf(interlocks, sizeof(interlocks),
           "Laser IL=%d OC=%d OTW=%d NTC=%d T=%.1fC | TEC ILDENY=%d",
           il, oc, otw, ntc, isnan(TntcC) ? 0.0f : TntcC, ilDenyTEC ? 1 : 0);

  // Draw single page
  drawDashboard(interlocks,
                (isetA < 0 ? 0 : isetA),
                (iA    < 0 ? 0 : iA),
                (isnan(Tset)  ? TEC_FIXED_SET_C : Tset),
                (isnan(Tmeas) ? 0 : Tmeas));

  delay(100);
}