#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <SoftwareSerial.h>

// ------------------- MCP2515 -------------------
#define CAN_CS     10
#define CAN_INT    2
MCP_CAN CAN0(CAN_CS);

#define MOTOR_ID     1
#define CAN_ID_CMD   (0x06000000UL + MOTOR_ID)  // SDO Tx către motor

#define CAN_BAUD     CAN_250KBPS
#define MCP_CLOCK    MCP_8MHZ     // MCP_16MHZ dacă modulul tău e 16MHz

// ------------------- KEYA scaling -------------------
// IMPORTANT: cu 39 nu ai rezoluție fină (salturi mari). Pentru mișcări fine, folosește ~10000/rot.
#define POS_PER_REV  936L       // ajustează dacă nu corespunde 1 rotație la 360.00°
#define DEG_LIMIT    720.0f       // limită de siguranță (±720°)

// Inversează sensul (dacă motorul merge invers față de ce vrei)
#define INVERT_DIR   1            // 1 = inversat, 0 = normal

// ------------------- Link Nano#1 -> Nano#2 -------------------
SoftwareSerial LINK(8, 7);        // RX=D8 (primește), TX=D7 (nefolosit)
#define LINK_BAUD    57600

// ------------------- Control -------------------
volatile bool motorEnabled = false;
volatile int32_t targetCdeg = 0;  // centi-grade, ex 3000 = 30.00°

static uint32_t lastRxMs  = 0;
static uint32_t lastCanMs = 0;

static int32_t lastSentPos = 0;
static int lastSentRpm = -1;

// ------------------- Helpers -------------------
static inline void canSend8(uint32_t id, const uint8_t d[8]) {
  CAN0.sendMsgBuf(id, 1, 8, (byte*)d);
}

// KEYA: enable + position mode
static void motorEnableNow() {
  const uint8_t en[8]      = {0x23,0x0D,0x20,0x01,0,0,0,0};
  const uint8_t posMode[8] = {0x03,0x0D,0x20,0x31,0,0,0,0};
  canSend8(CAN_ID_CMD, en);
  canSend8(CAN_ID_CMD, posMode);
}

// KEYA: disable
static void motorDisableNow() {
  const uint8_t dis[8] = {0x23,0x0C,0x20,0x01,0,0,0,0};
  canSend8(CAN_ID_CMD, dis);
}

// KEYA: speed set (RPM). (același format ca în codul tău Mega)
static void sendSpeedRPM(int rpm) {
  if (rpm < 1) rpm = 1;
  if (rpm > 100) rpm = 100;

  uint32_t v = (uint32_t)rpm;
  uint8_t f[8];
  f[0]=0x23; f[1]=0x00; f[2]=0x20; f[3]=0x01;
  f[4]=(uint8_t)(v & 0xFF);
  f[5]=(uint8_t)((v >> 8) & 0xFF);
  f[6]=(uint8_t)((v >> 16) & 0xFF);
  f[7]=(uint8_t)((v >> 24) & 0xFF);

  canSend8(CAN_ID_CMD, f);
}

// Convert centi-degree -> internal position counts (int32)
// internal = cdeg * POS_PER_REV / 36000
static int32_t cdegToInternal(int32_t cdeg) {
  // limit în grade înainte să convertești
  float deg = (float)cdeg / 100.0f;
  if (deg >  DEG_LIMIT) deg =  DEG_LIMIT;
  if (deg < -DEG_LIMIT) deg = -DEG_LIMIT;

  int32_t cdegLim = (int32_t)lround(deg * 100.0f);

  if (INVERT_DIR) cdegLim = -cdegLim;

  double v = (double)cdegLim * (double)POS_PER_REV / 36000.0;
  return (int32_t)lround(v);
}

// KEYA: position set (același format ca în codul tău Mega)
static void sendPositionInternal(int32_t pos) {
  uint32_t p = (uint32_t)pos;

  uint8_t f[8];
  f[0]=0x23; f[1]=0x02; f[2]=0x20; f[3]=0x01;
  f[4]=(uint8_t)(p & 0xFF);
  f[5]=(uint8_t)((p >> 8) & 0xFF);
  f[6]=(uint8_t)((p >> 16) & 0xFF);
  f[7]=(uint8_t)((p >> 24) & 0xFF);

  canSend8(CAN_ID_CMD, f);
}

// RPM dinamic după cât e de schimbat setpoint-ul (delta counts)
// delta mic => RPM mic (fine), delta mare => RPM mare (rapid)
static int rpmFromDelta(int32_t deltaCountsAbs) {
  // praguri aproximative; ajustezi după cum se simte pe tractor
  if (deltaCountsAbs <= 2)   return 5;    // foarte fin
  if (deltaCountsAbs <= 8)   return 10;
  if (deltaCountsAbs <= 20)  return 20;
  if (deltaCountsAbs <= 60)  return 35;
  if (deltaCountsAbs <= 150) return 55;
  return 80;
}

// ------------------- Serial parsing (EN=... ANG=...) -------------------
static bool readLineFromLink(char *out, size_t outSz) {
  static char buf[64];
  static size_t n = 0;
  static uint32_t lastByteMs = 0;

  while (LINK.available()) {
    char c = (char)LINK.read();
    lastByteMs = millis();

    if (c == '\r') continue;

    if (c == '\n') {
      buf[n] = 0;
      strncpy(out, buf, outSz);
      out[outSz-1] = 0;
      n = 0;
      return true;
    } else {
      if (n < sizeof(buf)-1) buf[n++] = c;
      else { // overflow: reset
        n = 0;
      }
    }
  }

  // finalize dacă nu vine newline (10ms pauză)
  if (n && (millis() - lastByteMs > 10)) {
    buf[n] = 0;
    strncpy(out, buf, outSz);
    out[outSz-1] = 0;
    n = 0;
    return true;
  }

  return false;
}

static bool parseENANG(const char *line, bool &enOut, int32_t &angCdegOut) {
  // aștept: "EN=1 ANG=3000"
  const char *pEN  = strstr(line, "EN=");
  const char *pANG = strstr(line, "ANG=");
  if (!pEN || !pANG) return false;

  enOut = (atoi(pEN + 3) != 0);
  angCdegOut = (int32_t)atol(pANG + 4);
  return true;
}

// ------------------- Setup/Loop -------------------
void setup() {
  Serial.begin(115200);
  LINK.begin(LINK_BAUD);

  Serial.println("=== Nano#2 EN+ANG -> KEYA (dynamic speed + invert dir) ===");
  Serial.print("LINK_BAUD="); Serial.println(LINK_BAUD);
  Serial.print("POS_PER_REV="); Serial.println(POS_PER_REV);
  Serial.print("INVERT_DIR="); Serial.println(INVERT_DIR);

  if (CAN0.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) != CAN_OK) {
    Serial.println("❌ CAN init fail");
    while (true);
  }
  CAN0.setMode(MCP_NORMAL);

  motorEnabled = false;
  targetCdeg = 0;
}

void loop() {
  // 1) RX from Nano#1
  char line[64];
  if (readLineFromLink(line, sizeof(line))) {
    bool en;
    int32_t ang;
    if (parseENANG(line, en, ang)) {
      motorEnabled = en;
      targetCdeg = ang;
      lastRxMs = millis();

      // Debug
      Serial.print("RX EN="); Serial.print(en ? 1 : 0);
      Serial.print(" ANG="); Serial.print(ang);
      Serial.print(" (deg="); Serial.print(ang / 100.0f, 2); Serial.println(")");
    }
  }

  // 2) enable/disable on change
  static bool lastEn = false;
  if (motorEnabled != lastEn) {
    if (motorEnabled) {
      motorEnableNow();
      Serial.println("EN=1 -> enable sent");
    } else {
      motorDisableNow();
      Serial.println("EN=0 -> disable sent");
    }
    lastEn = motorEnabled;
  }

  // 3) send pos at 50Hz
  const uint32_t CAN_PERIOD_MS = 20;
  uint32_t now = millis();

  if (motorEnabled && (now - lastCanMs >= CAN_PERIOD_MS)) {
    lastCanMs = now;

    int32_t pos = cdegToInternal(targetCdeg);
    int32_t delta = pos - lastSentPos;
    int32_t absDelta = (delta >= 0) ? delta : -delta;

    // viteză dinamică
    int rpm = rpmFromDelta(absDelta);

    // trimite RPM doar când se schimbă (reduce spam)
    if (rpm != lastSentRpm) {
      sendSpeedRPM(rpm);
      lastSentRpm = rpm;
      Serial.print("SPD rpm="); Serial.println(rpm);
    }

    // trimite poziția
    sendPositionInternal(pos);
    lastSentPos = pos;

    // Debug fin
    Serial.print("TX pos="); Serial.print(pos);
    Serial.print(" delta="); Serial.print(delta);
    Serial.print(" (deg="); Serial.print(targetCdeg/100.0f, 2); Serial.println(")");
  }
}
