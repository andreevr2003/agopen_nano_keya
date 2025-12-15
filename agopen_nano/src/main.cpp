#include <Arduino.h>
#include <SoftwareSerial.h>

#define AOG_BAUD 38400
#define DBG_BAUD 57600

SoftwareSerial dbg(7, 8); // TX=D8 -> Nano#2 RX

static void dbgPrintFrame(const uint8_t *f) {
  dbg.print(F("EN="));
  dbg.print(f[7]); // 0/1

  int16_t set_cdeg = (int16_t)((uint16_t)f[8] | ((uint16_t)f[9] << 8)); // 0.01 deg
  dbg.print(F("  SET="));
  dbg.print(set_cdeg / 100.0f, 2);
  dbg.print(F(" deg  raw="));
  dbg.print(set_cdeg);

  dbg.print(F("  HEX: "));
  const char* h="0123456789ABCDEF";
  for (int i=0;i<14;i++){
    dbg.write(h[(f[i]>>4)&0xF]); dbg.write(h[f[i]&0xF]); dbg.write(' ');
  }
  dbg.write('\n');
}

void setup() {
  Serial.begin(AOG_BAUD);  // AOG/AgIO
  dbg.begin(DBG_BAUD);     // debug către Nano#2
  dbg.println(F("\n=== Nano#1 AOG 14B parser ==="));
}

void loop() {
  static uint8_t f[14];
  static uint8_t idx = 0;
  static bool syncing = false;

  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    if (!syncing) {
      // caută 0x80 0x81
      if (idx == 0) {
        if (b == 0x80) { f[idx++] = b; }
      } else if (idx == 1) {
        if (b == 0x81) { f[idx++] = b; syncing = true; }
        else idx = 0;
      }
      continue;
    }

    // colectează restul până la 14 bytes
    f[idx++] = b;

    if (idx >= 14) {
      // validare minimă
      bool ok = (f[0]==0x80 && f[1]==0x81 && f[2]==0x7F && f[3]==0xFE && f[4]==0x08);

      if (ok) dbgPrintFrame(f);
      else {
        dbg.println(F("Bad frame (sync lost)"));
      }

      // reset pentru următorul frame
      idx = 0;
      syncing = false;
    }
  }
}
