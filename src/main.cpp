// ---------------------------------------------------------------------------
// LoRa AT modem firmware for LilyGO TTGO LoRa32 T3 v1.6.1 (433 MHz, SX1278)
//
// Implements the at-os3 raw SX1278 LoRa AT command set described in
// AT_COMMANDS.md, over the USB serial port, using the RadioLib driver.
//
//   - UART 115200 8N1, '\n' terminates a command, '\r' ignored.
//   - Uppercase, strict parsing. Malformed -> +ERR=4.
//   - Hex payloads for TX/RX (binary-safe).
//   - Unsolicited: +READY, +SENDED, +RCV=..., +ERR=1.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Build-time configuration (see platformio.ini build_flags) -------------
#ifndef AT_UART_BAUD
#define AT_UART_BAUD 115200
#endif
#ifndef PIN_LORA_SCK
#define PIN_LORA_SCK 5
#endif
#ifndef PIN_LORA_MISO
#define PIN_LORA_MISO 19
#endif
#ifndef PIN_LORA_MOSI
#define PIN_LORA_MOSI 27
#endif
#ifndef PIN_LORA_NSS
#define PIN_LORA_NSS 18
#endif
#ifndef PIN_LORA_RST
#define PIN_LORA_RST 23
#endif
#ifndef PIN_LORA_DIO0
#define PIN_LORA_DIO0 26
#endif
#ifndef PIN_LORA_DIO1
#define PIN_LORA_DIO1 33
#endif
#ifndef PIN_OLED_SDA
#define PIN_OLED_SDA 21
#endif
#ifndef PIN_OLED_SCL
#define PIN_OLED_SCL 22
#endif
#ifndef PIN_OLED_RST
#define PIN_OLED_RST -1
#endif
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C
#endif
#define OLED_W 128
#define OLED_H 64

static const char *FW_VERSION = "at-os3-0.1.0";
static const char *FW_UID = "000000000000";

// SX1278 subclass exposing a raw register read for the AT+REG diagnostic.
// RadioLib's getMod() is protected, but reachable from a derived class.
class SX1278AT : public SX1278 {
 public:
  SX1278AT(Module *mod) : SX1278(mod) {}
  uint8_t readReg(uint8_t addr) { return this->getMod()->SPIreadRegister(addr); }
};

// SX1278 radio over the default VSPI bus, DIO0 = RxDone/TxDone.
SX1278AT radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO0, PIN_LORA_RST, PIN_LORA_DIO1);

// SSD1306 OLED on the secondary I2C bus.
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, PIN_OLED_RST);
bool oledOk = false;

// Live indicators shown on the OLED. Updated from loop() context only.
struct Indicators {
  uint32_t txCount;     // accepted TX frames
  uint32_t rxCount;     // valid RX frames
  uint32_t crcErrCount; // +ERR=1 events
  int lastRssi;         // dBm of last RX
  int lastSnr;          // dB of last RX
  int lastFerr;         // Hz of last RX
  char lastEvent[22];   // short status line, e.g. "TX 4B" / "RX 4B" / "CRC err"
  char lastHex[18];     // last payload hex, truncated for display
};
Indicators ind = {0, 0, 0, 0, 0, 0, "boot", "--"};
bool displayDirty = true;
uint32_t lastRenderMs = 0;

// --- AT shadow configuration (power-on / AT+FACTORY defaults) --------------
struct ShadowConfig {
  uint32_t freqHz;    // carrier frequency in Hz
  uint8_t sf;         // spreading factor 6..12
  uint8_t bwCode;     // AT bandwidth code 0..9
  uint8_t crCode;     // AT coding rate code 1..4
  uint16_t preamble;  // 4..1023
  uint8_t power;      // 0..15 dBm (PA_BOOST)
  uint8_t iqi;        // 0 normal, 1 inverted
  uint8_t crc;        // 0 disabled, 1 enabled
  uint8_t ldro;       // 0 off, 1 on, 2 auto
  uint8_t len;        // 0 explicit header, 1..255 implicit length
  uint8_t syncword;   // 0..255
  uint16_t address;   // 0..65535 (shadow only)
  uint8_t networkid;  // 0..16 (shadow only)
  uint8_t mode;       // 0 RX active, 1 standby
};

static const ShadowConfig DEFAULTS = {
    /* freqHz   */ 433175000UL,
    /* sf       */ 12,
    /* bwCode   */ 7,
    /* crCode   */ 1,
    /* preamble */ 8,
    /* power    */ 15,
    /* iqi      */ 0,
    /* crc      */ 1,
    /* ldro     */ 2,
    /* len      */ 0,
    /* syncword */ 0x12,
    /* address  */ 0,
    /* networkid*/ 0,
    /* mode     */ 0,
};

ShadowConfig cfg;

// --- Receive interrupt flag ------------------------------------------------
volatile bool rxFlag = false;
bool seenAT = false;  // for startup resync to first "AT" in a line

ICACHE_RAM_ATTR void onDio0() { rxFlag = true; }

// --- Bandwidth code -> kHz mapping (see AT_COMMANDS.md "Bandwidth Codes") --
static float bwCodeToKhz(uint8_t code) {
  switch (code) {
    case 0: return 7.8f;
    case 1: return 10.4f;
    case 2: return 15.6f;
    case 3: return 20.8f;
    case 4: return 31.25f;
    case 5: return 41.7f;
    case 6: return 62.5f;
    case 7: return 125.0f;
    case 8: return 250.0f;
    case 9: return 500.0f;
    default: return 125.0f;
  }
}

// --- Serial response helpers ----------------------------------------------
static void sendLine(const char *s) {
  Serial.print(s);
  Serial.print("\r\n");
}
static void sendOK() { sendLine("+OK"); }
static void sendErr(int code) {
  Serial.print("+ERR=");
  Serial.print(code);
  Serial.print("\r\n");
}

// --- Apply the full shadow profile to the radio ----------------------------
// Returns true on success. Sequences sleep/configure as RadioLib requires.
static bool applyRadioConfig() {
  int state = radio.begin(cfg.freqHz / 1.0e6f,        // MHz
                          bwCodeToKhz(cfg.bwCode),     // kHz
                          cfg.sf,                      // spreading factor
                          cfg.crCode + 4,              // CR code 1..4 -> 5..8
                          cfg.syncword,                // sync word
                          cfg.power,                   // dBm (PA_BOOST)
                          cfg.preamble);               // preamble symbols
  if (state != RADIOLIB_ERR_NONE) return false;

  radio.setCRC(cfg.crc != 0);

  if (cfg.ldro == 0)      radio.forceLDRO(false);
  else if (cfg.ldro == 1) radio.forceLDRO(true);
  else                    radio.autoLDRO();

  if (cfg.len == 0) radio.explicitHeader();
  else              radio.implicitHeader(cfg.len);

  radio.invertIQ(cfg.iqi != 0);

  radio.setDio0Action(onDio0, RISING);
  return true;
}

// Re-arm continuous RX (mode 0) or go to standby (mode 1).
static void enterCurrentMode() {
  if (cfg.mode == 0) {
    radio.startReceive();
  } else {
    radio.standby();
  }
}

// ---------------------------------------------------------------------------
// OLED indicators
// ---------------------------------------------------------------------------

// Store a payload (binary) as a short uppercase-hex string for the display.
static void setLastHex(const uint8_t *data, size_t len) {
  static const char H[] = "0123456789ABCDEF";
  size_t maxBytes = (sizeof(ind.lastHex) - 3) / 2;  // leave room for ".."+NUL
  size_t shown = len < maxBytes ? len : maxBytes;
  size_t o = 0;
  for (size_t i = 0; i < shown; ++i) {
    ind.lastHex[o++] = H[data[i] >> 4];
    ind.lastHex[o++] = H[data[i] & 0x0F];
  }
  if (shown < len) {
    ind.lastHex[o++] = '.';
    ind.lastHex[o++] = '.';
  }
  ind.lastHex[o] = '\0';
}

static void initDisplay() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.display();
}

// Render the full status screen. Called from loop(), throttled.
static void renderDisplay() {
  if (!oledOk) return;

  char buf[24];
  oled.clearDisplay();

  // Header (inverted bar): frequency + mode.
  oled.fillRect(0, 0, OLED_W, 9, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(1, 1);
  float mhz = cfg.freqHz / 1.0e6f;
  snprintf(buf, sizeof(buf), "%.3fMHz", mhz);
  oled.print(buf);
  const char *modeStr = (cfg.mode == 0) ? "RX" : "STBY";
  oled.setCursor(OLED_W - (int)strlen(modeStr) * 6 - 1, 1);
  oled.print(modeStr);

  oled.setTextColor(SSD1306_WHITE);

  // Line 1: modulation profile.
  float bw = bwCodeToKhz(cfg.bwCode);
  snprintf(buf, sizeof(buf), "SF%u BW%g 4/%u", cfg.sf, bw, cfg.crCode + 4);
  oled.setCursor(0, 12);
  oled.print(buf);

  // Line 2: power / sync word / IQ / CRC.
  snprintf(buf, sizeof(buf), "P%u SW%02X IQ%c C%c", cfg.power, cfg.syncword,
           cfg.iqi ? 'i' : 'n', cfg.crc ? '1' : '0');
  oled.setCursor(0, 22);
  oled.print(buf);

  // Line 3: counters.
  snprintf(buf, sizeof(buf), "TX:%lu RX:%lu E:%lu",
           (unsigned long)ind.txCount, (unsigned long)ind.rxCount,
           (unsigned long)ind.crcErrCount);
  oled.setCursor(0, 32);
  oled.print(buf);

  // Line 4: last RX link metrics.
  snprintf(buf, sizeof(buf), "RSSI%d SNR%d", ind.lastRssi, ind.lastSnr);
  oled.setCursor(0, 42);
  oled.print(buf);

  // Line 5: last event + last payload hex.
  oled.setCursor(0, 52);
  oled.print(ind.lastEvent);
  snprintf(buf, sizeof(buf), " %s", ind.lastHex);
  oled.print(buf);

  oled.display();
}

static inline void markDisplay() { displayDirty = true; }

// ---------------------------------------------------------------------------
// Strict numeric parsing helpers
// ---------------------------------------------------------------------------

// Parse the whole NUL-terminated string as an unsigned decimal. Rejects
// empty input, leading/trailing junk, signs and overflow past 32 bits.
static bool parseDecAll(const char *s, uint32_t *out) {
  if (s == nullptr || *s == '\0') return false;
  uint64_t v = 0;
  for (const char *p = s; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    v = v * 10 + (uint64_t)(*p - '0');
    if (v > 0xFFFFFFFFULL) return false;
  }
  *out = (uint32_t)v;
  return true;
}

static int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

// Parse exactly two hex digits into a byte. Rejects anything else.
static bool parseHexByteAll(const char *s, uint8_t *out) {
  if (s == nullptr || s[0] == '\0' || s[1] == '\0' || s[2] != '\0') return false;
  int hi = hexNibble(s[0]);
  int lo = hexNibble(s[1]);
  if (hi < 0 || lo < 0) return false;
  *out = (uint8_t)((hi << 4) | lo);
  return true;
}

// Split a comma-separated argument string in place. Returns field count, or
// -1 if more than maxFields fields are present. Empty fields are kept.
static int splitCommas(char *s, char *fields[], int maxFields) {
  int n = 0;
  fields[n++] = s;
  for (char *p = s; *p; ++p) {
    if (*p == ',') {
      *p = '\0';
      if (n >= maxFields) return -1;
      fields[n++] = p + 1;
    }
  }
  return n;
}

// Print bytes as uppercase hex (no separators).
static void printHexUpper(const uint8_t *data, size_t len) {
  static const char H[] = "0123456789ABCDEF";
  for (size_t i = 0; i < len; ++i) {
    Serial.write(H[data[i] >> 4]);
    Serial.write(H[data[i] & 0x0F]);
  }
}

// ---------------------------------------------------------------------------
// Command handlers. Each returns void and emits its own response.
// `arg` is the text after the command name (may be nullptr if absent).
// ---------------------------------------------------------------------------

static void doReset() {
  sendLine("+RESET");
  bool ok = applyRadioConfig();
  enterCurrentMode();
  if (ok) sendLine("+READY");
  else sendErr(4);
}

static void doMode(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 1) { sendErr(4); return; }
  cfg.mode = (uint8_t)v;
  enterCurrentMode();
  sendOK();
}

static void doBand(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v < 137000000UL || v > 1020000000UL) {
    sendErr(4);
    return;
  }
  cfg.freqHz = v;
  if (radio.setFrequency(v / 1.0e6f) != RADIOLIB_ERR_NONE) { sendErr(4); return; }
  enterCurrentMode();
  sendOK();
}

static void doParameter(const char *arg) {
  char buf[64];
  if (arg == nullptr) { sendErr(4); return; }
  strncpy(buf, arg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *f[5];
  int n = splitCommas(buf, f, 5);
  if (n != 4) { sendErr(4); return; }

  uint32_t sf, bw, cr, pre;
  if (!parseDecAll(f[0], &sf) || sf < 6 || sf > 12) { sendErr(4); return; }
  if (!parseDecAll(f[1], &bw) || bw > 9) { sendErr(4); return; }
  if (!parseDecAll(f[2], &cr) || cr < 1 || cr > 4) { sendErr(4); return; }
  if (!parseDecAll(f[3], &pre) || pre < 4 || pre > 1023) { sendErr(4); return; }

  // Validate atomically, then commit.
  cfg.sf = (uint8_t)sf;
  cfg.bwCode = (uint8_t)bw;
  cfg.crCode = (uint8_t)cr;
  cfg.preamble = (uint16_t)pre;

  bool ok = true;
  ok &= radio.setSpreadingFactor(cfg.sf) == RADIOLIB_ERR_NONE;
  ok &= radio.setBandwidth(bwCodeToKhz(cfg.bwCode)) == RADIOLIB_ERR_NONE;
  ok &= radio.setCodingRate(cfg.crCode + 4) == RADIOLIB_ERR_NONE;
  ok &= radio.setPreambleLength(cfg.preamble) == RADIOLIB_ERR_NONE;
  if (!ok) { sendErr(4); return; }

  enterCurrentMode();
  sendOK();
}

static void doPkt(const char *arg) {
  char buf[32];
  if (arg == nullptr) { sendErr(4); return; }
  strncpy(buf, arg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *f[4];
  int n = splitCommas(buf, f, 4);
  if (n != 3) { sendErr(4); return; }

  uint32_t crc, ldro, len;
  if (!parseDecAll(f[0], &crc) || crc > 1) { sendErr(4); return; }
  if (!parseDecAll(f[1], &ldro) || ldro > 2) { sendErr(4); return; }
  if (!parseDecAll(f[2], &len) || len > 255) { sendErr(4); return; }

  cfg.crc = (uint8_t)crc;
  cfg.ldro = (uint8_t)ldro;
  cfg.len = (uint8_t)len;

  radio.setCRC(cfg.crc != 0);
  if (cfg.ldro == 0)      radio.forceLDRO(false);
  else if (cfg.ldro == 1) radio.forceLDRO(true);
  else                    radio.autoLDRO();
  if (cfg.len == 0) radio.explicitHeader();
  else              radio.implicitHeader(cfg.len);

  enterCurrentMode();
  sendOK();
}

static void doCrfop(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 15) { sendErr(4); return; }
  cfg.power = (uint8_t)v;
  if (radio.setOutputPower(cfg.power) != RADIOLIB_ERR_NONE) { sendErr(4); return; }
  sendOK();
}

static void doIqi(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 1) { sendErr(4); return; }
  cfg.iqi = (uint8_t)v;
  radio.invertIQ(cfg.iqi != 0);
  enterCurrentMode();
  sendOK();
}

static void doSyncword(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 255) { sendErr(4); return; }
  cfg.syncword = (uint8_t)v;
  if (radio.setSyncWord(cfg.syncword) != RADIOLIB_ERR_NONE) { sendErr(4); return; }
  enterCurrentMode();
  sendOK();
}

static void doAddress(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 65535UL) { sendErr(4); return; }
  cfg.address = (uint16_t)v;
  sendOK();
}

static void doNetworkId(const char *arg) {
  uint32_t v;
  if (!parseDecAll(arg, &v) || v > 16) { sendErr(4); return; }
  cfg.networkid = (uint8_t)v;
  sendOK();
}

static void doSend(const char *arg) {
  char buf[300];
  if (arg == nullptr) { sendErr(4); return; }
  strncpy(buf, arg, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  char *f[4];
  int n = splitCommas(buf, f, 4);
  if (n != 3) { sendErr(4); return; }

  uint32_t addr, len;
  if (!parseDecAll(f[0], &addr)) { sendErr(4); return; }   // parsed, ignored
  if (!parseDecAll(f[1], &len) || len < 1 || len > 128) { sendErr(4); return; }

  const char *hex = f[2];
  if (strlen(hex) != len * 2) { sendErr(4); return; }

  uint8_t payload[128];
  for (uint32_t i = 0; i < len; ++i) {
    int hi = hexNibble(hex[i * 2]);
    int lo = hexNibble(hex[i * 2 + 1]);
    if (hi < 0 || lo < 0) { sendErr(4); return; }
    payload[i] = (uint8_t)((hi << 4) | lo);
  }

  sendOK();  // accepted into the TX path

  int state = radio.transmit(payload, len);

  if (state == RADIOLIB_ERR_NONE) {
    sendLine("+SENDED");
    ind.txCount++;
    snprintf(ind.lastEvent, sizeof(ind.lastEvent), "TX %luB", (unsigned long)len);
    setLastHex(payload, len);
    markDisplay();
  }
  // TX timeout / unexpected IRQ is silent at the AT layer, per spec.

  rxFlag = false;  // DIO0 doubles as TxDone and has just triggered the RX ISR

  enterCurrentMode();  // radio FSM returns to RX/standby
}

static void doReg(const char *arg) {
  uint8_t addr;
  if (!parseHexByteAll(arg, &addr)) { sendErr(4); return; }
  uint8_t val = radio.readReg(addr);
  Serial.print("+REG=");
  printHexUpper(&val, 1);
  Serial.print("\r\n");
  sendOK();
}

static void doFactory() {
  cfg = DEFAULTS;
  sendLine("+FACTORY");  // no extra +OK, per spec
}

// ---------------------------------------------------------------------------
// Receive handling: emit +RCV or +ERR=1.
// ---------------------------------------------------------------------------
static void handleReceive() {
  uint8_t data[256];
  int state = radio.readData(data, sizeof(data));
  size_t len = radio.getPacketLength();

  if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    sendErr(1);  // PHY CRC error
    ind.crcErrCount++;
    strncpy(ind.lastEvent, "CRC err", sizeof(ind.lastEvent));
    markDisplay();
  } else if (state == RADIOLIB_ERR_NONE) {
    int rssi = (int)lroundf(radio.getRSSI());
    int snr = (int)radio.getSNR();
    int ferr = (int)lroundf(radio.getFrequencyError());

    ind.rxCount++;
    ind.lastRssi = rssi;
    ind.lastSnr = snr;
    ind.lastFerr = ferr;
    snprintf(ind.lastEvent, sizeof(ind.lastEvent), "RX %uB", (unsigned)len);
    setLastHex(data, len);
    markDisplay();

    Serial.print("+RCV=");
    Serial.print(cfg.address);  // shadow address, not sender
    Serial.print(',');
    Serial.print(len);
    Serial.print(',');
    printHexUpper(data, len);
    Serial.print(',');
    Serial.print(rssi);
    Serial.print(',');
    Serial.print(snr);
    Serial.print(',');
    Serial.print(ferr);
    Serial.print("\r\n");
  }
  // else: other errors are dropped silently

  // Re-arm RX if we are in RX mode.
  if (cfg.mode == 0) radio.startReceive();
}

// ---------------------------------------------------------------------------
// Command dispatch. `line` is a writable, NUL-terminated, '\r'-stripped line.
// ---------------------------------------------------------------------------

// Matches a command name then dispatches. The query/assignment suffix handling
// is per-command so we can keep strict parsing (reject trailing junk).
static void dispatch(char *line) {
  // Startup resync: before the first valid AT, allow leading garbage by
  // seeking to the first "AT" occurrence in the line.
  if (!seenAT) {
    char *at = strstr(line, "AT");
    if (at == nullptr) return;  // no AT yet; ignore line
    line = at;
  }

  if (strncmp(line, "AT", 2) != 0) { sendErr(4); return; }
  char *p = line + 2;

  // Bare "AT"
  if (*p == '\0') {
    seenAT = true;
    sendOK();
    return;
  }

  if (*p != '+') { sendErr(4); return; }
  p++;  // now at command name

  seenAT = true;

  // Helper lambdas via small macros for readability.
  // Assignment form: "NAME=<arg>" ; Query form: "NAME?" ; Bare: "NAME"
  auto isCmd = [](const char *s, const char *name, char **rest) -> bool {
    size_t n = strlen(name);
    if (strncmp(s, name, n) != 0) return false;
    *rest = (char *)s + n;
    return true;
  };

  char *rest;

  // --- Commands with =value and/or ? query ---
  if (isCmd(p, "MODE", &rest)) {
    if (rest[0] == '=') { doMode(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+MODE=");
      Serial.print(cfg.mode);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "BAND", &rest)) {
    if (rest[0] == '=') { doBand(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+BAND=");
      Serial.print(cfg.freqHz);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "PARAMETER", &rest)) {
    if (rest[0] == '=') { doParameter(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+PARAMETER=");
      Serial.print(cfg.sf); Serial.print(',');
      Serial.print(cfg.bwCode); Serial.print(',');
      Serial.print(cfg.crCode); Serial.print(',');
      Serial.print(cfg.preamble);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "PKT", &rest)) {
    if (rest[0] == '=') { doPkt(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+PKT=");
      Serial.print(cfg.crc); Serial.print(',');
      Serial.print(cfg.ldro); Serial.print(',');
      Serial.print(cfg.len);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "CRFOP", &rest)) {
    if (rest[0] == '=') { doCrfop(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+CRFOP=");
      Serial.print(cfg.power);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "IQI", &rest)) {
    if (rest[0] == '=') { doIqi(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+IQI=");
      Serial.print(cfg.iqi);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "SYNCWORD", &rest)) {
    if (rest[0] == '=') { doSyncword(rest + 1); return; }
    sendErr(4);  // no SYNCWORD? query per spec
    return;
  }

  if (isCmd(p, "ADDRESS", &rest)) {
    if (rest[0] == '=') { doAddress(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+ADDRESS=");
      Serial.print(cfg.address);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "NETWORKID", &rest)) {
    if (rest[0] == '=') { doNetworkId(rest + 1); return; }
    if (strcmp(rest, "?") == 0) {
      Serial.print("+NETWORKID=");
      Serial.print(cfg.networkid);
      Serial.print("\r\n");
      sendOK();
      return;
    }
    sendErr(4);
    return;
  }

  if (isCmd(p, "SEND", &rest)) {
    if (rest[0] == '=') { doSend(rest + 1); return; }
    sendErr(4);
    return;
  }

  if (isCmd(p, "REG", &rest)) {
    if (rest[0] == '=') { doReg(rest + 1); return; }
    sendErr(4);
    return;
  }

  // --- Query-only / no-arg commands ---
  if (strcmp(p, "VER?") == 0) {
    Serial.print("+VER=");
    Serial.print(FW_VERSION);
    Serial.print("\r\n");
    sendOK();
    return;
  }

  if (strcmp(p, "UID?") == 0) {
    Serial.print("+UID=");
    Serial.print(FW_UID);
    Serial.print("\r\n");
    sendOK();
    return;
  }

  if (strcmp(p, "RESET") == 0) { doReset(); return; }

  if (strcmp(p, "FACTORY") == 0) { doFactory(); return; }

  // AT+IPR compatibility no-op: accept any name starting with "IPR".
  if (strncmp(p, "IPR", 3) == 0) { sendOK(); return; }

  // Anything else is malformed/unsupported.
  sendErr(4);
}

// ---------------------------------------------------------------------------
// Line reader: 127 chars + NUL. '\r' ignored, '\n' executes. Overflow bytes
// before the newline are discarded.
// ---------------------------------------------------------------------------
static char lineBuf[128];
static size_t lineLen = 0;
static bool overflow = false;

static void feedByte(char c) {
  if (c == '\r') return;  // ignored
  if (c == '\n') {
    lineBuf[lineLen] = '\0';
    if (lineLen > 0 || overflow) {
      // Empty lines are ignored; only dispatch non-empty.
      if (lineLen > 0) dispatch(lineBuf);
    }
    lineLen = 0;
    overflow = false;
    return;
  }
  if (lineLen < sizeof(lineBuf) - 1) {
    lineBuf[lineLen++] = c;
  } else {
    overflow = true;  // discard extra bytes until newline
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(AT_UART_BAUD);

  // Explicit SPI pin mapping for the T3 v1.6.x LoRa module.
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);

  initDisplay();  // OLED is optional; failure is non-fatal

  cfg = DEFAULTS;

  bool ok = applyRadioConfig();
  enterCurrentMode();

  strncpy(ind.lastEvent, ok ? "ready" : "radio err", sizeof(ind.lastEvent));
  renderDisplay();

  // Modem firmware is ready.
  sendLine("+READY");
  if (!ok) {
    // Surface init failure so the host isn't left guessing.
    sendErr(4);
  }
}

void loop() {
  // Drain UART input.
  while (Serial.available() > 0) {
    feedByte((char)Serial.read());
  }

  // Service radio RX events.
  if (rxFlag) {
    rxFlag = false;
    handleReceive();
  }

  // Refresh the OLED: on demand after an event, and as a slow heartbeat so
  // config changes made via AT also show up. ~3 Hz keeps I2C off the RX path.
  uint32_t now = millis();
  if (displayDirty || (now - lastRenderMs) >= 333) {
    displayDirty = false;
    lastRenderMs = now;
    renderDisplay();
  }
}
