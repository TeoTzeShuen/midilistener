// BLE MIDI listener: connects to a BLE MIDI peripheral (e.g. M-Vave Chocolate Plus)
// as a central, subscribes to the MIDI characteristic, prints every message and
// mirrors each switch's CC state on a WS2812 strip. Auto-rescans and reconnects.
//
// Banks: the footswitch sends nothing on a bank change, so the board learns banks.
// A "key" is a (MIDI channel, CC) pair. On/off state is stored per key, because a CC
// shared between banks drives the same multiFX parameter. A bank maps switches 1-4 to
// four keys. The displayed bank changes when the first CC of a packet is not part of
// the current bank but belongs to another learned bank.
//
// Setup mode: press switch 1 (of any learned bank) 6 times within 2.5 s. Then change
// bank on the footswitch and press switches 1-4 twice each. The new bank is saved
// (replacing any bank with the same switch-1 key). Setup cancels after 30 s idle.

#ifndef LED_TEST  // the ledtest env builds src/ledtest.cpp instead

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include <freertos/queue.h>

static const NimBLEUUID MIDI_SERVICE_UUID("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
static const NimBLEUUID MIDI_CHAR_UUID("7772E5DB-3868-4112-A1A9-F2669D106BF3");

static const NimBLEAdvertisedDevice* g_target = nullptr;
static NimBLEClient* g_client = nullptr;
static volatile bool g_doConnect = false;
static volatile bool g_connected = false;

// ---------- LED config ----------
// 8-pixel WS2812 strip on GPIO14. Pixels 1,3,5,7 (index 0,2,4,6) show switch 1-4 of the
// current bank: off for value < 64, blue for >= 64. Pixels 2,4,6,8 (index 1,3,5,7) blink
// green after boot until that switch's key sends its first message ("unsure" indicator).
static const uint8_t LED_PIN = 14;
static const uint8_t NUM_PIXELS = 8;
static const uint8_t BRIGHTNESS = 51;  // 20% of 255
static const uint32_t BLINK_MS = 500;
// Unsure pixels at 5% absolute: the strip-wide brightness is 20%, so scale the colour by 5/20.
static const uint8_t UNSURE_LEVEL = 255 * 5 / 20;  // 63
// Onboard RGB LED (needs the "RGB" pad bridged): blinks red until BLE MIDI is subscribed,
// then steady purple; double red flash on a bank change. While disconnected the strip is
// dark apart from its last pixel blinking red.
static const uint8_t BOARD_LED_PIN = 48;
static const uint8_t BOARD_LED_RED = 40;
static const uint8_t BOARD_LED_PURPLE = 30;  // per channel (red + blue)

static Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
static Preferences prefs;

// ---------- Keys, banks, state ----------
static const uint16_t NUM_KEYS = 16 * 128;
static const uint16_t NO_KEY = 0xFFFF;
static const uint8_t MAX_BANKS = 8;

struct Bank { uint16_t keys[4]; };

static Bank g_banks[MAX_BANKS];
static uint8_t g_bankCount = 0;
static uint8_t g_curBank = 0;
static uint8_t g_onBits[NUM_KEYS / 8];
static uint8_t g_unsureBits[NUM_KEYS / 8];  // all set at boot; cleared per key on first message
static uint8_t g_savedOnBits[NUM_KEYS / 8];
static uint8_t g_savedCurBank = 0xFF;
static uint32_t g_saveAt = 0;  // millis() when a pending state save is due, 0 = none

static inline uint16_t makeKey(uint8_t ch, uint8_t cc) { return (uint16_t)(ch - 1) * 128 + cc; }
static inline uint8_t keyCh(uint16_t k) { return k / 128 + 1; }
static inline uint8_t keyCc(uint16_t k) { return k % 128; }
static inline bool getBit(const uint8_t* b, uint16_t k) { return b[k >> 3] & (1 << (k & 7)); }
static inline void setBit(uint8_t* b, uint16_t k, bool v) {
  if (v) b[k >> 3] |= 1 << (k & 7); else b[k >> 3] &= ~(1 << (k & 7));
}

static int switchInBank(uint8_t bank, uint16_t k) {
  for (int s = 0; s < 4; s++) if (g_banks[bank].keys[s] == k) return s;
  return -1;
}

// ---------- Modes and indicator timers ----------
enum Mode { MODE_NORMAL, MODE_SETUP };
static Mode g_mode = MODE_NORMAL;

static const uint8_t TRIGGER_PRESSES = 6;
static const uint32_t TRIGGER_WINDOW_MS = 2500;
static const uint32_t SETUP_TIMEOUT_MS = 30000;
static uint16_t g_trigKey = NO_KEY;
static uint32_t g_trigTimes[TRIGGER_PRESSES];
static uint8_t g_trigCount = 0;

static uint8_t g_setupSwitch = 0;       // switch being learned, 0..3
static uint16_t g_setupFirst = NO_KEY;  // key from the first of the two presses
static uint16_t g_setupKeys[4];
static uint32_t g_setupLastActivity = 0;

static uint32_t g_bankFlashStart = 0;   // board LED double red flash
static bool g_bankFlashActive = false;
static uint32_t g_stripFlashStart = 0;  // whole-strip double flash (setup done / cancelled)
static uint32_t g_stripFlashColor = 0;
static bool g_stripFlashActive = false;

// ---------- Persistence ----------

static void saveBanks() {
  prefs.begin("ledstate", false);
  prefs.putUChar("nbanks", g_bankCount);
  prefs.putBytes("banks", g_banks, sizeof(Bank) * g_bankCount);
  prefs.end();
}

static void saveState() {
  bool onChanged = memcmp(g_onBits, g_savedOnBits, sizeof(g_onBits)) != 0;
  bool curChanged = g_curBank != g_savedCurBank;
  if (!onChanged && !curChanged) return;
  prefs.begin("ledstate", false);
  if (onChanged) prefs.putBytes("onbits", g_onBits, sizeof(g_onBits));
  if (curChanged) prefs.putUChar("cur", g_curBank);
  prefs.end();
  memcpy(g_savedOnBits, g_onBits, sizeof(g_onBits));
  g_savedCurBank = g_curBank;
}

static void loadAll() {
  memset(g_onBits, 0, sizeof(g_onBits));
  memset(g_unsureBits, 0xFF, sizeof(g_unsureBits));
  prefs.begin("ledstate", true);
  g_bankCount = prefs.getUChar("nbanks", 0);
  if (g_bankCount > MAX_BANKS) g_bankCount = 0;
  if (g_bankCount) prefs.getBytes("banks", g_banks, sizeof(Bank) * g_bankCount);
  bool haveBits = prefs.getBytes("onbits", g_onBits, sizeof(g_onBits)) == sizeof(g_onBits);
  uint8_t legacyMask = prefs.getUChar("on", 0);
  g_curBank = prefs.getUChar("cur", 0);
  prefs.end();

  if (g_bankCount == 0) {
    // default bank: the original switch map (CH1 CC 48, 49, 54, 55)
    g_banks[0] = {{makeKey(1, 48), makeKey(1, 49), makeKey(1, 54), makeKey(1, 55)}};
    g_bankCount = 1;
  }
  if (!haveBits) {
    // migrate the old 4-bit mask from before banks existed
    for (int s = 0; s < 4; s++) setBit(g_onBits, g_banks[0].keys[s], legacyMask & (1 << s));
  }
  if (g_curBank >= g_bankCount) g_curBank = 0;
  memcpy(g_savedOnBits, g_onBits, sizeof(g_onBits));
  g_savedCurBank = g_curBank;

  Serial.printf("Loaded %u bank(s), current bank %u\n", g_bankCount, g_curBank + 1);
  for (int b = 0; b < g_bankCount; b++) {
    Serial.printf("  bank %d:", b + 1);
    for (int s = 0; s < 4; s++) {
      uint16_t k = g_banks[b].keys[s];
      Serial.printf("  sw%d=CH%u CC%u(%s)", s + 1, keyCh(k), keyCc(k), getBit(g_onBits, k) ? "on" : "off");
    }
    Serial.println();
  }
}

// ---------- Message processing (runs in loop, not on the BLE task) ----------

struct CcMsg { uint8_t ch, cc, val; bool first; };
static QueueHandle_t g_ccQueue;
static bool g_firstInPacket = false;  // only touched on the BLE task

static void startSetup() {
  g_mode = MODE_SETUP;
  g_setupSwitch = 0;
  g_setupFirst = NO_KEY;
  g_setupLastActivity = millis();
  g_trigCount = 0;
  Serial.println("== SETUP: change bank, then press switches 1-4 twice each ==");
}

static void endSetup(bool success) {
  g_mode = MODE_NORMAL;
  g_stripFlashStart = millis();
  g_stripFlashActive = true;
  g_stripFlashColor = success ? strip.Color(0, 255, 0) : strip.Color(255, 0, 0);
  if (!success) { Serial.println("== SETUP cancelled =="); return; }

  Bank nb;
  memcpy(nb.keys, g_setupKeys, sizeof(nb.keys));
  int slot = -1;
  for (int b = 0; b < g_bankCount; b++) if (g_banks[b].keys[0] == nb.keys[0]) slot = b;
  if (slot < 0) slot = g_bankCount < MAX_BANKS ? g_bankCount++ : MAX_BANKS - 1;
  g_banks[slot] = nb;
  g_curBank = slot;
  saveBanks();
  g_saveAt = millis() + 1;
  Serial.printf("== SETUP saved as bank %d ==\n", slot + 1);
}

static void setupPress(uint16_t k) {
  g_setupLastActivity = millis();
  if (g_setupFirst != k) {  // first press of this switch, or a different button: (re)start it
    g_setupFirst = k;
    Serial.printf("  setup sw%d: got CH%u CC%u, press again to confirm\n", g_setupSwitch + 1, keyCh(k), keyCc(k));
    return;
  }
  g_setupKeys[g_setupSwitch] = k;
  g_setupFirst = NO_KEY;
  Serial.printf("  setup sw%d = CH%u CC%u\n", g_setupSwitch + 1, keyCh(k), keyCc(k));
  if (++g_setupSwitch == 4) endSetup(true);
}

static bool checkTrigger(uint16_t k, uint32_t now) {
  bool isSwitch1 = false;
  for (int b = 0; b < g_bankCount; b++) if (g_banks[b].keys[0] == k) isSwitch1 = true;
  if (!isSwitch1) { g_trigCount = 0; g_trigKey = NO_KEY; return false; }
  if (k != g_trigKey) { g_trigKey = k; g_trigCount = 0; }
  // shift in the newest press time
  if (g_trigCount < TRIGGER_PRESSES) g_trigCount++;
  for (int i = TRIGGER_PRESSES - 1; i > 0; i--) g_trigTimes[i] = g_trigTimes[i - 1];
  g_trigTimes[0] = now;
  return g_trigCount == TRIGGER_PRESSES && now - g_trigTimes[TRIGGER_PRESSES - 1] <= TRIGGER_WINDOW_MS;
}

static void processCC(const CcMsg& m) {
  uint16_t k = makeKey(m.ch, m.cc);
  bool on = m.val >= 64;
  setBit(g_onBits, k, on);
  setBit(g_unsureBits, k, false);
  g_saveAt = millis() + 1000;  // debounce flash writes
  uint32_t now = millis();

  if (g_mode == MODE_SETUP) {
    if (m.first) setupPress(k);  // the first CC of a packet identifies the button
    return;
  }
  if (!m.first) return;  // e.g. CC 56 riding along with CC 55: state only, no bank switching

  if (switchInBank(g_curBank, k) < 0) {
    for (int b = 0; b < g_bankCount; b++) {
      if (switchInBank(b, k) >= 0) {
        g_curBank = b;
        g_bankFlashStart = now;
        g_bankFlashActive = true;
        Serial.printf("  -> bank %d\n", b + 1);
        break;
      }
    }
  }
  int s = switchInBank(g_curBank, k);
  if (s >= 0) Serial.printf("  bank %d switch %d -> %s\n", g_curBank + 1, s + 1, on ? "ON" : "off");
  if (checkTrigger(k, now)) startSetup();
}

// ---------- Rendering ----------

static void renderLeds() {
  static uint32_t lastRender = 0;
  uint32_t now = millis();
  if (now - lastRender < 20) return;
  lastRender = now;
  bool blink = (now / BLINK_MS) % 2;

  // board LED
  uint8_t br = 0, bb = 0;
  if (!g_connected) {
    br = blink ? BOARD_LED_RED : 0;
  } else if (g_bankFlashActive && now - g_bankFlashStart < 400) {
    uint32_t ph = (now - g_bankFlashStart) / 100;  // on, off, on, off
    br = (ph % 2 == 0) ? BOARD_LED_RED : 0;
  } else {
    g_bankFlashActive = false;
    br = BOARD_LED_PURPLE; bb = BOARD_LED_PURPLE;
  }
  static int lastBoard = -1;
  int boardCode = (br << 8) | bb;
  if (boardCode != lastBoard) { rgbLedWrite(BOARD_LED_PIN, br, 0, bb); lastBoard = boardCode; }

  // strip
  strip.clear();
  if (!g_connected) {
    if (blink) strip.setPixelColor(NUM_PIXELS - 1, strip.Color(255, 0, 0));
  } else if (g_stripFlashActive && now - g_stripFlashStart < 600) {
    if (((now - g_stripFlashStart) / 150) % 2 == 0)
      for (int i = 0; i < NUM_PIXELS; i++) strip.setPixelColor(i, g_stripFlashColor);
  } else if (g_mode == MODE_SETUP) {
    g_stripFlashActive = false;
    bool fast = (now / 200) % 2;
    for (int s = 0; s < 4; s++) {
      if (s < g_setupSwitch) strip.setPixelColor(s * 2, strip.Color(255, 255, 255));
      else if (s == g_setupSwitch) {
        // white blink while waiting; once the first press is in, the pair pixel lights too
        if (fast) strip.setPixelColor(s * 2, strip.Color(255, 255, 255));
        if (g_setupFirst != NO_KEY) strip.setPixelColor(s * 2 + 1, strip.Color(255, 255, 255));
      }
    }
  } else {
    g_stripFlashActive = false;
    const Bank& bank = g_banks[g_curBank];
    for (int s = 0; s < 4; s++) {
      uint16_t k = bank.keys[s];
      if (getBit(g_onBits, k)) strip.setPixelColor(s * 2, strip.Color(0, 0, 255));
      if (getBit(g_unsureBits, k) && blink) strip.setPixelColor(s * 2 + 1, strip.Color(0, UNSURE_LEVEL, 0));
    }
  }
  strip.show();
}

// ---------- MIDI parsing ----------

static void handleCC(uint8_t ch, uint8_t cc, uint8_t value) {
  CcMsg m{ch, cc, value, g_firstInPacket};
  g_firstInPacket = false;
  xQueueSend(g_ccQueue, &m, 0);
}

static void printMidi(uint8_t status, uint8_t d1, uint8_t d2, int len) {
  uint8_t type = status & 0xF0;
  uint8_t ch = (status & 0x0F) + 1;
  switch (type) {
    case 0xB0: Serial.printf("CH%u CC %u = %u\n", ch, d1, d2); handleCC(ch, d1, d2); break;
    case 0x90: Serial.printf("CH%u NoteOn %u vel %u\n", ch, d1, d2); break;
    case 0x80: Serial.printf("CH%u NoteOff %u vel %u\n", ch, d1, d2); break;
    case 0xC0: Serial.printf("CH%u ProgramChange %u\n", ch, d1); break;
    case 0xE0: Serial.printf("CH%u PitchBend %u\n", ch, (d2 << 7) | d1); break;
    default:
      Serial.printf("status 0x%02X", status);
      if (len > 1) Serial.printf(" %u", d1);
      if (len > 2) Serial.printf(" %u", d2);
      Serial.println();
  }
}

static int dataLenFor(uint8_t status) {
  uint8_t t = status & 0xF0;
  if (t == 0xC0 || t == 0xD0) return 1;
  if (t >= 0x80 && t <= 0xE0) return 2;
  if (status == 0xF1 || status == 0xF3) return 1;
  if (status == 0xF2) return 2;
  return 0;  // realtime / other system messages
}

// BLE MIDI packet: [header][timestamp][status][data...]([timestamp][status][data...])...
// Running status allowed; a timestamp byte (bit7 set) precedes each status byte.
static void parseBleMidi(const uint8_t* p, size_t len) {
  if (len < 3) return;
  size_t i = 1;  // skip header
  uint8_t running = 0;
  while (i < len) {
    // timestamp byte (bit7 set) followed by a status byte (bit7 set)
    if ((p[i] & 0x80) && i + 1 < len && (p[i + 1] & 0x80)) {
      i++;  // skip timestamp
      running = p[i++];
    } else if (p[i] & 0x80) {
      // lone timestamp before running-status data
      i++;
      if (i >= len) break;
    }
    if (running == 0xF0) {  // SysEx: skip to end
      while (i < len && p[i] != 0xF7) i++;
      i++;
      running = 0;
      continue;
    }
    int n = dataLenFor(running);
    if (running == 0) break;
    if (n == 0) { printMidi(running, 0, 0, 1); continue; }
    if (i + n > len) break;
    uint8_t d1 = p[i];
    uint8_t d2 = n > 1 ? p[i + 1] : 0;
    i += n;
    printMidi(running, d1, d2, n + 1);
  }
}

static void onMidiNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  Serial.print("raw:");
  for (size_t i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
  Serial.print("  -> ");
  g_firstInPacket = true;
  parseBleMidi(data, len);
}

// ---------- BLE callbacks ----------

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* c) override {
    Serial.printf("Connected to %s\n", c->getPeerAddress().toString().c_str());
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    Serial.printf("Disconnected (reason %d), rescanning...\n", reason);
    // Unsure flags are deliberately kept: the ESP32 stayed powered, and a footswitch that
    // is on would be broadcasting, so a BLE drop alone doesn't make the state unsure.
    g_connected = false;
    NimBLEDevice::getScan()->start(0, false, true);
  }
} g_clientCb;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    std::string name = dev->getName();
    if (!name.empty()) {
      Serial.printf("  seen: '%s' [%s] RSSI %d svcUUIDs=%d\n", name.c_str(),
                    dev->getAddress().toString().c_str(), dev->getRSSI(), dev->getServiceUUIDCount());
    }
    std::string lower = name;
    for (auto& ch : lower) ch = tolower(ch);
    bool nameMatch = lower.find("chocolate") != std::string::npos ||
                     lower.find("m-vave") != std::string::npos ||
                     lower.find("mvave") != std::string::npos ||
                     lower.find("footctrl") != std::string::npos ||
                     lower.find("midi") != std::string::npos;
    if (dev->isAdvertisingService(MIDI_SERVICE_UUID) || nameMatch) {
      Serial.printf("Found BLE MIDI device: '%s' [%s] RSSI %d\n",
                    dev->getName().c_str(), dev->getAddress().toString().c_str(), dev->getRSSI());
      NimBLEDevice::getScan()->stop();
      g_target = dev;
      g_doConnect = true;
    }
  }
  void onScanEnd(const NimBLEScanResults&, int) override {
    if (!g_doConnect) NimBLEDevice::getScan()->start(0, false, true);
  }
} g_scanCb;

static bool connectToTarget() {
  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    g_client->setClientCallbacks(&g_clientCb, false);
    g_client->setConnectionParams(12, 12, 0, 400);  // 15ms interval
    g_client->setConnectTimeout(5000);
  }
  if (!g_client->connect(g_target, false)) {
    Serial.println("Connect failed");
    return false;
  }
  NimBLERemoteService* svc = g_client->getService(MIDI_SERVICE_UUID);
  if (!svc) { Serial.println("MIDI service not found"); g_client->disconnect(); return false; }
  NimBLERemoteCharacteristic* chr = svc->getCharacteristic(MIDI_CHAR_UUID);
  if (!chr) { Serial.println("MIDI characteristic not found"); g_client->disconnect(); return false; }
  if (!chr->canNotify() || !chr->subscribe(true, onMidiNotify)) {
    Serial.println("Subscribe failed");
    g_client->disconnect();
    return false;
  }
  Serial.println("Subscribed to MIDI notifications. Press a switch.");
  g_connected = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBLE MIDI listener starting");
  rgbLedWrite(BOARD_LED_PIN, BOARD_LED_RED, 0, 0);  // red from the start, blinks until connected
  g_ccQueue = xQueueCreate(32, sizeof(CcMsg));
  loadAll();
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  // boot animation: one white pixel sweeps left -> right -> left in 2 s
  // (0..7..0 = 15 frames at ~133 ms each)
  const int frames = NUM_PIXELS * 2 - 1;
  for (int f = 0; f < frames; f++) {
    int i = f < NUM_PIXELS ? f : frames - 1 - f;
    strip.clear();
    strip.setPixelColor(i, strip.Color(255, 255, 255));
    strip.show();
    delay(2000 / frames);
  }
  strip.clear();
  strip.show();
  NimBLEDevice::init("ESP32-MIDI-Listener");
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&g_scanCb, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(100);
  scan->start(0, false, true);
  Serial.println("Scanning for BLE MIDI service...");
}

void loop() {
  if (g_doConnect) {
    g_doConnect = false;
    if (!connectToTarget()) {
      delay(1000);
      NimBLEDevice::getScan()->start(0, false, true);
    }
  }
  CcMsg m;
  while (xQueueReceive(g_ccQueue, &m, 0) == pdTRUE) processCC(m);

  if (g_mode == MODE_SETUP && (!g_connected || millis() - g_setupLastActivity > SETUP_TIMEOUT_MS))
    endSetup(false);
  if (g_saveAt && (int32_t)(millis() - g_saveAt) >= 0) {
    g_saveAt = 0;
    saveState();
  }
  renderLeds();
  delay(5);
}

#endif  // LED_TEST
