// BLE MIDI listener: connects to a BLE MIDI peripheral (e.g. M-Vave Chocolate Plus)
// as a central, subscribes to the MIDI characteristic, prints every message and
// mirrors each switch's CC state on a WS2812 strip. Auto-rescans and reconnects.

#ifndef LED_TEST  // the ledtest env builds src/ledtest.cpp instead

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

static const NimBLEUUID MIDI_SERVICE_UUID("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
static const NimBLEUUID MIDI_CHAR_UUID("7772E5DB-3868-4112-A1A9-F2669D106BF3");

static const NimBLEAdvertisedDevice* g_target = nullptr;
static NimBLEClient* g_client = nullptr;
static volatile bool g_doConnect = false;

// ---------- LEDs ----------
// 8-pixel WS2812 strip on GPIO14. Pixels 1,3,5,7 (index 0,2,4,6) show switch 1-4 state:
// off for value < 64, blue for >= 64. Pixels 2,4,6,8 (index 1,3,5,7) blink green after
// boot until that switch sends its first message ("unsure" indicator).
static const uint8_t LED_PIN = 14;
static const uint8_t NUM_PIXELS = 8;
static const uint8_t BRIGHTNESS = 51;  // 20% of 255
static const uint8_t MIDI_CHANNEL = 1;
static const uint8_t SWITCH_CC[4] = {48, 49, 54, 55};  // switch 4 also sends CC 56; ignored
static const uint32_t BLINK_MS = 500;
// Unsure pixels at 5% absolute: the strip-wide brightness is 20%, so scale the colour by 5/20.
static const uint8_t UNSURE_LEVEL = 255 * 5 / 20;  // 63
// Onboard RGB LED (needs the "RGB" pad bridged): blinks red until BLE MIDI is subscribed,
// then steady purple. While disconnected the strip is dark apart from its last pixel blinking red.
static const uint8_t BOARD_LED_PIN = 48;
static const uint8_t BOARD_LED_RED = 40;
static const uint8_t BOARD_LED_PURPLE = 30;  // per channel (red + blue)
static volatile bool g_connected = false;

static Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
static Preferences prefs;

static volatile bool g_on[4] = {false, false, false, false};
static volatile bool g_unsure[4] = {true, true, true, true};
static volatile bool g_dirty = true;      // redraw needed
static volatile bool g_saveNeeded = false;

static void loadState() {
  prefs.begin("ledstate", true);
  uint8_t mask = prefs.getUChar("on", 0);
  prefs.end();
  for (int s = 0; s < 4; s++) g_on[s] = mask & (1 << s);
  Serial.printf("Restored LED state mask 0x%X\n", mask);
}

static void saveState() {
  uint8_t mask = 0;
  for (int s = 0; s < 4; s++) if (g_on[s]) mask |= 1 << s;
  prefs.begin("ledstate", false);
  if (prefs.getUChar("on", 0xFF) != mask) prefs.putUChar("on", mask);
  prefs.end();
}

static void handleCC(uint8_t ch, uint8_t cc, uint8_t value) {
  if (ch != MIDI_CHANNEL) return;
  for (int s = 0; s < 4; s++) {
    if (SWITCH_CC[s] == cc) {
      g_on[s] = value >= 64;
      g_unsure[s] = false;
      g_dirty = true;
      g_saveNeeded = true;
      Serial.printf("  switch %d -> %s\n", s + 1, value >= 64 ? "ON" : "off");
    }
  }
}

static void renderLeds() {
  static bool lastBlink = false;
  bool blink = (millis() / BLINK_MS) % 2;
  if (!g_dirty && blink == lastBlink) return;
  lastBlink = blink;
  g_dirty = false;
  if (!g_connected) {
    // no BLE link: strip dark except the last pixel blinking red, in step with the board LED
    strip.clear();
    if (blink) strip.setPixelColor(NUM_PIXELS - 1, strip.Color(255, 0, 0));
    strip.show();
    rgbLedWrite(BOARD_LED_PIN, blink ? BOARD_LED_RED : 0, 0, 0);
    return;
  }
  for (int s = 0; s < 4; s++) {
    strip.setPixelColor(s * 2, g_on[s] ? strip.Color(0, 0, 255) : 0);
    strip.setPixelColor(s * 2 + 1, (g_unsure[s] && blink) ? strip.Color(0, UNSURE_LEVEL, 0) : 0);
  }
  strip.show();
  rgbLedWrite(BOARD_LED_PIN, BOARD_LED_PURPLE, 0, BOARD_LED_PURPLE);  // connected
}

// ---------- MIDI parsing ----------

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
    g_dirty = true;
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
  g_dirty = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\nBLE MIDI listener starting");
  rgbLedWrite(BOARD_LED_PIN, BOARD_LED_RED, 0, 0);  // red from the start, blinks until connected
  loadState();
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
  renderLeds();
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
  if (g_saveNeeded) {
    g_saveNeeded = false;
    saveState();
  }
  renderLeds();
  delay(10);
}

#endif  // LED_TEST
