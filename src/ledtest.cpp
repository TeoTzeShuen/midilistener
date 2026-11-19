// LED / GPIO diagnostic, no BLE. Build/flash with:
//   python -m platformio run -e ledtest -t upload --upload-port COM7
// Repeats a 9 s cycle, printing each phase over serial:
//   1. GPIO14 held HIGH for 3 s  (multimeter GPIO14 -> GND should read ~3.3 V)
//   2. GPIO14 held LOW  for 3 s  (should read ~0 V)
//   3. Strip on GPIO14 painted R,G,B,R,G,B,R,G at 30% for 3 s
// The onboard RGB (GPIO48) is set to purple via the core's rgbLedWrite; on many clone
// boards it only works once the "RGB" solder jumper is bridged.
#ifdef LED_TEST

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

static const uint8_t PIN = 14;
static Adafruit_NeoPixel strip(8, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nLED diagnostic starting");
  rgbLedWrite(48, 40, 0, 40);
}

void loop() {
  Serial.println("Phase 1: GPIO14 HIGH (expect ~3.3V)");
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  delay(3000);

  Serial.println("Phase 2: GPIO14 LOW (expect ~0V)");
  digitalWrite(PIN, LOW);
  delay(3000);

  Serial.println("Phase 3: strip R,G,B,R,G,B,R,G");
  strip.begin();
  strip.setBrightness(77);  // 30%
  const uint32_t colours[3] = {strip.Color(255, 0, 0), strip.Color(0, 255, 0), strip.Color(0, 0, 255)};
  for (int i = 0; i < 8; i++) strip.setPixelColor(i, colours[i % 3]);
  for (int k = 0; k < 6; k++) { strip.show(); delay(500); }
}

#endif  // LED_TEST
