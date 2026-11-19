# ESP32 MIDI LED Indicator: Handoff Note

Last updated: 18 November 2026

## Goal

Add per-switch LED indicators to an M-Vave Chocolate Plus footswitch. The Chocolate Plus only shows the last switch pressed on its display. An ESP32-S3 will connect to it over BLE MIDI as a listener and drive four individually addressable LEDs that mirror each switch's on/off state. The footswitch stays wired to the multiFX as normal, so the ESP32 is never in the signal path. If it crashes, the rig keeps working.

## Confirmed so far

- The Chocolate Plus broadcasts MIDI over BLE and over its wired output at the same time. Tested with a phone MIDI monitor app while it was plugged in.
- It is visible in nRF Connect after a reset (it was not visible while the phone was already bonded or connected). It needs no PIN or authentication on first pairing, so the firmware needs no bonding or security callbacks.
- BLE MIDI peripherals normally accept one central at a time. The phone must be fully disconnected whenever the ESP32 is meant to connect.
- The footswitch is used in CC mode. Each switch toggles between two values on successive presses. Switch 1 sends CH1, CC 17: value 0 (off), then 64 (on), alternating.
- LED logic is therefore stateless: value >= 64 means LED on, anything lower means off.

## Hardware (all received)

- ESP32-S3-DevKitC-1 (Espressif pinout diagram in hand)
- WS2812B addressable LEDs, 4 pixels
- 10V 680µF electrolytic capacitor (fine for a 5V rail; the 10V rating gives roughly 2x headroom)
- 300-470Ω resistor for the data line
- 3-pin JST connectors
- Bidirectional logic level converter (BSS138-style, HV/LV sides)

## Wiring plan

JST pin order, Data / VCC / GND:

| JST pin | Board pin |
|---|---|
| Data in | GPIO14 |
| VCC | 5V0 |
| GND | GND |

On the DevKitC-1 diagram, GPIO14, 5V0 and GND sit together on the bottom-left header, in that order going down.

- GPIO14 is a plain GPIO with no strapping function. The S3 strapping pins are GPIO0, 3, 45 and 46. Any GPIO can drive WS2812B data via the RMT peripheral.
- 5V0 is fed from USB, so it is only live when the board is powered over USB.
- Put a 300-470Ω resistor in series on the data line, close to the first pixel.
- Put the 680µF capacitor across 5V and GND at the LED end. The stripe on the can marks negative, which goes to GND. Reversing it can make it fail or pop.
- Before connecting the LEDs, check that the JST pad order matches the pixel pad order. Some suppliers use 5V/Data/GND and others Data/5V/GND. Swapping 5V and data can kill a pixel. Also check for roughly 5V between VCC and GND on the JST with a multimeter.
- Cap brightness at about 20-30%. Four pixels at full white could draw about 240mA.

### Logic level (3.3V data into 5V pixels)

The S3 outputs 3.3V, while a WS2812B powered at 5V formally wants about 3.5V or more on data. Plan, in order:

1. Test direct, with no shifter. It often works with four pixels and short wires.
2. If it flickers or shows wrong colours, add the level converter:
   - HV to the same 5V0 feeding the pixels
   - LV to a 3V3 pin (the 3V3 pins are on the top-left of the board, so use a jumper lead)
   - GND to GND
   - GPIO14 to LV1, then HV1 to the series resistor, then to the first pixel's Data In
3. BSS138-style modules are slow and can round off the 800kHz signal. If they still misbehave, use a 74AHCT125 or 74HCT245 buffer instead.
4. Alternative trick if not using a shifter: put a silicon diode (e.g. 1N4001) in series on the pixels' 5V feed. This drops the supply to about 4.3V and lowers the data threshold to about 3.0V. Do not combine this with a shifter, because the HV rail should stay at full 5V.

## Firmware plan (not yet written)

- Arduino framework on ESP32-S3, NimBLE-Arduino stack
- Act as a BLE central. Scan for the BLE MIDI service UUID `03B80E5A-EDE8-4B33-A751-6CE34EC4C700` rather than a device name.
- Connect, then subscribe to notifications on the BLE MIDI characteristic.
- Parse the BLE MIDI packets: strip the header and timestamp bytes. A single packet can contain several messages.
- Map CC numbers to the four LEDs (pixel indices on GPIO14) and set each from its value.
- Use FastLED or Adafruit NeoPixel. FastLED gives more flexible colour handling. A possible scheme is green for on and a dim red or off for off.
- Auto-rescan and reconnect on disconnect or when the footswitch powers up late. Treat this as essential, not optional.

## Known caveats

- **Startup sync:** the footswitch holds its own toggle state and only reveals it on the next press. After ESP32 boot or a BLE reconnect, an LED may disagree with the real switch state until that switch is pressed once. To check whether the Chocolate sends its current states on connect, log what arrives in a MIDI monitor right after connecting. Otherwise, boot with all LEDs off and accept first-press sync.
- **Preset or state drift:** the LEDs reflect the footswitch's messages, not the multiFX's actual effect state. Changing a preset or toggling something on the multiFX itself will not update them, unless the multiFX sends state back over MIDI (not investigated).
- **Latency:** BLE adds roughly 7-30ms, which is irrelevant for LEDs because the wired path to the multiFX is unaffected.

## BLE listener test (18 November 2026): working

- Sketch in `src/main.cpp` (PlatformIO project, `platformio.ini`). Build/flash: `python -m platformio run -t upload --upload-port COM7`.
- Board is used on the **UART** port (CH343, COM7). Serial goes to UART0 (`ARDUINO_USB_CDC_ON_BOOT=0`), and auto-reset/flash works there. The native USB port needed a manual BOOT+RESET to flash.
- Advertised BLE name: **FootCtrlPlus**. The scan never matched on the MIDI service UUID, so the sketch also matches by name ("footctrl", "chocolate", "m-vave", "midi"). It connected with no pairing or bonding.
- Packets come one message per notify, e.g. `80 80 B0 30 40` = CH1 CC 48 value 64. Observed CC 48 (0/64) and CC 49 (0/64), so the current bank/mode does not use the CC 17 noted above.
- The footswitch may need to be in a fresh "pending pairing" state, not connected to the phone, for the ESP32 to find it.

### Switch-to-CC map (current bank, CH1, values toggle 0/64)

| Switch | CC | Notes |
|---|---|---|
| 1 | 48 | |
| 2 | 49 | |
| 3 | 54 | |
| 4 | 55 + 56 | Sends both CCs in one BLE packet (running status: `B0 37 40 38 40`), always with the same value. Drive LED 4 from CC 55 and ignore 56. |

The toggle state is held in the footswitch, so the first press after connecting can send either 0 or 64. That confirms the first-press sync caveat below.

## Board quirk: clone, not an official DevKitC-1

- The board is an ESP32-S3 N16R8 clone (16MB flash, 8MB PSRAM, CH343 USB-UART), YD-ESP32-S3 style.
- Out of the box its **5V pin carried no USB power**, so the strip was dark. Fix: bridge the **IN-OUT** solder pad. From now on, never feed outside 5V into that pin while USB is connected.
- The onboard RGB LED (GPIO48) only works once the **RGB** pad is bridged (now bridged).
- Diagnostic build: `python -m platformio run -e ledtest -t upload --upload-port COM7` (GPIO14 high/low phases plus a strip colour test, see `src/ledtest.cpp`).

## LED behaviour (implemented 18 November 2026)

- 8-pixel WS2812 strip, data on GPIO14. Adafruit NeoPixel, brightness 51/255 (20%).
- Boot animation: a single white pixel sweeps left to right and back in 2 s (15 frames), then the saved state is shown.
- Pixels 1, 3, 5, 7 show switches 1-4: blue if value >= 64, off otherwise.
- Pixels 2, 4, 6, 8 are "unsure" indicators at 5% absolute brightness (green scaled to 63 under the 20% strip brightness). After boot each one blinks green (500ms) until its own switch sends a message.
- Not connected (from boot, or after any disconnect, until the BLE MIDI subscription is up): the strip is dark except pixel 8, which blinks red in step with the onboard RGB LED (GPIO48, via `rgbLedWrite`). Once connected, the onboard LED is steady dim purple and the strip shows switch states. A BLE drop does not re-arm the unsure indicators; only a reboot of the ESP32 does.
- Indicator states are saved to NVS (Preferences namespace `ledstate`, key `on`, bitmask) whenever they change, and restored at boot. Tested: switches 1 and 4 left on, reset, mask 0x9 restored.
- On a BLE disconnect, all four unsure indicators start blinking again until each switch reports.

## Open items

1. CC numbers for all four switches (only switch 1 is known: CC 17, CH1).
2. Whether more than one bank or mode is used. Banks would change the CC numbers and may need bank-change handling.
3. The footswitch's advertised BLE name (noted in nRF Connect, if it is useful to target it specifically).
4. Whether direct 3.3V data drive works, or whether the level converter (or diode trick) is needed.
5. Which USB port to power the board from. Either port supplies 5V.
6. Write the sketch. It can be done as soon as items 1 and 2 are known.

## Suggested build order

1. Wire the LEDs and run a simple colour-test sketch on GPIO14. This verifies the power, the data signal and the logic level before any BLE work.
2. Get a BLE MIDI client sketch that only prints incoming messages over serial. Confirm it connects while the footswitch is also wired to the multiFX.
3. Combine them with the CC-to-LED mapping.
4. Add reconnect handling, then soak-test by power-cycling the footswitch and ESP32 in different orders.
