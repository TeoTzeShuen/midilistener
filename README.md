# Program for ESP32-S3 boards to listen to BLE MIDI signals from a M-vave Chocolate Plus footswitch.

One issue I found with the foodswitch is that it doesn't have per-switch LEDS to tell you states. This program intends to solve that by listening to the board's BLE broadcasts and updating the LEDs based on the received 

You will need: 

- An ESP32-S3 (On some clones of the devkitC, you may need to bridge the LED and IN>OUT Pins on the board. There is no need to bridge USB OTG).
- A 8 LED WS2812B LED Strip. (These need a 5v data by right, but as it's just 8 leds we can get away with 3.3v)

Hardware preparation: Connect the following pin pairings: 5V to VCC, GND to GND, GPIO14 to DIN (Data IN)

The mapping is set in the source code, currently
1 	48 	
2 	49 	
3 	54 	
4 	55 + 56

This is the setup for my hotone, change it to fit your mapping.
