#include <Adafruit_GFX.h>
#include "Font4x7Fixed.h"

#define CLOCK R_PORT1, 4
#define STROBE R_PORT1, 5
#define SHA R_PORT1, 2
#define SHB R_PORT1, 7
#define A R_PORT4, 10
#define B R_PORT4, 11
#define C R_PORT1, 3
#define D R_PORT1, 6

#define _PIN_SET(port, pin, value) if (value) { port->POSR = bit(pin); } else { port->PORR = bit(pin); }
#define PIN_SET(...) _PIN_SET(__VA_ARGS__)

#define _OUTPUT(port, pin) port->PDR |= bit(pin)
#define OUTPUT(...) _OUTPUT(__VA_ARGS__)

GFXcanvas1 buffer(64, 7); 

char *message = "An efficient font";

void setup() {
  OUTPUT(STROBE);
  OUTPUT(CLOCK);
  OUTPUT(SHA);
  OUTPUT(SHB);
  OUTPUT(A);
  OUTPUT(B);
  OUTPUT(C);
  OUTPUT(D);

  PIN_SET(SHA, 1);
  PIN_SET(SHB, 1);
  PIN_SET(SHA, 0);
  PIN_SET(SHB, 0);

  buffer.setFont(&Font4x7Fixed);
  buffer.setTextSize(1);
}

void loop() {
  buffer.fillScreen(0);
  buffer.setCursor(0, 7);
  buffer.print(message);

  for (int row=0; row < 7; row++) {
      for (int m=0; m < 1;  m++) {
        for (int col=0; col < 64; col++) {
          int state = buffer.getPixel(col, row);
          PIN_SET(SHA, state);
          PIN_SET(SHB, state);

          // Take a few cycles to make sure this is long enough
          PIN_SET(CLOCK, 1);
          PIN_SET(CLOCK, 0);
        }
      }

      // Set row address via BCD lines
      PIN_SET(A, row & 1);
      PIN_SET(B, row & 2);
      PIN_SET(C, row & 4);

      // Strobe
      PIN_SET(STROBE, HIGH);
      PIN_SET(STROBE, LOW);
      delayMicroseconds(1000); 
    }
}
