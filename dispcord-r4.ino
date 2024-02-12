#include <Adafruit_GFX.h>
#include "Font4x7Fixed.h"

/**
 *
 * Drive both A and B sections together. We'll assume they are both
 * (8*8)x(7*2), 64x14 pixels, during rendering.
 *
 * Each display chain uses two shift registers which are driven in
 * parallel for some reason. For the A chain, we'll call these ports
 * SHAA and SHAB. For the B chain, they will be called SHBA, SHBB.
 *
 * +------------------+------------------+------------------+
 * |  A2              |  A1              |   A0             |  <--- Input
 * +------------------+------------------+------------------+
 * |  A5              |  A4              |   A3             |
 * +------------------+------------------+------------------+
 * |  B2              |  B1              |   B0             |  <--- Input
 * +------------------+------------------+------------------+
 *                    |  B3              |
 *                    +------------------+
 *
 * We can probably share a single pin for the pairs of shift registers.
 */

#define CLOCK R_PORT1, 4
#define STROBE R_PORT1, 5
#define SHAA R_PORT1, 2
#define SHAB R_PORT1, 7
#define SHBA R_PORT1, 2
#define SBBB R_PORT1, 7
#define A R_PORT4, 10
#define B R_PORT4, 11
#define C R_PORT1, 3
#define D R_PORT1, 6

//These macros help do the pin setting for an ARM based Arduino (e.g. R4).
#define _PIN_SET(port, pin, value) if (value) { port->POSR = bit(pin); } else { port->PORR = bit(pin); }
#define _OUTPUT(port, pin) port->PDR |= bit(pin)

#define PIN_SET(...) _PIN_SET(__VA_ARGS__)
#define OUTPUT(...) _OUTPUT(__VA_ARGS__)

GFXcanvas1 buffer(64*8, 7*4);

char *message = "An efficient font";

void setup() {
  OUTPUT(STROBE);
  OUTPUT(CLOCK);
  OUTPUT(SHAA);
  OUTPUT(SHAB);
  OUTPUT(SHBA);
  OUTPUT(SHBB);
  OUTPUT(A);
  OUTPUT(B);
  OUTPUT(C);
  OUTPUT(D);

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
          PIN_SET(SHAA, state);
          PIN_SET(SHAB, state);

	  // Pulse the clock
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

      // The delay is important to avoid ghosting between rows
      delayMicroseconds(650);
    }
}
