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
 * |//////////////////|  B3              |//////////////////|
 * +------------------+------------------+------------------+
 *
 * We can probably share a single pin for the pairs of shift registers.
 */

#define CLOCK R_PORT3, 1   // D0
#define SHA R_PORT3, 2     // D1
#define A R_PORT1, 4       // D2
#define B R_PORT1, 5       // D3
#define C R_PORT1, 6       // D4
#define STROBE R_PORT1, 7  // D5
#define SHB R_PORT1, 11    // D6

// In the PCB version, we swap STROBE and SHB to make the board nicer
/* #define STROBE R_PORT1, 11  // D6 */
/* #define SHB R_PORT1, 7      // D5 */

//These macros help do the pin setting for an ARM based Arduino (e.g. R4).
#define _PIN_SET(port, pin, value) if (value) { port->POSR = bit(pin); } else { port->PORR = bit(pin); }
#define _OUTPUT(port, pin) port->PDR |= bit(pin)

#define PIN_SET(...) _PIN_SET(__VA_ARGS__)
#define OUTPUT(...) _OUTPUT(__VA_ARGS__)

#define PULSE(port) _PIN_SET(port, HIGH); _PIN_SET(port, LOW)

GFXcanvas1 buffer(8*8*3, 7*4);

char *message = "An efficient font hello hello hello";

void setup() {
  OUTPUT(STROBE); OUTPUT(CLOCK);
  OUTPUT(SHA); OUTPUT(SHB);
  OUTPUT(A); OUTPUT(B); OUTPUT(C);// OUTPUT(D);

  buffer.setFont(&Font4x7Fixed);
  buffer.setTextSize(1);
}

void loop() {
  int x,y;
  bool stateA, stateB;

  //buffer.fillScreen(1);
  buffer.setCursor(0, 7);
  buffer.print(message);

  buffer.setCursor(0, 14);
  buffer.print("Hello fom the train display");

  buffer.setCursor(0, 21);
  buffer.print("Bottom line");

  // Draw a grid
  // for (int gx = 0; gx < 192; gx += 4) {
  //    buffer.drawFastVLine(gx, 0, 14, 1);
  // }
  // for (int gy = 0; gy < 14; gy += 4) {
  //    buffer.drawFastHLine(gy, 0, 192, 1);
  // }

  for (int row=0; row < 7; row++) {
    for (int col=0; col < (8*8*3*2); col++) {
      // The rows have to be swapped because of the order the bits are
      // pushed out.
      x = col % 192;
      y = row + (col < 192 ? 7 : 0);

      stateA = buffer.getPixel(x, y);
      PIN_SET(SHA, stateA);

      // We need to skip 2 lines (+14), but also like above, we need
      // to swap the two lines because they are drawn from the bottom
      // up.
      x = col % 192;
      y = row + (col < 192 ? 7 : 0) + 14;

      // Since we have to push out the same number of bits for top and
      // bottom lines, we want to repeat the bottom-middle section
      // three times. This will ensure that when the bottom-upper line
      // is pushed out, it will have data in it for the bottom-middle
      // section. We can do this by checking whether we are on the
      // bottom row, and then modulo 64 the x.
      //
      // Essential what we want is the following (refer to the diagram
      // at the top): B3 B3 B3 B2 B1 B0
      if (col < 192) x = (x % 64) + 64;

      stateB = buffer.getPixel(x, y);
      PIN_SET(SHB, stateB);

      // Pulse the clock
      PULSE(CLOCK);
    }

    // Set row address via BCD lines
    PIN_SET(A, row & 1);
    PIN_SET(B, row & 2);
    PIN_SET(C, row & 4);

    // Latch the data
    PULSE(STROBE);

    // The delay is important to avoid ghosting between rows
    delayMicroseconds(1000);
  }
}
