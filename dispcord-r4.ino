#include "WiFiS3.h"
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <ArduinoJson.h>
#include "Font5x7Fixed.h"
#include <RTC.h>
#include <NTPClient.h>
#include "FspTimer.h"

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
#define STROBE R_PORT1, 11 // D6
#define SHB R_PORT1, 7     // D5

// WiFi credentials
#define SSID ""
#define PASS ""
#define MQTT_HOST ""
#define MQTT_PORT 1883
#define NAME "DepartureBoard"

#define STATUS_TOPIC "nh/status"
#define CHANNEL_TOPIC "nh/discord/rx"
#define DOORBELL_TOPIC "nh/gk/DoorButton"
#define TEMPERATURE_TOPIC "nh/temperature"
#define DEPARTURE_TOPIC "nh/tdb/NOT"

//These macros help do the pin setting for an ARM based Arduino (e.g. R4).
#define NOP __asm__("nop")
#define _PIN_SET(port, pin, value) if (value) { port->POSR = bit(pin); } else { port->PORR = bit(pin); }
#define _OUTPUT(port, pin) port->PDR |= bit(pin)
#define PIN_SET(...) _PIN_SET(__VA_ARGS__)
#define OUTPUT(...) _OUTPUT(__VA_ARGS__)
#define PULSE(port) _PIN_SET(port, HIGH); NOP; NOP; NOP; NOP; NOP; NOP; _PIN_SET(port, LOW)

#define WIDTH 8*8*3
#define HEIGHT 7*4

// Global objects
GFXcanvas1 buffer(WIDTH, HEIGHT);
volatile uint8_t *rawBuffer;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WiFiUDP Udp;
NTPClient timeClient(Udp);
FspTimer timer;

//#define DEBUG_HOME

// State
char discordMessage[4096];
char discordUsername[64];
char discordChannel[64];
char doorbell[32];
char temperatureArea[32];
char temperatureValue[32];
char departures[512];
bool updateTimeOnly; // Used while redrawing the departure board

unsigned long clearTimer; // time to clear the doorbell (or other) message
unsigned long redrawTimer; // Optionally force a redraw at millis()
int lineOffset;
volatile int row;
volatile int col;

void drawBufferISR(timer_callback_args_t __attribute((unused)) *p_args);

bool beginTimer(float rate) {
  uint8_t timer_type = AGT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0){
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0){
    return false;
  }

  if(!timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, drawBufferISR)){
    return false;
  }

  if (!timer.setup_overflow_irq()){
    return false;
  }

  if (!timer.open()){
    return false;
  }

  if (!timer.start()){
    return false;
  }
  return true;
}

void setup() {
  OUTPUT(STROBE); OUTPUT(CLOCK);
  OUTPUT(SHA); OUTPUT(SHB);
  OUTPUT(A); OUTPUT(B); OUTPUT(C);

  buffer.setFont(&Font5x7Fixed);
  buffer.setTextSize(1);
  rawBuffer = buffer.getBuffer();

  Serial.begin(115200); // USB
  while (!Serial);
  Serial.println("Setup...");

  randomSeed(micros());

  clearTimer = 0;
  redrawTimer = 0;
  lineOffset = 0;
  row = 0;
  col = 0;

  if (!beginTimer(8000)) {
    Serial.println("Issue with timer interrupt");
  }

  doorbell[0] = '\0';
  temperatureArea[0] = '\0';
  temperatureValue[0] = '\0';
  departures[0] = '\0';

  checkWiFi();
  checkMqtt();

  RTC.begin();
  timeClient.begin();
  timeClient.update();

  auto unixTime = timeClient.getEpochTime();
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);
}

/**
 * Right, I know this is horrible. Let me explain:
 *
 * - It used to be `for (int i=0; i < 384; i++) { ... }` but this
 *   required a modulo operation with a non-power of two. That
 *   required like clock cycles rather than 2. The loop was split into
 *   two parts, 0-191 and 192 to 383. Much faster.
 *
 * - While trying to get this to run as an ISR, the WiFi kept
 *   disconnecting. The ISR was taking too long, so I split each of
 *   the loops into 4 parts. This means if the interrupt timer is
 *   firing at 8KHz, we are doing 8000 / 2 / 4 / 7 frames per
 *   second. The seven comes from the number of rows. Roughly 140fps,
 *   quite good.
 */
void drawBufferISR(timer_callback_args_t __attribute((unused)) *p_args) {
  int x,y;
  volatile uint8_t *ptr;

  if (col >= 0 && col < 192) {
    for (int i=0; i < 48; i++) {
      // The rows have to be swapped because of the order the bits are
      // pushed out.
      x = col;
      y = row + 7;

      ptr = &rawBuffer[(x / 8) + y * ((WIDTH + 7) / 8)];
      PIN_SET(SHA, ((*ptr) & (0x80 >> (x & 7))) != 0);

      // We need to skip 2 lines (+14), but also like above, we need
      // to swap the two lines because they are drawn from the bottom
      // up.
      y = row + 21;

      // Essential what we want is the following (refer to the diagram
      // at the top): B3 B3 B3 B2 B1 B0
      x = (x % 64) + 64;

      ptr = &rawBuffer[(x / 8) + y * ((WIDTH + 7) / 8)];
      PIN_SET(SHB, ((*ptr) & (0x80 >> (x & 7))) != 0);

      // Pulse the clock
      PULSE(CLOCK);
      
      col++;
    }
    return;
  }
  if (col >= 192 && col < 384) {
    for (int i=0; i < 48; i++) {
      // The rows have to be swapped because of the order the bits are
      // pushed out.
      x = col - 192;
      y = row;

      ptr = &rawBuffer[(x / 8) + y * ((WIDTH + 7) / 8)];
      PIN_SET(SHA, ((*ptr) & (0x80 >> (x & 7))) != 0);

      // We need to skip 2 lines (+14), but also like above, we need
      // to swap the two lines because they are drawn from the bottom
      // up.
      y = row + 14;

      ptr = &rawBuffer[(x / 8) + y * ((WIDTH + 7) / 8)];
      PIN_SET(SHB, ((*ptr) & (0x80 >> (x & 7))) != 0);

      // Pulse the clock
      PULSE(CLOCK);
      col++;
    }
    return;
  }

  if (col >= 384) {
    // Set row address via BCD lines
    PIN_SET(A, row & 1);
    PIN_SET(B, row & 2);
    PIN_SET(C, row & 4);

    // Latch the data
    PULSE(STROBE);

    row++;
    if (row == 7) row = 0;
    
    col = 0;
  }
}

void drawBufferASCII() {
  Serial.println();
  for (int y = 0; y < HEIGHT; y++) {
    if ((y % 7) == 0)
      Serial.println();
    for (int x = 0; x < WIDTH; x++) {
      if ((x % 64) == 0)
        Serial.print("|");
      Serial.print(buffer.getPixel(x, y) ? "█" : " ");
    }
    Serial.println();
  }
  Serial.println();
}

void print(char *msg) {
  buffer.setCursor(0, 7);

  buffer.fillScreen(0x0000);
  buffer.print(msg);

  Serial.println(msg);

#ifdef DEBUG_HOME
  drawBufferASCII();
#endif
}

void checkWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  print("Waiting for WiFi :)");

  while (WiFi.begin(SSID, PASS) != WL_CONNECTED) {
    delay(100);
  }

  print("WiFi OK :D");

  long rssi = WiFi.RSSI();

  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);
}

void checkMqtt() {
  if (mqtt.connected()) return;
  mqtt.setBufferSize(2048);

  print("Waiting for MQTT... :'(");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  while (!mqtt.connected()) {
    String clientId = NAME;
    clientId += String(random(0xffff), HEX);

    if (mqtt.connect(clientId.c_str())) {
      print("MQTT has connected :D");

      mqtt.subscribe(STATUS_TOPIC "/req");
      mqtt.subscribe(CHANNEL_TOPIC "/#");
      mqtt.subscribe(DOORBELL_TOPIC "/#");
      //mqtt.subscribe(TEMPERATURE_TOPIC "/#");
      mqtt.subscribe(DEPARTURE_TOPIC);

      mqtt.publish("nh/discord/tx/pm/asjackson", NAME " - Restarted");
      mqtt.publish("nh/irc/tx/pm/asjackson", NAME " - Restarted");
    } else {
      print("MQTT connection failed :'(");
      delay(1000);
    }
  }

  print("Waiting patiently for messages :D");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, STATUS_TOPIC "/req") == 0 && memcmp(payload, "STATUS", length) == 0) {
    mqtt.publish(STATUS_TOPIC "/res", "Running: " NAME);
    return;
  }

  redrawTimer = 0;
  lineOffset = 0;

  if (strncmp(topic, DEPARTURE_TOPIC, strlen(DEPARTURE_TOPIC)) == 0) {
    strncpy(departures, (const char*)payload, length);
    clearTimer = millis() + 60*1000;
    return;
  }

  if (strncmp(topic, CHANNEL_TOPIC, strlen(CHANNEL_TOPIC)) == 0) {
    memset(discordUsername, 0, sizeof discordUsername);
    memset(discordChannel, 0, sizeof discordChannel);
    memset(discordMessage, 0, sizeof discordMessage);

    // I hate string processing in C so much.
    char suffix[100];
    strcpy(suffix, topic + strlen(CHANNEL_TOPIC) + 1);
    char *sep = strchr(suffix, '/');
    if ((sep-suffix) < 0) return; // no username part (e.g. "nh/discord/rx/general")
    strncpy(discordChannel, suffix, sep-suffix);
    strcpy(discordUsername, sep + 1);

    if (length > 4095) length = 4095;
    strncpy(discordMessage, (const char*)payload, length);

    return;
  }

  if (strncmp(topic, DOORBELL_TOPIC, strlen(DOORBELL_TOPIC)) == 0) {
    strncpy(doorbell, (const char*)payload, length);
    return;
  }

  if (strncmp(topic, TEMPERATURE_TOPIC, strlen(TEMPERATURE_TOPIC)) == 0) {
    strcpy(temperatureArea, topic + strlen(TEMPERATURE_TOPIC) + 1);
    strncpy(temperatureValue, (const char*)payload, length);
    return;
  }
}

void drawDoorbell() {
  buffer.fillScreen(0x0000);
  buffer.setCursor(0, 21);
  buffer.setTextSize(3);
  buffer.print(doorbell);

  buffer.setTextSize(1);
  buffer.setCursor(64, 28);
  buffer.print("DOORBELL");

  // Prevent a redraw for 30 seconds or so
  redrawTimer = millis() + 30e3;
  doorbell[0] = '\0';
}

void drawTemperature() {
  buffer.fillScreen(0x0000);
  buffer.setCursor(0, 21);
  buffer.setTextSize(3);

  String value = String(temperatureValue);
  value += "c";

  buffer.print(value);

  buffer.setTextSize(1);
  buffer.setCursor(64, 28);
  buffer.print(temperatureArea);

  redrawTimer = millis() + 3000;
  temperatureArea[0] = '\0';
}

void drawDiscord() {
  buffer.fillScreen(0x0000);
  buffer.setCursor(0, 7 - lineOffset);
  buffer.setTextWrap(true);

  if (strlen(discordMessage) == 0) {
    buffer.print("No messages yet... :'(");
    return;
  }

  char message[2048];
  sprintf(message, "<%s>  %s", discordUsername, discordMessage);

  String _message = String(message);
  _message.replace("😄", ":D");
  _message.replace("🙂", ":)");
  _message.replace("😭", ":'(");
  _message.replace("😢", ":'(");
  _message.replace("\n\n", "\n");

  noInterrupts();
  buffer.print(_message.c_str());

  // We want to check if line scrolling is needed, so we do a "quick" check
  bool moreLines = false;
  for (int x = 0; x < WIDTH && !moreLines; x++) {
    if (buffer.getPixel(x, 25)) moreLines = true;
  }

  if (moreLines || (lineOffset % 7) != 0) {
    if ((lineOffset % 7) == 0)
      redrawTimer = millis() + 2500;
    else
      redrawTimer = millis() + 35;
    lineOffset++;
  } else {
    if (lineOffset > 0) redrawTimer = millis() + 4000;
    lineOffset = 0;
  }

  buffer.fillRect(0, 21, WIDTH, 7, 0x0000);

  buffer.setCursor(64, 28);
  buffer.print(discordChannel);
  interrupts();
}

void drawDepartureBoard() {
  buffer.setTextWrap(false);

  if (! updateTimeOnly) {
    buffer.fillScreen(0x0000);

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, departures);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      redrawTimer = 0;
      updateTimeOnly = false;
      return;
    }

    if (doc.size() == 0) {
      redrawTimer = 0;
      updateTimeOnly = false;
      return;
    }

    for (int d = 0; d < doc.size(); d++) {
      JsonObject dep = doc[d].as<JsonObject>();

      buffer.setCursor(0, 7 * (d + 1));
      buffer.print((const char*)dep["std"]);

      buffer.setCursor(30, 7 * (d + 1));
      buffer.print((const char*)dep["destination"]);

      buffer.fillRect(142, 7 * d, 50, 7, 0x0000);
      buffer.setCursor(145, 7 * (d + 1));
      buffer.print((const char*)dep["platform"]);

      buffer.setCursor(160, 7 * (d + 1));
      buffer.print((const char*)dep["etd"]);
    }

    updateTimeOnly = true;
  }

  RTCTime currentTime;
  RTC.getTime(currentTime);

  buffer.fillRect(0, 21, WIDTH, 7, 0x0000);
  buffer.setCursor(64+10, 28);
  buffer.print(String(currentTime).c_str() + 11);

  redrawTimer = millis() + 1000;
  if ((clearTimer > 0 && clearTimer < millis())) {
    clearTimer = 0;
    departures[0] = '\0';
    updateTimeOnly = false;
  }
}

unsigned long poll = 0;
unsigned long ntpRefresh = 0;
void loop() {
  unsigned long m = millis();
  if (poll < m) {
    checkWiFi();
    checkMqtt();
    mqtt.loop();
    poll = m + 500;
  }

  if (ntpRefresh < m) {
    timeClient.update();
    auto unixTime = timeClient.getEpochTime();
    RTCTime timeToSet = RTCTime(unixTime);
    RTC.setTime(timeToSet);
    ntpRefresh = m + 60*10*1000; // 10 minutes
  }

  if (redrawTimer < m) {
    // This can be overridden by a draw function, e.g. for scrolling text
    redrawTimer = m + 20e3;

    if (strlen(doorbell) > 0) {
      drawDoorbell();
    } else if (strlen(departures) > 0) {
      drawDepartureBoard();
    } else if (strlen(temperatureArea) > 0) {
      drawTemperature();
    } else { // default to last discord message
      drawDiscord();
    }

#ifdef DEBUG_HOME
    drawBufferASCII();
#endif
  }
}
