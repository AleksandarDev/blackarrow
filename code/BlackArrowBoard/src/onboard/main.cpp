//
// BlackArrow OnBoard
//

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Servo.h>
#include "../common/radioDefines.h"

#define LEDBOARD        13
#define DIR_CONTROL     6
#define SPEED_CONTROL   9

#define SPEED_RX_MIN          0
#define SPEED_RX_MAX          254
#define SPEED_RX_NEUTRAL_MIN  34    // (254 / 3) - 50 (+-50 deadzone)
#define SPEED_RX_NEUTRAL_MAX  134   // (254 / 3) + 50 (+-50 deadzone)

#define ESC_MAX         2000
#define ESC_NEUTRAL     1500
#define ESC_MIN         1000

#define FRAME_WAIT_MS   10
#define FRAME_DROP_MS   1000

Servo esc;
byte requestedSpeed;
byte currentSpeed = 0;
byte missedFramesCount = 0;

void frameReceived();
void frameMissed();
void setSpeed(byte val);
void clearSpeed();

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LEDBOARD, OUTPUT);
  pinMode(DIR_CONTROL, OUTPUT);
  esc.attach(SPEED_CONTROL);

  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(DIR_CONTROL, LOW);
  esc.writeMicroseconds(ESC_NEUTRAL);

  // Reset ESC
  clearSpeed();

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop() {

  byte radioSpeedPacketBuff[4];
  byte len = sizeof(radioSpeedPacketBuff);
  if (rf95.waitAvailableTimeout(FRAME_WAIT_MS)) {
    // Should be a reply message for us now   
    if (rf95.recv(radioSpeedPacketBuff, &len))
    {
      if (radioSpeedPacketBuff[0] != 0x10 &&
          radioSpeedPacketBuff[2] != 0x47 &&
          radioSpeedPacketBuff[3] != 0xFF) {

        // Signal frame was missed (invalid)
        frameMissed();
        Serial.println("Got reply with invalid frame.");
      } else {
        // Signal frame was received
        frameReceived();

        // Set new requested speed
        requestedSpeed = radioSpeedPacketBuff[1];

        // Debug print
        Serial.print("Got speed command: "); 
        Serial.print(requestedSpeed); 
        Serial.print("\t"); 
        Serial.print("RSSI: "); 
        Serial.println(rf95.lastRssi(), DEC);
      }
    }
  } else {
    // Signal frame was missed (not available)
    frameMissed();
  }

  // Update speed
  if (currentSpeed != requestedSpeed) {
    currentSpeed = requestedSpeed;
    setSpeed(currentSpeed);
  }
}

void frameReceived() {
  missedFramesCount = 0;
}

void frameMissed() {
  missedFramesCount++;
  if (missedFramesCount * FRAME_WAIT_MS > FRAME_DROP_MS) {
    missedFramesCount = 0;
    clearSpeed(); 

    Serial.println("Too many frames missed. Cleared speed.");
  }
}

void clearSpeed() {
  requestedSpeed = currentSpeed = SPEED_RX_MIN + (SPEED_RX_MAX - SPEED_RX_MIN) / 2; // Set to neutral
  setSpeed(requestedSpeed);
}

void setSpeed(byte val) {
  int out;
  if (val < SPEED_RX_NEUTRAL_MIN) {
    //out = map(val, SPEED_RX_MIN, SPEED_RX_NEUTRAL_MIN, ESC_MIN, ESC_NEUTRAL);
    out = ESC_MIN;
  } else if (val > SPEED_RX_NEUTRAL_MAX) {
    out = map(val, SPEED_RX_NEUTRAL_MAX, SPEED_RX_MAX, ESC_NEUTRAL, ESC_MAX);

    // Ease (x^3)
    float diff = ESC_MAX - ESC_NEUTRAL;
    float currDiff = out - ESC_NEUTRAL;
    float perc = currDiff / diff;
    float percEased = perc * perc * perc;
    out = (int)(percEased * diff) + ESC_NEUTRAL;
  } else {
    // Set neutral
    out = ESC_NEUTRAL;
  } 

  analogWrite(LEDBOARD, map(val, ESC_MIN, ESC_MAX, 0, 255));
  esc.writeMicroseconds(out);

  // Debug 
  Serial.print("ESC: ");
  Serial.println(out);
}