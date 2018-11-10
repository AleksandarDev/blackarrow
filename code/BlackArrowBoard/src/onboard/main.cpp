//
// BlackArrow OnBoard
//

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "../common/radioDefines.h"

#define LEDBOARD        13
#define DIR_CONTROL     6
#define SPEED_CONTROL   9

void setSpeed(byte val);

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LEDBOARD, OUTPUT);
  pinMode(DIR_CONTROL, OUTPUT);
  pinMode(SPEED_CONTROL, OUTPUT);

  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(DIR_CONTROL, LOW);
  digitalWrite(SPEED_CONTROL, LOW);

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
  if (rf95.waitAvailableTimeout(1000)) {
    // Should be a reply message for us now   
    if (rf95.recv(radioSpeedPacketBuff, &len))
    {
      if (radioSpeedPacketBuff[0] != 0x10 &&
          radioSpeedPacketBuff[2] != 0x47 &&
          radioSpeedPacketBuff[3] != 0xFF) {
        Serial.println("Got reply with invalid frame.");
      } else {
        byte speedVal = radioSpeedPacketBuff[1];
        Serial.print("Got speed command: "); Serial.print(speedVal); Serial.print("\t"); Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC);

        setSpeed(speedVal);
      }
    }
  } else {
    setSpeed(0);
  }

  delay(1);
}

void setSpeed(byte val) {
  analogWrite(LEDBOARD, val);
  analogWrite(SPEED_CONTROL, val);
}