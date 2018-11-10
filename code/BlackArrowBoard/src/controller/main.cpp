//
// BlackArrow Controller 
//

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "../common/radioDefines.h"

#define LEDBOARD    13
#define LEDR        6
#define LEDG        9
#define LEDB        10
#define SPEED_INPUT A4
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


int counter = 0;

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LEDBOARD, OUTPUT);
  pinMode(SPEED_INPUT, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1) {
      digitalWrite(LEDR, HIGH);
      delay(1000);
      digitalWrite(LEDR, LOW);
      delay(1000);
    }
  }

  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1) {
      digitalWrite(LEDR, HIGH);
      delay(250);
      digitalWrite(LEDR, LOW);
      delay(250);
      digitalWrite(LEDR, HIGH);
      delay(250);
      digitalWrite(LEDR, LOW);
      delay(1000);
    }
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  digitalWrite(LEDG, LOW);
}

void loop() {
  int val = analogRead(SPEED_INPUT);
  byte valByte = (byte)map(val, 0, 1023, 0, 254);

  analogWrite(LEDB, valByte);

  counter++;
  if (counter > 100) {
    Serial.println(val);
    counter = 0;

    // Send packet
    byte radioSpeedPacket[4];
    radioSpeedPacket[0] = 0x10;
    radioSpeedPacket[1] = valByte;
    radioSpeedPacket[2] = 0x47;
    radioSpeedPacket[3] = 0xFF;
    rf95.send((uint8_t*)radioSpeedPacket, 4);
    rf95.waitPacketSent();
  }

  delay(1);
}