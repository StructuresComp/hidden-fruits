#include <Arduino.h>
#include "lepton.h"
#include <Preferences.h>
Preferences prefs;

uint8_t frame_8bit[160 * 120];  // move to global (top of file)


const int kPinLedR = 0;  // overlaps with strapping pin

const int kPinI2cScl = 22;
const int kPinI2cSda = 21;

const int kPinLepRst = 26;
const int kPinLepPwrdn = 27;
const int kPinLepCs = 15;
const int kPinLepVsync = 25;
const int kPinLepSck = 14;
const int kPinLepMosi = 13;
const int kPinLepMiso = 12;
size_t width = 160, height = 120;
SPIClass spi(HSPI);
TwoWire i2c(0);

FlirLepton lepton(i2c, spi, kPinLepCs, kPinLepRst, kPinLepPwrdn);
uint8_t vospiBuf[160*120*3] = {0};  

String serialBuffer = "";

void setup() {
  //Serial.begin(921600);
  Serial.begin(921600);


  // wait for post-flash reset
  pinMode(kPinLedR, OUTPUT);
  digitalWrite(kPinLedR, LOW);
  delay(2000);
  digitalWrite(kPinLedR, HIGH);

  spi.begin(kPinLepSck, kPinLepMiso, -1, -1);
  i2c.begin(kPinI2cSda, kPinI2cScl, 400000);

  Serial.println("Lepton start");
  pinMode(kPinLepVsync, INPUT);
  assert(lepton.begin());

  while (!lepton.isReady()) {
    delay(1);
  }
  Serial.println("Lepton ready");

  Serial.print("Lepton Serial = ");
  Serial.print(lepton.getFlirSerial());
  Serial.println("");

  Serial.print("Lepton Part Number = ");
  Serial.print(lepton.getFlirPartNum());
  Serial.println("");

  assert(lepton.enableVsync());
  
  while (!digitalRead(kPinLepVsync));  // seems necessary
}

void loop() {
  bool readResult = lepton.readVoSpi(sizeof(vospiBuf), vospiBuf);
  if (readResult) {
    digitalWrite(kPinLedR, !digitalRead(kPinLedR));
    Serial.write(0xFF);
    Serial.write(0xD8);
    
    uint16_t min = 65535, max = 0;

    // Find min and max
    for (size_t i = 0; i < 160 * 120; i++) {
      uint16_t pixel = ((uint16_t)vospiBuf[2*i] << 8) | vospiBuf[2*i + 1];
      if (pixel < min) min = pixel;
      if (pixel > max) max = pixel;
    }

    // Normalize to 8-bit
    for (size_t i = 0; i < 160 * 120; i++) {
      uint16_t pixel = ((uint16_t)vospiBuf[2*i] << 8) | vospiBuf[2*i + 1];
      frame_8bit[i] = (uint8_t)(((pixel - min) * 255) / (max - min));
    }
    
    Serial.write(frame_8bit, sizeof(frame_8bit));


  
  }

}