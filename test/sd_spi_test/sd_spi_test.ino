/**
 * sd_spi_test.ino
 *
 * SPI-mode sanity check for the Adafruit 4682 microSD breakout on the
 * Feather RP2350. Uses the Feather's default SPI pins at a conservative
 * 4 MHz, writes test.txt, reads it back, and lists the card.
 *
 * Wiring (breakout -> Feather):
 *   3V        -> 3.3V
 *   GND       -> GND
 *   CLK       -> SCK  (GPIO22)
 *   SI  (CMD) -> MO   (GPIO23)
 *   SO  (D0)  -> MI   (GPIO20)
 *   CS  (D3)  -> D25  (GPIO25)
 *   D1, DAT2  -> leave unconnected
 */

#include <SdFat.h>

#define SD_CS_PIN   25
#define SD_CONFIG   SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4))

SdFs sd;
FsFile file;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Serial.println("SD SPI test (4 MHz)");
  delay(250);

  if (!sd.begin(SD_CONFIG)) {
    uint8_t code = sd.sdErrorCode();
    Serial.printf("Mount FAILED, code 0x%02X (", code);
    printSdErrorSymbol(&Serial, code);
    Serial.printf("), data 0x%02X\n", sd.sdErrorData());
    return;
  }
  Serial.printf("Mount OK, FAT type %d, %.1f GB\n",
                sd.fatType(), sd.card()->sectorCount() * 512e-9);

  // Write test.txt
  if (!file.open("test.txt", O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("ERROR: could not open test.txt for writing");
    return;
  }
  file.println("hello, world");
  file.close();
  Serial.println("Wrote test.txt");

  // Read it back
  if (!file.open("test.txt", O_RDONLY)) {
    Serial.println("ERROR: could not open test.txt for reading");
    return;
  }
  Serial.print("Read back: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  Serial.println("Root directory:");
  sd.ls(LS_SIZE);
  Serial.println("Done.");
}

void loop() {
}
