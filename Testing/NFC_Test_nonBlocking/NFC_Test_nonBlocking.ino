//https://github.com/Udinic/smartShadeController/blob/master/smartShadeController.ino
//https://github.com/adafruit/Adafruit-PN532
#include <Adafruit_PN532.h>
// PIN definitions
#define PN532_IRQ   (4)
#define PN532_RESET (5)

// Config
const boolean NFC_DISABLED = false;

// State
bool listeningToNFC = false;
uint8_t irqCurr;
uint8_t irqPrev;

// Init the object that controls the PN532 NFC chip
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


void startListeningToNFC() {
  if (NFC_DISABLED) {
    return;
  }
  listeningToNFC = true;
  irqCurr = digitalRead(PN532_IRQ);
  irqPrev = irqCurr;

  Serial.println("START listening to NFC tags..");
  nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
}

void stopListeningToNFC() {
  if (NFC_DISABLED) {
    return;
  }
  listeningToNFC = false;
  Serial.println("STOP listening to NFC tags..");
  digitalWrite(PN532_RESET, HIGH);

}


uint32_t getCardId(uint8_t uid[], uint8_t uidLength) {
  if (uidLength == 4)
  {
    // We probably have a Mifare Classic card ...
    uint32_t cardid = uid[0];
    cardid <<= 8;
    cardid |= uid[1];
    cardid <<= 8;
    cardid |= uid[2];
    cardid <<= 8;
    cardid |= uid[3];
    return cardid;
  } else {
    return -1;
  }
}

void printCardInfo(uint8_t uid[], uint8_t uidLength) {
  Serial.println("***********************");
  Serial.println("Found an ISO14443A card !!");
  Serial.print("  UID Length: "); Serial.print(uidLength, DEC); Serial.println(" bytes");
  Serial.print("  UID Value: ");
  nfc.PrintHex(uid, uidLength);

  if (uidLength == 4)
  {
    // We probably have a Mifare Classic card ...
    uint32_t cardid = uid[0];
    cardid <<= 8;
    cardid |= uid[1];
    cardid <<= 8;
    cardid |= uid[2];
    cardid <<= 8;
    cardid |= uid[3];
    Serial.print("Seems to be a Mifare Classic card #");
    Serial.println(cardid);
  }
  Serial.println("");
  Serial.println("***********************");
}



void handleNFCDetected() {
  Serial.println("**********");
  Serial.println("Got NFC IRQ");
  Serial.println("**********");

  uint8_t success = false;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // read the NFC tag's info
  success = nfc.readDetectedPassiveTargetID(uid, &uidLength);
  Serial.println(success ? "Read successful" : "Read failed (not a card?)");

  if (success) {
    uint32_t cardId = getCardId(uid, uidLength);
    Serial.print("Found card : ");
    Serial.println(cardId );
  }

  if (listeningToNFC) {
    delay(500);
    //      Serial.println("Start listening for cards again");
    startListeningToNFC();
  }
}




void setup() {
  Serial.begin(115200);
  while (!Serial);


  if (!NFC_DISABLED) {
    Serial.println("Initializing NFC chip...");
    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
      Serial.print("Didn't find PN53x board");
      while (1); // halt
    }

    // configure board to read RFID tags
    nfc.SAMConfig();

    // Setting the NFC IRQ pin.
    pinMode(PN532_IRQ, INPUT_PULLUP);
  }

  Serial.println("Started listening to input..");

  startListeningToNFC();
}

void loop() {
  //Serial.println(".");
  delay(500);

  
  irqCurr = digitalRead(PN532_IRQ);

  if (listeningToNFC && irqCurr == LOW && irqPrev == HIGH) {
    handleNFCDetected();
  } else if (irqCurr == LOW && irqPrev == HIGH) {
    Serial.println("##### Got IRQ while not listening..");
  }

  irqPrev = irqCurr;

}
