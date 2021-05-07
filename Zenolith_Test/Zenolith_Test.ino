
#include <SPI.h>
#include <Wire.h>

//datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <SD.h>
const int chipSelect = 10; //m4


//display/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
#define BUTTON_A  9
#define BUTTON_B  6
//#define BUTTON_C  5

//motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
//https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/arduino-usage
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m3 = AFMS.getMotor(3);

boolean testMotors = false;

//NFC/////////////////////////////////////////////////////////////////////////////////////////////////////////
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
uint32_t cardId;

// Init the object that controls the PN532 NFC chip
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

//TTS/////////////////////////////////////////////////////////////////////////////////////////////////////////
//setup serial2
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
Uart Serial2 (&sercom0, A5, A4, SERCOM_RX_PAD_2, UART_TX_PAD_0);

void SERCOM0_0_Handler()
{
  Serial2.IrqHandler();
}
//
void SERCOM0_2_Handler()
{
  Serial2.IrqHandler();
}

//RTC/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RTClib.h"
RTC_DS3231 rtc;

//IMU/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long lastXUpdate;
#define BNO055_SAMPLERATE_DELAY_MS (100)
sensors_event_t event;

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  delay(3000);
  Serial.println("Hello");
  delay(50);

  //datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");


  //motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
  AFMS.begin();  // create with the default frequency 1.6KHz
  delay(50);

  //RTC/////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  delay(50);

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));



  //display/////////////////////////////////////////////////////////////////////////////////////////////////////////


  //Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  //Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(500);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Hello there.");
  display.println();
  display.print("I am Zenolith.");
  display.println();

  display.write("Press Button A to begin Motor Test");
  display.println();


  DateTime now = rtc.now();
  display.print(now.year(), DEC);
  display.print('/');
  display.print(now.month(), DEC);
  display.print('/');
  display.print(now.day(), DEC);
  display.print(" - ");

  display.print(now.hour(), DEC);
  display.print(':');
  display.print(now.minute(), DEC);
  display.print(':');
  display.print(now.second(), DEC);
  display.println();


  display.display(); // actually display all of the above
  delay(50);


  //TTS/////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial2.begin(9600);
  pinPeripheral(A5, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  //delay(3000);
  Serial2.write("W180\n");//Wx Set speaking rate (words/minute): x = 75 to 600
  delay(50);
  Serial2.write("V10\n");//Vx Set audio volume (dB): x = -48 to 18
  delay(50);
  Serial2.write("SHELLO there. I am zenolith. Press Button A. To begin Motor Test\n");





  //IMU/////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);


  //NFC/////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  //motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (testMotors) {
    motorTest(m1);
    motorTest(m2);
    motorTest(m3);
  }

  //NFC/////////////////////////////////////////////////////////////////////////////////////////////////////////


  irqCurr = digitalRead(PN532_IRQ);

  if (listeningToNFC && irqCurr == LOW && irqPrev == HIGH) {
    handleNFCDetected();
  } else if (irqCurr == LOW && irqPrev == HIGH) {
    Serial.println("##### Got IRQ while not listening..");
  }

  irqPrev = irqCurr;



  //display/////////////////////////////////////////////////////////////////////////////////////////////////////////
  display.clearDisplay();
  display.setCursor(0, 0);

  if (!digitalRead(BUTTON_A)) {
    display.print("Testing Motors");
    Serial2.write("STesting Motors\n");
    //testMotors = true;
    delay(100);
  }

  if (!digitalRead(BUTTON_B)) {
    display.print("Button B");
    //Serial2.write("SB\n");

    delay(100);
  }

  //yield();

  //IMU/////////////////////////////////////////////////////////////////////////////////////////////////////////

  if ((millis() - lastXUpdate) > BNO055_SAMPLERATE_DELAY_MS) {
    bno.getEvent(&event);
    lastXUpdate = millis();
  }

  /* Display the floating point data */
  display.print("X: ");
  display.println(event.orientation.x, 4);
  display.print("Y: ");
  display.println(event.orientation.y, 4);
  display.print("Z: ");
  display.println(event.orientation.z, 4);

  display.print("UUID: ");
  display.println(cardId);

  display.display();

  //datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////
  logData();
}





//motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
void motorTest(Adafruit_DCMotor *m) {
  uint8_t i;
  int d = 20; //lower values here stalls motor

  m->run(FORWARD);
  for (i = 0; i < 255; i++) {
    m->setSpeed(i);
    delay(d);
  }
  delay(500);
  for (i = 255; i != 0; i--) {
    m->setSpeed(i);
    delay(d);
  }

  m->run(BACKWARD);
  for (i = 0; i < 255; i++) {
    m->setSpeed(i);
    delay(d);
  }
  delay(500);
  for (i = 255; i != 0; i--) {
    m->setSpeed(i);
    delay(d);
  }

  m->run(RELEASE);
  delay(1000);
}


//NFC/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    cardId = getCardId(uid, uidLength);
    Serial.print("Found card : ");
    Serial.println(cardId );
  }

  if (listeningToNFC) {
    delay(500);
    //      Serial.println("Start listening for cards again");
    startListeningToNFC();
  }
}


//datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////
void logData() {
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  dataString += event.orientation.x;
  dataString += ",";
  dataString += event.orientation.y;
  dataString += ",";
  dataString += event.orientation.z;
  dataString += ",";
  dataString += cardId;



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
