#include <Wire.h>
#include <SPI.h>

float margin = 5;

//battery
#define VBATPIN A6
float measuredvbat;
boolean chargeMode = false;

//display/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
//#include <Fonts/Picopixel.h>
//#include <Fonts/Org_01.h>
//#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/TomThumb.h>

Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
#define BUTTON_A  9
#define BUTTON_B  6
//#define BUTTON_C  5


//json  /////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ArduinoJson.h>
#include <SD.h>
const int chipSelect = 10; //m4
int id = 0;

//struct Config {
//  char UUID[64];
//  float xSetpoint;
//  float ySetpoint;
//  float zSetpoint;
//
//};
const char *filename = "UUID.txt";  // <- SD library uses 8.3 filenames
const char *settings = "settings.txt";  // <- SD library uses 8.3 filenames

//Config config;                         // <- global configuration object

// PID/////////////////////////////////////////////////////////////////////////////////////////////////////////
//https://github.com/br3ttb/Arduino-PID-Library
#include <PID_v1.h>

//Define Variables we'll be connecting to
double x, xSetpoint, xInput, xOutput;
double y, ySetpoint, yInput, yOutput;
double z, zSetpoint, zInput, zOutput;

float  xSetpointFuture;
float  ySetpointFuture;
float  zSetpointFuture;

//Specify the links and initial tuning parameters
/*
  1.Set all gains to zero.
  2.Increase the P gain until the response to a disturbance is steady oscillation.
  3.Increase the D gain until the the oscillations go away (i.e. it's critically damped).
  4.Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
  5.Set P and D to the last stable values.
  6.Increase the I gain until it brings you to the setpoint with the number of oscillations desired
  (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot)
*/

//float Kp = 5.6;
//float Ki = 0.0;
//float Kd = 0.26;
//
//float Kp = 5.5;
//float Ki = 0.0;
//float Kd = 0.3;


//float Kp = 8.1;
//float Ki = .31;
//float Kd = 1.2;

//float Kp = 9.0;
//float Ki = .7;
//float Kd = 2.7;


////single axis
//float Kp = 14.90;
//float Ki = 0.2;
//float Kd = 4.3;

//all axis + 4/6 shells
//float Kp = 15.70;
//float Ki = 0.2;
//float Kd = 4.9;

//single axis half shell
float Kp = 9.20;
float Ki = 0.1;
float Kd = 4.0;



PID xPID(&xInput, &xOutput, &xSetpoint, Kp, Ki, Kd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, Kp, Ki, Kd, REVERSE);
PID zPID(&zInput, &zOutput, &zSetpoint, Kp, Ki, Kd, REVERSE);


//motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
//https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/arduino-usage
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m4 = AFMS.getMotor(4);

boolean enableMotors = false;
boolean enableMotorX = true;
boolean enableMotorY = true;
boolean enableMotorZ = true;

boolean enableParallelSetpoints = false;
int motorSequence = 0;

void runMotorX() {
  if (enableMotorX) {
    if (xOutput < 0)  m1->run(BACKWARD);
    else  m1->run(FORWARD);
    m1->setSpeed( abs(xOutput) );
  }
  else {
    m1->setSpeed(0);
  }
}


void runMotorY() {
  if (enableMotorY) {
    if (yOutput < 0)  m2->run(BACKWARD);
    else  m2->run(FORWARD);
    m2->setSpeed( abs(yOutput) );

  } else {
    m2->setSpeed(0);
  }
}

void runMotorZ() {
  if (enableMotorZ) {
    if (zOutput < 0)  m4->run(BACKWARD);
    else  m4->run(FORWARD);
    m4->setSpeed( abs(zOutput) );
  } else {
    m4->setSpeed(0);
  }
}


//TTS/////////////////////////////////////////////////////////////////////////////////////////////////////////
//setup serial2
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
Uart Serial2 (&sercom0, A5, A4, SERCOM_RX_PAD_2, UART_TX_PAD_0);
long lastTalk;
int volume = 3;

void SERCOM0_0_Handler()
{
  Serial2.IrqHandler();
}
//
void SERCOM0_2_Handler()
{
  Serial2.IrqHandler();
}

void setTTSVolume() {

  if (volume == 3)  Serial2.write("V18\n"); //Vx Set audio volume (dB): x = -48 to 18
  else if (volume == 2)  Serial2.write("V10\n"); //Vx Set audio volume (dB): x = -48 to 18
  else if (volume == 1)  Serial2.write("V0\n"); //Vx Set audio volume (dB): x = -48 to 18
  else if (volume == 0)  Serial2.write("V-10\n"); //Vx Set audio volume (dB): x = -48 to 18

  //  char c[8];
  //  sprintf(c, "V%\n", volume);//add + to positive numbers, leading zeros

  // Serial2.write(c);//Vx Set audio volume (dB): x = -48 to 18

  Serial.println("Volume: ");
  Serial.println(volume);



}

//IMU  /////////////////////////////////////////////////////////////////////////////////////////////////////////

//ranges
//YAW X 0 360
//PITCH X 0 -90 0 90 0
//RoLL X -180 0 180

float XOffset = 0;
float YOffset = 0;
float ZOffset = 0;

float lastX = 0;
float lastY = 0;
float lastZ = 0;





//json  /////////////////////////////////////////////////////////////////////////////////////////////////////////

void loadConfiguration(const char *filename, String UUID) {
  // Open file for reading
  File file = SD.open(filename);
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<20240> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println(F("Failed to read file, using default configuration"));
    display.print("Failed to read file");
    display.println();
    display.display(); // actually display all of the above
    delay(50);
  }
  id = doc[UUID]["id"] | -1;

  volume += doc[UUID]["volume"] | 0;
  if (volume < 0)volume = 0;
  else if (volume > 3)volume = 3;


  margin += doc[UUID]["margin"] | 0;



  setTTSVolume();

  const char *say = doc[UUID]["say"] | "error";

  //if it's a setpoint card enable motors

  if (id >= 0) {
    char c[20];
    sprintf(c, "S%d", id);
    Serial2.write(c); //say ID
    Serial2.write("\n");

    enableMotors = true;
    motorSequence = 0;

  } else {
    Serial2.write("S");
    Serial2.write(say); //say
    Serial2.write("\n");
  }



  //  if (strcmp(say, "error") == 1) {
  //    file.close();
  //    return;
  //  }

  xSetpointFuture = doc[UUID]["xSetpoint"] | xSetpoint;
  ySetpointFuture = doc[UUID]["ySetpoint"] | ySetpoint;
  zSetpointFuture = doc[UUID]["zSetpoint"] | zSetpoint;

  Kp = doc[UUID]["P"] | Kp;
  Ki = doc[UUID]["I"] | Ki;
  Kd = doc[UUID]["D"] | Kd;

  float pd = doc[UUID]["Pdelta"] | 0.0;
  float id = doc[UUID]["Idelta"] | 0.0;
  float dd = doc[UUID]["Ddelta"] | 0.0;

  //Serial.println(pd);

  Kp = Kp + pd;
  Ki = Ki + id;
  Kd = Kd + dd;

  xPID.SetTunings( Kp,  Ki,  Kd);
  yPID.SetTunings( Kp,  Ki,  Kd);
  zPID.SetTunings( Kp,  Ki,  Kd);

  //Serial.println(config.xSetpoint );
  enableMotors = doc[UUID]["enableMotors"] | int(enableMotors);

  enableMotorX = doc[UUID]["enableMotorX"] | int(enableMotorX);
  enableMotorY = doc[UUID]["enableMotorY"] | int(enableMotorY);
  enableMotorZ = doc[UUID]["enableMotorZ"] | int(enableMotorZ);


  boolean enableAllMotors = doc[UUID]["enableAllMotors"] | 0 ;
  if (enableAllMotors) {
    enableMotors = true;
    enableMotorX = true;
    enableMotorY = true;
    enableMotorZ = true;
  }

  enableParallelSetpoints = doc[UUID]["enableParallelSetpoints"] | int(enableParallelSetpoints);

  if (!enableParallelSetpoints) {
    motorSequence = 0;
  }

  boolean resetZenolith = doc[UUID]["reset"] | 0;
  if (resetZenolith) {


    XOffset = -x + XOffset;
    YOffset = -y + YOffset;
    ZOffset = -z + ZOffset;
    lastX = -x + XOffset;
    lastY = -y + YOffset;
    lastZ = -z + ZOffset;



    motorSequence = 0;

    enableMotors = false;
    enableMotorX = true;
    enableMotorY = true;
    enableMotorZ = true;

    loadSettings(settings);


  }


  boolean save = doc[UUID]["save"] | 0;
  if (save) saveSettings(settings);




  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}


void loadSettings(const char *filename) {
  // Open file for reading
  File file = SD.open(filename);
  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.println(F("Failed to read settings"));
    display.print("Failed to read settings");
    display.println();
    display.display(); // actually display all of the above
    delay(50);
  }

  Kp = doc["Kp"] | Kp;
  Ki = doc["Ki"] | Ki;
  Kd = doc["Kd"] | Kd;
  volume = doc["volume"] | volume;
  margin = doc["margin"] | margin;

  setTTSVolume();

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}


// Saves the configuration to a file
void saveSettings(const char *filename) {
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove(filename);

  // Open file for writing
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    display.print("Failed to create file");
    display.println();
    display.display(); // actually display all of the above
    delay(50);

    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Set the values in the document
  doc["Kp"] = Kp;
  doc["Ki"] = Ki;
  doc["Kd"] = Kd;
  doc["volume"] = volume;
  doc["margin"] = margin;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    display.print("Failed to write to  file");
    display.println();
    display.display(); // actually display all of the above
    delay(50);
  }

  // Close the file
  file.close();
}




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

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET); // Init the object that controls the PN532 NFC chip

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

    loadConfiguration(filename, String(cardId));

    //Serial2.write("SBEEP\n");
  }

  if (listeningToNFC) {
    delay(1000);
    //Serial.println("Start listening for cards again");
    startListeningToNFC();
  }
}



//IMU/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define BNO055_SAMPLERATE_DELAY_MS (10) //10-1000

unsigned long lastUpdate;

boolean gyroCalibrated = false;


void updateIMU() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  x = euler.x() + XOffset;
  y = euler.y() + YOffset;
  z = euler.z() + ZOffset;



  // Get absolute X
  float diff = x - lastX;
  if (diff > 180)
  {
    XOffset -= 360;
  }
  else if (diff < -180)
  {
    XOffset += 360;
  }

  //  diff = y - lastY;
  //  if (diff > 180)
  //  {
  //    YOffset -= 360;
  //  }
  //  else if (diff < -180)
  //  {
  //    YOffset += 360;
  //  }
  //

  diff = z - lastZ;
  if (diff > 180)
  {
    ZOffset -= 360;
  }
  else if (diff < -180)
  {
    ZOffset += 360;
  }


  x = euler.x() + XOffset;
  y = euler.y() + YOffset;
  z = euler.z() + ZOffset;

  lastX = x;
  lastY = y;
  lastZ = z;
}



int gyroStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* Display the individual values */
  //  Serial.print(" G:");
  //  Serial.println(gyro, DEC);
  return gyro;
}




//datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////
//#include <SD.h>
//const int chipSelect = 10; //m4

void logData() {
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  //  dataString += event.orientation.x;
  //  dataString += ",";
  //  dataString += event.orientation.y;
  //  dataString += ",";
  //  dataString += event.orientation.z;
  //  dataString += ",";

  dataString += id;
  dataString += ",";
  dataString += cardId;
  dataString += ",";
  dataString += gyroStatus();
  dataString += ",";

  dataString += x;
  dataString += ",";
  dataString += y;
  dataString += ",";
  dataString +=  z;
  dataString += ",";

  dataString += xSetpoint;
  dataString += ",";
  dataString += ySetpoint;
  dataString += ",";
  dataString += zSetpoint;
  dataString += ",";


  dataString += Kp;
  dataString += ",";
  dataString += Ki;
  dataString += ",";
  dataString += Kd;





  //  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //
  //  dataString += ",";
  //  dataString += accel.x();
  //  dataString += ",";
  //  dataString += accel.y();
  //  dataString += ",";
  //  dataString += accel.z();




  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}




//RTC/////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RTClib.h"
//RTC_DS3231 rtc;
RTC_PCF8523 rtc;







/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP /////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
  delay(5000);
  Serial.println("Hello");

  //display/////////////////////////////////////////////////////////////////////////////////////////////////////////


  //Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //  display.display();
  //  delay(500);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("ZENOLITH V4.0");
  display.println();

  //  display.write("Press Button A to begin Motor Test");
  //  display.println();


  DateTime now = rtc.now();
  display.print(now.year(), DEC);
  display.print('/');
  display.print(now.month(), DEC);
  display.print('/');
  display.print(now.day(), DEC);
  display.print("  ");
  display.print(now.hour(), DEC);
  display.print(':');
  display.print(now.minute(), DEC);
  display.print(':');
  display.print(now.second(), DEC);
  display.println();

  display.display(); // actually display all of the above
  delay(50);


  //IMU/////////////////////////////////////////////////////////////////////////////////////////////////////////
  /* Initialise the sensor */
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))
    //if (!bno.begin())

  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    display.print("no BNO055 detected");
    display.println();

    display.display(); // actually display all of the above
    delay(50);

    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);



  //gyroStatus();


  //datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    display.print("Card failed, or not present");
    display.println();
    display.display(); // actually display all of the above
    delay(50);
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");




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

  //rtc.adjust(DateTime(2020, 5, 14, 3, 36, 0));
  rtc.start();

  delay(50);





  //TTS/////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial2.begin(9600);
  pinPeripheral(A5, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  //delay(3000);
  Serial2.write("W180\n");//Wx Set speaking rate (words/minute): x = 75 to 600
  delay(50);
  //  Serial2.write("V");//Vx Set audio volume (dB): x = -48 to 18
  //  Serial2.write(volume);//Vx Set audio volume (dB): x = -48 to 18
  //  Serial2.write("\n");//Vx Set audio volume (dB): x = -48 to 18
  setTTSVolume();
  delay(50);
  //Serial2.write("SI am zenolith.\n");




  //NFC/////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (!NFC_DISABLED) {
    Serial.println("Initializing NFC chip...");
    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
      Serial.print("Didn't find PN53x board");
      display.print("Didn't find PN53x board");
      display.println();
      display.display(); // actually display all of the above
      delay(50);

      while (1); // halt
    }

    // configure board to read RFID tags
    nfc.SAMConfig();

    // Setting the NFC IRQ pin.
    pinMode(PN532_IRQ, INPUT_PULLUP);
  }

  Serial.println("Started listening to input..");

  startListeningToNFC();

  //motor/////////////////////////////////////////////////////////////////////////////////////////////////////////
  AFMS.begin();  // create with the default frequency 1.6KHz
  delay(50);
  Serial.println("motors on");


  //PID /////////////////////////////////////////////////////////////////////////////////////////////////////////
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-255, 255);
  yPID.SetMode(AUTOMATIC);
  yPID.SetOutputLimits(-255, 255);
  zPID.SetMode(AUTOMATIC);
  zPID.SetOutputLimits(-255, 255);
  xSetpoint = 0;
  ySetpoint = 0;
  zSetpoint = 0;


  loadSettings(settings);





}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////LOOP////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  //chargemode/////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (chargeMode) {
    m1->setSpeed(0);
    m2->setSpeed(0);
    m4->setSpeed(0);
    //
    //    delay(20);

    //battery
    if (millis() % 1000 < 100)
    {
      measuredvbat = analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      //      measuredvbat = measuredvbat - 3.20; //3.2-4.2
      //      measuredvbat = measuredvbat * 100;
    }

    display.setFont(); //normal font
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BATT: " );
    display.print(measuredvbat, 2);
    display.print("V ");


    display.display();


  } else {


    //gyro calibration/////////////////////////////////////////////////////////////////////////////////////////////////////////
    if ( !gyroCalibrated ) {
      if (gyroStatus() == 3 && millis() - lastTalk > 6000) {
        //delay(5000);
        Serial2.write("SReady\n");
        gyroCalibrated = true;
      }
      else if (millis() < 20000 && millis() - lastTalk > 6000) {
        Serial2.write("SCalibrating gyro. Stand still.\n");
        lastTalk = millis();
      }
    }

    //Serial.println(".");

    //PID/////////////////////////////////////////////////////////////////////////////////////////////////////////

    //updateY();
    //updateZ();


    if ((millis() - lastUpdate) > BNO055_SAMPLERATE_DELAY_MS) {
      //update IMU and PID together
      updateIMU();
      xInput = x;
      xPID.Compute();
      yInput = y;
      yPID.Compute();
      zInput = z;
      zPID.Compute();
      lastUpdate = millis();

      //motor/////////////////////////////////////////////////////////////////////////////////////////////////////////

      if (enableMotors) {

        //test tracking
        //if(millis()>10000 && xSetpoint<180) xSetpoint+=2;

        if (enableParallelSetpoints) {
          xSetpoint = xSetpointFuture;
          ySetpoint = ySetpointFuture;
          zSetpoint = zSetpointFuture;

        }
        else {

          //margin = 3; //margin
          if (motorSequence < 10) {
            xSetpoint = xSetpointFuture;

            //check if X is close to setpoint
            if (x >= xSetpoint - margin && x <= xSetpoint + margin ) motorSequence++;
          }
          else if (motorSequence < 20) {
            xSetpoint = xSetpointFuture;
            ySetpoint = ySetpointFuture;

            if (y >= ySetpoint - margin && y <= ySetpoint + margin ) motorSequence++;
          }
          else {
            xSetpoint = xSetpointFuture;
            ySetpoint = ySetpointFuture;
            zSetpoint = zSetpointFuture;

          }
        }

        runMotorX();
        runMotorY();
        runMotorZ();

        //datalogger/////////////////////////////////////////////////////////////////////////////////////////////////////////
        logData();
      }
      else {
        m1->setSpeed(0);
        m2->setSpeed(0);
        m4->setSpeed(0);
      }
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
    display.println();

    display.setFont(); //normal font
    display.print("ID: ");
    display.print(id);
    display.print(": ");

    display.println(cardId);

    //display.setCursor(0, 6);
    display.setTextSize(1);
    //display.setFont(&Picopixel); //tiny font
    //display.setFont(&FreeMono9pt7b); //mono font
    display.setFont(&TomThumb); //mono font

    int xCursor = 0;
    int yCursor = 14;
    int colSpacing = 43;
    int rowSpacing = 7;

    display.setCursor(xCursor, yCursor);

    display.print("IMU Y: ");
    char c[12];
    sprintf(c, " % +06.1f", x);//add + to positive numbers, leading zeros
    display.print(c);

    display.setCursor(xCursor + colSpacing+6 , yCursor );
    display.print(" P: ");
    sprintf(c, " % +06.1f", y);
    display.print(c);

    display.setCursor(xCursor + colSpacing * 2, yCursor);
    display.print(" R: ");
    sprintf(c, " % +06.1f", z);
    display.print(c);

    //display.println();
    yCursor += rowSpacing;
    xCursor = 0;
    display.setCursor(xCursor, yCursor);

    display.print("STP X: ");
    sprintf(c, " % +06.1f", xSetpoint);//add + to positive numbers, leading zeros
    display.print(c);

    display.setCursor(xCursor + colSpacing+6 , yCursor );

    display.print(" Y: ");
    sprintf(c, " % +06.1f", ySetpoint);
    display.print(c);

    display.setCursor(xCursor + colSpacing * 2, yCursor);

    display.print(" Z: ");
    sprintf(c, " % +06.1f", zSetpoint);
    display.print(c);

    //display.println();

    yCursor += rowSpacing;
    xCursor = 0;

    display.setCursor(xCursor, yCursor);
    if (enableMotors) {


      display.print("SPD X: " );
      sprintf(c, " % +04.0f%%", xOutput / 255.0 * 100.0); //add + to positive numbers, leading zeros
      if (enableMotorX) display.print(c);
      else display.print("  XXX%");

    display.setCursor(xCursor + colSpacing+6 , yCursor );
      display.print(" Y: ");
      sprintf(c, " % +04.0f%%", yOutput / 255.0 * 100.0);
      if (enableMotorY) display.print(c);
      else display.print("  XXX%");

    display.setCursor(xCursor + colSpacing * 2, yCursor);
      display.print(" Z: ");
      sprintf(c, " % +04.0f%%", zOutput / 255.0 * 100.0);
      if (enableMotorZ) display.print(c);
      else display.print("  XXX%");

    } else {
      display.print("MOTORS DISABLED" );
    }
    //display.println();


    yCursor += rowSpacing;
    xCursor = 0;
    display.setCursor(xCursor, yCursor);
    int d = 2;

    display.print("P: ");
    display.print(Kp, d);
    display.print( " I: " );
    display.print(Ki, d);
    display.print( " D: " );
    display.print(Kd, d);

    display.println();


    display.print("MARGIN: " );
    display.print(margin, 1);
    display.print("  |  ");
    //display.println();

    //gyro
    display.print( "GYRO: " );
    display.print(gyroStatus());

    display.println();

    //battery
    if (millis() % 10000 < 100)
    {
      measuredvbat = analogRead(VBATPIN);
      measuredvbat *= 2;    // we divided by 2, so multiply back
      measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
      measuredvbat /= 1024; // convert to voltage
      //      measuredvbat = measuredvbat - 3.20; //3.2-4.2
      //      measuredvbat = measuredvbat * 100;
    }
    display.print("BAT: " );
    display.print(measuredvbat, 2);
    display.print("V ");

    //display.print(" | ");
    display.println();




    DateTime now = rtc.now();
    display.print("UTC: ");
    display.print(now.year(), DEC);
    display.print('/');
    display.print(now.month(), DEC);
    display.print('/');
    display.print(now.day(), DEC);
    display.print("  ");
    display.print(now.hour(), DEC);
    display.print(':');
    display.print(now.minute(), DEC);
    display.print(':');
    display.print(now.second(), DEC);
    display.println();

    //bubble level
    int xPos = display.width() - 20;
    int yPos = display.height() - 20;

    display.drawCircle(xPos, yPos, 10, 1);
    float xArrow = cos(radians(xSetpoint - x)) * 15;
    float yArrow = sin(radians(xSetpoint - x)) * 15;

    display.drawLine(xPos, yPos, xPos - xArrow, yPos + yArrow, 1);

    float xOff = (ySetpoint - y) * .2;
    float yOff = (zSetpoint - z) * .2;

    float cXPos = xPos - xOff;
    float cYPos = yPos - yOff;

    display.fillCircle(cXPos, cYPos, 7, 1);
    display.drawFastHLine(cXPos, cYPos, -5, 0);

    display.display();


    //serial/////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  Serial.print(", X_Set_Point: ");
    //  Serial.print( xSetpoint );
    //  Serial.print(", Current_X: ");
    //  Serial.print(x);
    //  Serial.print(", X_Speed: ");
    //  Serial.println(xOutput);



  }
  //buttons/////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (!digitalRead(BUTTON_A)) {
    Kp += 0.1;
    xPID.SetTunings( Kp,  Ki,  Kd);
    // Debounce
    delay(100);

    m1->setSpeed(0);
    m2->setSpeed(0);
    m4->setSpeed(0);
    delay(1000);
  }

  if (!digitalRead(BUTTON_B)) {
    //    Kp -= 0.1;
    //    xPID.SetTunings( Kp,  Ki,  Kd);
    chargeMode = !chargeMode;
    // Debounce
    delay(100);


  }

}
