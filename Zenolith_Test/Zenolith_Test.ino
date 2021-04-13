//display//////////////////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);


#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5



//motor//////////////////////////////////////////////////////////////////

//https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/arduino-usage
//Zenolith Motor Test

#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m3 = AFMS.getMotor(3);

boolean testMotors = false;


//TTS//////////////////////////////////////////////////////////////////
//setup serial2
#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

//m4 pin mapping
//https://learn.adafruit.com/assets/78438

//set serial https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
//https://forums.adafruit.com/viewtopic.php?f=62&t=154259&p=761240&hilit=samd51+sercom+uart#p761240
//A2->TX
//A3->RX
//Uart Serial2 (&sercom4, A3, A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
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



void setup() {
  //motor//////////////////////////////////////////////////////////////////
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz


  //display//////////////////////////////////////////////////////////////////
  Serial.begin(115200);

  Serial.println("128x64 OLED FeatherWing test");
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);
  Serial.println("Button test");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Hello there. I am Zenolith.");
  display.display(); // actually display all of the above


  //TTS//////////////////////////////////////////////////////////////////
  Serial2.begin(9600);
  pinPeripheral(A5, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);
  //delay(3000);
  Serial2.write("W180\n");//Wx Set speaking rate (words/minute): x = 75 to 600
  delay(50);
  Serial2.write("V15\n");//Vx Set audio volume (dB): x = -48 to 18
  delay(50);
  Serial2.write("SHELLO there. I am zenolith. Press Button A. To begin Motor Test\n");
}

void loop() {
  //motor//////////////////////////////////////////////////////////////////
  if (testMotors) {
    motorTest(m1);
    motorTest(m2);
    motorTest(m3);
  }
  //display//////////////////////////////////////////////////////////////////

  if (!digitalRead(BUTTON_A)) {
    display.print("Testing Motors");
    Serial2.write("STesting Motors\n");
    testMotors = true;

  }
  delay(100);
  yield();
  display.display();
}





//motor//////////////////////////////////////////////////////////////////
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
