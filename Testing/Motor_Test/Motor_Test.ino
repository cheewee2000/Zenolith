//https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/arduino-usage
//Zenolith Motor Test

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m4 = AFMS.getMotor(4);


void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

}

void loop() {
  motorTest(m1);
  motorTest(m2);
  motorTest(m4);
}


void motorTest(Adafruit_DCMotor *m) {
  uint8_t i;
  int d = 20; //lower values here stalls motor

  m->run(FORWARD);
  for (i = 0; i < 255; i+=5) {
    m->setSpeed(i);
    delay(d);
  }
  delay(500);
  for (i = 255; i != 0; i-=5) {
    m->setSpeed(i);
    delay(d);
  }

  m->run(BACKWARD);
  for (i = 0; i < 255; i+=5) {
    m->setSpeed(i);
    delay(d);
  }
  delay(500);
  for (i = 255; i != 0; i-=5) {
    m->setSpeed(i);
    delay(d);
  }

  m->run(RELEASE);
  delay(1000);




}
