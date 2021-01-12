//may need to reset feather before uploading code. (double click reset button on feather)
//

#include <AutoPID.h>

//pid settings and gains
#define OUTPUT_MIN -255
#define OUTPUT_MAX 255
#define KP .12
#define KI .0003
#define KD 0


//#define KP .6
//#define KI .001
//#define KD 0


double x, setxPoint, outputValX;
//input/output variables passed by reference, so they are updated automatically
AutoPID xPID(&x, &setxPoint, &outputValX, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

double y, setyPoint, outputValY;
//input/output variables passed by reference, so they are updated automatically
AutoPID yPID(&y, &setyPoint, &outputValY, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

double z, setzPoint, outputValZ;
//input/output variables passed by reference, so they are updated automatically
AutoPID zPID(&z, &setzPoint, &outputValZ, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);



unsigned long lastXUpdate, lastYUpdate, lastZUpdate;

float lastX = 0;
float lastY = 0;
float lastZ = 0;


//9dof sensor
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
#define BNO055_SAMPLERATE_DELAY_MS (100)

//motors
#define FLYMOTOR_X_DIR 11 // Motor B PWM Speed
#define FLYMOTOR_X_SPEED 12 // Motor B speed

#define FLYMOTOR_Y_DIR 9 // Motor B PWM Speed
#define FLYMOTOR_Y_SPEED 10 // Motor B speed

#define FLYMOTOR_Z_DIR 5 // Motor B PWM Speed
#define FLYMOTOR_Z_SPEED 6 // Motor B speed




//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateX() {
  if ((millis() - lastXUpdate) > BNO055_SAMPLERATE_DELAY_MS) {
    //9dof sensor
    sensors_event_t event;
    bno.getEvent(&event);
    //Serial.println(event.orientation.x, 4);
    //x = event.orientation.x;
    //Serial.println(outputVal);


    //get absolute x
    float diff = event.orientation.x - lastX;
    if (diff > 300) {
      diff = diff - 360;
    }
    else   if (diff < -300) {
      diff = diff + 360;
      //x -= event.orientation.x - lastx - 360;
    }

    x += diff;

    Serial.println(x, 4);
    lastXUpdate = millis();

    lastX = event.orientation.x;

    return true;
  }
  return false;
}//void updatex


bool updateY() {
  if ((millis() - lastYUpdate) > BNO055_SAMPLERATE_DELAY_MS) {
    //9dof sensor
    sensors_event_t event;
    bno.getEvent(&event);

    //get absolute x
    float diff = event.orientation.y - lastY;
    if (diff > 300) {
      diff = diff - 360;
    }
    else   if (diff < -300) {
      diff = diff + 360;
      //x -= event.orientation.x - lastx - 360;
    }

    y += diff;

    Serial.println(y, 4);
    lastYUpdate = millis();

    lastY = event.orientation.y;

    return true;
  }
  return false;
}//void updaty


bool updateZ() {
  if ((millis() - lastZUpdate) > BNO055_SAMPLERATE_DELAY_MS) {
    //9dof sensor
    sensors_event_t event;
    bno.getEvent(&event);

    //get absolute x
    float diff = event.orientation.z - lastZ;
    if (diff > 300) {
      diff = diff - 360;
    }
    else   if (diff < -300) {
      diff = diff + 360;
      //x -= event.orientation.x - lastx - 360;
    }

    z += diff;

    Serial.println(z, 4);
    lastZUpdate = millis();

    lastZ = event.orientation.z;

    return true;
  }
  return false;
}//void updaty



void setup() {
  //motors
  pinMode( FLYMOTOR_X_DIR, OUTPUT );
  pinMode( FLYMOTOR_X_SPEED, OUTPUT );
  digitalWrite( FLYMOTOR_X_SPEED, LOW );
  digitalWrite( FLYMOTOR_X_DIR, LOW );

  pinMode( FLYMOTOR_Y_DIR, OUTPUT );
  pinMode( FLYMOTOR_Y_SPEED, OUTPUT );
  digitalWrite( FLYMOTOR_Y_SPEED, LOW );
  digitalWrite( FLYMOTOR_Y_DIR, LOW );

  pinMode( FLYMOTOR_Z_DIR, OUTPUT );
  pinMode( FLYMOTOR_Z_SPEED, OUTPUT );
  digitalWrite( FLYMOTOR_Z_SPEED, LOW );
  digitalWrite( FLYMOTOR_Z_DIR, LOW );



  //9d0f sensor
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);


  //if x is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  //myPID.setBangBang(45);
  //set PID update interval
  xPID.setTimeStep(1);
  yPID.setTimeStep(1);
  zPID.setTimeStep(1);

}


int motorSpeed = 0;
int lastDir = 0;
int dir = 0;

//float x;
//float lastX;

void loop() {

  updateX();
  setxPoint = 0;
  updateY();
  setyPoint = 0;
  updateZ();
  setzPoint = 0;

  // Serial.println(setPoint);
  xPID.run(); //call every loop, updates automatically at certain time interval
  zPID.run();
  yPID.run();


  if (millis() > 10000) {
    //int s = int(outputVal - 255.0);
    int xSpeed;
    if (outputValX < 0)xSpeed = -255 - outputValX;
    else xSpeed = 255 - outputValX;


    int ySpeed;
    if (outputValY < 0)ySpeed = -255 - outputValY;
    else ySpeed = 255 - outputValY;


    int zSpeed;
    if (outputValZ < 0)zSpeed = -255 - outputValZ;
    else zSpeed = 255 - outputValZ;



    FM(xSpeed, FLYMOTOR_X_DIR, FLYMOTOR_X_SPEED);
    FM(ySpeed, FLYMOTOR_Y_DIR, FLYMOTOR_Y_SPEED);
    FM(zSpeed, FLYMOTOR_Z_DIR, FLYMOTOR_Z_SPEED);

  }





}


//motor
void FM(int mSpeed, int motorDirPin, int motorSpeedPin) {
  if (mSpeed < 0) {
    analogWrite( motorSpeedPin,  mSpeed);
    analogWrite( motorDirPin, 0 );
  }
  else if (mSpeed > 0) {
    analogWrite( motorSpeedPin,  255 - abs(mSpeed));
    analogWrite( motorDirPin,  255);
  } else {
    analogWrite( motorSpeedPin,  0);
    analogWrite( motorDirPin, 0 );
  }
}



//sensor status
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
