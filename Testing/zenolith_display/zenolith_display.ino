#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
 
Adafruit_SH110X display = Adafruit_SH110X(64, 128, &Wire);
 

  #define BUTTON_A  9
  #define BUTTON_B  6
  #define BUTTON_C  5
 
void setup() {
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
  display.setCursor(0,0);
  display.print("Connecting to SSID\n'adafruit':");
  display.print("connected!");
  display.println("IP: 10.0.1.23");
  display.println("Sending val #0");
  display.display(); // actually display all of the above
}
 
void loop() {
  if(!digitalRead(BUTTON_A)) display.print("A");
//  if(!digitalRead(BUTTON_B)) display.print("B");
//  if(!digitalRead(BUTTON_C)) display.print("C");
  delay(100);
  yield();
  display.display();
}
