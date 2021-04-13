
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
v

oid SERCOM0_0_Handler()
{
  Serial2.IrqHandler();
}
//
void SERCOM0_2_Handler()
{
  Serial2.IrqHandler();
}


//emic2 datasheet
//https://cdn.sparkfun.com/datasheets/Components/General/30016-Emic2TextToSpeech-v1.1.pdf


void setup() {
  Serial2.begin(9600);

  pinPeripheral(A5, PIO_SERCOM_ALT);
  pinPeripheral(A4, PIO_SERCOM_ALT);

  delay(3000);
  //Serial2.write("V10\n");//Vx Set audio volume (dB): x = -48 to 18
  Serial2.write("SHELLO there. I am zenolith\n");
}

void loop() {



}
