/*
  Module for testing the interface of a cheap Nordic nRF24AP (ANT+) UART module from Goodluckbuy (and various other places).

  Copyright 2013 Brody Kenrick.

  This module here lacks any documentation.....
  http://www.goodluckbuy.com/nrf24ap2-networking-module-zigbee-module-with-ant-transceiver-.html
  (and very likely [not tested] http://www.goodluckbuy.com/nrf24ap2-wireless-module-zigbee-module.html  )

  It appears to be a board to this design:
  http://www.nordicsemi.com/eng/nordic/download_resource/12549/1/38968289

  Chip datasheet is here:
  http://www.nordicsemi.com/eng/nordic/content_download/2745/34254/file/nRF24AP2_Product_Specification_v1_2.pdf


  The connector on the board is (looking from the front, pin 1 is marked []):
  GND(=VSS) | VDD(=3.3 volts)
  UART_TX   | UART_RX
  !(SUSP)   | SLEEP
  RTS       | !(RESET)
  Determined through trace analysis.

  Wiring up on an Arduino Pro Mini 3v3 to connections as in the pin consts below. In additon GND<>GND and VCC<>VDD were connected.

  Baud rate of the unit is 9600. Trace analysis shows BR1 and BR3 are high and BR2 is LOW.


*/
#include <SoftwareSerial.h>

//Arduino Pro Mini pins to the nrf24AP2 modules pinouts
const int RTS_PIN     = 2;
const int TX_PIN     = 10; //Using software serial for the UART
const int RX_PIN     = 11; //Ditto

const int BAUD_RATE = 9600;
volatile int state = 1;
SoftwareSerial mySerial(TX_PIN, RX_PIN); // RX, TX on Arduino is opposite on other module
SoftwareSerial piSerial(9, 8); // Send Data HRM to Raspbery pi

void isr_ant()
{
  state = 1;
}

int state_counter;
boolean clear_to_send = false;

void setup()
{
  // Open local serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
  {
    ;
  }

  Serial.println("nrf24AP2 Tester!");

  pinMode(RTS_PIN, INPUT);

  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each message
  attachInterrupt(0/*0==pin2==RTS_PIN*/, isr_ant, RISING);

  // Set the data rate for the SoftwareSerial port
  mySerial.begin(BAUD_RATE);
  //piSerial.begin(4800);

  Serial.println("Setup() Complete!");
}

void loop() // run over and over
{
  //Print out everything received from the ANT chip
  if (mySerial.available())
  {
    Serial.print("RECV: 0x");
    while (mySerial.available())
    {
      Serial.print("_");
      Serial.print( mySerial.read(), HEX );
    }
    Serial.println(".");
  }


  if (state == 1)
  {
    //Clear the ISR
    //state = 0;

    if ( digitalRead(RTS_PIN) == 0 )
    {
      Serial.println("Host CTS (ANT is ready to receive again).");
      clear_to_send = true;
    }
    else
    {
      Serial.print("Waiting for ANT to let us send again.");
      //Need to make sure it is low again
      while ( digitalRead(RTS_PIN) != 0 )
      {
        Serial.print(".");
        delay(50);
      }
      clear_to_send = true;
    }
  }

  if ( clear_to_send )
  {
    if ((state_counter % 2) == 0)
    {
      Serial.println("Wait...");
      delay(500);
    }
    else if (state_counter == 3)
    {
      Serial.println("RESET_SYSTEM()");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x01);
      mySerial.write((uint8_t)0x4A);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0xEF);
      //Padding
      mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 5)
    {
      Serial.println("REQUEST( CAPS )");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x02);
      mySerial.write((uint8_t)0x4D);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x54);
      mySerial.write((uint8_t)0xBF);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 7)
    {
      Serial.println("CHANNEL(ASSIGN)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x03);
      mySerial.write((uint8_t)0x42);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0xE5);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 9)
    {
      Serial.println("CHANNEL ID(SET)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x05);
      mySerial.write((uint8_t)0x51);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)120);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0x88);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 11)
    {
      Serial.println("NET WORK KEY(SET)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x09);
      mySerial.write((uint8_t)0x46);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0xB9);
      mySerial.write((uint8_t)0xA5);
      mySerial.write((uint8_t)0x21);
      mySerial.write((uint8_t)0xFB);
      mySerial.write((uint8_t)0xBD);
      mySerial.write((uint8_t)0x72);
      mySerial.write((uint8_t)0xC3);
      mySerial.write((uint8_t)0x45);
      mySerial.write((uint8_t)0x64);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 13)
    {
      Serial.println("SEARCH TIME OUT(SET)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x02);
      mySerial.write((uint8_t)0x44);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)12);
      mySerial.write((uint8_t)0xEE);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 15)
    {
      Serial.println("RF FREQUENCY(SET)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x02);
      mySerial.write((uint8_t)0x45);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)57);
      mySerial.write((uint8_t)0xDA);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 17)
    {
      Serial.println("Channel Period(SET)");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x03);
      mySerial.write((uint8_t)0x43);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)(32280 & 0x00FF));
      mySerial.write((uint8_t)((32280 & 0xFF00) >> 8));
      mySerial.write((uint8_t)0x82);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    else if (state_counter == 19)
    {
      Serial.println("OPEN CHANNEL");
      mySerial.write((uint8_t)0xA4);
      mySerial.write((uint8_t)0x01);
      mySerial.write((uint8_t)0x4B);
      mySerial.write((uint8_t)0x00);
      mySerial.write((uint8_t)0xEE);
      //Padding
      //mySerial.write((uint8_t)0x00);
      state = 0;
      //clear_to_send = false;
    }
    state_counter++;
  }
}




