/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

boolean pressed = true;
int bval = 1;
int oldbval = 1;
int PWMALL=10;
int inp1=5;
int inp2=6;
int inp3=9;
int inp4=11;
int inmag = 50;

void setup(void)
{
  //  while (!Serial);  // required for Flora & Micro

pinMode(inp1, OUTPUT);
pinMode(inp2, OUTPUT);
pinMode(inp3, OUTPUT);
pinMode(inp4, OUTPUT);
pinMode(PWMALL, OUTPUT);
  Serial.begin(115200);
  BTSetup();

}
void BTSetup()
{
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
    delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}
/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void moveForward()
{
  digitalWrite(inp1,HIGH);
  digitalWrite (inp2,LOW );
  digitalWrite(inp3,LOW  );
  digitalWrite (inp4,HIGH);//insert functino forward here.
  Serial.println("Forward");

}

void moveBack()
{
  digitalWrite(inp1, LOW);
  digitalWrite (inp2, HIGH);
  digitalWrite(inp3, HIGH );
  digitalWrite (inp4, LOW );

  Serial.println("Backward");
  digitalWrite(13, HIGH);
}

void turnRight() {
  digitalWrite(inp1,LOW); //5,6,9,11
  digitalWrite (inp2,HIGH );
  digitalWrite(inp3,LOW  );
  digitalWrite (inp4,HIGH);
  Serial.println("Initiate Right");

}

void turnLeft()
{
  digitalWrite(inp1,HIGH);
  digitalWrite (inp2,LOW );
  digitalWrite(inp3,HIGH  );
  digitalWrite (inp4,LOW);
  Serial.println("Initiate Left");
}

void nomove(){
  digitalWrite(inp1,LOW);
digitalWrite (inp2,LOW );
digitalWrite(inp3,LOW  );
digitalWrite (inp4,LOW);

}


void increase() {
  inmag = inmag + 10;
  if (inmag>250) {inmag = 255;};
  
}

void decrease() {
  inmag = inmag - 10;
  if (inmag<50) {inmag = 50;};
}

void processMove(int bval1)
{
  switch (bval1)
  {
    case 5:
      {
        moveForward(); break;
      }
    case 6:
      {
        moveBack(); break;

      }
    case 1:
      {
        nomove();
        Serial.println("Stop");
        
      }

  }
}


void processTurn(int bval1, int pressed1)
{
  Serial.print("In turn (bval1, pressed1)"); Serial.print(bval1); Serial.print(","); Serial.println(pressed1);

  if (pressed)
  {
    switch (bval1)
    {
      case 7:
        {
          turnLeft(); break;
        }
      case 8:
        {
          turnRight();  break;
        }
    }
  }
  else
  {
    Serial.println("Stop Turning");
  }
}


void processSpeed(int bval1, int pressed1)
{
  Serial.print("In Speed (bval1, pressed1)"); Serial.print(bval1); Serial.print(","); Serial.println(pressed1);

  if (pressed)
  {
    switch (bval1)
    {
      case 2:
        {
          increase(); break;
        }
      case 4:
        {
          decrease();  break;
        }
    }
  }
  else
  {
    Serial.println("Stop Turning");
  }
}

void loop(void)
{
analogWrite(PWMALL, inmag);
/* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //  if (len == 0) return;
  if ((bval == 5) || (bval == 6) || (bval == 1)) //forward, backward, stop
  {
    oldbval = bval; processMove(bval);
  }
  if (((bval == 7) || (bval == 8)) && (pressed)) //turn and on
  {
    processTurn(bval, pressed);
  }
  if (((bval == 7) || (bval == 8)) && (! pressed)) //turn and off
  {
    processTurn(bval, pressed); bval = oldbval;
  }
  if (((bval == 2) || (bval == 4)) && (pressed)) //turn and on
  {
    processSpeed(bval, pressed);
  }
  if (((bval == 2) || (bval == 4)) && (! pressed)) //turn and off
  {
    processSpeed(bval, pressed); bval = oldbval;
  }



  // Buttons
  if (packetbuffer[1] == 'B') {
    bval = packetbuffer[2] - '0';
    pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(bval);
    if (pressed) {
      Serial.println(" pressed");

    }

    else {
      Serial.println(" released");

    }
  }
  else
  {
    //Serial.println("Nothing received");
    if ((bval == 5) || (bval == 6) || (bval == 1)) {
      bval = oldbval; //in motion, continue motion (could be rest)
    }
    if ((bval == 7) || (bval == 8)) {
      Serial.println("In turn, leave it alone to processTurn");
    }
  }

  delay(10);

}
