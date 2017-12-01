#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_MMA8452Q.h> 

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Define battery pin
#define VBATPIN A9

MMA8452Q accel;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Motor Mapping
/*
 * driveMotor1 : M1
 * driveMotor2 : M2
 * driveMotor3 : M3
 * driveMotor4 : M4
 */
//

Adafruit_DCMotor *driveMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *driveMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *driveMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *driveMotor4 = AFMS.getMotor(4);

//


// Initialize LEDs
const int redLED = 11;   // 11 on small, 9 on std
const int greenLED = 10; // 10 on small, 6 on std
const int blueLED = 5;  // 5 on small, 3 on std
const int MAX_BRIGHTNESS = 255;
///

///////////BEGIN ADAFRUIT BLUEFRUIT INITIALIZATION//////////////////
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void error(const __FlashStringHelper*err) {
 // Serial.println(err);
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
void setup(void)
{
  
  //accel.init();
  AFMS.begin();

  // Set the speed to start, from 0 (off) to 255 (max speed)
  driveMotor1->setSpeed(150);
  driveMotor2->setSpeed(150);
  driveMotor3->setSpeed(150);
  driveMotor4->setSpeed(150);

  driveMotor1->run(FORWARD);
  driveMotor1->run(RELEASE);

  driveMotor2->run(FORWARD);
  driveMotor2->run(RELEASE);

  driveMotor3->run(FORWARD);
  driveMotor3->run(RELEASE);

  driveMotor4->run(FORWARD);
  driveMotor4->run(RELEASE);

//// Only run this if using Arduino 
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  while (!Serial);  // required for Flora & Micro
  delay(500);
#endif
////
  Serial.begin(115200); // serial monitor

  /* Initialise the module */
 // Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
 // Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
 //   Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

//  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

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
  //  Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
//  Serial.println(F("Switching to DATA mode!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

///  Serial.println(F("******************************"));

}
///////////END ADAFRUIT BLUEFRUIT INITIALIZATION//////////////////


volatile int currentMotorSpeed = 150;
float prevvbat = 0;

void loop()
{

/// Accelerometer code
/*
  if (accel.available())
  {
    // First, use accel.read() to read the new variables:
    accel.read();
    
    // accel.read() will update two sets of variables. 
    // * int's x, y, and z will store the signed 12-bit values 
    //   read out of the accelerometer.
    // * floats cx, cy, and cz will store the calculated 
    //   acceleration from those 12-bit values. These variables 
    //   are in units of g's.
    // Check the two function declarations below for an example
    // of how to use these variables.
    printCalculatedAccels();
    //printAccels(); // Uncomment to print digital readings
    
    // The library also supports the portrait/landscape detection
    //  of the MMA8452Q. Check out this function declaration for
    //  an example of how to use that.
    printOrientation();
    
    Serial.println(); // Print new line every time.
  }
*/
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  // Read packets sent from app and assign their values to the LED variables 
  // acceptable values: 0-255
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];

    analogWrite(redLED, red);
    analogWrite(greenLED, green);
    analogWrite(blueLED, blue);   
    
  }
////
    // Buttons
    // Button Mapping
    /*
     * Button 1 : 1
     * Button 2 : 2
     * Button 3 : 3
     * Button 4 : 4
     * Button 5 : UP 
     * Button 6 : DOWN
     * Button 7 : LEFT
     * Button 8 : RIGHT
     */

//Serial.println(currentMotorSpeed);
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (pressed) {

      //Speed Control Modes
      if(buttnum == 1){
        currentMotorSpeed = 150;
      }

      if(buttnum == 2){
        currentMotorSpeed = 180;
      }

      if(buttnum == 3){
        currentMotorSpeed = 250;
      }

      if(buttnum == 4){
        currentMotorSpeed = 115;
      }

      // forward (prototype1) 
      if(buttnum == 5){
        
        driveMotor1->setSpeed(currentMotorSpeed);
        driveMotor2->setSpeed(currentMotorSpeed);
        driveMotor3->setSpeed(currentMotorSpeed);
        driveMotor4->setSpeed(currentMotorSpeed);

        driveMotor1->run(BACKWARD);
        driveMotor4->run(BACKWARD);

        driveMotor2->run(FORWARD);
        driveMotor3->run(FORWARD);

        analogWrite(greenLED, MAX_BRIGHTNESS);

      }

      // back
      if(buttnum == 6){

        driveMotor1->setSpeed(currentMotorSpeed);
        driveMotor2->setSpeed(currentMotorSpeed);
        driveMotor3->setSpeed(currentMotorSpeed);
        driveMotor4->setSpeed(currentMotorSpeed);

        
        driveMotor1->run(FORWARD);
        driveMotor4->run(FORWARD);

        driveMotor2->run(BACKWARD);
        driveMotor3->run(BACKWARD);

        analogWrite(redLED, MAX_BRIGHTNESS);

      }

      // right 
      if(buttnum == 8){

        
        driveMotor1->setSpeed(currentMotorSpeed);
        driveMotor2->setSpeed(currentMotorSpeed);
        driveMotor3->setSpeed(currentMotorSpeed);
        driveMotor4->setSpeed(currentMotorSpeed);
        
        analogWrite(greenLED, MAX_BRIGHTNESS);
        driveMotor1->run(FORWARD);
        driveMotor2->run(FORWARD);
        driveMotor3->run(FORWARD);
        driveMotor4->run(FORWARD);


        analogWrite(blueLED, MAX_BRIGHTNESS);
      }

      // left
      if(buttnum == 7){

        driveMotor1->setSpeed(currentMotorSpeed);
        driveMotor2->setSpeed(currentMotorSpeed);
        driveMotor3->setSpeed(currentMotorSpeed);
        driveMotor4->setSpeed(currentMotorSpeed);

        analogWrite(redLED, MAX_BRIGHTNESS);
        driveMotor1->run(BACKWARD);
        driveMotor2->run(BACKWARD);
        driveMotor3->run(BACKWARD);
        driveMotor4->run(BACKWARD);

        analogWrite(redLED, MAX_BRIGHTNESS);
        analogWrite(greenLED, 0);
        analogWrite(blueLED, MAX_BRIGHTNESS);
      }
      
    } else {
      // button release states

      // forward release
      if(buttnum == 5){
        driveMotor1->run(RELEASE);
        driveMotor2->run(RELEASE);
        driveMotor3->run(RELEASE);
        driveMotor4->run(RELEASE);

        analogWrite(greenLED, 0);
      } 
      
      // backward release
      if(buttnum == 6){
        //driveMotor1->setSpeed(0);
        driveMotor1->run(RELEASE);
        driveMotor2->run(RELEASE);
        driveMotor3->run(RELEASE);
        driveMotor4->run(RELEASE);

        analogWrite(redLED, 0);
      }

      // left release
      if(buttnum == 7){
        
        driveMotor1->run(RELEASE);
        driveMotor2->run(RELEASE);
        driveMotor3->run(RELEASE);
        driveMotor4->run(RELEASE);
        
        analogWrite(blueLED, 0);
        analogWrite(redLED, 0);

      }

      // right release
      if(buttnum == 8){
        
        driveMotor1->run(RELEASE);
        driveMotor2->run(RELEASE);
        driveMotor3->run(RELEASE);
        driveMotor4->run(RELEASE);
        
        analogWrite(redLED, 0);
        analogWrite(greenLED, 0);
        analogWrite(blueLED, 0);
      }      
    }
  }

///
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
// battery voltage stats 
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  ble.print("Battery: " ); ble.println(measuredvbat);
#endif
///

}

/// Accel Helper Functions
/*
///
// The function demonstrates how to use the accel.x, accel.y and
//  accel.z variables.
// Before using these variables you must call the accel.read()
//  function!
void printAccels(){
  Serial.print(accel.x, 3);
  Serial.print("\t");
  Serial.print(accel.y, 3);
  Serial.print("\t");
  Serial.print(accel.z, 3);
  Serial.print("\t");
}

// This function demonstrates how to use the accel.cx, accel.cy,
//  and accel.cz variables.
// Before using these variables you must call the accel.read()
//  function!
void printCalculatedAccels(){ 
  Serial.print(accel.cx, 3);
  Serial.print("\t");
  Serial.print(accel.cy, 3);
  Serial.print("\t");
  Serial.print(accel.cz, 3);
  Serial.print("\t");
}

// This function demonstrates how to use the accel.readPL()
// function, which reads the portrait/landscape status of the
// sensor.
void printOrientation(){
  // accel.readPL() will return a byte containing information
  // about the orientation of the sensor. It will be either
  // PORTRAIT_U, PORTRAIT_D, LANDSCAPE_R, LANDSCAPE_L, or
  // LOCKOUT.
  byte pl = accel.readPL();
  switch (pl)
  {
  case PORTRAIT_U:
    Serial.print("Portrait Up");
    break;
  case PORTRAIT_D:
    Serial.print("Portrait Down");
    break;
  case LANDSCAPE_R:
    Serial.print("Landscape Right");
    break;
  case LANDSCAPE_L:
    Serial.print("Landscape Left");
    break;
  case LOCKOUT:
    Serial.print("Flat");
    break;
  }
}
*/


