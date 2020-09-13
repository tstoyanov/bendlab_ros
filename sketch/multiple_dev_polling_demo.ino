/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  This example shows how read two sensors at the same time.
  This requires that you've already run the ChangeAddress example and
  set the address of one sensor to 45 (0x2D). The other sensor
  should remain at the default address of 19 (0x13).

  This example presumes you're using two 2-axis sensors but can be pared
  down to 1-axis sensors.

  This example uses software polling but checkout the Interrupts example
  to see how to check the DRDY pins.

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a sensor from SparkFun!
  https://www.sparkfun.com/products/15245 (2-axis sensor)
  https://www.sparkfun.com/products/15244 (1-axis sensor)

  Hardware Connections:
  Use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to connect to the RedBoard Qwiic and the following pins on the ADS:
  SCL: Yellow wire on Qwiic cable
  SDA: Blue
  VCC: Red
  GND: Black

  Single axis pinout: https://cdn.sparkfun.com/assets/9/f/8/2/d/Bendlabs_Single_Axis_Flex_Sensor_Pinout.png
  Dual axis pintout: https://cdn.sparkfun.com/assets/f/f/9/e/6/Bendlabs_Dual_Axis_Flex_Sensor_Pinout.png

  Open the serial monitor at 115200 baud to see the output
*/

#define I2C1  /// use sda1 and scl1

#include <Wire.h>
#ifdef I2C1
extern TwoWire Wire1;
#define Wire Wire1
#endif



#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

#define ADDR_SENSOR_1 0x13
#define ADDR_SENSOR_2 0x14
#define ADDR_SENSOR_3 0x15
#define ADDR_SENSOR_4 0x16

ADS myFlexSensor1; //Create object of the ADS class
ADS myFlexSensor2; //Create object of the ADS class
ADS myFlexSensor3; //Create object of the ADS class
ADS myFlexSensor4; //Create object of the ADS class

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("SparkFun Displacement Sensor Example"));

  Wire.begin();

  //Setup first sensor - look for it at the default address of 0x13 = 19
  if (myFlexSensor1.begin(ADDR_SENSOR_1) == false)
  {
    Serial.println(F("First sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }

  //Setup second sensor - look for it at the I2C address of 45. You should have set this up in example 6
  if (myFlexSensor2.begin(ADDR_SENSOR_2) == false)
  {
    Serial.println(F("Second sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  if (myFlexSensor3.begin(ADDR_SENSOR_3) == false)
  {
    Serial.println(F("3st sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  if (myFlexSensor4.begin(ADDR_SENSOR_4) == false)
  {
    Serial.println(F("4th sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
}

void loop()
{
  if (myFlexSensor1.available() == true && myFlexSensor2.available() == true&& myFlexSensor3.available() == true&& myFlexSensor4.available() == true)
  {
    Serial.print("1,");
    Serial.print(myFlexSensor1.getX());
    Serial.print(",");
    Serial.print(myFlexSensor1.getY());
    
    Serial.print("; 2,");
    Serial.print(myFlexSensor2.getX());
    Serial.print(",");
    Serial.print(myFlexSensor2.getY());
    
    Serial.print("; 3,");
    Serial.print(myFlexSensor3.getX());
    Serial.print(",");
    Serial.print(myFlexSensor3.getY());
    
    Serial.print("; 4,");
    Serial.print(myFlexSensor4.getX());
    Serial.print(",");
    Serial.print(myFlexSensor4.getY());
    Serial.println();
  }
  delay(10);
}
