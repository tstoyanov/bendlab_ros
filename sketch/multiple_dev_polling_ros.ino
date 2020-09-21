//Use the following line if the device has a native USB port, the following line should above #include <ros.h>
#define USE_USBCON

#include <ros.h>
#include <rosserial_msgs/BendingSensorReadings.h>


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

#define DEVICE_NUM 4

ADS myFlexSensor1; //Create object of the ADS class
ADS myFlexSensor2; //Create object of the ADS class
ADS myFlexSensor3; //Create object of the ADS class
ADS myFlexSensor4; //Create object of the ADS class


//Set up the ros node and publisher
rosserial_msgs::BendingSensorReadings sensor_reading_msg;
ros::Publisher pub_temp("BendingSensorReadings", &sensor_reading_msg);
ros::NodeHandle nh;


void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_temp);
  sensor_reading_msg.values = (float*)malloc(sizeof(float) * DEVICE_NUM * 2);
  sensor_reading_msg.values_length = DEVICE_NUM * 2;
  
  Wire.begin();

  //Setup first sensor - look for it at the default address of 0x13 = 19
  if (myFlexSensor1.begin(ADDR_SENSOR_1) == false)
  {
    nh.loginfo("First sensor not detected. Check wiring. Freezing...");
    while (1)
      ;
  }
  nh.loginfo("Two Axis ADS 1 initialization succeeded");

  //Setup second sensor - look for it at the I2C address of 45. You should have set this up in example 6
  if (myFlexSensor2.begin(ADDR_SENSOR_2) == false)
  {    
    nh.loginfo("Second sensor not detected. Check wiring. Freezing...");
    while (1)
      ;
  }
  nh.loginfo("Two Axis ADS 2 initialization succeeded");
  
  if (myFlexSensor3.begin(ADDR_SENSOR_3) == false)
  {    
    nh.loginfo("3st sensor not detected. Check wiring. Freezing...");
    while (1)
      ;
  }
  nh.loginfo("Two Axis ADS 3 initialization succeeded");
  
  if (myFlexSensor4.begin(ADDR_SENSOR_4) == false)
  {    
    nh.loginfo("4th sensor not detected. Check wiring. Freezing...");
    while (1)
      ;
  }
  nh.loginfo("Two Axis ADS 4 initialization succeeded");
}

void loop()
{
  if (myFlexSensor1.available() == true && myFlexSensor2.available() == true&& myFlexSensor3.available() == true&& myFlexSensor4.available() == true)
  {

    sensor_reading_msg.header.seq = 0;
    sensor_reading_msg.header.stamp = nh.now();
    sensor_reading_msg.values[0] = myFlexSensor1.getX();
    sensor_reading_msg.values[1] = myFlexSensor1.getY();
    
    sensor_reading_msg.values[2] = myFlexSensor2.getX();
    sensor_reading_msg.values[3] = myFlexSensor2.getY();
    
    sensor_reading_msg.values[4] = myFlexSensor3.getX();
    sensor_reading_msg.values[5] = myFlexSensor3.getY();
    
    sensor_reading_msg.values[6] = myFlexSensor4.getX();
    sensor_reading_msg.values[7] = myFlexSensor4.getY();
    
    pub_temp.publish(&sensor_reading_msg);
  }
  delay(10);
  nh.spinOnce();
}
