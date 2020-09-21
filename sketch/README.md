Copy the libarary folder to your Arduino libraries. Libaray has been modified so it fits multiple devices communication.

Install rosserial and rosserial_arduino (see http://wiki.ros.org/rosserial_arduino)

Use the following line if the device has a native USB port, the following line should above #include <ros.h> in Arduino .ino file
#define USE_USBCON



i2c_scanner.ino, scanner the address of connected devices
standalone_demo.ino, , demo for connect one sensor and communicating through serial port,
multiple_dev_polling_demo, demo for connect multiple(four) sensors and communicating through serial port, using polling mode of the sensors
multiple_dev_polling_ros, demo for connect multiple(four) sensors and communicating through ROS topic

for multiple device connection, polling mode should be used instead of the interrupt mode.
when using polling mode, please check the SparkFun library, in function bool ADS::begin(uint8_t deviceAddress, TwoWire &wirePort) of SparkFun_Displacement_Sensor,Arduino_Library.cpp, the line 69 poll(); is used while line 67 run(); is commented.

examples, check the library: https://github.com/sparkfun/SparkFun_Displacement_Sensor_Arduino_Library
