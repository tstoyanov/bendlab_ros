Copy the libarary folder to your Arduino libraries. Libaray has been modified so it fits multiple devices communication.

Install rosserial and rosserial_arduino (see http://wiki.ros.org/rosserial_arduino)

Use the following line if the device has a native USB port, the following line should above #include <ros.h> in Arduino .ino file
#define USE_USBCON



i2c_scanner.ino, scanner the address of connected devices
standalone_demo.ino, , demo for connect one sensor and communicating through serial port
multiple_dev_demo, demo for connect multiple(two) sensors and communicating through serial port
multiple_dev_ros, demo for connect multiple(two) sensors and communicating through ROS topic
