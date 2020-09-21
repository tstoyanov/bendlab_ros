# bendlab_ros

Contains ROS drivers for the bendlab two-axis sensor over an arduino interface.

Requires rosserial and rosserial_arduino (see <http://wiki.ros.org/rosserial_arduino>)

There is a customized ros message BendingSensorReadings.msg, please copy rosserial_msgs folder here to rosserial-xxx(your ros version)-devel. Basically you need to replace the CMakeLists.txt and package.xml in rosserial_msgs. Then add BendingSensorReadings.msg to rosserial_msgs/msg/ . Then Generate ros_lib and put it to Arduino libraries folder.

.ino file needs to be uploaded to arduino
