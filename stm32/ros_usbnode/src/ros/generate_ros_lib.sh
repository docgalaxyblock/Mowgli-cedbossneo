
# copy rosserial from our ROS host
scp -r cedric@192.168.64.6:/home/cedric/MowgliRover/libraries/ros_lib/mowgli ros_lib/mowgli

# overwrite arduino code with STM32 code
#cp extra/* ros_lib
rm ros_lib/ArduinoHardware.h
rm ros_lib/ArduinoTcpHardware.h
