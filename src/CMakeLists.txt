cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

generate_arduino_firmware(garry-firmware
	SRCS garry_uno_firmware.cpp ${ROS_LIB_DIR}/time.cpp
	BOARD uno
	PORT /dev/ttyACM0
)
