cmake_minimum_required(VERSION 2.8.12)
project(polhemus_ros_driver)

add_compile_options(-std=c++11)

find_package(PkgConfig REQUIRED)

pkg_check_modules(libusb-1.0 libusb-1.0 QUIET)

if(NOT ${libusb_1_FOUND})
  message(FATAL_ERROR "Could not find libusb1")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2
  tf2_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polhemus
  CATKIN_DEPENDS geometry_msgs tf2 tf2_ros roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_1_INCLUDE_DIRS}
)

add_library(polhemus
  src/liberty.cpp
  src/viper.cpp
  src/polhemus.cpp
)

add_executable(polhemus_tf_broadcaster src/polhemus_tf_broadcaster.cpp)
target_link_libraries(polhemus_tf_broadcaster
  ${catkin_LIBRARIES}
  ${LIBUSB_1_LIBRARIES}
  usb-1.0
  polhemus
)

install(DIRECTORY include/polhemus_ros_driver
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS polhemus_tf_broadcaster polhemus
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES launch/start.launch 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
