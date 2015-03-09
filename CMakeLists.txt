cmake_minimum_required(VERSION 2.8.3)
project(polhemus_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

if (USB_LIBRARIES AND USB_INCLUDE_DIRS)
  # in cache already
  set(USB_FOUND TRUE)
else (USB_LIBRARIES AND USB_INCLUDE_DIRS)
  find_path(USB_INCLUDE_DIR
    NAMES
      usb.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(USB_LIBRARY
    NAMES
      usb
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(USB_INCLUDE_DIRS
    ${USB_INCLUDE_DIR}
  )
  set(USB_LIBRARIES
    ${USB_LIBRARY}
)

  if (USB_INCLUDE_DIRS AND USB_LIBRARIES)
     set(USB_FOUND TRUE)
  endif (USB_INCLUDE_DIRS AND USB_LIBRARIES)

  if (USB_FOUND)
    if (NOT USB_FIND_QUIETLY)
      message(STATUS "Found USB: ${USB_LIBRARIES}")
    endif (NOT USB_FIND_QUIETLY)
  else (USB_FOUND)
    if (USB_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find USB")
    endif (USB_FIND_REQUIRED)
  endif (USB_FOUND)

  # show the USB_INCLUDE_DIRS and USB_LIBRARIES variables only in the advanced view
  mark_as_advanced(USB_INCLUDE_DIRS USB_LIBRARIES)

endif (USB_LIBRARIES AND USB_INCLUDE_DIRS)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf
  roscpp
  kdl_parser
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polhemus_ros
  CATKIN_DEPENDS geometry_msgs tf roscpp kdl_parser
  DEPENDS system_lib orocos_kdl eigen
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${USB_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} 
	src/liberty.cpp
)


add_executable(polhemus_ros_node src/polhemus_ros_node.cpp src/liberty.cpp src/polhemus_ros.cpp)

target_link_libraries(polhemus_ros_node
  ${catkin_LIBRARIES}
  ${USB_LIBRARIES}
)
